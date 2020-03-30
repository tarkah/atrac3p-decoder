use crate::data::*;
use crate::{ChannelUnit, ChannelUnitType, Context, Error, GCContext, GainInfo, IPQFChannelCtx};

use rustdct::mdct::{window_fn, MDCTNaive, MDCT};

pub(crate) fn decode_residual_spectrum(
    channel_unit: &mut ChannelUnit,
    out: &mut [[f32; FRAME_SAMPLES]; 2],
    num_channels: usize,
) -> Result<(), Error> {
    if channel_unit.mute_flag > 0 {
        return Ok(());
    }

    let mut sb_rng_index = [0; SUBBANDS];

    let mut rng_index = 0;
    for qu in 0..channel_unit.used_quant_units as usize {
        rng_index +=
            channel_unit.channels[0].qu_sf_idx[qu] + channel_unit.channels[1].qu_sf_idx[qu];
    }

    for sb in 0..channel_unit.num_coded_subbands as usize {
        sb_rng_index[sb] = rng_index & 0x3FC;
        rng_index += 128;
    }

    for ch in 0..num_channels as usize {
        for qu in 0..channel_unit.used_quant_units as usize {
            let nspeclines = QU_TO_SPEC_POS[qu + 1] - QU_TO_SPEC_POS[qu];

            if channel_unit.channels[ch].qu_wordlen[qu] > 0 {
                let q = SF_TAB[channel_unit.channels[ch].qu_sf_idx[qu] as usize]
                    * MANT_TAB[channel_unit.channels[ch].qu_wordlen[qu] as usize];

                let dst = &mut out[ch][QU_TO_SPEC_POS[qu] as usize..];
                let src = &channel_unit.channels[ch].spectrum[QU_TO_SPEC_POS[qu] as usize..];

                for i in 0..nspeclines as usize {
                    dst[i] = src[i] as f32 * q;
                }
            }
        }

        for sb in 0..channel_unit.num_coded_subbands as usize {
            power_compensation(
                channel_unit,
                ch,
                &mut out[ch],
                sb_rng_index[sb] as usize,
                sb,
            )?;
        }
    }

    if channel_unit.unit_type == ChannelUnitType::Stereo {
        for sb in 0..channel_unit.num_coded_subbands as usize {
            if channel_unit.swap_channels[sb] > 0 {
                for i in 0..SUBBAND_SAMPLES {
                    let swap_0 = out[0][sb * SUBBAND_SAMPLES + i];
                    let swap_1 = out[1][sb * SUBBAND_SAMPLES + i];

                    out[0][sb * SUBBAND_SAMPLES + i] = swap_1;
                    out[1][sb * SUBBAND_SAMPLES + i] = swap_0;
                }
            }

            if channel_unit.negate_coeffs[sb] > 0 {
                for i in 0..SUBBAND_SAMPLES {
                    out[1][sb * SUBBAND_SAMPLES + i] = -(out[1][sb * SUBBAND_SAMPLES + i]);
                }
            }
        }
    }

    Ok(())
}

fn power_compensation(
    channel_unit: &mut ChannelUnit,
    ch_index: usize,
    sp: &mut [f32],
    mut rng_index: usize,
    sb: usize,
) -> Result<(), Error> {
    let mut pwcsp: [f32; SUBBAND_SAMPLES] = [0.0; SUBBAND_SAMPLES];

    let swap_ch = if channel_unit.unit_type == ChannelUnitType::Stereo
        && channel_unit.swap_channels[sb] > 0
    {
        1
    } else {
        0
    };

    if channel_unit.channels[ch_index ^ swap_ch].power_levs[SUBBAND_TO_POWGRP[sb] as usize]
        == POWER_COMP_OFF
    {
        return Ok(());
    }

    for i in 0..SUBBAND_SAMPLES as usize {
        pwcsp[i] = NOISE_TAB[rng_index & 0x3FF];
        rng_index += 1;
    }

    let g1 = &channel_unit.channels[ch_index ^ swap_ch].gain_data[sb];
    let g2 = &channel_unit.channels[ch_index ^ swap_ch].gain_data_prev[sb];

    let gain_lev = if g1.num_points > 0 {
        6 - g1.lev_code[0]
    } else {
        0
    };

    let mut gcv: i32 = 0;
    for i in 0..g2.num_points as usize {
        gcv = gcv.max(gain_lev - (g2.lev_code[i] - 6));
    }

    for i in 0..g1.num_points as usize {
        gcv = gcv.max(6 - g1.lev_code[i]);
    }

    let grp_lev = PWC_LEVS[channel_unit.channels[ch_index ^ swap_ch].power_levs
        [SUBBAND_TO_POWGRP[sb] as usize] as usize]
        / (1 << gcv) as f32;

    for qu in (SUBBAND_TO_QU[sb] + if !(sb > 0) { 2 } else { 0 })..SUBBAND_TO_QU[sb + 1] {
        if channel_unit.channels[ch_index].qu_wordlen[qu as usize] <= 0 {
            continue;
        }

        let qu_lev = SF_TAB[channel_unit.channels[ch_index].qu_sf_idx[qu as usize] as usize]
            * MANT_TAB[channel_unit.channels[ch_index].qu_wordlen[qu as usize] as usize]
            / (1 << channel_unit.channels[ch_index].qu_wordlen[qu as usize]) as f32
            * grp_lev;

        let mut dst = &mut sp[QU_TO_SPEC_POS[qu as usize] as usize..];
        let nsp = QU_TO_SPEC_POS[qu as usize + 1] - QU_TO_SPEC_POS[qu as usize];

        vector_fmac_scalar(&mut dst, &pwcsp, qu_lev, nsp as usize);
    }

    Ok(())
}

pub(crate) fn reconstruct_frame(
    ctx: &mut Context,
    ch_block: usize,
    num_channels: usize,
) -> Result<(), Error> {
    let num_subbands = ctx.ch_units[ch_block].as_ref().unwrap().num_subbands;

    for ch in 0..num_channels {
        for sb in 0..num_subbands as usize {
            let wnd_shape = ctx.ch_units[ch_block].as_ref().unwrap().channels[ch].wnd_shape[sb];
            let wnd_shape_prev =
                ctx.ch_units[ch_block].as_ref().unwrap().channels[ch].wnd_shape_prev[sb];

            let wind_id = (wnd_shape_prev << 1) + wnd_shape;

            {
                let input = &mut ctx.samples[ch][sb * SUBBAND_SAMPLES..];
                let output = &mut ctx.mdct_buf[ch][sb * SUBBAND_SAMPLES..];

                imdct(input, output, wind_id, sb)?;
            }

            let ch_unit = &mut ctx.ch_units[ch_block].as_mut().unwrap();

            gain_compensation(
                &ctx.gainc_ctx,
                &ctx.mdct_buf[ch][sb * SUBBAND_SAMPLES..],
                &mut ch_unit.prev_buf[ch][sb * SUBBAND_SAMPLES..],
                &ch_unit.channels[ch].gain_data_prev[sb],
                &ch_unit.channels[ch].gain_data[sb],
                SUBBAND_SAMPLES,
                &mut ctx.time_buf[ch][sb * SUBBAND_SAMPLES..],
            )?;
        }

        let mut ch_unit = &mut ctx.ch_units[ch_block].as_mut().unwrap();

        for i in 0..((SUBBANDS - ch_unit.num_subbands as usize) * SUBBAND_SAMPLES) {
            ch_unit.prev_buf[ch][(ch_unit.num_subbands as usize * SUBBAND_SAMPLES) + i] = 0.0;
        }

        for i in 0..((SUBBANDS - ch_unit.num_subbands as usize) * SUBBAND_SAMPLES) {
            ctx.time_buf[ch][(ch_unit.num_subbands as usize * SUBBAND_SAMPLES) + i] = 0.0;
        }

        // println!("Ch {} MDCT", ch);
        // for i in 0..ctx.mdct_buf[ch].len() {
        //     print!("{},", ctx.mdct_buf[ch][i]);
        // }
        // println!("\nCh {} Time", ch);
        // for i in 0..ctx.time_buf[ch].len() {
        //     print!("{},", ctx.time_buf[ch][i]);
        // }
        // println!();

        if ch_unit.waves_info.tones_present > 0 || ch_unit.waves_info_prev.tones_present > 0 {
            for sb in 0..ch_unit.num_subbands as usize {
                if ch_unit.channels[ch].tones_info[sb].num_wavs > 0
                    || ch_unit.channels[ch].tones_info_prev[sb].num_wavs > 0
                {
                    generate_tones(&mut ch_unit, ch, sb, &mut ctx.time_buf[ch][sb * 128..])?;
                }
            }
        }

        ipqf(
            &mut ch_unit.ipqf_ctx[ch],
            &ctx.time_buf[ch],
            &mut ctx.outp_buf[ch],
        )?;
    }

    let mut ch_unit = &mut ctx.ch_units[ch_block].as_mut().unwrap();

    for ch in 0..num_channels {
        for i in 0..ch_unit.channels[ch].wnd_shape.len() {
            let swap0 = ch_unit.channels[ch].wnd_shape[i];
            let swap1 = ch_unit.channels[ch].wnd_shape_prev[i];

            ch_unit.channels[ch].wnd_shape[i] = swap1;
            ch_unit.channels[ch].wnd_shape_prev[i] = swap0;
        }

        for i in 0..ch_unit.channels[ch].gain_data.len() {
            let swap0 = ch_unit.channels[ch].gain_data[i];
            let swap1 = ch_unit.channels[ch].gain_data_prev[i];

            ch_unit.channels[ch].gain_data[i] = swap1;
            ch_unit.channels[ch].gain_data_prev[i] = swap0;
        }

        for i in 0..ch_unit.channels[ch].tones_info.len() {
            let swap0 = ch_unit.channels[ch].tones_info[i];
            let swap1 = ch_unit.channels[ch].tones_info_prev[i];

            ch_unit.channels[ch].tones_info[i] = swap1;
            ch_unit.channels[ch].tones_info_prev[i] = swap0;
        }
    }

    {
        let swap0 = ch_unit.waves_info;
        let swap1 = ch_unit.waves_info_prev;

        ch_unit.waves_info = swap1;
        ch_unit.waves_info_prev = swap0;
    }

    Ok(())
}

fn imdct(input: &mut [f32], output: &mut [f32], wind_id: u8, sb: usize) -> Result<(), Error> {
    let mut _output: [f32; MDCT_SIZE] = [0.0; MDCT_SIZE];

    if (sb & 1) > 0 {
        for i in 0..SUBBAND_SAMPLES / 2 {
            let swap0: f32 = input[i];
            let swap1: f32 = input[SUBBAND_SAMPLES - 1 - i];

            input[i] = swap1;
            input[SUBBAND_SAMPLES - 1 - i] = swap0;
        }
    }

    let mdct = MDCTNaive::<f32>::new(SUBBAND_SAMPLES, window_fn::one);

    mdct.process_imdct(&input[..SUBBAND_SAMPLES], &mut _output[..]);

    //println!("{:?}", &_output[..]);

    if (wind_id & 2) > 0 {
        for i in 0..32 {
            _output[i] = 0.0;
        }
        vector_fmul(&mut _output[32..], &SINE_64[..], 64);
    } else {
        vector_fmul(&mut _output[..], &SINE_128[..], MDCT_SIZE / 2);
    }

    if (wind_id & 1) > 0 {
        vector_fmul_reverse(&mut _output[160..], &SINE_64[..], 64);
        for i in 0..32 {
            _output[224 + i] = 0.0;
        }
    } else {
        vector_fmul_reverse(&mut _output[128..], &SINE_128[..], MDCT_SIZE / 2);
    }

    //println!("{:?}", &_output[..]);

    for i in 0..SUBBAND_SAMPLES {
        output[i] = _output[i];
    }

    Ok(())
}

fn gain_compensation(
    gctx: &GCContext,
    input: &[f32],
    prev: &mut [f32],
    gc_now: &GainInfo,
    gc_next: &GainInfo,
    num_samples: usize,
    output: &mut [f32],
) -> Result<(), Error> {
    let gc_scale = if gc_next.num_points > 0 {
        gctx.gain_tab1[gc_next.lev_code[0] as usize]
    } else {
        1.0
    };

    if !(gc_now.num_points > 0) {
        for pos in 0..num_samples {
            output[pos] = input[pos] * gc_scale + prev[pos];
        }
    } else {
        let mut pos = 0usize;

        for i in 0..gc_now.num_points as usize {
            let lastpos = gc_now.loc_code[i] << gctx.loc_scale;

            let mut lev = gctx.gain_tab1[gc_now.lev_code[i] as usize];
            let gain_inc = gctx.gain_tab2[(if (i + 1) < gc_now.num_points as usize {
                gc_now.lev_code[i + 1]
            } else {
                gctx.id2exp_offset
            }) as usize
                - gc_now.lev_code[i] as usize
                + 15];

            while pos < lastpos as usize {
                output[pos] = (input[pos] * gc_scale + prev[pos]) * lev;
                pos += 1;
            }

            while pos < (lastpos + gctx.loc_size) as usize {
                output[pos] = (input[pos] * gc_scale + prev[pos]) * lev;
                lev *= gain_inc;
                pos += 1;
            }
        }

        while pos < num_samples {
            output[pos] = input[pos] * gc_scale + prev[pos];
            pos += 1;
        }
    }

    for i in 0..num_samples {
        if i + num_samples < input.len() {
            prev[i] = input[i + num_samples];
        }
    }

    Ok(())
}

fn generate_tones(
    ch_unit: &mut ChannelUnit,
    ch_num: usize,
    sb: usize,
    output: &mut [f32],
) -> Result<(), Error> {
    unimplemented!();
}

fn ipqf(hist: &mut IPQFChannelCtx, input: &[f32], output: &mut [f32]) -> Result<(), Error> {
    unsafe { std::ptr::write_bytes(output.as_mut_ptr(), 0, FRAME_SAMPLES) };

    let mut idct_in: [f32; SUBBANDS] = [0.0; SUBBANDS];

    for s in 0..SUBBAND_SAMPLES {
        for sb in 0..SUBBANDS {
            idct_in[sb] = input[sb * SUBBAND_SAMPLES + s];
        }

        // println!("pre idmct_half input");
        // for i in 0..idct_in.len() {
        //     print!("{},", idct_in[i]);
        // }
        // println!();

        let mut idct_out: [f32; SUBBANDS * 2] = [0.0; SUBBANDS * 2];
        let mdct =
            MDCTNaive::<f32>::new(SUBBANDS, |len| (0..len).map(|_| 32.0 / 32768.0).collect());

        mdct.process_imdct(&idct_in[..], &mut idct_out[..]);
        let idct_out = &idct_out[8..24];

        // println!("post idmct_half output");
        // for i in 8..24 {
        //     print!("{:.6},", idct_out[i]);
        // }
        // println!();

        for i in 0..8 {
            hist.buf1[hist.pos as usize][i] = idct_out[i + 8];
            hist.buf2[hist.pos as usize][i] = idct_out[7 - i];
        }

        let mut pos_now = hist.pos as usize;
        let mut pos_next = MOD23_LUT[pos_now + 2] as usize;

        for t in 0..PQF_FIR_LEN {
            for i in 0..8 {
                output[s * 16 + i + 0] += hist.buf1[pos_now][i] * IPQF_COEFFS1[t][i]
                    + hist.buf2[pos_next][i] * IPQF_COEFFS2[t][i];

                output[s * 16 + i + 8] += hist.buf1[pos_now][7 - i] * IPQF_COEFFS1[t][i + 8]
                    + hist.buf2[pos_next][7 - i] * IPQF_COEFFS2[t][i + 8];
            }

            pos_now = MOD23_LUT[pos_next + 2] as usize;
            pos_next = MOD23_LUT[pos_now + 2] as usize;
        }

        hist.pos = MOD23_LUT[hist.pos as usize];
    }

    Ok(())
}

fn vector_fmac_scalar(dst: &mut [f32], src: &[f32], mul: f32, len: usize) {
    for i in 0..len {
        dst[i] += src[i] * mul;
    }
}

fn vector_fmul(dst: &mut [f32], src1: &[f32], len: usize) {
    for i in 0..len {
        dst[i] = dst[i] * src1[i];
    }
}

fn vector_fmul_reverse(dst: &mut [f32], src1: &[f32], len: usize) {
    for i in 0..len {
        dst[i] = dst[i] * src1[len - 1 - i];
    }
}
