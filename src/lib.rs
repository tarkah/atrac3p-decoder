use bitstream_io::{huffman::ReadHuffmanTree, BigEndian, BitReader};
use riff_wave_reader::RiffWaveReader;
use rustdct::{
    mdct::{window_fn, MDCTViaDCT4},
    DCTplanner,
};

use std::io::{Read, Seek, SeekFrom};

mod error;
pub use error::Error;

mod data;
use data::*;

mod decoder;
use decoder::*;

#[allow(clippy::unreadable_literal)]
const WAV_ATRAC3P_GUID: u128 = 0x62cee401faff19a14471cb58e923aabf;

pub struct Decoder<R: Read + Seek> {
    bit_reader: BitReader<R, BigEndian>,
    spec: Spec,
    context: Context,
    frame: Frame,
}

impl<R: Read + Seek> Decoder<R> {
    pub fn new(reader: R) -> Result<Decoder<R>, Error> {
        let riff_reader = RiffWaveReader::new(reader)?;

        let spec = get_spec_from_riff(&riff_reader)?;

        if !(spec.block_align > 0) {
            return Err(Error::BlockAlignNotSet);
        }

        let bit_reader = BitReader::new(riff_reader.into_reader());

        init_static();

        let mut context = Context::default();

        context.gainc_ctx = GCContext::init(6, 2);

        let mut planner = DCTplanner::new();
        let inner_dct4 = planner.plan_dct4(SUBBAND_SAMPLES);
        let mdct_ctx = MDCTViaDCT4::<f32>::new(inner_dct4, window_fn::one);
        context.mdct_ctx = Some(mdct_ctx);

        let mut planner = DCTplanner::new();
        let inner_dct4 = planner.plan_dct4(SUBBANDS);
        let ipqf_dct_ctx = MDCTViaDCT4::<f32>::new(inner_dct4, |len| {
            (0..len).map(|_| -32.0 / 32768.0).collect()
        });
        context.ipqf_dct_ctx = Some(ipqf_dct_ctx);

        set_channel_params(&mut context, &spec)?;

        for i in 0..context.num_channel_blocks as usize {
            let mut ch_unit = ChannelUnit::default();

            for ch in 0..2 {
                ch_unit.channels[ch].ch_num = ch as i32;
            }

            context.ch_units[i] = Some(ch_unit);
        }

        let frame = Frame::new();

        Ok(Decoder {
            bit_reader,
            spec,
            context,
            frame,
        })
    }

    fn next_frame(&mut self) -> Result<(), Error> {
        let block_align = self.spec.block_align as f32;
        let data_size = self.spec.data_size;
        let file_size = self.spec.file_size;

        take_mut::take(&mut self.bit_reader, |bit_reader| {
            align_to_block(bit_reader, block_align, data_size, file_size).unwrap()
        });

        decode_frame(&mut self.bit_reader, &mut self.context, &mut self.frame)?;

        Ok(())
    }
}

impl<R: Read + Seek> Iterator for Decoder<R> {
    type Item = f32;

    fn next(&mut self) -> Option<Self::Item> {
        let remaining_samples: usize = self.frame.samples.iter().map(|v| v.len()).sum();

        if remaining_samples == 0 {
            self.frame.channel_to_interleave = 0;

            if let Err(e) = self.next_frame() {
                if let Error::IOError(e) = e {
                    if e.kind() == std::io::ErrorKind::UnexpectedEof {
                        log::info!(
                            "END OF SONG, {} frames processed",
                            self.context.frame_number,
                        );
                    } else {
                        log::error!("IO ERROR: {:?}", e.kind());
                    }
                    return None;
                } else {
                    //println!("{}", self.context.ch_units[0].as_ref().unwrap());
                    log::error!("ERROR frame {}: {}", self.context.frame_number, e);
                    //panic!();
                }

                self.next()
            } else {
                self.next()
            }
        } else {
            let sample = self.frame.samples[self.frame.channel_to_interleave as usize].remove(0);

            self.frame.channel_to_interleave =
                (self.frame.channel_to_interleave + 1) % self.frame.samples.len() as u8;

            Some(sample)
        }
    }
}

impl<R: Read + Seek> rodio::Source for Decoder<R> {
    #[inline]
    fn current_frame_len(&self) -> Option<usize> {
        Some(self.frame.samples.iter().map(|v| v.len()).sum())
    }

    #[inline]
    fn channels(&self) -> u16 {
        self.spec.channels
    }

    #[inline]
    fn sample_rate(&self) -> u32 {
        self.spec.sample_rate
    }

    #[inline]
    fn total_duration(&self) -> Option<std::time::Duration> {
        let millis = (1000 * self.spec.data_size as u64 * 8) / (self.spec.byte_rate as u64 * 8);
        Some(std::time::Duration::from_millis(millis))
    }
}

#[derive(Debug, Clone, Copy)]
struct Spec {
    file_size: u32,
    data_size: u32,
    channels: u16,
    sample_rate: u32,
    block_align: u16,
    bits_per_coded_sample: u16,
    channel_mask: u32,
    byte_rate: u32,
}

fn get_spec_from_riff<R: Read + Seek>(riff_reader: &RiffWaveReader<R>) -> Result<Spec, Error> {
    let extended_info = riff_reader
        .fmt_chunk
        .extended_info
        .as_ref()
        .ok_or(Error::RiffErrorNoExtendedInfo)?;

    if extended_info.sub_format != WAV_ATRAC3P_GUID {
        return Err(Error::NotAtrac3PlusFile);
    };

    let spec = Spec {
        file_size: riff_reader.riff_chunk.file_size + 8,
        data_size: riff_reader.data_chunk.data_size,
        channels: riff_reader.fmt_chunk.num_channels,
        sample_rate: riff_reader.fmt_chunk.sample_rate,
        block_align: riff_reader.fmt_chunk.block_align,
        bits_per_coded_sample: extended_info.bits_per_coded_sample,
        channel_mask: extended_info.channel_mask,
        byte_rate: riff_reader.fmt_chunk.byte_rate,
    };

    Ok(spec)
}

unsafe impl Send for Context {}

struct Context {
    samples: [[f32; FRAME_SAMPLES]; 2],
    mdct_buf: [[f32; FRAME_SAMPLES]; 2],
    time_buf: [[f32; FRAME_SAMPLES]; 2],
    outp_buf: [[f32; FRAME_SAMPLES]; 2],
    ch_units: [Option<ChannelUnit>; 5],
    num_channel_blocks: u8,
    num_channels: u8,
    channel_blocks: [Option<ChannelUnitType>; 5],
    gainc_ctx: GCContext,
    frame_number: u64,
    mdct_ctx: Option<MDCTViaDCT4<f32>>,
    ipqf_dct_ctx: Option<MDCTViaDCT4<f32>>,
}

impl Default for Context {
    fn default() -> Self {
        Context {
            samples: [[0.0; FRAME_SAMPLES]; 2],
            mdct_buf: [[0.0; FRAME_SAMPLES]; 2],
            time_buf: [[0.0; FRAME_SAMPLES]; 2],
            outp_buf: [[0.0; FRAME_SAMPLES]; 2],
            ch_units: [None; 5],
            num_channel_blocks: Default::default(),
            num_channels: Default::default(),
            channel_blocks: [None; 5],
            gainc_ctx: Default::default(),
            frame_number: Default::default(),
            mdct_ctx: None,
            ipqf_dct_ctx: None,
        }
    }
}

#[derive(Default)]
struct GCContext {
    gain_tab1: [f32; 16],
    gain_tab2: [f32; 31],
    id2exp_offset: i32,
    loc_scale: i32,
    loc_size: i32,
}

impl GCContext {
    fn init(id2exp_offset: i32, loc_scale: i32) -> Self {
        let mut gctx = Self::default();

        gctx.loc_scale = loc_scale;
        gctx.loc_size = 1 << loc_scale;
        gctx.id2exp_offset = id2exp_offset;

        for i in 0..16 {
            gctx.gain_tab1[i] = (2.0 as f32).powf(id2exp_offset as f32 - i as f32);
        }

        let mut i: i32 = -15;
        while i < 16 {
            gctx.gain_tab2[(i + 15) as usize] =
                (2.0 as f32).powf(-1.0 / gctx.loc_size as f32 * i as f32);
            i += 1;
        }

        gctx
    }
}

fn set_channel_params(ctx: &mut Context, spec: &Spec) -> Result<(), Error> {
    ctx.num_channels = spec.channels as u8;

    match ctx.num_channels {
        1 => {
            ctx.channel_blocks[0] = Some(ChannelUnitType::Mono);
            ctx.num_channel_blocks = 1;
        }
        2 => {
            ctx.channel_blocks[0] = Some(ChannelUnitType::Stereo);
            ctx.num_channel_blocks = 1;
        }
        3 => {
            ctx.channel_blocks[0] = Some(ChannelUnitType::Stereo);
            ctx.channel_blocks[1] = Some(ChannelUnitType::Mono);
            ctx.num_channel_blocks = 2;
        }
        4 => {
            ctx.channel_blocks[0] = Some(ChannelUnitType::Stereo);
            ctx.channel_blocks[1] = Some(ChannelUnitType::Mono);
            ctx.channel_blocks[2] = Some(ChannelUnitType::Mono);
            ctx.num_channel_blocks = 3;
        }
        6 => {
            ctx.channel_blocks[0] = Some(ChannelUnitType::Stereo);
            ctx.channel_blocks[1] = Some(ChannelUnitType::Mono);
            ctx.channel_blocks[2] = Some(ChannelUnitType::Stereo);
            ctx.channel_blocks[3] = Some(ChannelUnitType::Mono);
            ctx.num_channel_blocks = 4;
        }
        7 => {
            ctx.channel_blocks[0] = Some(ChannelUnitType::Stereo);
            ctx.channel_blocks[1] = Some(ChannelUnitType::Mono);
            ctx.channel_blocks[2] = Some(ChannelUnitType::Stereo);
            ctx.channel_blocks[3] = Some(ChannelUnitType::Mono);
            ctx.channel_blocks[4] = Some(ChannelUnitType::Mono);
            ctx.num_channel_blocks = 5;
        }
        8 => {
            ctx.channel_blocks[0] = Some(ChannelUnitType::Stereo);
            ctx.channel_blocks[1] = Some(ChannelUnitType::Mono);
            ctx.channel_blocks[2] = Some(ChannelUnitType::Stereo);
            ctx.channel_blocks[3] = Some(ChannelUnitType::Stereo);
            ctx.channel_blocks[4] = Some(ChannelUnitType::Mono);
            ctx.num_channel_blocks = 5;
        }
        _ => {
            return Err(Error::UnsupportedChannelCount(spec.channels));
        }
    }

    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ChannelUnitType {
    Mono,
    Stereo,
    Extension,
    Terminator,
}

impl Default for ChannelUnitType {
    fn default() -> ChannelUnitType {
        ChannelUnitType::Mono
    }
}

impl ChannelUnitType {
    fn from_bits(bits: u8) -> Result<ChannelUnitType, Error> {
        match bits {
            0 => Ok(ChannelUnitType::Mono),
            1 => Ok(ChannelUnitType::Stereo),
            2 => Ok(ChannelUnitType::Extension),
            3 => Ok(ChannelUnitType::Terminator),
            _ => Err(Error::InvalidChannelUnitTypeBits),
        }
    }
}

#[derive(Clone, Copy)]
struct ChannelUnit {
    unit_type: ChannelUnitType,
    num_quant_units: i32,
    num_subbands: u8,
    used_quant_units: i32,
    num_coded_subbands: u8,
    mute_flag: i32,
    use_full_table: i32,
    noise_present: i32,
    noise_level_index: i32,
    noise_table_index: i32,
    swap_channels: [u8; SUBBANDS],
    negate_coeffs: [u8; SUBBANDS],
    channels: [ChannelParams; 2],
    waves_info: WaveSynthParams,
    waves_info_prev: WaveSynthParams,
    ipqf_ctx: [IPQFChannelCtx; 2],
    prev_buf: [[f32; FRAME_SAMPLES]; 2],
}

impl Default for ChannelUnit {
    fn default() -> Self {
        ChannelUnit {
            unit_type: Default::default(),
            num_quant_units: Default::default(),
            num_subbands: Default::default(),
            used_quant_units: Default::default(),
            num_coded_subbands: Default::default(),
            mute_flag: Default::default(),
            use_full_table: Default::default(),
            noise_present: Default::default(),
            noise_level_index: Default::default(),
            noise_table_index: Default::default(),
            swap_channels: Default::default(),
            negate_coeffs: Default::default(),
            channels: Default::default(),
            waves_info: Default::default(),
            waves_info_prev: Default::default(),
            ipqf_ctx: [Default::default(); 2],
            prev_buf: [[0.0; FRAME_SAMPLES]; 2],
        }
    }
}

#[derive(Default, Clone, Copy)]
struct IPQFChannelCtx {
    buf1: [[f32; 8]; PQF_FIR_LEN * 2],
    buf2: [[f32; 8]; PQF_FIR_LEN * 2],
    pos: i32,
}

struct Frame {
    samples: Vec<Vec<f32>>,
    channel_to_interleave: u8,
}

impl Frame {
    fn new() -> Frame {
        Frame {
            samples: vec![],
            channel_to_interleave: 0,
        }
    }
}

fn decode_frame<'a, R: Read + Seek>(
    mut bit_reader: &'a mut BitReader<R, BigEndian>,
    mut ctx: &'a mut Context,
    frame: &'a mut Frame,
) -> Result<(), Error> {
    frame.samples.drain(..);

    let start_marker = bit_reader.read_bit()?;

    // False = bit 0, start marker must be 0
    if start_marker {
        return Err(Error::InvalidStartBit);
    }

    ctx.frame_number += 1;

    let mut ch_block = 0usize;
    let mut ch_unit_type = ChannelUnitType::from_bits(bit_reader.read(2)?)?;
    while ch_unit_type != ChannelUnitType::Terminator {
        if ch_unit_type == ChannelUnitType::Extension {
            return Err(Error::UnsupportedChannelUnitExtension);
        }

        let mut channel_unit =
            &mut ctx.ch_units[ch_block]
                .as_mut()
                .ok_or(Error::OtherFormat(format!(
                    "Terminator not reached, tried accessing ch_block: {}",
                    ch_block
                )))?;
        channel_unit.unit_type = ch_unit_type;
        let channels_to_process: usize = match ch_unit_type {
            ChannelUnitType::Mono => 0,
            ChannelUnitType::Stereo => 1,
            _ => 0,
        } + 1;

        decode_channel_unit(&mut bit_reader, &mut channel_unit, channels_to_process)?;

        decode_residual_spectrum(&mut channel_unit, &mut ctx.samples, channels_to_process)?;

        reconstruct_frame(&mut ctx, ch_block, channels_to_process)?;

        for i in 0..channels_to_process {
            frame.samples.push(ctx.outp_buf[i].to_vec());
        }

        ch_block += 1;
        ch_unit_type = ChannelUnitType::from_bits(bit_reader.read(2)?)?;
    }

    Ok(())
}

fn decode_channel_unit<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    mut channel_unit: &'a mut ChannelUnit,
    num_channels: usize,
) -> Result<(), Error> {
    channel_unit.num_quant_units = bit_reader.read::<i32>(5)? + 1;

    if channel_unit.num_quant_units > 28 && channel_unit.num_quant_units < 32 {
        return Err(Error::OtherFormat(format!(
            "Invalid number of quantization units: {}!",
            channel_unit.num_quant_units
        )));
    }

    channel_unit.mute_flag = bit_reader.read::<i32>(1)?;

    decode_quant_wordlen(bit_reader, &mut channel_unit, num_channels)?;

    channel_unit.num_subbands = QU_TO_SUBBAND[channel_unit.num_quant_units as usize - 1] + 1;
    channel_unit.num_coded_subbands = if channel_unit.used_quant_units > 0 {
        QU_TO_SUBBAND[channel_unit.used_quant_units as usize - 1] + 1
    } else {
        0
    };

    decode_scale_factors(bit_reader, &mut channel_unit, num_channels)?;

    decode_code_table_indexes(bit_reader, &mut channel_unit, num_channels)?;

    decode_spectrum(bit_reader, &mut channel_unit, num_channels)?;

    if num_channels == 2 {
        get_subband_flags(
            bit_reader,
            &mut channel_unit.swap_channels,
            channel_unit.num_coded_subbands as usize,
        )?;

        get_subband_flags(
            bit_reader,
            &mut channel_unit.negate_coeffs,
            channel_unit.num_coded_subbands as usize,
        )?;
    }

    decode_window_shape(bit_reader, &mut channel_unit, num_channels)?;

    decode_gainc_data(bit_reader, &mut channel_unit, num_channels)?;

    decode_tones_info(bit_reader, &mut channel_unit, num_channels)?;

    channel_unit.noise_present = bit_reader.read::<i32>(1)?;
    if channel_unit.noise_present > 0 {
        channel_unit.noise_level_index = bit_reader.read::<i32>(4)?;
        channel_unit.noise_table_index = bit_reader.read::<i32>(4)?;
    }

    Ok(())
}

fn decode_quant_wordlen<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    num_channels: usize,
) -> Result<(), Error> {
    for ch_num in 0..num_channels {
        for i in 0..channel_unit.channels[ch_num].qu_wordlen.len() {
            channel_unit.channels[ch_num].qu_wordlen[i] = 0;
        }

        decode_channel_wordlen(bit_reader, channel_unit, ch_num)?;
    }

    let mut i = channel_unit.num_quant_units - 1;
    while i >= 0 {
        if channel_unit.channels[0].qu_wordlen[i as usize] > 0
            || (num_channels == 2 && channel_unit.channels[1].qu_wordlen[i as usize] > 0)
        {
            break;
        }

        i -= 1;
    }
    channel_unit.used_quant_units = i + 1;

    Ok(())
}

fn decode_channel_wordlen<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
) -> Result<(), Error> {
    let mut weight_index: Option<u8> = None;

    channel_unit.channels[ch_num].fill_mode = 0;

    let coding_mode = bit_reader.read::<u8>(2)?;
    match coding_mode {
        0 => {
            for i in 0..channel_unit.num_quant_units {
                let mut chan = &mut channel_unit.channels[ch_num];

                chan.qu_wordlen[i as usize] = bit_reader.read::<i32>(3)?;
            }
        }
        1 => {
            if ch_num > 0 {
                num_coded_units(bit_reader, ch_num, channel_unit)?;

                let ref_chan = channel_unit.channels[0];
                let mut chan = &mut channel_unit.channels[ch_num];

                if chan.num_coded_vals > 0 {
                    let vlc_sel = bit_reader.read::<u8>(2)? as usize;
                    let vlc_tab = &WL_VLC_TABS[vlc_sel];

                    for i in 0..chan.num_coded_vals as usize {
                        let delta = bit_reader.read_huffman(&vlc_tab)?;
                        chan.qu_wordlen[i] = (ref_chan.qu_wordlen[i] + delta) & 7;
                    }
                }
            } else {
                weight_index = Some(bit_reader.read::<u8>(2)?);

                num_coded_units(bit_reader, ch_num, channel_unit)?;

                let mut chan = &mut channel_unit.channels[ch_num];

                if chan.num_coded_vals > 0 {
                    let pos = bit_reader.read::<i32>(5)?;

                    if pos > chan.num_coded_vals {
                        return Err(Error::Other("WL mode 1: invalid position!"));
                    }

                    let delta_bits = bit_reader.read::<u8>(2)?;
                    let min_value = bit_reader.read::<u16>(3)?;

                    for i in 0..pos as usize {
                        chan.qu_wordlen[i] = bit_reader.read::<i32>(3)?;
                    }

                    for i in pos..chan.num_coded_vals {
                        chan.qu_wordlen[i as usize] =
                            (min_value as i32 + bit_reader.read::<i32>(delta_bits as _)?) & 7;
                    }
                }
            }
        }
        2 => {
            num_coded_units(bit_reader, ch_num, channel_unit)?;

            let ref_chan = channel_unit.channels[0];
            let mut chan = &mut channel_unit.channels[ch_num];

            if ch_num > 0 && chan.num_coded_vals > 0 {
                let vlc_tab = &WL_VLC_TABS[bit_reader.read::<u8>(2)? as usize];
                let delta = bit_reader.read_huffman(&vlc_tab)?;
                chan.qu_wordlen[0] = (ref_chan.qu_wordlen[0] + delta) & 7;

                for i in 1..chan.num_coded_vals as usize {
                    let diff = ref_chan.qu_wordlen[i] - ref_chan.qu_wordlen[i - 1];
                    let delta = bit_reader.read_huffman(&vlc_tab)?;
                    chan.qu_wordlen[i] = (chan.qu_wordlen[i - 1] + diff + delta) & 7;
                }
            } else if chan.num_coded_vals > 0 {
                let flag = bit_reader.read_bit()?;
                let vlc_tab = &WL_VLC_TABS[bit_reader.read::<u8>(1)? as usize];

                let start_val = bit_reader.read::<u8>(3)?;
                unpack_vq_shape(
                    start_val,
                    &WL_SHAPES[start_val as usize][bit_reader.read::<u8>(4)? as usize],
                    &mut chan.qu_wordlen,
                    chan.num_coded_vals as _,
                );

                if !flag {
                    for i in 0..chan.num_coded_vals as usize {
                        let delta = bit_reader.read_huffman(&vlc_tab)?;
                        chan.qu_wordlen[i] = (chan.qu_wordlen[i] + delta) & 7;
                    }
                } else {
                    let mut i = 0usize;
                    while i < (chan.num_coded_vals as isize & -2) as usize {
                        if !bit_reader.read_bit()? {
                            chan.qu_wordlen[i as usize] = (chan.qu_wordlen[i as usize]
                                + bit_reader.read_huffman(&vlc_tab)? as i32)
                                & 7;

                            chan.qu_wordlen[i as usize + 1] = (chan.qu_wordlen[i as usize + 1]
                                + bit_reader.read_huffman(&vlc_tab)? as i32)
                                & 7;
                        }

                        i += 2;
                    }

                    if chan.num_coded_vals & 1 > 0 {
                        chan.qu_wordlen[i as usize] = (chan.qu_wordlen[i as usize]
                            + bit_reader.read_huffman(&vlc_tab)? as i32)
                            & 7;
                    }
                }
            }
        }
        3 => {
            weight_index = Some(bit_reader.read::<u8>(2)?);

            num_coded_units(bit_reader, ch_num, channel_unit)?;

            let mut chan = &mut channel_unit.channels[ch_num];

            if chan.num_coded_vals > 0 {
                let vlc_tab = &WL_VLC_TABS[bit_reader.read::<u8>(2)? as usize];

                chan.qu_wordlen[0] = bit_reader.read::<i32>(3)?;

                for i in 1..chan.num_coded_vals as usize {
                    let delta = bit_reader.read_huffman(&vlc_tab)?;
                    chan.qu_wordlen[i] = (chan.qu_wordlen[i - 1] + delta) & 7;
                }
            }
        }
        _ => {} //unreachable
    }

    {
        let mut chan = &mut channel_unit.channels[ch_num];

        if chan.fill_mode == 2 {
            for i in chan.num_coded_vals..channel_unit.num_quant_units {
                chan.qu_wordlen[i as usize] = if ch_num > 0 {
                    bit_reader.read::<i32>(1)?
                } else {
                    1
                }
            }
        } else if chan.fill_mode == 3 {
            let mut pos = if ch_num > 0 {
                chan.num_coded_vals + chan.split_point
            } else {
                channel_unit.num_quant_units - chan.split_point
            } as usize;

            if pos > ff_array_elems(&chan.qu_wordlen) {
                pos = ff_array_elems(&chan.qu_wordlen);
            }

            for i in chan.num_coded_vals as usize..pos {
                chan.qu_wordlen[i] = 1;
            }
        }
    }

    if let Some(idx) = weight_index {
        if idx > 0 {
            add_wordlen_weights(channel_unit, ch_num, idx as usize)?
        }
    }

    Ok(())
}

fn num_coded_units<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    ch_num: usize,
    channel_unit: &'a mut ChannelUnit,
) -> Result<(), Error> {
    let mut chan = &mut channel_unit.channels[ch_num];

    chan.fill_mode = bit_reader.read::<i32>(2)?;

    if !(chan.fill_mode > 0) {
        chan.num_coded_vals = channel_unit.num_quant_units;
    } else {
        chan.num_coded_vals = bit_reader.read::<i32>(5)?;

        if chan.num_coded_vals > channel_unit.num_quant_units {
            return Err(Error::Other("Invalid number of transmitted units!"));
        }

        if chan.fill_mode == 3 {
            let bits = bit_reader.read::<i32>(2)?;
            chan.split_point = bits + (chan.ch_num << 1) + 1;
        }
    }

    Ok(())
}

fn add_wordlen_weights<'a>(
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
    wtab_idx: usize,
) -> Result<(), Error> {
    let mut chan = &mut channel_unit.channels[ch_num];

    let weights_tab = &WL_WEIGHTS[chan.ch_num as usize * 3 + wtab_idx - 1];

    for i in 0..channel_unit.num_quant_units {
        chan.qu_wordlen[i as usize] += weights_tab[i as usize] as i32;

        if chan.qu_wordlen[i as usize] < 0 || chan.qu_wordlen[i as usize] > 7 {
            return Err(Error::OtherFormat(format!(
                "WL index out of range: pos={}, val={}!",
                i, chan.qu_wordlen[i as usize],
            )));
        }
    }

    Ok(())
}

fn unpack_vq_shape(start_val: u8, shape_vec: &[i16], dst: &mut [i32], num_values: usize) {
    if num_values > 0 {
        dst[0] = start_val as _;
        dst[1] = start_val as _;
        dst[2] = start_val as _;

        for i in 3..num_values {
            dst[i] = start_val as i32 - shape_vec[QU_NUM_TO_SEG[i] as usize - 1] as i32;
        }
    }
}

fn decode_scale_factors<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    num_channels: usize,
) -> Result<(), Error> {
    if !(channel_unit.used_quant_units > 0) {
        return Ok(());
    }

    for ch_num in 0..num_channels {
        for i in 0..channel_unit.channels[ch_num].qu_sf_idx.len() {
            channel_unit.channels[ch_num].qu_sf_idx[i] = 0;
        }

        decode_channel_sf_idx(bit_reader, channel_unit, ch_num)?;
    }

    Ok(())
}

fn decode_channel_sf_idx<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
) -> Result<(), Error> {
    let mut weight_index: Option<u8> = None;

    let coding_mode = bit_reader.read::<u8>(2)?;
    match coding_mode {
        0 => {
            let mut chan = &mut channel_unit.channels[ch_num];

            for i in 0..channel_unit.used_quant_units as usize {
                chan.qu_sf_idx[i] = bit_reader.read::<i32>(6)?;
            }
        }
        1 => {
            if ch_num > 0 {
                let vlc_tab = &SF_VLC_TABS[bit_reader.read::<u8>(2)? as usize];

                let ref_chan = channel_unit.channels[0];
                let mut chan = &mut channel_unit.channels[ch_num];

                for i in 0..channel_unit.used_quant_units as usize {
                    let delta = bit_reader.read_huffman(&vlc_tab)?;
                    chan.qu_sf_idx[i] = (ref_chan.qu_sf_idx[i] + delta) & 0x3F;
                }
            } else {
                weight_index = Some(bit_reader.read::<u8>(2)?);
                if weight_index.unwrap() == 3 {
                    let mut chan = &mut channel_unit.channels[ch_num];

                    let start_val = bit_reader.read::<u8>(6)?;
                    unpack_vq_shape(
                        start_val,
                        &SF_SHAPES[bit_reader.read::<u8>(6)? as usize],
                        &mut chan.qu_sf_idx,
                        channel_unit.used_quant_units as usize,
                    );

                    let num_long_vals = bit_reader.read::<i32>(5)?;
                    let delta_bits = bit_reader.read::<u8>(2)?;
                    let min_val = bit_reader.read::<i32>(4)? - 7;

                    for i in 0..num_long_vals as usize {
                        chan.qu_sf_idx[i] =
                            (chan.qu_sf_idx[i] + bit_reader.read::<i32>(4)? - 7) & 0x3F;
                    }

                    for i in num_long_vals..channel_unit.used_quant_units {
                        chan.qu_sf_idx[i as usize] = (chan.qu_sf_idx[i as usize]
                            + min_val
                            + bit_reader.read::<i32>(delta_bits as _)?)
                            & 0x3F;
                    }
                } else {
                    let num_long_vals = bit_reader.read::<i32>(5)?;
                    let delta_bits = bit_reader.read::<u32>(3)?;
                    let min_val = bit_reader.read::<i32>(6)?;

                    if num_long_vals > channel_unit.used_quant_units || delta_bits == 7 {
                        return Err(Error::Other("SF mode 1: invalid parameters!"));
                    }

                    let mut chan = &mut channel_unit.channels[ch_num];

                    for i in 0..num_long_vals as usize {
                        chan.qu_sf_idx[i] = bit_reader.read::<i32>(6)?;
                    }

                    for i in num_long_vals..channel_unit.used_quant_units {
                        chan.qu_sf_idx[i as usize] =
                            (min_val + bit_reader.read::<i32>(delta_bits)?) & 0x3F;
                    }
                }
            }
        }
        2 => {
            if ch_num > 0 {
                let vlc_tab = &SF_VLC_TABS[bit_reader.read::<u8>(2)? as usize];

                let ref_chan = channel_unit.channels[0];
                let mut chan = &mut channel_unit.channels[ch_num];

                let mut delta = bit_reader.read_huffman(&vlc_tab)?;
                chan.qu_sf_idx[0] = (ref_chan.qu_sf_idx[0] + delta) & 0x3F;

                for i in 1..channel_unit.used_quant_units as usize {
                    let diff = ref_chan.qu_sf_idx[i] - ref_chan.qu_sf_idx[i - 1];
                    delta = bit_reader.read_huffman(&vlc_tab)?;
                    chan.qu_sf_idx[i] = (chan.qu_sf_idx[i - 1] + diff + delta) & 0x3F;
                }
            } else {
                let vlc_tab = &SF_VLC_TABS[bit_reader.read::<u8>(2)? as usize + 4];

                let mut chan = &mut channel_unit.channels[ch_num];

                let start_val = bit_reader.read::<u8>(6)?;
                unpack_vq_shape(
                    start_val,
                    &SF_SHAPES[bit_reader.read::<u8>(6)? as usize],
                    &mut chan.qu_sf_idx,
                    channel_unit.used_quant_units as usize,
                );

                for i in 0..channel_unit.used_quant_units as usize {
                    let delta = bit_reader.read_huffman::<i32>(&vlc_tab)?;
                    chan.qu_sf_idx[i] = (chan.qu_sf_idx[i] + sign_extend(delta, 4)) & 0x3F;
                }
            }
        }
        3 => {
            if ch_num > 0 {
                let ref_chan = channel_unit.channels[0];
                let mut chan = &mut channel_unit.channels[ch_num];

                for i in 0..channel_unit.used_quant_units {
                    chan.qu_sf_idx[i as usize] = ref_chan.qu_sf_idx[i as usize];
                }
            } else {
                let mut chan = &mut channel_unit.channels[ch_num];

                weight_index = Some(bit_reader.read::<u8>(2)?);
                let vlc_sel = bit_reader.read::<u8>(2)? as usize;
                let mut vlc_tab = &SF_VLC_TABS[vlc_sel];

                if weight_index.unwrap() == 3 {
                    vlc_tab = &SF_VLC_TABS[vlc_sel + 4];

                    let start_val = bit_reader.read::<u8>(6)?;
                    unpack_vq_shape(
                        start_val,
                        &SF_SHAPES[bit_reader.read::<u8>(6)? as usize],
                        &mut chan.qu_sf_idx,
                        channel_unit.used_quant_units as usize,
                    );

                    let mut diff = (bit_reader.read::<i32>(4)? + 56) & 0x3F;
                    chan.qu_sf_idx[0] = (chan.qu_sf_idx[0] + diff) & 0x3F;

                    for i in 1..channel_unit.used_quant_units {
                        let delta = bit_reader.read_huffman(&vlc_tab)?;
                        diff = (diff + sign_extend(delta, 4)) & 0x3F;
                        chan.qu_sf_idx[i as usize] = (diff + chan.qu_sf_idx[i as usize]) & 0x3F;
                    }
                } else {
                    chan.qu_sf_idx[0] = bit_reader.read::<i32>(6)?;

                    for i in 1..channel_unit.used_quant_units {
                        let delta = bit_reader.read_huffman(&vlc_tab)?;
                        chan.qu_sf_idx[i as usize] =
                            (chan.qu_sf_idx[i as usize - 1] + delta) & 0x3F;
                    }
                }
            }
        }
        _ => {} // unreachable
    }

    if let Some(idx) = weight_index {
        if idx > 0 && idx < 3 {
            subtract_sf_weights(channel_unit, ch_num, idx as usize)?;
        }
    }

    Ok(())
}

fn subtract_sf_weights<'a>(
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
    wtab_idx: usize,
) -> Result<(), Error> {
    let mut chan = &mut channel_unit.channels[ch_num];

    let weights_tab = &SF_WEIGHTS[wtab_idx - 1];

    for i in 0..channel_unit.used_quant_units {
        chan.qu_sf_idx[i as usize] -= weights_tab[i as usize] as i32;

        if chan.qu_sf_idx[i as usize] < 0 || chan.qu_sf_idx[i as usize] > 63 {
            return Err(Error::OtherFormat(format!(
                "SF index out of range: pos={}, val={}!",
                i, chan.qu_sf_idx[i as usize]
            )));
        }
    }

    Ok(())
}

fn decode_code_table_indexes<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    num_channels: usize,
) -> Result<(), Error> {
    if !(channel_unit.used_quant_units > 0) {
        return Ok(());
    }

    channel_unit.use_full_table = bit_reader.read::<i32>(1)?;

    for ch_num in 0..num_channels {
        for i in 0..channel_unit.channels[ch_num].qu_tab_idx.len() {
            channel_unit.channels[ch_num].qu_tab_idx[i] = 0;
        }

        decode_channel_code_tab(bit_reader, channel_unit, ch_num)?;
    }

    Ok(())
}

fn decode_channel_code_tab<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
) -> Result<(), Error> {
    let mask = if channel_unit.use_full_table > 0 {
        7
    } else {
        3
    };

    let mut pred = 0;

    {
        let mut chan = &mut channel_unit.channels[ch_num];
        chan.table_type = bit_reader.read::<i32>(1)?;
    }

    let coding_mode = bit_reader.read::<i32>(2)?;
    match coding_mode {
        0 => {
            let num_bits = channel_unit.use_full_table as u32 + 2;

            let operation = CodeTabOperation::Direct { num_bits };
            dec_ct_idx_common(bit_reader, channel_unit, ch_num, mask, &mut pred, operation)?;
        }
        1 => {
            let vlc_tab = if channel_unit.use_full_table > 0 {
                &CT_VLC_TABS[1]
            } else {
                &CT_VLC_TABS[0]
            };

            let operation = CodeTabOperation::Vlc { vlc_tab };
            dec_ct_idx_common(bit_reader, channel_unit, ch_num, mask, &mut pred, operation)?;
        }
        2 => {
            let (vlc_tab, delta_vlc_tab) = if channel_unit.use_full_table > 0 {
                (&CT_VLC_TABS[1], &CT_VLC_TABS[2])
            } else {
                (&CT_VLC_TABS[0], &CT_VLC_TABS[0])
            };

            let operation = CodeTabOperation::VlcDelta {
                vlc_tab,
                delta_vlc_tab,
            };
            dec_ct_idx_common(bit_reader, channel_unit, ch_num, mask, &mut pred, operation)?;
        }
        3 => {
            if ch_num > 0 {
                let vlc_tab = if channel_unit.use_full_table > 0 {
                    &CT_VLC_TABS[3]
                } else {
                    &CT_VLC_TABS[0]
                };

                let operation = CodeTabOperation::VlcDiff { vlc_tab };
                dec_ct_idx_common(bit_reader, channel_unit, ch_num, mask, &mut pred, operation)?;
            }
        }
        _ => {} // unreachable
    }

    Ok(())
}

fn get_subband_flags<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    out: &mut [u8],
    num_flags: usize,
) -> Result<(), Error> {
    for i in 0..num_flags {
        out[i] = 0;
    }

    let result = bit_reader.read_bit()?;
    if result {
        if bit_reader.read_bit()? {
            for i in 0..num_flags {
                out[i] = bit_reader.read::<u8>(1)?;
            }
        } else {
            for i in 0..num_flags {
                out[i] = 1;
            }
        }
    }

    Ok(())
}

enum CodeTabOperation<'a> {
    Direct {
        num_bits: u32,
    },
    Vlc {
        vlc_tab: &'a [ReadHuffmanTree<BigEndian, i32>],
    },
    VlcDelta {
        vlc_tab: &'a [ReadHuffmanTree<BigEndian, i32>],
        delta_vlc_tab: &'a [ReadHuffmanTree<BigEndian, i32>],
    },
    VlcDiff {
        vlc_tab: &'a [ReadHuffmanTree<BigEndian, i32>],
    },
}

impl<'a> CodeTabOperation<'a> {
    fn get_idx<R: Read + Seek>(
        &self,
        bit_reader: &'a mut BitReader<R, BigEndian>,
        ref_chan: ChannelParams,
        i: usize,
        mask: i32,
        pred: &'a mut i32,
    ) -> Result<i32, Error> {
        match self {
            CodeTabOperation::Direct { num_bits } => Ok(bit_reader.read::<i32>(*num_bits)?),
            CodeTabOperation::Vlc { vlc_tab } => Ok(bit_reader.read_huffman(vlc_tab)?),
            CodeTabOperation::VlcDelta {
                vlc_tab,
                delta_vlc_tab,
            } => {
                *pred = if !(i > 0) {
                    bit_reader.read_huffman(vlc_tab)?
                } else {
                    (*pred + bit_reader.read_huffman(delta_vlc_tab)?) & mask
                };
                Ok(*pred)
            }
            CodeTabOperation::VlcDiff { vlc_tab } => {
                Ok((ref_chan.qu_tab_idx[i] + bit_reader.read_huffman(vlc_tab)?) & mask)
            }
        }
    }
}

fn dec_ct_idx_common<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
    mask: i32,
    mut pred: &'a mut i32,
    operation: CodeTabOperation,
) -> Result<(), Error> {
    let num_vals = get_num_ct_values(bit_reader, channel_unit)?;

    let ref_chan = channel_unit.channels[0];
    let mut chan = &mut channel_unit.channels[ch_num];

    for i in 0..num_vals as usize {
        if chan.qu_wordlen[i] > 0 {
            chan.qu_tab_idx[i] = operation.get_idx(bit_reader, ref_chan, i, mask, &mut pred)?;
        } else if ch_num > 0 && ref_chan.qu_wordlen[i] > 0 {
            chan.qu_tab_idx[i] = bit_reader.read::<i32>(1)?;
        }
    }

    Ok(())
}

fn get_num_ct_values<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
) -> Result<i32, Error> {
    if bit_reader.read_bit()? {
        let num_coded_vals = bit_reader.read::<i32>(5)?;
        if num_coded_vals > channel_unit.used_quant_units {
            return Err(Error::OtherFormat(format!(
                "Invalid number of code table indexes: {}!",
                num_coded_vals
            )));
        }

        Ok(num_coded_vals)
    } else {
        Ok(channel_unit.used_quant_units)
    }
}

fn decode_spectrum<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    num_channels: usize,
) -> Result<(), Error> {
    let mut num_specs;

    for ch_num in 0..num_channels {
        {
            let chan = &mut channel_unit.channels[ch_num];

            for i in 0..chan.spectrum.len() {
                chan.spectrum[i] = 0;
            }

            for i in 0..chan.power_levs.len() {
                chan.power_levs[i] = POWER_COMP_OFF;
            }
        }

        for qu in 0..channel_unit.used_quant_units as usize {
            num_specs = QU_TO_SPEC_POS[qu + 1] - QU_TO_SPEC_POS[qu];

            let wordlen = channel_unit.channels[ch_num].qu_wordlen[qu];
            let mut codetab = channel_unit.channels[ch_num].qu_tab_idx[qu];

            if wordlen > 0 {
                let chan = &mut channel_unit.channels[ch_num];

                if !(channel_unit.use_full_table > 0) {
                    codetab = CT_RESTRICTED_TO_FULL[chan.table_type as usize][wordlen as usize - 1]
                        [codetab as usize] as i32;
                }

                let mut tab_index = (chan.table_type * 8 + codetab) * 7 + wordlen - 1;
                let tab = &SPECTRA_TABS[tab_index as usize];

                if tab.redirect >= 0 {
                    tab_index = tab.redirect;
                }

                let out_idx = QU_TO_SPEC_POS[qu] as usize;
                let out = &mut chan.spectrum[out_idx..];

                decode_qu_spectra(
                    bit_reader,
                    tab,
                    &SPEC_VLC_TABS[tab_index as usize],
                    out,
                    num_specs as usize,
                )?;
            } else if ch_num > 0 && channel_unit.channels[0].qu_wordlen[qu] > 0 && !(codetab > 0) {
                {
                    let src_chan = channel_unit.channels[0];
                    let src = &src_chan.spectrum[QU_TO_SPEC_POS[qu] as usize..];

                    let chan = &mut channel_unit.channels[ch_num];
                    let dst = &mut chan.spectrum[QU_TO_SPEC_POS[qu] as usize..];

                    for i in 0..src.len() {
                        dst[i] = src[i];
                    }
                }

                let src_chan = channel_unit.channels[0];
                let mut chan = &mut channel_unit.channels[ch_num];
                chan.qu_wordlen[qu] = src_chan.qu_wordlen[qu];
            }
        }

        if channel_unit.used_quant_units > 2 {
            let mut chan = &mut channel_unit.channels[ch_num];

            num_specs = SUBBAND_TO_NUM_POWGRPS[channel_unit.num_coded_subbands as usize - 1] as u16;
            for i in 0..num_specs as usize {
                chan.power_levs[i] = bit_reader.read::<u8>(4)?;
            }
        }
    }

    Ok(())
}

fn decode_qu_spectra<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    tab: &'a SpecCodeTab,
    vlc_tab: &'a Option<Box<[ReadHuffmanTree<BigEndian, u16>]>>,
    out: &'a mut [i16],
    num_specs: usize,
) -> Result<(), Error> {
    let group_size = tab.group_size;
    let num_coeffs = tab.num_coeffs;
    let bits = tab.bits;
    let is_signed = tab.is_signed;

    let mut pos = 0;
    while pos < num_specs {
        if group_size == 1 || bit_reader.read_bit()? {
            for _ in 0..group_size {
                // NEED TO RESEARCH, NULL VLC_TAB
                let mut val = if let Some(tab) = vlc_tab {
                    bit_reader.read_huffman(tab)?
                } else {
                    0
                } as i32;

                for _ in 0..num_coeffs {
                    let mut cf = val & ((1 << bits) - 1);

                    if is_signed > 0 {
                        cf = sign_extend(val, bits as usize);
                    } else if cf > 0 && bit_reader.read_bit()? {
                        cf = -cf;
                    }

                    out[pos] = cf as i16;
                    pos += 1;

                    val >>= bits;
                }
            }
        } else {
            pos += (group_size * num_coeffs) as usize;
        }
    }

    Ok(())
}

fn decode_window_shape<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    num_channels: usize,
) -> Result<(), Error> {
    for ch_num in 0..num_channels {
        get_subband_flags(
            bit_reader,
            &mut channel_unit.channels[ch_num].wnd_shape,
            channel_unit.num_subbands as usize,
        )?;
    }

    Ok(())
}

fn decode_gainc_data<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    num_channels: usize,
) -> Result<(), Error> {
    for ch_num in 0..num_channels {
        for i in 0..channel_unit.channels[ch_num].gain_data.len() {
            channel_unit.channels[ch_num].gain_data[i] = GainInfo::default();
        }

        if bit_reader.read_bit()? {
            let coded_subbands = bit_reader.read::<i32>(4)? + 1;

            if bit_reader.read_bit()? {
                channel_unit.channels[ch_num].num_gain_subbands = bit_reader.read::<i32>(4)? + 1;
            } else {
                channel_unit.channels[ch_num].num_gain_subbands = coded_subbands;
            }

            decode_gainc_npoints(bit_reader, channel_unit, ch_num, coded_subbands as usize)?;
            decode_gainc_levels(bit_reader, channel_unit, ch_num, coded_subbands as usize)?;
            decode_gainc_loc_codes(bit_reader, channel_unit, ch_num, coded_subbands as usize)?;

            if coded_subbands > 0 {
                for sb in coded_subbands..channel_unit.channels[ch_num].num_gain_subbands {
                    channel_unit.channels[ch_num].gain_data[sb as usize] =
                        channel_unit.channels[ch_num].gain_data[sb as usize - 1];
                }
            }
        } else {
            channel_unit.channels[ch_num].num_gain_subbands = 0;
        }
    }

    Ok(())
}

fn decode_gainc_npoints<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
    coded_subbands: usize,
) -> Result<(), Error> {
    let ref_chan = channel_unit.channels[0];
    let mut chan = &mut channel_unit.channels[ch_num];

    let coding_mode = bit_reader.read::<u8>(2)?;
    match coding_mode {
        0 => {
            for i in 0..coded_subbands {
                chan.gain_data[i].num_points = bit_reader.read::<i32>(3)?;
            }
        }
        1 => {
            for i in 0..coded_subbands {
                let vlc_tab = &GAIN_VLC_TABS[0];
                chan.gain_data[i].num_points = bit_reader.read_huffman(&vlc_tab)? as i32;
            }
        }
        2 => {
            if ch_num > 0 {
                for i in 0..coded_subbands {
                    let vlc_tab = &GAIN_VLC_TABS[1];
                    let delta = bit_reader.read_huffman(&vlc_tab)? as i32;

                    chan.gain_data[i].num_points = (ref_chan.gain_data[i].num_points + delta) & 7;
                }
            } else {
                let vlc_tab = &GAIN_VLC_TABS[0];
                chan.gain_data[0].num_points = bit_reader.read_huffman(&vlc_tab)? as i32;

                for i in 1..coded_subbands {
                    let vlc_tab = &GAIN_VLC_TABS[1];
                    let delta = bit_reader.read_huffman(&vlc_tab)? as i32;

                    chan.gain_data[i].num_points = (chan.gain_data[i - 1].num_points + delta) & 7;
                }
            }
        }
        3 => {
            if ch_num > 0 {
                for i in 0..coded_subbands {
                    chan.gain_data[i].num_points = ref_chan.gain_data[i].num_points;
                }
            } else {
                let delta_bits = bit_reader.read::<u32>(2)?;
                let min_val = bit_reader.read::<i32>(3)?;

                for i in 0..coded_subbands {
                    chan.gain_data[i].num_points = min_val + bit_reader.read::<i32>(delta_bits)?;

                    if chan.gain_data[i].num_points > 7 {
                        return Err(Error::InvalidData);
                    }
                }
            }
        }
        _ => {} // unreachable
    }

    Ok(())
}

fn decode_gainc_levels<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
    coded_subbands: usize,
) -> Result<(), Error> {
    let ref_chan = channel_unit.channels[0];
    let mut chan = &mut channel_unit.channels[ch_num];

    let coding_mode = bit_reader.read::<u8>(2)?;
    match coding_mode {
        0 => {
            for sb in 0..coded_subbands {
                for i in 0..chan.gain_data[sb].num_points as usize {
                    chan.gain_data[sb].lev_code[i] = bit_reader.read::<i32>(4)?;
                }
            }
        }
        1 => {
            if ch_num > 0 {
                for sb in 0..coded_subbands {
                    for i in 0..chan.gain_data[sb].num_points as usize {
                        let vlc_tab = &GAIN_VLC_TABS[5];
                        let delta = bit_reader.read_huffman(vlc_tab)?;

                        let pred = if i >= ref_chan.gain_data[sb].num_points as usize {
                            7
                        } else {
                            ref_chan.gain_data[sb].lev_code[i]
                        };

                        chan.gain_data[sb].lev_code[i] = (pred + delta as i32) & 0xF;
                    }
                }
            } else {
                for sb in 0..coded_subbands {
                    // mode1m
                    {
                        let dst = &mut chan.gain_data[sb];

                        if dst.num_points > 0 {
                            let vlc_tab = &GAIN_VLC_TABS[2];
                            dst.lev_code[0] = bit_reader.read_huffman(vlc_tab)? as i32;
                        }

                        for i in 1..dst.num_points as usize {
                            let vlc_tab = &GAIN_VLC_TABS[3];
                            let delta = bit_reader.read_huffman(vlc_tab)? as i32;

                            dst.lev_code[i] = (dst.lev_code[i - 1] + delta) & 0xF;
                        }
                    }
                }
            }
        }
        2 => {
            if ch_num > 0 {
                for sb in 0..coded_subbands {
                    if chan.gain_data[sb].num_points > 0 {
                        if bit_reader.read_bit()? {
                            // mode1m
                            {
                                let dst = &mut chan.gain_data[sb];

                                if dst.num_points > 0 {
                                    let vlc_tab = &GAIN_VLC_TABS[2];
                                    dst.lev_code[0] = bit_reader.read_huffman(vlc_tab)? as i32;
                                }

                                for i in 1..dst.num_points as usize {
                                    let vlc_tab = &GAIN_VLC_TABS[3];
                                    let delta = bit_reader.read_huffman(vlc_tab)? as i32;

                                    dst.lev_code[i] = (dst.lev_code[i - 1] + delta) & 0xF;
                                }
                            }
                        } else {
                            // mode3s
                            {
                                let _ref = &ref_chan.gain_data[sb];
                                let dst = &mut chan.gain_data[sb];

                                for i in 0..dst.num_points as usize {
                                    dst.lev_code[i] = if i >= _ref.num_points as usize {
                                        7
                                    } else {
                                        _ref.lev_code[i]
                                    };
                                }
                            }
                        }
                    }
                }
            } else {
                if chan.gain_data[0].num_points > 0 {
                    // mode1m
                    {
                        let dst = &mut chan.gain_data[0];

                        if dst.num_points > 0 {
                            let vlc_tab = &GAIN_VLC_TABS[2];
                            dst.lev_code[0] = bit_reader.read_huffman(vlc_tab)? as i32;
                        }

                        for i in 1..dst.num_points as usize {
                            let vlc_tab = &GAIN_VLC_TABS[3];
                            let delta = bit_reader.read_huffman(vlc_tab)? as i32;

                            dst.lev_code[i] = (dst.lev_code[i - 1] + delta) & 0xF;
                        }
                    }
                }

                for sb in 1..coded_subbands {
                    for i in 0..chan.gain_data[sb].num_points as usize {
                        let vlc_tab = &GAIN_VLC_TABS[4];
                        let delta = bit_reader.read_huffman(vlc_tab)? as i32;

                        let pred = if i >= chan.gain_data[sb - 1].num_points as usize {
                            7
                        } else {
                            chan.gain_data[sb - 1].lev_code[i]
                        };

                        chan.gain_data[sb].lev_code[i] = (pred + delta) & 0xF;
                    }
                }
            }
        }
        3 => {
            if ch_num > 0 {
                for sb in 0..coded_subbands {
                    // mode3s
                    {
                        let _ref = &ref_chan.gain_data[sb];
                        let dst = &mut chan.gain_data[sb];

                        for i in 0..dst.num_points as usize {
                            dst.lev_code[i] = if i >= _ref.num_points as usize {
                                7
                            } else {
                                _ref.lev_code[i]
                            };
                        }
                    }
                }
            } else {
                let delta_bits = bit_reader.read::<u32>(2)?;
                let min_val = bit_reader.read::<i32>(4)?;

                for sb in 0..coded_subbands {
                    for i in 0..chan.gain_data[sb].num_points as usize {
                        chan.gain_data[sb].lev_code[i] =
                            min_val + bit_reader.read::<i32>(delta_bits)?;

                        if chan.gain_data[sb].lev_code[i] > 15 {
                            return Err(Error::InvalidData);
                        }
                    }
                }
            }
        }
        _ => {} // uncreachable
    }

    Ok(())
}

fn decode_gainc_loc_codes<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
    coded_subbands: usize,
) -> Result<(), Error> {
    let ref_chan = channel_unit.channels[0];
    let mut chan = &mut channel_unit.channels[ch_num];

    let coding_mode = bit_reader.read::<u8>(2)?;
    match coding_mode {
        0 => {
            for sb in 0..coded_subbands {
                for i in 0..chan.gain_data[sb].num_points as usize {
                    //loc_mode0
                    {
                        let dst = &mut chan.gain_data[sb];
                        let pos = i;

                        if !(pos > 0) || dst.loc_code[pos - 1] < 15 {
                            dst.loc_code[pos] = bit_reader.read::<i32>(5)?;
                        } else if dst.loc_code[pos - 1] >= 30 {
                            dst.loc_code[pos] = 31;
                        } else {
                            let delta_bits = log2(30 - dst.loc_code[pos - 1] as u32) + 1;
                            dst.loc_code[pos] = dst.loc_code[pos - 1]
                                + bit_reader.read::<i32>(delta_bits as u32)?
                                + 1;
                        }
                    }
                }
            }
        }
        1 => {
            if ch_num > 0 {
                for sb in 0..coded_subbands {
                    if chan.gain_data[sb].num_points <= 0 {
                        continue;
                    }

                    let _ref = &ref_chan.gain_data[sb];
                    let dst = &mut chan.gain_data[sb];

                    let vlc_tab = &GAIN_VLC_TABS[10];
                    let delta = bit_reader.read_huffman(vlc_tab)?;

                    let pred = if _ref.num_points > 0 {
                        _ref.loc_code[0]
                    } else {
                        0
                    };
                    dst.loc_code[0] = (pred + delta as i32) & 0x1F;

                    for i in 1..dst.num_points as usize {
                        let more_than_ref = i >= _ref.num_points as usize;
                        if dst.lev_code[i] > dst.lev_code[i - 1] {
                            if more_than_ref {
                                let vlc_tab = &GAIN_VLC_TABS[9];
                                let delta = bit_reader.read_huffman(vlc_tab)?;

                                dst.loc_code[i] = dst.loc_code[i - 1] + delta as i32;
                            } else {
                                if bit_reader.read_bit()? {
                                    //loc_mode0
                                    {
                                        let pos = i;

                                        if !(pos > 0) || dst.loc_code[pos - 1] < 15 {
                                            dst.loc_code[pos] = bit_reader.read::<i32>(5)?;
                                        } else if dst.loc_code[pos - 1] >= 30 {
                                            dst.loc_code[pos] = 31;
                                        } else {
                                            let delta_bits =
                                                log2(30 - dst.loc_code[pos - 1] as u32) + 1;
                                            dst.loc_code[pos] = dst.loc_code[pos - 1]
                                                + bit_reader.read::<i32>(delta_bits as u32)?
                                                + 1;
                                        }
                                    }
                                } else {
                                    dst.loc_code[i] = _ref.loc_code[i];
                                }
                            }
                        } else {
                            let vlc_tab = if more_than_ref {
                                &GAIN_VLC_TABS[7]
                            } else {
                                &GAIN_VLC_TABS[10]
                            };
                            let delta = bit_reader.read_huffman(vlc_tab)?;

                            if more_than_ref {
                                dst.loc_code[i] = dst.loc_code[i - 1] + delta as i32;
                            } else {
                                dst.loc_code[i] = (_ref.loc_code[i] + delta as i32) & 0x1F;
                            }
                        }
                    }
                }
            } else {
                for sb in 0..coded_subbands {
                    // loc_mode1
                    {
                        let dst = &mut chan.gain_data[sb];

                        if dst.num_points > 0 {
                            dst.loc_code[0] = bit_reader.read::<i32>(5)?;

                            for i in 1..dst.num_points as usize {
                                let vlc_tab = if dst.lev_code[i] <= dst.lev_code[i - 1] {
                                    &GAIN_VLC_TABS[7]
                                } else {
                                    &GAIN_VLC_TABS[9]
                                };

                                dst.loc_code[i] =
                                    dst.loc_code[i - 1] + bit_reader.read_huffman(vlc_tab)? as i32;
                            }
                        }
                    }
                }
            }
        }
        2 => {
            if ch_num > 0 {
                for sb in 0..coded_subbands {
                    if chan.gain_data[sb].num_points <= 0 {
                        continue;
                    }

                    let _ref = &ref_chan.gain_data[sb];
                    let dst = &mut chan.gain_data[sb];

                    if dst.num_points > _ref.num_points || bit_reader.read_bit()? {
                        // loc_mode1
                        {
                            if dst.num_points > 0 {
                                dst.loc_code[0] = bit_reader.read::<i32>(5)?;

                                for i in 1..dst.num_points as usize {
                                    let vlc_tab = if dst.lev_code[i] <= dst.lev_code[i - 1] {
                                        &GAIN_VLC_TABS[7]
                                    } else {
                                        &GAIN_VLC_TABS[9]
                                    };

                                    dst.loc_code[i] = dst.loc_code[i - 1]
                                        + bit_reader.read_huffman(vlc_tab)? as i32;
                                }
                            }
                        }
                    } else {
                        for i in 0..dst.num_points as usize {
                            dst.loc_code[i] = _ref.loc_code[i];
                        }
                    }
                }
            } else {
                for i in 0..chan.gain_data[0].num_points as usize {
                    //loc_mode0
                    {
                        let dst = &mut chan.gain_data[0];
                        let pos = i;

                        if !(pos > 0) || dst.loc_code[pos - 1] < 15 {
                            dst.loc_code[pos] = bit_reader.read::<i32>(5)?;
                        } else if dst.loc_code[pos - 1] >= 30 {
                            dst.loc_code[pos] = 31;
                        } else {
                            let delta_bits = log2(30 - dst.loc_code[pos - 1] as u32) + 1;
                            dst.loc_code[pos] = dst.loc_code[pos - 1]
                                + bit_reader.read::<i32>(delta_bits as u32)?
                                + 1;
                        }
                    }
                }

                for sb in 1..coded_subbands {
                    if chan.gain_data[sb].num_points <= 0 {
                        continue;
                    }

                    let mut vlc_tab = &GAIN_VLC_TABS[6];
                    let delta = bit_reader.read_huffman(vlc_tab)?;

                    let pred = if chan.gain_data[sb - 1].num_points > 0 {
                        chan.gain_data[sb - 1].loc_code[0]
                    } else {
                        0
                    };

                    {
                        let dst = &mut chan.gain_data[sb];
                        dst.loc_code[0] = (pred + delta as i32) & 0x1F;
                    }

                    for i in 1..chan.gain_data[sb].num_points as usize {
                        let more_than_ref = i >= chan.gain_data[sb - 1].num_points as usize;

                        vlc_tab = &GAIN_VLC_TABS[if chan.gain_data[sb].lev_code[i]
                            > chan.gain_data[sb].lev_code[i - 1]
                        {
                            1
                        } else {
                            0
                        } * 2
                            + if more_than_ref { 1 } else { 0 }
                            + 6];

                        let delta = bit_reader.read_huffman(vlc_tab)? as i32;

                        if more_than_ref {
                            chan.gain_data[sb].loc_code[i] =
                                chan.gain_data[sb].loc_code[i - 1] + delta;
                        } else {
                            chan.gain_data[sb].loc_code[i] =
                                (chan.gain_data[sb - 1].loc_code[i] + delta) & 0x1F;
                        }
                    }
                }
            }
        }
        3 => {
            if ch_num > 0 {
                for sb in 0..coded_subbands {
                    for i in 0..chan.gain_data[sb].num_points as usize {
                        if i >= ref_chan.gain_data[sb].num_points as usize {
                            //loc_mode0
                            {
                                let dst = &mut chan.gain_data[sb];
                                let pos = i;

                                if !(pos > 0) || dst.loc_code[pos - 1] < 15 {
                                    dst.loc_code[pos] = bit_reader.read::<i32>(5)?;
                                } else if dst.loc_code[pos - 1] >= 30 {
                                    dst.loc_code[pos] = 31;
                                } else {
                                    let delta_bits = log2(30 - dst.loc_code[pos - 1] as u32) + 1;
                                    dst.loc_code[pos] = dst.loc_code[pos - 1]
                                        + bit_reader.read::<i32>(delta_bits as u32)?
                                        + 1;
                                }
                            }
                        } else {
                            chan.gain_data[sb].loc_code[i] = ref_chan.gain_data[sb].loc_code[i];
                        }
                    }
                }
            } else {
                let delta_bits = bit_reader.read::<u32>(2)? + 1;
                let min_val = bit_reader.read::<i32>(5)?;

                for sb in 0..coded_subbands {
                    for i in 0..chan.gain_data[sb].num_points as usize {
                        chan.gain_data[sb].loc_code[i] =
                            min_val + i as i32 + bit_reader.read::<i32>(delta_bits)?;
                    }
                }
            }
        }
        _ => {} // unreachable
    }

    for sb in 0..coded_subbands {
        let dst = &chan.gain_data[sb];

        for i in 0..dst.num_points as usize {
            if dst.loc_code[i] < 0
                || dst.loc_code[i] > 31
                || (i > 0 && dst.loc_code[i] <= dst.loc_code[i - 1])
            {
                return Err(Error::OtherFormat(format!(
                    "Invalid gain location: ch={} sb={} pos={} val={}",
                    ch_num, sb, i, dst.loc_code[i]
                )));
            }
        }
    }

    Ok(())
}

fn decode_tones_info<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    num_channels: usize,
) -> Result<(), Error> {
    for ch_num in 0..num_channels {
        for i in 0..channel_unit.channels[ch_num].tones_info.len() {
            channel_unit.channels[ch_num].tones_info[i] = WavesData::default();
        }
    }

    channel_unit.waves_info.tones_present = bit_reader.read::<i32>(1)?;

    if !(channel_unit.waves_info.tones_present > 0) {
        return Ok(());
    }

    for i in 0..channel_unit.waves_info.waves.len() {
        channel_unit.waves_info.waves[i] = WaveParam::default();
    }

    channel_unit.waves_info.amplitude_mode = bit_reader.read::<i32>(1)?;
    if !(channel_unit.waves_info.amplitude_mode > 0) {
        return Err(Error::Other("GHA amplitude mode 0"));
    }

    let vlc_tab = &TONE_VLC_TABS[0];
    channel_unit.waves_info.num_tone_bands = bit_reader.read_huffman(vlc_tab)? as i32 + 1;

    if num_channels == 2 {
        get_subband_flags(
            bit_reader,
            &mut channel_unit.waves_info.tone_sharing,
            channel_unit.waves_info.num_tone_bands as usize,
        )?;
        get_subband_flags(
            bit_reader,
            &mut channel_unit.waves_info.tone_master,
            channel_unit.waves_info.num_tone_bands as usize,
        )?;
        get_subband_flags(
            bit_reader,
            &mut channel_unit.waves_info.invert_phase,
            channel_unit.waves_info.num_tone_bands as usize,
        )?;
    }

    channel_unit.waves_info.tones_index = 0;

    let mut band_has_tones = [0; 16];
    for ch_num in 0..num_channels {
        for i in 0..channel_unit.waves_info.num_tone_bands as usize {
            band_has_tones[i] = if !(ch_num > 0) {
                1
            } else {
                if channel_unit.waves_info.tone_sharing[i] > 0 {
                    0
                } else {
                    1
                }
            };
        }

        decode_tones_envelope(bit_reader, channel_unit, ch_num, &band_has_tones)?;
        decode_band_numwavs(bit_reader, channel_unit, ch_num, &band_has_tones)?;
        decode_tones_frequency(bit_reader, channel_unit, ch_num, &band_has_tones)?;
        decode_tones_amplitude(bit_reader, channel_unit, ch_num, &band_has_tones)?;
        decode_tones_phase(bit_reader, channel_unit, ch_num, &band_has_tones)?;
    }

    if num_channels == 2 {
        for i in 0..channel_unit.waves_info.num_tone_bands as usize {
            if channel_unit.waves_info.tone_sharing[i] > 0 {
                channel_unit.channels[1].tones_info[i] = channel_unit.channels[0].tones_info[i];
            }

            if channel_unit.waves_info.tone_master[i] > 0 {
                let info_0 = channel_unit.channels[0].tones_info[i];
                let info_1 = channel_unit.channels[1].tones_info[i];

                channel_unit.channels[0].tones_info[i] = info_1;
                channel_unit.channels[1].tones_info[i] = info_0;
            }
        }
    }

    Ok(())
}

fn decode_tones_envelope<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
    band_has_tones: &'a [i32],
) -> Result<(), Error> {
    if !(ch_num > 0) || !bit_reader.read_bit()? {
        let dst = &mut channel_unit.channels[ch_num].tones_info;

        for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
            if !(band_has_tones[sb] > 0) {
                continue;
            }

            dst[sb].pend_env.has_start_point = bit_reader.read::<i32>(1)?;
            dst[sb].pend_env.start_pos = if dst[sb].pend_env.has_start_point > 0 {
                bit_reader.read::<i32>(5)?
            } else {
                -1
            };

            dst[sb].pend_env.has_stop_point = bit_reader.read::<i32>(1)?;
            dst[sb].pend_env.stop_pos = if dst[sb].pend_env.has_stop_point > 0 {
                bit_reader.read::<i32>(5)?
            } else {
                32
            };
        }
    } else {
        for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
            if !(band_has_tones[sb] > 0) {
                continue;
            }

            let has_start_point = channel_unit.channels[0].tones_info[sb]
                .pend_env
                .has_start_point;
            let start_pos = channel_unit.channels[0].tones_info[sb].pend_env.start_pos;
            let has_stop_point = channel_unit.channels[0].tones_info[sb]
                .pend_env
                .has_stop_point;
            let stop_pos = channel_unit.channels[0].tones_info[sb].pend_env.stop_pos;

            let dst = &mut channel_unit.channels[ch_num].tones_info;

            dst[sb].pend_env.has_start_point = has_start_point;
            dst[sb].pend_env.has_stop_point = has_stop_point;
            dst[sb].pend_env.start_pos = start_pos;
            dst[sb].pend_env.stop_pos = stop_pos;
        }
    }

    Ok(())
}

fn decode_band_numwavs<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
    band_has_tones: &'a [i32],
) -> Result<(), Error> {
    let mode = bit_reader.read::<u8>(ch_num as u32 + 1)?;
    match mode {
        0 => {
            let dst = &mut channel_unit.channels[ch_num].tones_info;

            for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
                if band_has_tones[sb] > 0 {
                    dst[sb].num_wavs = bit_reader.read::<i32>(4)?;
                }
            }
        }
        1 => {
            let dst = &mut channel_unit.channels[ch_num].tones_info;

            for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
                if band_has_tones[sb] > 0 {
                    let vlc_tab = &TONE_VLC_TABS[1];
                    dst[sb].num_wavs = bit_reader.read_huffman(&vlc_tab)? as i32;
                }
            }
        }
        2 => {
            for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
                if band_has_tones[sb] > 0 {
                    let vlc_tab = &TONE_VLC_TABS[2];
                    let mut delta = bit_reader.read_huffman(&vlc_tab)? as i32;
                    delta = sign_extend(delta, 3);

                    let ref_num_wavs = channel_unit.channels[0].tones_info[sb].num_wavs;

                    let dst = &mut channel_unit.channels[ch_num].tones_info;
                    dst[sb].num_wavs = (ref_num_wavs + delta) & 0xF;
                }
            }
        }
        3 => {
            for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
                if band_has_tones[sb] > 0 {
                    let ref_num_wavs = channel_unit.channels[0].tones_info[sb].num_wavs;

                    let dst = &mut channel_unit.channels[ch_num].tones_info;
                    dst[sb].num_wavs = ref_num_wavs;
                }
            }
        }
        _ => {} // unreachable
    }

    let dst = &mut channel_unit.channels[ch_num].tones_info;

    for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
        if band_has_tones[sb] > 0 {
            if channel_unit.waves_info.tones_index + dst[sb].num_wavs > 48 {
                return Err(Error::OtherFormat(format!(
                    // ADD NUM FRAMES TO CTX
                    "Too many tones: {} (max. 48), frame: UNIMPL!",
                    channel_unit.waves_info.tones_index + dst[sb].num_wavs
                )));
            }

            dst[sb].start_index = channel_unit.waves_info.tones_index;
            channel_unit.waves_info.tones_index += dst[sb].num_wavs;
        }
    }

    Ok(())
}

fn decode_tones_frequency<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
    band_has_tones: &'a [i32],
) -> Result<(), Error> {
    if !(ch_num > 0) || !bit_reader.read_bit()? {
        let dst = &mut channel_unit.channels[ch_num].tones_info;

        for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
            if !(band_has_tones[sb] > 0) || !(dst[sb].num_wavs > 0) {
                continue;
            }

            let mut iwav = &mut channel_unit.waves_info.waves[dst[sb].start_index as usize..];
            let direction = if dst[sb].num_wavs > 1 {
                bit_reader.read::<i32>(1)?
            } else {
                0
            };

            if direction > 0 {
                if dst[sb].num_wavs > 0 {
                    iwav[dst[sb].num_wavs as usize - 1].freq_index = bit_reader.read::<i32>(10)?;
                }

                let mut i = dst[sb].num_wavs - 2;
                while i >= 0 {
                    let nbits = log2(iwav[i as usize + 1].freq_index as u32) + 1;
                    iwav[i as usize].freq_index = bit_reader.read::<i32>(nbits as u32)?;
                    i -= 1;
                }
            } else {
                for i in 0..dst[sb].num_wavs as usize {
                    if !(i > 0) || iwav[i - 1].freq_index < 512 {
                        iwav[i].freq_index = bit_reader.read::<i32>(10)?;
                    } else {
                        let nbits = log2(1023 - iwav[i - 1].freq_index as u32) + 1;
                        iwav[i].freq_index =
                            bit_reader.read::<i32>(nbits as u32)? + 1024 - (1 << nbits);
                    }
                }
            }
        }
    } else {
        for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
            if !(band_has_tones[sb] > 0)
                || !(channel_unit.channels[ch_num].tones_info[sb].num_wavs > 0)
            {
                continue;
            }

            for i in 0..channel_unit.channels[ch_num].tones_info[sb].num_wavs as usize {
                let vlc_tab = &TONE_VLC_TABS[6];
                let mut delta = bit_reader.read_huffman(vlc_tab)? as i32;
                delta = sign_extend(delta, 8);

                let iwav = &channel_unit.waves_info.waves
                    [channel_unit.channels[0].tones_info[sb].start_index as usize..];
                let ref_num_wavs = channel_unit.channels[0].tones_info[sb].num_wavs;

                let pred = if i < ref_num_wavs as usize {
                    iwav[i].freq_index
                } else {
                    if ref_num_wavs > 0 {
                        iwav[ref_num_wavs as usize - 1].freq_index
                    } else {
                        0
                    }
                };

                let owav = &mut channel_unit.waves_info.waves
                    [channel_unit.channels[ch_num].tones_info[sb].start_index as usize..];
                owav[i].freq_index = (pred + delta) & 0x3FF;
            }
        }
    }

    Ok(())
}

fn decode_tones_amplitude<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
    band_has_tones: &'a [i32],
) -> Result<(), Error> {
    let mut refwaves: [i32; 48] = [0; 48];

    if ch_num > 0 {
        let _ref = &channel_unit.channels[0].tones_info;
        let dst = &channel_unit.channels[ch_num].tones_info;

        for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
            if !(band_has_tones[sb] > 0) || !(dst[sb].num_wavs > 0) {
                continue;
            }

            let wsrc = &channel_unit.waves_info.waves[dst[sb].start_index as usize..];
            let wref = &channel_unit.waves_info.waves[_ref[sb].start_index as usize..];

            let mut maxdiff = 1024;
            let mut fi = 0;
            for j in 0..dst[sb].num_wavs as usize {
                for i in 0.._ref[sb].num_wavs as usize {
                    let diff = (wsrc[j].freq_index - wref[i].freq_index).abs();
                    if diff < maxdiff {
                        maxdiff = diff;
                        fi = i as i32;
                    }
                }

                if maxdiff < 0 {
                    refwaves[dst[sb].start_index as usize + j] = fi + _ref[sb].start_index;
                } else if j < _ref[sb].num_wavs as usize {
                    refwaves[dst[sb].start_index as usize + j] = j as i32 + _ref[sb].start_index;
                } else {
                    refwaves[dst[sb].start_index as usize + j] = -1;
                }
            }
        }
    }

    let dst = &mut channel_unit.channels[ch_num].tones_info;

    let mode = bit_reader.read::<u8>(ch_num as u32 + 1)?;
    match mode {
        0 => {
            for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
                if !(band_has_tones[sb] > 0) || !(dst[sb].num_wavs > 0) {
                    continue;
                }

                if channel_unit.waves_info.amplitude_mode > 0 {
                    for i in 0..dst[sb].num_wavs as usize {
                        channel_unit.waves_info.waves[dst[sb].start_index as usize + i].amp_sf =
                            bit_reader.read::<i32>(6)?;
                    }
                } else {
                    channel_unit.waves_info.waves[dst[sb].start_index as usize].amp_sf =
                        bit_reader.read::<i32>(6)?;
                }
            }
        }
        1 => {
            for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
                if !(band_has_tones[sb] > 0) || !(dst[sb].num_wavs > 0) {
                    continue;
                }

                if channel_unit.waves_info.amplitude_mode > 0 {
                    for i in 0..dst[sb].num_wavs as usize {
                        let vlc_tab = &TONE_VLC_TABS[3];
                        channel_unit.waves_info.waves[dst[sb].start_index as usize + i].amp_sf =
                            bit_reader.read_huffman(vlc_tab)? as i32 + 20;
                    }
                } else {
                    let vlc_tab = &TONE_VLC_TABS[4];
                    channel_unit.waves_info.waves[dst[sb].start_index as usize].amp_sf =
                        bit_reader.read_huffman(vlc_tab)? as i32 + 24;
                }
            }
        }
        2 => {
            for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
                if !(band_has_tones[sb] > 0) || !(dst[sb].num_wavs > 0) {
                    continue;
                }

                for i in 0..dst[sb].num_wavs as usize {
                    let vlc_tab = &TONE_VLC_TABS[5];
                    let mut delta = bit_reader.read_huffman(vlc_tab)? as i32;
                    delta = sign_extend(delta, 5);
                    let pred = if refwaves[dst[sb].start_index as usize + i] >= 0 {
                        channel_unit.waves_info.waves
                            [refwaves[dst[sb].start_index as usize + i] as usize]
                            .amp_sf
                    } else {
                        34
                    };

                    channel_unit.waves_info.waves[dst[sb].start_index as usize + i].amp_sf =
                        (pred + delta) & 0x3F;
                }
            }
        }
        3 => {
            for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
                if !(band_has_tones[sb] > 0) {
                    continue;
                }

                for i in 0..dst[sb].num_wavs as usize {
                    channel_unit.waves_info.waves[dst[sb].start_index as usize + i].amp_sf =
                        if refwaves[dst[sb].start_index as usize + i] >= 0 {
                            channel_unit.waves_info.waves
                                [refwaves[dst[sb].start_index as usize + i] as usize]
                                .amp_sf
                        } else {
                            32
                        };
                }
            }
        }
        _ => {} // unreachable
    }

    Ok(())
}

fn decode_tones_phase<'a, R: Read + Seek>(
    bit_reader: &'a mut BitReader<R, BigEndian>,
    channel_unit: &'a mut ChannelUnit,
    ch_num: usize,
    band_has_tones: &'a [i32],
) -> Result<(), Error> {
    let dst = &channel_unit.channels[ch_num].tones_info;

    for sb in 0..channel_unit.waves_info.num_tone_bands as usize {
        if !(band_has_tones[sb] > 0) {
            continue;
        }

        let wparam = &mut channel_unit.waves_info.waves[dst[sb].start_index as usize..];
        for i in 0..dst[sb].num_wavs as usize {
            wparam[i].phase_index = bit_reader.read::<i32>(5)?;
        }
    }

    Ok(())
}

impl std::fmt::Display for ChannelUnit {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "--- Channel Block ---
Unit Type:        {:?}
Quant Units:      {}
Used Quant Units: {}
Mute Flag:        {}
# Subbands:       {}
# Coded Subbands: {}
Use Full Table:   {}
Noise Present:    {}
Noise Level Idx:  {}
Noise Table Idx:  {}
Swap Channels:    {:?}
Negate Coeffs:    {:?}

------- Chn 0 -------
Coded Vals:       {}
Fill Mode:        {}
Split Point:      {}
Table Type:       {}
QU Wordlen:       {:?}
QU SF Idx:        {:?}
QU Tab Idx:       {:?}
Power Levels:     {:?}
Window Shape:     {:?}
Gain Data:        {:?}
# Gain Subbands:  {}

------- Chn 1 -------
Coded Vals:       {}
Fill Mode:        {}
Split Point:      {}
Table Type:       {}
QU Wordlen:       {:?}
QU SF Idx:        {:?}
QU Tab Idx:       {:?}
Power Levels:     {:?}
Window Shape:     {:?}
Gain Data:        {:?}
# Gain Subbands:  {}

----- Waves Info ----
Tones Present:    {}
Amplitude Mode:   {}
Num Tone Bands:   {}
Tone Sharing:     {:?}
Tone Master:      {:?}
Invert Phase:     {:?}",
            self.unit_type,
            self.num_quant_units,
            self.used_quant_units,
            self.mute_flag,
            self.num_subbands,
            self.num_coded_subbands,
            self.use_full_table,
            self.noise_present,
            self.noise_level_index,
            self.noise_table_index,
            self.swap_channels,
            self.negate_coeffs,
            self.channels[0].num_coded_vals,
            self.channels[0].fill_mode,
            self.channels[0].split_point,
            self.channels[0].table_type,
            self.channels[0].qu_wordlen,
            self.channels[0].qu_sf_idx,
            self.channels[0].qu_tab_idx,
            self.channels[0].power_levs,
            self.channels[0].wnd_shape,
            self.channels[0].gain_data,
            self.channels[0].num_gain_subbands,
            self.channels[1].num_coded_vals,
            self.channels[1].fill_mode,
            self.channels[1].split_point,
            self.channels[1].table_type,
            self.channels[1].qu_wordlen,
            self.channels[1].qu_sf_idx,
            self.channels[1].qu_tab_idx,
            self.channels[1].power_levs,
            self.channels[1].wnd_shape,
            self.channels[1].gain_data,
            self.channels[1].num_gain_subbands,
            self.waves_info.tones_present,
            self.waves_info.amplitude_mode,
            self.waves_info.num_tone_bands,
            self.waves_info.tone_sharing,
            self.waves_info.tone_master,
            self.waves_info.invert_phase,
        )?;

        for ch in 0..2 {
            writeln!(f, "\nCh {} spectrum", ch)?;
            for x in self.channels[ch].spectrum.iter() {
                write!(f, ",{}", x)?;
            }
        }

        Ok(())
    }
}

const POWER_COMP_OFF: u8 = 15;

#[derive(Clone, Copy)]
struct ChannelParams {
    ch_num: i32,
    num_coded_vals: i32,
    fill_mode: i32,
    split_point: i32,
    table_type: i32,
    qu_wordlen: [i32; 32],
    qu_sf_idx: [i32; 32],
    qu_tab_idx: [i32; 32],
    spectrum: [i16; 2048],
    power_levs: [u8; 5],
    wnd_shape: [u8; SUBBANDS],
    wnd_shape_prev: [u8; SUBBANDS],
    gain_data: [GainInfo; SUBBANDS],
    gain_data_prev: [GainInfo; SUBBANDS],
    num_gain_subbands: i32,
    tones_info: [WavesData; SUBBANDS],
    tones_info_prev: [WavesData; SUBBANDS],
}

impl Default for ChannelParams {
    fn default() -> ChannelParams {
        ChannelParams {
            ch_num: Default::default(),
            num_coded_vals: Default::default(),
            fill_mode: Default::default(),
            split_point: Default::default(),
            table_type: Default::default(),
            qu_wordlen: Default::default(),
            qu_sf_idx: Default::default(),
            qu_tab_idx: Default::default(),
            spectrum: [0; 2048],
            power_levs: [POWER_COMP_OFF; 5],
            wnd_shape: Default::default(),
            wnd_shape_prev: Default::default(),
            gain_data: Default::default(),
            gain_data_prev: Default::default(),
            num_gain_subbands: Default::default(),
            tones_info: Default::default(),
            tones_info_prev: Default::default(),
        }
    }
}

#[derive(Default, Clone, Copy)]
struct GainInfo {
    num_points: i32,
    lev_code: [i32; 7],
    loc_code: [i32; 7],
}

impl std::fmt::Debug for GainInfo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        Ok(write!(
            f,
            "({}, {:?}, {:?})",
            self.num_points, self.lev_code, self.loc_code
        )?)
    }
}

#[derive(Clone, Copy)]
struct WaveSynthParams {
    tones_present: i32,
    amplitude_mode: i32,
    num_tone_bands: i32,
    tone_sharing: [u8; SUBBANDS],
    tone_master: [u8; SUBBANDS],
    invert_phase: [u8; SUBBANDS],
    tones_index: i32,
    waves: [WaveParam; 48],
}

impl Default for WaveSynthParams {
    fn default() -> Self {
        WaveSynthParams {
            tones_present: Default::default(),
            amplitude_mode: Default::default(),
            num_tone_bands: Default::default(),
            tone_sharing: Default::default(),
            tone_master: Default::default(),
            invert_phase: Default::default(),
            tones_index: Default::default(),
            waves: [Default::default(); 48],
        }
    }
}

#[derive(Default, Clone, Copy)]
struct WaveParam {
    freq_index: i32,
    amp_sf: i32,
    amp_index: i32,
    phase_index: i32,
}

#[derive(Default, Clone, Copy)]
struct WavesData {
    pend_env: WavesEnvelope,
    curr_env: WavesEnvelope,
    num_wavs: i32,
    start_index: i32,
}

#[derive(Default, Clone, Copy)]
struct WavesEnvelope {
    has_start_point: i32,
    has_stop_point: i32,
    start_pos: i32,
    stop_pos: i32,
}

fn ff_array_elems<T>(a: &[T]) -> usize {
    std::mem::size_of_val(a) / std::mem::size_of_val(&a[0])
}

fn sign_extend(val: i32, bits: usize) -> i32 {
    let shift = 8 * std::mem::size_of_val(&val) - bits;
    let v = SignExtend {
        u: (val << shift) as u32,
    };

    let s = unsafe { v.s };
    s >> shift
}

union SignExtend {
    u: u32,
    s: i32,
}

fn log2(mut v: u32) -> i32 {
    let mut n = 0;

    if v & 0xffff0000 > 0 {
        v >>= 16;
        n += 16;
    }
    if v & 0xff00 > 0 {
        v >>= 8;
        n += 8;
    }
    n += LOG2_TAB[v as usize] as i32;

    n
}

const LOG2_TAB: [u8; 256] = [
    0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
];

fn align_to_block<R: Read + Seek>(
    mut bit_reader: BitReader<R, BigEndian>,
    block_align: f32,
    data_size: u32,
    file_size: u32,
) -> Result<BitReader<R, BigEndian>, Error> {
    bit_reader.byte_align();

    let mut reader = bit_reader.into_reader();
    let mut pos = reader.seek(SeekFrom::Current(0))? as u32;

    let offset = file_size - data_size;
    pos -= offset;

    let calc_blocks = pos as f32 / block_align;
    let next_block = calc_blocks.ceil();

    let block_delta = next_block - calc_blocks;
    let bytes_to_align = block_align * block_delta;

    reader.seek(SeekFrom::Current(bytes_to_align.round() as i64))?;

    Ok(BitReader::new(reader))
}

fn init_static() {
    lazy_static::initialize(&WL_VLC_TABS);
    lazy_static::initialize(&SF_VLC_TABS);
    lazy_static::initialize(&CT_VLC_TABS);
    lazy_static::initialize(&GAIN_VLC_TABS);
    lazy_static::initialize(&TONE_VLC_TABS);
    lazy_static::initialize(&SINE_64);
    lazy_static::initialize(&SINE_128);
    lazy_static::initialize(&SINE_TABLE);
    lazy_static::initialize(&HANN_WINDOW);
    lazy_static::initialize(&AMP_SF_TAB);
    lazy_static::initialize(&SPECTRA_TABS);
    lazy_static::initialize(&SPEC_VLC_TABS);
}
