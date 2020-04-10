use anyhow::Error;
use env_logger::Env;
use structopt::StructOpt;

use std::fs::File;
use std::io::{BufReader, Write};
use std::path::PathBuf;

fn main() -> Result<(), Error> {
    let opts = Opts::from_args();

    env_logger::Builder::from_env(Env::default().default_filter_or("info"))
        .format(|buf, record| writeln!(buf, "{}", record.args()))
        .init();

    match opts.command {
        Command::Play { input } => {
            let file = File::open(input)?;
            let reader = BufReader::new(file);

            let decoder = atrac3p_decoder::Decoder::new(reader)?;

            let device = rodio::default_output_device().unwrap();
            let sink = rodio::Sink::new(&device);

            sink.append(decoder);
            sink.play();
            sink.sleep_until_end();
        }
    }

    Ok(())
}

#[derive(StructOpt)]
#[structopt(name = "atrac3p-decoder-cli")]
struct Opts {
    #[structopt(subcommand)]
    command: Command,
}

#[derive(StructOpt)]
enum Command {
    Play {
        #[structopt(parse(from_os_str))]
        input: PathBuf,
    },
}
