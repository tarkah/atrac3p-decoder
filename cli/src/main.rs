use anyhow::Error;
use structopt::StructOpt;

use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;

use atrac3p::Atrac3Plus;

fn main() -> Result<(), Error> {
    let opts = Opts::from_args();

    match opts.command {
        Command::Play { input } => {
            let file = File::open(input)?;
            let reader = BufReader::new(file);

            let atrac = Atrac3Plus::new(reader)?;

            let device = rodio::default_output_device().unwrap();
            let sink = rodio::Sink::new(&device);

            sink.append(atrac);
            sink.play();
            sink.sleep_until_end();
        }
    }

    Ok(())
}

#[derive(StructOpt)]
#[structopt(name = "atrac3p-decoder")]
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
