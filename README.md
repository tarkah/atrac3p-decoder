# Atrac3+ Decoder
[![Crates.io](https://img.shields.io/crates/v/atrac3p-decoder)](https://crates.io/crates/atrac3p-decoder)
[![](https://docs.rs/atrac3p-decoder/badge.svg)](https://docs.rs/atrac3p-decoder)

Atrac3+ Decoder in Rust. Can be used as a source for Rodio. Currently only supports
the `.at3` / `.wav` RIFF WAV container format.


## Rodio Source

`Decoder` implies rodio's `Source` trait, as well as `Iterator`. 

## Example with Rodio

```rust
use anyhow::Error;

use std::fs::File;
use std::io::BufReader;

fn main() -> Result<(), Error> {
    let file = File::open("path/to/song.at3")?;
    let reader = BufReader::new(file);

    let decoder = atrac3p_decoder::Decoder::new(reader)?;

    let device = rodio::default_output_device().unwrap();
    let sink = rodio::Sink::new(&device);

    sink.append(decoder);
    sink.play();
    sink.sleep_until_end();

    Ok(())
}
```