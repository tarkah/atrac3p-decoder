// Atrac3+ Decoder
//
// Copyright (c) 2020 Cory Forsstrom <cforsstrom18@gmail.com>
// Copyright (c) 2010-2013 Maxim Poliakovski
//
// The following code is a derivative work of the code from the FFmpeg project,
// which is licensed LGPL v2.1. This code therefore is also licensed under the terms
// of the GNU Lesser General Public License, verison 2.1.

use std::io;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum Error {
    #[error("IO error reading file: {0}")]
    IOError(io::Error),
    #[error("Error reading riff file: {0}")]
    RiffError(riff_wave_reader::Error),
    #[error("No extended info in riff file, can't determine spec")]
    RiffErrorNoExtendedInfo,
    #[error("File supplied is not Atrac3Plus")]
    NotAtrac3PlusFile,
    #[error("Block align is not set")]
    BlockAlignNotSet,
    #[error("Invalid Start Bit")]
    InvalidStartBit,
    #[error("Unsupported channel unit extension")]
    UnsupportedChannelUnitExtension,
    #[error("Channel Unit Type can only be 2 bits")]
    InvalidChannelUnitTypeBits,
    #[error("Invalid Data")]
    InvalidData,
    #[error("Unsupported channel count: {0}")]
    UnsupportedChannelCount(u16),
    #[error("{0}")]
    Other(&'static str),
    #[error("{0}")]
    OtherFormat(String),
}

impl From<riff_wave_reader::Error> for Error {
    fn from(error: riff_wave_reader::Error) -> Self {
        Error::RiffError(error)
    }
}

impl From<std::io::Error> for Error {
    fn from(error: std::io::Error) -> Self {
        Error::IOError(error)
    }
}
