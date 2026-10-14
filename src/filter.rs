//! The digital filters behind the ADC: which processing block runs, and the coefficients of its
//! first biquad (section 10.3.12, pp. 31 to 36).
//!
//! A processing block is a fixed signal chain: a decimation filter, a first-order IIR, and
//! depending on the block up to five biquads or an FIR filter (Table 6, p. 31). The coefficients
//! live in page 4 and all reset to a flat response (p. 35).
//!
//! Only biquad A is programmable through this driver so far. The first-order IIR, biquads B to E,
//! the FIR filters, the AGC's filter and the programmable miniDSP mode are left at their reset
//! values.

use embedded_hal::i2c::I2c;

use crate::register::{self, biquad_a, processing_block};
use crate::{Error, Register, Tlv320adc3101};

/// One of the 18 fixed processing blocks, PRB_R1 to PRB_R18: page 0, register 61 (Table 63,
/// p. 57; what each block contains is in Table 6, p. 31).
///
/// The block limits the clock tree: it names the AOSR values it works with, and `MADC * AOSR` has
/// to be larger than its instruction count (Table 6; Figure 28 note, p. 24).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProcessingBlock(u8);

impl ProcessingBlock {
    /// PRB_R1, the reset value: stereo, decimation filter A, first-order IIR, no biquads; AOSR 128
    /// or 64, 188 instructions (Table 6).
    pub const PRB_R1: Self = Self(processing_block::PRB_R1);
    /// PRB_R2: stereo, decimation filter A, first-order IIR and five biquads; AOSR 128 or 64, 240
    /// instructions (Table 6).
    pub const PRB_R2: Self = Self(processing_block::PRB_R2);

    /// PRB_R`number`. `None` unless `number` is 1 to 18: 0 selects the programmable miniDSP mode,
    /// which needs instructions this driver cannot load, and 19 to 31 are reserved (Table 63).
    pub const fn new(number: u8) -> Option<Self> {
        if number >= 1 && number <= 18 {
            Some(Self(number))
        } else {
            None
        }
    }

    /// The block's number, 1 to 18.
    pub const fn number(self) -> u8 {
        self.0
    }
}

/// One of the chip's two ADC channels.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel {
    /// The left channel.
    Left,
    /// The right channel.
    Right,
}

/// The five coefficients of one biquad section (Table 8, p. 35).
///
/// The transfer function is `(N0 + 2*N1*z^-1 + N2*z^-2) / (2^15 - 2*D1*z^-1 - D2*z^-2)`
/// (Equation 4, p. 35). Each coefficient is a 16-bit two's complement number in 1.15 format, -1.0
/// (`0x8000`) to just under 1.0 (`0x7FFF`), so every `i16` is a valid coefficient (p. 35,
/// Figure 40).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Biquad {
    /// N0.
    pub n0: i16,
    /// N1. The transfer function doubles it.
    pub n1: i16,
    /// N2.
    pub n2: i16,
    /// D1. The transfer function doubles it.
    pub d1: i16,
    /// D2.
    pub d2: i16,
}

impl Biquad {
    /// A section that passes the signal through unchanged: N0 just under 1.0 and the rest zero.
    ///
    /// These are the reset values of the left channel's biquad A (Table 107, p. 69, registers 14
    /// to 23), and the datasheet says every biquad is flat at 0 dB with its default coefficients
    /// (p. 35). The same table prints zeros for the right channel's N0 (p. 71, registers 78 and
    /// 79), which would be silence and contradicts that sentence; it has not been read from a
    /// chip. Writing this constant settles the question for the channel it is written to.
    pub const FLAT: Self = Self {
        n0: 0x7FFF,
        n1: 0,
        n2: 0,
        d1: 0,
        d2: 0,
    };
}

impl<I2C: I2c> Tlv320adc3101<I2C> {
    /// Selects the processing block: page 0, register 61 (Table 63, p. 57).
    ///
    /// Select it while the ADC channels are powered down, which is where the datasheet's design
    /// procedure puts it (section 11.2.2.2, step 2, p. 80).
    pub fn set_processing_block(
        &mut self,
        block: ProcessingBlock,
    ) -> Result<(), Error<I2C::Error>> {
        self.write(register::PROCESSING_BLOCK, block.number())
    }

    /// Writes the coefficients of biquad A for one channel: page 4, registers 14 to 23 (left) or
    /// 78 to 87 (right) (Table 8, p. 35).
    ///
    /// Biquad A is the first of the biquads in the processing blocks that have any, PRB_R2 among
    /// them (Table 6, p. 31; Figure 32, p. 32). In a block without biquads the coefficients have
    /// no effect, and in a block with an FIR filter the same registers hold FIR taps (Table 9,
    /// p. 36).
    ///
    /// Write them while the ADC channels are powered down, between the reset and the power-up of
    /// the channels. The datasheet only describes changing coefficients of a running filter for
    /// its adaptive mode (section
    /// 10.3.12.3, p. 34), which this driver does not set up. After a reset wait at least 100
    /// microseconds before writing: the coefficient memory is not valid earlier (section 10.6.4,
    /// p. 68); [`reset`](Self::reset) waits longer than that.
    pub fn set_biquad_a(
        &mut self,
        channel: Channel,
        coefficients: &Biquad,
    ) -> Result<(), Error<I2C::Error>> {
        let registers = match channel {
            Channel::Left => [
                biquad_a::LEFT_N0_MSB,
                biquad_a::LEFT_N1_MSB,
                biquad_a::LEFT_N2_MSB,
                biquad_a::LEFT_D1_MSB,
                biquad_a::LEFT_D2_MSB,
            ],
            Channel::Right => [
                biquad_a::RIGHT_N0_MSB,
                biquad_a::RIGHT_N1_MSB,
                biquad_a::RIGHT_N2_MSB,
                biquad_a::RIGHT_D1_MSB,
                biquad_a::RIGHT_D2_MSB,
            ],
        };
        let values = [
            coefficients.n0,
            coefficients.n1,
            coefficients.n2,
            coefficients.d1,
            coefficients.d2,
        ];
        for (high_register, value) in registers.into_iter().zip(values) {
            // The high byte goes first and the low byte immediately after it, into the next
            // address; the chip requires that order even when only one byte changes (p. 69).
            let [high, low] = value.to_be_bytes();
            let low_register = Register::new(high_register.page(), high_register.addr() + 1);
            self.write(high_register, high)?;
            self.write(low_register, low)?;
        }
        Ok(())
    }
}
