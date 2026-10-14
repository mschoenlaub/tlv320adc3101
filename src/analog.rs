//! The analog side, page 1: microphone bias, which input pins reach the two PGAs, and the PGA
//! gain (sections 10.3.9 to 10.3.11, pp. 25 to 30; Figure 30, p. 29).
//!
//! Each PGA sits behind a small mixer. Seven sources can be switched onto it, each at 0 dB or
//! -6 dB: four single-ended pins and three differential pairs (Table 4, p. 29). Switching on one
//! source selects it; switching on several mixes them, and the sum must stay within 2 Vpp
//! single-ended or 4 Vpp differential (p. 28). A pair cannot be differential for one PGA and
//! single-ended for the other at the same time (p. 29).

use embedded_hal::i2c::I2c;

use crate::filter::Channel;
use crate::register::{
    self, input_level, left_pga_input_1, left_pga_input_2, micbias_control, pga, right_pga_input_1,
    right_pga_input_2,
};
use crate::{Error, Register, Tlv320adc3101};

/// What one of the two microphone bias outputs delivers (Table 93, p. 65; section 10.3.11,
/// p. 30). Each output can supply 4 mA.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MicBias {
    /// Powered down, the reset value.
    Off,
    /// 2 V, derived from the chip's band-gap reference.
    V2,
    /// 2.5 V, derived from the chip's band-gap reference.
    V2_5,
    /// Connected to AVDD through a switch in the chip.
    Avdd,
}

impl MicBias {
    /// The two-bit code both fields of register 51 use (Table 93).
    const fn code(self) -> u8 {
        match self {
            MicBias::Off => 0b00,
            MicBias::V2 => 0b01,
            MicBias::V2_5 => 0b10,
            MicBias::Avdd => 0b11,
        }
    }

    /// The setting a two-bit code of register 51 stands for (Table 93). All four codes are
    /// defined, so every register value decodes; bits above the two lowest are ignored.
    pub(crate) const fn from_code(code: u8) -> Self {
        match code & 0b11 {
            0b00 => MicBias::Off,
            0b01 => MicBias::V2,
            0b10 => MicBias::V2_5,
            _ => MicBias::Avdd,
        }
    }
}

/// How one source is switched onto a PGA: the "input level control" in front of the PGA's mixer
/// (p. 28; Tables 94, 96, 97 and 99, pp. 66 to 67).
///
/// It is a coarse level setting and not a volume control (p. 29, note). A single-ended source is
/// amplified by two so that it arrives at the level of a differential one (Table 94, note 1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InputLevel {
    /// Not connected, the reset value.
    Off,
    /// Connected at 0 dB.
    ZeroDb,
    /// Connected at -6 dB.
    Minus6Db,
}

impl InputLevel {
    /// The two-bit code, to be shifted to its field (Tables 94, 96, 97 and 99).
    const fn code(self) -> u8 {
        match self {
            InputLevel::Off => input_level::NOT_CONNECTED,
            InputLevel::ZeroDb => input_level::ZERO_DB,
            InputLevel::Minus6Db => input_level::MINUS_6_DB,
        }
    }

    /// The level a two-bit field stands for; bits above the two lowest are ignored. The chip
    /// has two codes for "not connected", `10` and `11` (Tables 94, 96, 97 and 99), and both
    /// decode to [`InputLevel::Off`], so a register holding `10` does not encode back to the
    /// same byte.
    pub(crate) const fn from_code(code: u8) -> Self {
        match code & input_level::MASK {
            input_level::ZERO_DB => InputLevel::ZeroDb,
            input_level::MINUS_6_DB => InputLevel::Minus6Db,
            _ => InputLevel::Off,
        }
    }
}

/// What is switched onto the left PGA: page 1, registers 52 and 54 (Tables 94 and 96, p. 66).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LeftInputs {
    /// The IN1L(P) pin, single-ended.
    pub in1l: InputLevel,
    /// The IN2L(P) pin, single-ended.
    pub in2l: InputLevel,
    /// The IN3L(M) pin, single-ended.
    pub in3l: InputLevel,
    /// The IN1R(M) pin, single-ended.
    pub in1r: InputLevel,
    /// The differential pair IN2L(P) plus, IN3L(M) minus.
    pub diff_in2l_in3l: InputLevel,
    /// The differential pair IN1L(P) plus, IN1R(M) minus.
    pub diff_in1l_in1r: InputLevel,
    /// The differential pair IN2R(P) plus, IN3R(M) minus.
    pub diff_in2r_in3r: InputLevel,
    /// Whether the unselected left inputs are weakly held at the ADC's common-mode voltage, which
    /// keeps their coupling capacitors charged: register 54, bit D6 (Table 96; section 10.3.10,
    /// p. 30).
    pub bias_unselected: bool,
    /// Whether the PGA is bypassed and the IN2L(P)/IN3L(M) pair feeds the ADC unbuffered: register
    /// 54, bit D7 (Table 96).
    pub pga_bypass: bool,
}

impl LeftInputs {
    /// Nothing connected, the reset value (Tables 94 and 96).
    pub const NONE: Self = Self {
        in1l: InputLevel::Off,
        in2l: InputLevel::Off,
        in3l: InputLevel::Off,
        in1r: InputLevel::Off,
        diff_in2l_in3l: InputLevel::Off,
        diff_in1l_in1r: InputLevel::Off,
        diff_in2r_in3r: InputLevel::Off,
        bias_unselected: false,
        pga_bypass: false,
    };

    /// The value of register 52 (Table 94).
    const fn select_1_bits(&self) -> u8 {
        (self.diff_in2l_in3l.code() << left_pga_input_1::DIFF_IN2L_IN3L_SHIFT)
            | (self.in3l.code() << left_pga_input_1::IN3L_SHIFT)
            | (self.in2l.code() << left_pga_input_1::IN2L_SHIFT)
            | (self.in1l.code() << left_pga_input_1::IN1L_SHIFT)
    }

    /// The value of register 54 (Table 96).
    const fn select_2_bits(&self) -> u8 {
        let mut bits = (self.diff_in1l_in1r.code() << left_pga_input_2::DIFF_IN1L_IN1R_SHIFT)
            | (self.diff_in2r_in3r.code() << left_pga_input_2::DIFF_IN2R_IN3R_SHIFT)
            | (self.in1r.code() << left_pga_input_2::IN1R_SHIFT);
        if self.bias_unselected {
            bits |= left_pga_input_2::UNSELECTED_INPUTS_BIASED;
        }
        if self.pga_bypass {
            bits |= left_pga_input_2::PGA_BYPASS;
        }
        bits
    }

    /// What the values of registers 52 and 54 connect to the left PGA (Tables 94 and 96): the
    /// inverse of the two encoders above, for a read-back.
    pub(crate) const fn from_bits(select_1: u8, select_2: u8) -> Self {
        Self {
            in1l: InputLevel::from_code(select_1 >> left_pga_input_1::IN1L_SHIFT),
            in2l: InputLevel::from_code(select_1 >> left_pga_input_1::IN2L_SHIFT),
            in3l: InputLevel::from_code(select_1 >> left_pga_input_1::IN3L_SHIFT),
            in1r: InputLevel::from_code(select_2 >> left_pga_input_2::IN1R_SHIFT),
            diff_in2l_in3l: InputLevel::from_code(
                select_1 >> left_pga_input_1::DIFF_IN2L_IN3L_SHIFT,
            ),
            diff_in1l_in1r: InputLevel::from_code(
                select_2 >> left_pga_input_2::DIFF_IN1L_IN1R_SHIFT,
            ),
            diff_in2r_in3r: InputLevel::from_code(
                select_2 >> left_pga_input_2::DIFF_IN2R_IN3R_SHIFT,
            ),
            bias_unselected: select_2 & left_pga_input_2::UNSELECTED_INPUTS_BIASED != 0,
            pga_bypass: select_2 & left_pga_input_2::PGA_BYPASS != 0,
        }
    }
}

/// What is switched onto the right PGA: page 1, registers 55 and 57 (Tables 97 and 99, p. 67).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RightInputs {
    /// The IN1R(M) pin, single-ended.
    pub in1r: InputLevel,
    /// The IN2R(P) pin, single-ended.
    pub in2r: InputLevel,
    /// The IN3R(M) pin, single-ended.
    pub in3r: InputLevel,
    /// The IN1L(P) pin, single-ended.
    pub in1l: InputLevel,
    /// The differential pair IN2R(P) plus, IN3R(M) minus.
    pub diff_in2r_in3r: InputLevel,
    /// The differential pair IN1L(P) plus, IN1R(M) minus.
    pub diff_in1l_in1r: InputLevel,
    /// The differential pair IN2L(P) plus, IN3L(M) minus.
    pub diff_in2l_in3l: InputLevel,
    /// Whether the unselected right inputs are weakly held at the ADC's common-mode voltage:
    /// register 57, bit D6 (Table 99; section 10.3.10, p. 30).
    pub bias_unselected: bool,
    /// Whether the PGA is bypassed and the IN2R(P)/IN3R(M) pair feeds the ADC unbuffered: register
    /// 57, bit D7 (Table 99).
    pub pga_bypass: bool,
}

impl RightInputs {
    /// Nothing connected, the reset value (Tables 97 and 99).
    pub const NONE: Self = Self {
        in1r: InputLevel::Off,
        in2r: InputLevel::Off,
        in3r: InputLevel::Off,
        in1l: InputLevel::Off,
        diff_in2r_in3r: InputLevel::Off,
        diff_in1l_in1r: InputLevel::Off,
        diff_in2l_in3l: InputLevel::Off,
        bias_unselected: false,
        pga_bypass: false,
    };

    /// The value of register 55 (Table 97).
    const fn select_1_bits(&self) -> u8 {
        (self.diff_in2r_in3r.code() << right_pga_input_1::DIFF_IN2R_IN3R_SHIFT)
            | (self.in3r.code() << right_pga_input_1::IN3R_SHIFT)
            | (self.in2r.code() << right_pga_input_1::IN2R_SHIFT)
            | (self.in1r.code() << right_pga_input_1::IN1R_SHIFT)
    }

    /// The value of register 57 (Table 99).
    const fn select_2_bits(&self) -> u8 {
        let mut bits = (self.diff_in1l_in1r.code() << right_pga_input_2::DIFF_IN1L_IN1R_SHIFT)
            | (self.diff_in2l_in3l.code() << right_pga_input_2::DIFF_IN2L_IN3L_SHIFT)
            | (self.in1l.code() << right_pga_input_2::IN1L_SHIFT);
        if self.bias_unselected {
            bits |= right_pga_input_2::UNSELECTED_INPUTS_BIASED;
        }
        if self.pga_bypass {
            bits |= right_pga_input_2::PGA_BYPASS;
        }
        bits
    }

    /// What the values of registers 55 and 57 connect to the right PGA (Tables 97 and 99): the
    /// inverse of the two encoders above, for a read-back.
    pub(crate) const fn from_bits(select_1: u8, select_2: u8) -> Self {
        Self {
            in1r: InputLevel::from_code(select_1 >> right_pga_input_1::IN1R_SHIFT),
            in2r: InputLevel::from_code(select_1 >> right_pga_input_1::IN2R_SHIFT),
            in3r: InputLevel::from_code(select_1 >> right_pga_input_1::IN3R_SHIFT),
            in1l: InputLevel::from_code(select_2 >> right_pga_input_2::IN1L_SHIFT),
            diff_in2r_in3r: InputLevel::from_code(
                select_1 >> right_pga_input_1::DIFF_IN2R_IN3R_SHIFT,
            ),
            diff_in1l_in1r: InputLevel::from_code(
                select_2 >> right_pga_input_2::DIFF_IN1L_IN1R_SHIFT,
            ),
            diff_in2l_in3l: InputLevel::from_code(
                select_2 >> right_pga_input_2::DIFF_IN2L_IN3L_SHIFT,
            ),
            bias_unselected: select_2 & right_pga_input_2::UNSELECTED_INPUTS_BIASED != 0,
            pga_bypass: select_2 & right_pga_input_2::PGA_BYPASS != 0,
        }
    }
}

/// The gain of an analog PGA in steps of 0.5 dB: 0 to 80 steps, 0 dB to 40 dB (Tables 101 and
/// 102, p. 68; p. 25).
///
/// The register field is seven bits wide and the datasheet calls 81 to 127 reserved. Gain that
/// keeps rising up to 119 has been observed on a production board, but this type stays with
/// what the datasheet defines.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct PgaGain(u8);

impl PgaGain {
    /// 0 dB, the lowest gain and the reset value (Tables 101 and 102).
    pub const MIN: Self = Self(0);
    /// 40 dB, the highest gain the datasheet defines (Tables 101 and 102).
    pub const MAX: Self = Self(pga::GAIN_MAX_40_DB);

    /// A gain of `half_db` times 0.5 dB. `None` above 80 (40 dB).
    pub const fn from_half_db(half_db: u8) -> Option<Self> {
        if half_db <= pga::GAIN_MAX_40_DB {
            Some(Self(half_db))
        } else {
            None
        }
    }

    /// The gain in steps of 0.5 dB, 0 to 80.
    pub const fn half_db(self) -> u8 {
        self.0
    }
}

/// The PGA register of a channel (Tables 101 and 102).
const fn pga_register(channel: Channel) -> Register {
    match channel {
        Channel::Left => register::LEFT_PGA,
        Channel::Right => register::RIGHT_PGA,
    }
}

impl<I2C: I2c> Tlv320adc3101<I2C> {
    /// Sets the two microphone bias outputs: page 1, register 51, written whole (Table 93,
    /// p. 65).
    ///
    /// The datasheet's design procedure programs the bias before the PGAs (section 11.2.2.2,
    /// step 3, p. 80); it is independent of the channel power, so switching it on last and off
    /// first works as well.
    pub fn set_mic_bias(
        &mut self,
        micbias1: MicBias,
        micbias2: MicBias,
    ) -> Result<(), Error<I2C::Error>> {
        let bits = (micbias1.code() << micbias_control::MICBIAS1_SHIFT)
            | (micbias2.code() << micbias_control::MICBIAS2_SHIFT);
        self.write(register::MICBIAS_CONTROL, bits)
    }

    /// Connects inputs to the two PGAs: page 1, registers 52, 54, 55 and 57, each written whole
    /// (Tables 94, 96, 97 and 99, pp. 66 to 67).
    pub fn set_input_routing(
        &mut self,
        left: &LeftInputs,
        right: &RightInputs,
    ) -> Result<(), Error<I2C::Error>> {
        self.write(register::LEFT_PGA_INPUT_1, left.select_1_bits())?;
        self.write(register::LEFT_PGA_INPUT_2, left.select_2_bits())?;
        self.write(register::RIGHT_PGA_INPUT_1, right.select_1_bits())?;
        self.write(register::RIGHT_PGA_INPUT_2, right.select_2_bits())
    }

    /// Sets the gain of one PGA and leaves its mute bit as it is: page 1, register 59 or 60
    /// (Tables 101 and 102, p. 68).
    ///
    /// Unless soft-stepping is disabled, the PGA walks to the new gain one 0.5 dB step at a time
    /// (p. 25; [`Tlv320adc3101::set_soft_stepping`]).
    pub fn set_pga_gain(
        &mut self,
        channel: Channel,
        gain: PgaGain,
    ) -> Result<(), Error<I2C::Error>> {
        self.modify(pga_register(channel), pga::GAIN_MASK, gain.half_db())
    }

    /// Mutes or unmutes one PGA and leaves its gain as it is: page 1, register 59 or 60, bit D7
    /// (Tables 101 and 102, p. 68). Both PGAs are muted after a reset.
    pub fn set_pga_mute(&mut self, channel: Channel, muted: bool) -> Result<(), Error<I2C::Error>> {
        let mute = if muted { pga::MUTE } else { 0 };
        self.modify(pga_register(channel), pga::MUTE, mute)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pure decoding, no bus: a routing survives the trip through its two register values, and
    /// the chip's second code for "not connected" decodes like the first.
    #[test]
    fn routing_decodes_back_from_the_bits_it_encodes_to() {
        let left = LeftInputs {
            in1l: InputLevel::Minus6Db,
            in3l: InputLevel::ZeroDb,
            diff_in1l_in1r: InputLevel::Minus6Db,
            bias_unselected: true,
            ..LeftInputs::NONE
        };
        assert_eq!(
            LeftInputs::from_bits(left.select_1_bits(), left.select_2_bits()),
            left
        );
        let right = RightInputs {
            in2r: InputLevel::ZeroDb,
            diff_in2r_in3r: InputLevel::Minus6Db,
            diff_in2l_in3l: InputLevel::ZeroDb,
            pga_bypass: true,
            ..RightInputs::NONE
        };
        assert_eq!(
            RightInputs::from_bits(right.select_1_bits(), right.select_2_bits()),
            right
        );
        // 0xAA is `10` in all four fields of register 52 and 0x2A in the three of register 54.
        assert_eq!(LeftInputs::from_bits(0xAA, 0x2A), LeftInputs::NONE);
        assert_eq!(RightInputs::from_bits(0xAA, 0x2A), RightInputs::NONE);
    }

    /// Pure decoding, no bus: the four codes of a microphone bias field (Table 93).
    #[test]
    fn mic_bias_codes_decode_to_the_setting_that_encodes_them() {
        for bias in [MicBias::Off, MicBias::V2, MicBias::V2_5, MicBias::Avdd] {
            assert_eq!(MicBias::from_code(bias.code()), bias);
        }
    }
}
