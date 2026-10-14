//! The digital side of the two ADC channels, page 0: channel power, soft-stepping, mute, and the
//! two digital gain controls (sections 10.3.8 and 10.3.9, pp. 25 to 26).
//!
//! Behind each ADC sit a volume control from -12 dB to +20 dB in 0.5 dB steps and a fine trim from
//! 0 dB to -0.4 dB in 0.1 dB steps, which is there to match two channels with a resolution of
//! 0.1 dB (p. 26). All gains here are integers in the chip's own step size, so nothing is rounded.

use embedded_hal::i2c::I2c;

use crate::filter::Channel;
use crate::register::{self, adc_digital, adc_fine_volume, adc_volume};
use crate::{Error, Tlv320adc3101};

/// How gain changes are applied, for the digital volume and the analog PGA alike: page 0,
/// register 81, bits D1 to D0 (Table 67, p. 58; pp. 25 to 26).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SoftStepping {
    /// One gain step per sample, the reset value.
    PerSample,
    /// One gain step per two samples.
    PerTwoSamples,
    /// Gain changes apply at once.
    Disabled,
}

impl SoftStepping {
    /// The field's bits, in place (Table 67).
    const fn bits(self) -> u8 {
        match self {
            SoftStepping::PerSample => adc_digital::SOFT_STEP_PER_SAMPLE,
            SoftStepping::PerTwoSamples => adc_digital::SOFT_STEP_PER_TWO_SAMPLES,
            SoftStepping::Disabled => adc_digital::SOFT_STEP_DISABLED,
        }
    }

    /// The mode a value of register 81 selects. `None` for the reserved code `11` (Table 67).
    pub(crate) const fn from_bits(adc_digital: u8) -> Option<Self> {
        match adc_digital & adc_digital::SOFT_STEP_MASK {
            adc_digital::SOFT_STEP_PER_SAMPLE => Some(SoftStepping::PerSample),
            adc_digital::SOFT_STEP_PER_TWO_SAMPLES => Some(SoftStepping::PerTwoSamples),
            adc_digital::SOFT_STEP_DISABLED => Some(SoftStepping::Disabled),
            _ => None,
        }
    }
}

/// The digital volume of a channel in steps of 0.5 dB: -24 to 40 steps, -12 dB to +20 dB
/// (Table 2, p. 25; Tables 69 and 70, pp. 58 to 59).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct DigitalVolume(i8);

impl DigitalVolume {
    /// -12 dB, the lowest setting (Table 2).
    pub const MIN: Self = Self(-24);
    /// 0 dB, the reset value (Table 2).
    pub const ZERO_DB: Self = Self(0);
    /// +20 dB, the highest setting (Table 2).
    pub const MAX: Self = Self(40);

    /// A volume of `half_db` times 0.5 dB. `None` below -24 (-12 dB) or above 40 (+20 dB); the
    /// codes beyond that are reserved (Tables 69 and 70).
    pub const fn from_half_db(half_db: i8) -> Option<Self> {
        if half_db >= Self::MIN.0 && half_db <= Self::MAX.0 {
            Some(Self(half_db))
        } else {
            None
        }
    }

    /// The volume in steps of 0.5 dB, -24 to 40.
    pub const fn half_db(self) -> i8 {
        self.0
    }

    /// The register value: the step count as a seven-bit two's complement number, with the
    /// reserved bit D7 zero (Tables 69 and 70, note 1). -24 becomes `0x68` and 40 becomes `0x28`.
    const fn bits(self) -> u8 {
        self.0.to_le_bytes()[0] & adc_volume::VALUE_MASK
    }

    /// The volume register 83 or 84 holds: a seven-bit two's complement step count, with bit D7
    /// reserved and ignored here (Tables 69 and 70, note 1). `None` for the codes outside -12 dB
    /// to +20 dB, which the tables do not define as a volume.
    pub(crate) const fn from_bits(register: u8) -> Option<Self> {
        /// Bit D6, the sign of the seven-bit number (Tables 69 and 70).
        const SIGN: u8 = 0b0100_0000;
        let value = register & adc_volume::VALUE_MASK;
        // Copying the sign into bit D7 turns the seven-bit number into the same number in eight
        // bits.
        let extended = if value & SIGN != 0 {
            value | !adc_volume::VALUE_MASK
        } else {
            value
        };
        Self::from_half_db(i8::from_le_bytes([extended]))
    }
}

/// The fine trim of a channel: 0 to 4 steps of 0.1 dB *downwards*, 0 dB to -0.4 dB (Table 68,
/// p. 58; section 10.3.9.2, p. 26).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct FineGain(u8);

impl FineGain {
    /// No trim, 0 dB: the reset value (Table 68).
    pub const NONE: Self = Self(0);
    /// The largest trim, -0.4 dB (Table 68).
    pub const MAX: Self = Self(4);

    /// A trim of `tenths_db` times -0.1 dB. `None` above 4; the codes 5 to 7 are reserved
    /// (Table 68).
    pub const fn from_tenths_db_down(tenths_db: u8) -> Option<Self> {
        if tenths_db <= Self::MAX.0 {
            Some(Self(tenths_db))
        } else {
            None
        }
    }

    /// The attenuation in steps of 0.1 dB, 0 to 4.
    pub const fn tenths_db_down(self) -> u8 {
        self.0
    }
}

impl<I2C: I2c> Tlv320adc3101<I2C> {
    /// Selects how gain changes are applied and leaves the other bits of the register as they
    /// are: page 0, register 81, bits D1 to D0 (Table 67, p. 58).
    pub fn set_soft_stepping(&mut self, stepping: SoftStepping) -> Result<(), Error<I2C::Error>> {
        self.modify(
            register::ADC_DIGITAL,
            adc_digital::SOFT_STEP_MASK,
            stepping.bits(),
        )
    }

    /// Powers the two ADC channels up or down and leaves the other bits of the register as they
    /// are: page 0, register 81, bits D7 and D6 (Table 67, p. 58).
    ///
    /// A channel needs its clocks: the datasheet powers the channels after the clock tree (section
    /// 11.2.2.2, step 4, p. 80). When a channel is powered down the chip first ramps its PGA to
    /// mute and its digital volume to -12 dB by itself (pp. 25 to 26).
    pub fn set_channel_power(&mut self, left: bool, right: bool) -> Result<(), Error<I2C::Error>> {
        let mut power = 0;
        if left {
            power |= adc_digital::LEFT_POWER_UP;
        }
        if right {
            power |= adc_digital::RIGHT_POWER_UP;
        }
        self.modify(
            register::ADC_DIGITAL,
            adc_digital::LEFT_POWER_UP | adc_digital::RIGHT_POWER_UP,
            power,
        )
    }

    /// Mutes or unmutes the digital output of one channel and leaves the rest of the register as
    /// it is: page 0, register 82, bit D7 (left) or D3 (right) (Table 68, p. 58). Both channels
    /// are muted after a reset.
    pub fn set_digital_mute(
        &mut self,
        channel: Channel,
        muted: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let mask = match channel {
            Channel::Left => adc_fine_volume::LEFT_MUTE,
            Channel::Right => adc_fine_volume::RIGHT_MUTE,
        };
        self.modify(
            register::ADC_FINE_VOLUME,
            mask,
            if muted { mask } else { 0 },
        )
    }

    /// Sets the fine trim of one channel and leaves the rest of the register as it is: page 0,
    /// register 82, bits D6 to D4 (left) or D2 to D0 (right) (Table 68, p. 58).
    pub fn set_fine_gain(
        &mut self,
        channel: Channel,
        gain: FineGain,
    ) -> Result<(), Error<I2C::Error>> {
        let (mask, bits) = match channel {
            Channel::Left => (
                adc_fine_volume::LEFT_FINE_GAIN_MASK,
                gain.tenths_db_down() << adc_fine_volume::LEFT_FINE_GAIN_SHIFT,
            ),
            Channel::Right => (adc_fine_volume::RIGHT_FINE_GAIN_MASK, gain.tenths_db_down()),
        };
        self.modify(register::ADC_FINE_VOLUME, mask, bits)
    }

    /// Sets the digital volume of one channel: page 0, register 83 (left) or 84 (right), written
    /// whole (Tables 69 and 70, pp. 58 to 59). The reset value is 0 dB.
    pub fn set_digital_volume(
        &mut self,
        channel: Channel,
        volume: DigitalVolume,
    ) -> Result<(), Error<I2C::Error>> {
        let reg = match channel {
            Channel::Left => register::LEFT_ADC_VOLUME,
            Channel::Right => register::RIGHT_ADC_VOLUME,
        };
        self.write(reg, volume.bits())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    /// Pins the digital gain writes: the volume as a seven-bit two's complement step count in its
    /// own register, and trim and mute as read-modify-writes that keep each other's bits.
    #[test]
    fn volume_trim_and_mute_write_the_chip_encodings() {
        const ADDRESS: u8 = 0x18;
        let expectations = [
            Transaction::write(ADDRESS, vec![0, 0]),
            // -12 dB is -24 steps, 0x68 in seven bits; +20 dB is 40 steps, 0x28.
            Transaction::write(ADDRESS, vec![83, 0x68]),
            Transaction::write(ADDRESS, vec![84, 0x28]),
            // Left trim -0.4 dB into bits D6 to D4 of an unmuted register.
            Transaction::write_read(ADDRESS, vec![82], vec![0x00]),
            Transaction::write(ADDRESS, vec![82, 0x40]),
            // Right trim -0.1 dB into bits D2 to D0; the left trim stays.
            Transaction::write_read(ADDRESS, vec![82], vec![0x40]),
            Transaction::write(ADDRESS, vec![82, 0x41]),
            // Right mute is bit D3; both trims stay.
            Transaction::write_read(ADDRESS, vec![82], vec![0x41]),
            Transaction::write(ADDRESS, vec![82, 0x49]),
        ];
        let mut adc = Tlv320adc3101::new(Mock::new(&expectations), ADDRESS);
        adc.set_digital_volume(Channel::Left, DigitalVolume::MIN)
            .unwrap();
        adc.set_digital_volume(Channel::Right, DigitalVolume::MAX)
            .unwrap();
        adc.set_fine_gain(Channel::Left, FineGain::MAX).unwrap();
        adc.set_fine_gain(Channel::Right, FineGain::from_tenths_db_down(1).unwrap())
            .unwrap();
        adc.set_digital_mute(Channel::Right, true).unwrap();
        adc.release().done();
    }

    /// Pure decoding, no bus: the volume's seven-bit two's complement and its undefined codes
    /// (Tables 69 and 70), and the soft-stepping field with its reserved code (Table 67).
    #[test]
    fn volume_and_soft_stepping_decode_from_their_register_values() {
        for volume in [
            DigitalVolume::MIN,
            DigitalVolume::ZERO_DB,
            DigitalVolume::MAX,
        ] {
            assert_eq!(DigitalVolume::from_bits(volume.bits()), Some(volume));
        }
        // 0x7F is -1 step, -0.5 dB. The reserved bit D7 does not change the value.
        assert_eq!(
            DigitalVolume::from_bits(0x7F).map(DigitalVolume::half_db),
            Some(-1)
        );
        assert_eq!(
            DigitalVolume::from_bits(0xFF),
            DigitalVolume::from_bits(0x7F)
        );
        // One step above +20 dB (0x29) and one below -12 dB (0x67) are not volumes.
        assert_eq!(DigitalVolume::from_bits(0x29), None);
        assert_eq!(DigitalVolume::from_bits(0x67), None);

        assert_eq!(SoftStepping::from_bits(0xC2), Some(SoftStepping::Disabled));
        assert_eq!(SoftStepping::from_bits(0xC0), Some(SoftStepping::PerSample));
        assert_eq!(SoftStepping::from_bits(0x03), None);
    }
}
