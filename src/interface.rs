//! The serial audio interface: format, word length, who drives the clocks, where in the frame
//! this chip's two channels sit, and the DOUT pin (section 10.3.6, pp. 14 to 22).
//!
//! # Several chips on one data line
//!
//! The chip can release DOUT outside its own words and start its words a programmable number of
//! bit clocks after the word clock, which lets several chips take turns on one data line
//! (p. 15). Each chip then gets [`InterfaceConfig::tristate`] and its own
//! [`InterfaceConfig::slot_offset`], and one of them drives BCLK and WCLK for all.
//!
//! # What the driver does not cover
//!
//! Time-slot mode (page 0, register 38, bit D0, with the second offset in register 37) is always
//! written as off, so the second channel follows the first directly. The secondary audio interface
//! (registers 31 to 33) is not touched.

use embedded_hal::i2c::I2c;

use crate::register::{
    self, dout_control, i2s_tdm_control, interface_control_1, interface_control_2,
};
use crate::{Error, Tlv320adc3101};

/// The serial format: page 0, register 27, bits D7 to D6 (Table 33, p. 49).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    /// I2S, the reset value: each channel starts one bit clock after its edge of the word clock
    /// (section 10.3.6.3, p. 19).
    I2s,
    /// DSP: the rising edge of the word clock starts the left channel and the right channel
    /// follows it immediately (section 10.3.6.4, p. 20).
    ///
    /// With [`InterfaceConfig::slot_offset`] 0 this is what ALSA calls DSP mode B; an offset of 1
    /// gives DSP mode A.
    Dsp,
    /// Right-justified (section 10.3.6.1, p. 16).
    RightJustified,
    /// Left-justified (section 10.3.6.2, p. 16).
    LeftJustified,
}

impl Format {
    /// The field's bits, in place (Table 33).
    const fn bits(self) -> u8 {
        match self {
            Format::I2s => interface_control_1::FORMAT_I2S,
            Format::Dsp => interface_control_1::FORMAT_DSP,
            Format::RightJustified => interface_control_1::FORMAT_RIGHT_JUSTIFIED,
            Format::LeftJustified => interface_control_1::FORMAT_LEFT_JUSTIFIED,
        }
    }
}

/// The word length: page 0, register 27, bits D5 to D4 (Table 33, p. 49).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WordLength {
    /// 16 bits, the reset value.
    Bits16,
    /// 20 bits.
    Bits20,
    /// 24 bits.
    Bits24,
    /// 32 bits.
    Bits32,
}

impl WordLength {
    /// The field's bits, in place (Table 33).
    const fn bits(self) -> u8 {
        match self {
            WordLength::Bits16 => interface_control_1::WORD_LENGTH_16,
            WordLength::Bits20 => interface_control_1::WORD_LENGTH_20,
            WordLength::Bits24 => interface_control_1::WORD_LENGTH_24,
            WordLength::Bits32 => interface_control_1::WORD_LENGTH_32,
        }
    }
}

/// Whether the chip listens to a clock pin or drives it: page 0, register 27, bit D3 for BCLK and
/// bit D2 for WCLK (Table 33, p. 49).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockDirection {
    /// The pin is an input and another device provides the clock. The reset value.
    Input,
    /// The pin is an output and this chip provides the clock.
    Output,
}

/// What feeds the BCLK N divider, BDIV_CLKIN: page 0, register 29, bits D1 to D0 (Table 35,
/// p. 50).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BclkDividerInput {
    /// ADC_CLK, the output of NADC. The reset value.
    AdcClk,
    /// ADC_MOD_CLK, the output of MADC.
    AdcModClk,
}

impl BclkDividerInput {
    /// The field's bits, in place (Table 35).
    const fn bits(self) -> u8 {
        match self {
            BclkDividerInput::AdcClk => interface_control_2::BDIV_CLKIN_ADC_CLK,
            BclkDividerInput::AdcModClk => interface_control_2::BDIV_CLKIN_ADC_MOD_CLK,
        }
    }
}

/// Which of the two channels the chip sends: page 0, register 38, bits D3 to D2 (Table 44,
/// p. 52).
///
/// The two middle variants are named after their bits and not after a channel, because the
/// datasheet disagrees with itself about which channel each one leaves on the bus:
///
/// - the register table reads `01` as "left channel enabled" and `10` as "right channel enabled"
///   (Table 44);
/// - the description of the same register says that setting bit D2 disables the *first* channel,
///   and draws the second one still being sent (p. 15 and Figure 12), which makes `01` the right
///   channel and `10` the left one as long as the channels are not swapped.
///
/// Nobody has checked it on a chip yet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChannelEnable {
    /// `00`: both channels are sent. The reset value.
    Both,
    /// `01`, bit D2 set: one channel is sent. See the type's description for which.
    Bits01,
    /// `10`, bit D3 set: the other channel is sent. See the type's description for which.
    Bits10,
    /// `11`: nothing is sent.
    Neither,
}

impl ChannelEnable {
    /// The field's bits, in place (Table 44).
    const fn bits(self) -> u8 {
        match self {
            ChannelEnable::Both => i2s_tdm_control::CHANNELS_BOTH,
            ChannelEnable::Bits01 => i2s_tdm_control::CHANNELS_LEFT,
            ChannelEnable::Bits10 => i2s_tdm_control::CHANNELS_RIGHT,
            ChannelEnable::Neither => i2s_tdm_control::CHANNELS_NONE,
        }
    }
}

/// The serial interface of one chip: registers 27, 28, 29 and 38 of page 0.
///
/// The datasheet leaves two conditions to the designer, because they depend on the number of bit
/// clocks per frame (pp. 16 to 21): a frame must be longer than two words, and
/// [`slot_offset`](Self::slot_offset) plus the word length must fit into it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InterfaceConfig {
    /// The serial format.
    pub format: Format,
    /// The word length.
    pub word_length: WordLength,
    /// Whether this chip drives the bit clock.
    pub bclk: ClockDirection,
    /// Whether this chip drives the word clock.
    pub wclk: ClockDirection,
    /// Whether DOUT goes high-impedance after the last bit of each of this chip's words and stays
    /// so for the rest of the frame: register 27, bit D0 (Table 33, p. 49; pp. 15 and 21). Chips
    /// that share a data line need it, because without it a chip keeps driving the line.
    pub tristate: bool,
    /// Whether DOUT goes high-impedance half a bit clock earlier than [`tristate`](Self::tristate)
    /// alone would make it: register 38, bit D1 (Table 44, p. 52; p. 15). On after a reset.
    pub early_tristate: bool,
    /// Ch_Offset_1: how many bit clocks after the word clock this chip's first channel starts.
    /// The register takes any value from 0 to 255 (Table 34, p. 50).
    pub slot_offset: u8,
    /// Whether the bit clock is inverted: register 29, bit D3 (Table 35, p. 50).
    pub bclk_inverted: bool,
    /// Whether BCLK and WCLK keep running while the ADC is powered down: register 29, bit D2
    /// (Table 35, p. 50). Only meaningful on a chip that drives them; chips that are clocked from
    /// its BCLK need it (p. 16).
    pub clocks_active_when_powered_down: bool,
    /// What feeds the BCLK N divider.
    pub bclk_divider_input: BclkDividerInput,
    /// Which channels are sent.
    pub channels: ChannelEnable,
    /// Whether the two channels swap places on the bus: register 38, bit D4 (Table 44, p. 52).
    pub channel_swap: bool,
}

impl InterfaceConfig {
    /// The interface after a reset: I2S, 16 bits, both clocks inputs, DOUT driven all the time,
    /// no offset, both channels (Tables 33, 34, 35 and 44, pp. 49 to 52).
    pub const RESET: Self = Self {
        format: Format::I2s,
        word_length: WordLength::Bits16,
        bclk: ClockDirection::Input,
        wclk: ClockDirection::Input,
        tristate: false,
        early_tristate: true,
        slot_offset: 0,
        bclk_inverted: false,
        clocks_active_when_powered_down: false,
        bclk_divider_input: BclkDividerInput::AdcClk,
        channels: ChannelEnable::Both,
        channel_swap: false,
    };

    /// The value of register 27 (Table 33).
    const fn control_1_bits(&self) -> u8 {
        let mut bits = self.format.bits() | self.word_length.bits();
        if matches!(self.bclk, ClockDirection::Output) {
            bits |= interface_control_1::BCLK_OUTPUT;
        }
        if matches!(self.wclk, ClockDirection::Output) {
            bits |= interface_control_1::WCLK_OUTPUT;
        }
        if self.tristate {
            bits |= interface_control_1::DOUT_TRISTATE;
        }
        bits
    }

    /// The value of register 29 (Table 35).
    const fn control_2_bits(&self) -> u8 {
        let mut bits = self.bclk_divider_input.bits();
        if self.bclk_inverted {
            bits |= interface_control_2::BCLK_INVERTED;
        }
        if self.clocks_active_when_powered_down {
            bits |= interface_control_2::CLOCKS_ACTIVE_WHEN_POWERED_DOWN;
        }
        bits
    }

    /// The value of register 38, with time-slot mode off (Table 44).
    const fn tdm_bits(&self) -> u8 {
        let mut bits = self.channels.bits();
        if self.channel_swap {
            bits |= i2s_tdm_control::CHANNEL_SWAP;
        }
        if self.early_tristate {
            bits |= i2s_tdm_control::EARLY_TRISTATE;
        }
        bits
    }
}

impl<I2C: I2c> Tlv320adc3101<I2C> {
    /// Programs the serial interface: registers 27, 28, 29 and 38 of page 0, in that order, each
    /// written whole.
    pub fn configure_interface(
        &mut self,
        interface: &InterfaceConfig,
    ) -> Result<(), Error<I2C::Error>> {
        self.write(register::INTERFACE_CONTROL_1, interface.control_1_bits())?;
        self.write(register::CH_OFFSET_1, interface.slot_offset)?;
        self.write(register::INTERFACE_CONTROL_2, interface.control_2_bits())?;
        self.write(register::I2S_TDM_CONTROL, interface.tdm_bits())
    }

    /// Turns the DOUT pin into the audio data output (`true`) or powers its output buffer down
    /// (`false`): page 0, register 53 (Table 57, p. 55).
    ///
    /// After a reset the pin is the data output already. Switching it off is for the time a chip
    /// is being configured while it shares the data line with others: keep DOUT off until the
    /// clocks run and switch it on as the last step.
    ///
    /// The bus keeper stays disabled, its reset value. The pin's other functions (general-purpose
    /// output, CLKOUT, interrupts, secondary clocks) are not offered.
    pub fn set_data_output(&mut self, enabled: bool) -> Result<(), Error<I2C::Error>> {
        let function = if enabled {
            dout_control::FUNCTION_PRIMARY_DOUT
        } else {
            dout_control::FUNCTION_DISABLED
        };
        self.write(
            register::DOUT_CONTROL,
            dout_control::BUS_KEEPER_DISABLED | function,
        )
    }
}
