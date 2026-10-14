//! The clock tree: where the codec clock comes from, the PLL, and the dividers down to the sample
//! rate (section 10.3.7, pp. 22 to 24; the whole tree is drawn in Figure 28, p. 24).
//!
//! The sample rate is `CODEC_CLKIN / (NADC * MADC * AOSR)`. CODEC_CLKIN is the MCLK pin, the BCLK
//! pin or the PLL's output, and the PLL multiplies its own input by `R * J.D / P` (p. 22). A chip
//! that drives the bit clock derives it from ADC_CLK or ADC_MOD_CLK through the BCLK N divider.
//!
//! Every value type here can only hold what its register field can hold, so
//! [`Tlv320adc3101::configure_clocks`] cannot be handed a value the chip has no encoding for. What
//! the types do not check is whether the resulting frequencies are in range, because that depends
//! on the input clock, which the driver does not know:
//!
//! - with D = 0 the PLL needs 512 kHz <= PLL_CLKIN / P <= 20 MHz, an output of 80 MHz to 110 MHz
//!   and 4 <= J <= 55 (p. 22);
//! - with D != 0 it needs 10 MHz <= PLL_CLKIN / P <= 20 MHz, the same output range, 4 <= J <= 11
//!   and R = 1 (p. 23);
//! - `MADC * AOSR` has to be larger than the instruction count of the selected processing block
//!   (Figure 28 note, p. 24; the counts are in Table 6, p. 31);
//! - AOSR has to be a multiple of the processing block's decimation (Table 27 note, p. 48).

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::register::{self, clock_gen_mux, divider, pll_d_msb, pll_j, pll_p_r};
use crate::{Error, Tlv320adc3101};

/// How long [`Tlv320adc3101::configure_clocks`] waits after powering the PLL, in milliseconds.
///
/// The datasheet asks for "approximately 10 ms" between the PLL's power-up command and its clocks
/// being available (section 10.3.2, p. 13). 15 ms is used, because "approximately" leaves no
/// margin and 5 ms more costs nothing in a start-up sequence.
pub const PLL_STARTUP_MS: u32 = 15;

/// Where the PLL takes its input from, PLL_CLKIN: page 0, register 4, bits D3 to D2 (Table 19,
/// p. 47).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PllInput {
    /// The MCLK pin, the reset value.
    Mclk,
    /// The BCLK pin.
    Bclk,
    /// Tied to logic 0, for a PLL that is not used.
    Low,
}

impl PllInput {
    /// The field's bits, in place (Table 19).
    const fn bits(self) -> u8 {
        match self {
            PllInput::Mclk => clock_gen_mux::PLL_CLKIN_MCLK,
            PllInput::Bclk => clock_gen_mux::PLL_CLKIN_BCLK,
            PllInput::Low => clock_gen_mux::PLL_CLKIN_ZERO,
        }
    }

    /// The PLL input a value of register 4 selects. `None` for the reserved code `10`
    /// (Table 19).
    pub(crate) const fn from_bits(mux: u8) -> Option<Self> {
        match mux & clock_gen_mux::PLL_CLKIN_MASK {
            clock_gen_mux::PLL_CLKIN_MCLK => Some(PllInput::Mclk),
            clock_gen_mux::PLL_CLKIN_BCLK => Some(PllInput::Bclk),
            clock_gen_mux::PLL_CLKIN_ZERO => Some(PllInput::Low),
            _ => None,
        }
    }
}

/// Where the codec takes its clock from, CODEC_CLKIN, the clock NADC divides: page 0, register 4,
/// bits D1 to D0 (Table 19, p. 47).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CodecInput {
    /// The MCLK pin, the reset value.
    Mclk,
    /// The BCLK pin: how a chip follows another chip's bit clock without a master clock of its own.
    Bclk,
    /// The PLL's output.
    Pll,
}

impl CodecInput {
    /// The field's bits, in place (Table 19).
    const fn bits(self) -> u8 {
        match self {
            CodecInput::Mclk => clock_gen_mux::CODEC_CLKIN_MCLK,
            CodecInput::Bclk => clock_gen_mux::CODEC_CLKIN_BCLK,
            CodecInput::Pll => clock_gen_mux::CODEC_CLKIN_PLL,
        }
    }

    /// The codec input a value of register 4 selects. `None` for the reserved code `10`
    /// (Table 19).
    pub(crate) const fn from_bits(mux: u8) -> Option<Self> {
        match mux & clock_gen_mux::CODEC_CLKIN_MASK {
            clock_gen_mux::CODEC_CLKIN_MCLK => Some(CodecInput::Mclk),
            clock_gen_mux::CODEC_CLKIN_BCLK => Some(CodecInput::Bclk),
            clock_gen_mux::CODEC_CLKIN_PLL => Some(CodecInput::Pll),
            _ => None,
        }
    }
}

/// The PLL's input divider P, 1 to 8 (Table 20, p. 47; range on p. 22).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PllP(u8);

impl PllP {
    /// P = 1, the reset value (Table 20).
    pub const RESET: Self = Self(1);

    /// `None` unless `p` is 1 to 8, the values the three-bit field encodes (Table 20).
    pub const fn new(p: u8) -> Option<Self> {
        if p >= 1 && p <= 8 {
            Some(Self(p))
        } else {
            None
        }
    }

    /// The divider, 1 to 8.
    pub const fn get(self) -> u8 {
        self.0
    }

    /// The field's bits, in place: 1 to 7 are themselves and 8 is written as 0 (Table 20).
    const fn bits(self) -> u8 {
        ((self.0 & 0b111) << pll_p_r::P_SHIFT) & pll_p_r::P_MASK
    }

    /// The divider a value of register 5 holds: the field's 1 to 7 are themselves and 0 is 8
    /// (Table 20), so every value decodes.
    pub(crate) const fn from_bits(p_r: u8) -> Self {
        match (p_r & pll_p_r::P_MASK) >> pll_p_r::P_SHIFT {
            0 => Self(8),
            p => Self(p),
        }
    }
}

/// The PLL's multiplier R, 1 to 16 (Table 20, p. 47; range on p. 22).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PllR(u8);

impl PllR {
    /// R = 1, the reset value (Table 20).
    pub const RESET: Self = Self(1);

    /// `None` unless `r` is 1 to 16, the values the four-bit field encodes (Table 20).
    pub const fn new(r: u8) -> Option<Self> {
        if r >= 1 && r <= 16 {
            Some(Self(r))
        } else {
            None
        }
    }

    /// The multiplier, 1 to 16.
    pub const fn get(self) -> u8 {
        self.0
    }

    /// The field's bits, in place: 1 to 15 are themselves and 16 is written as 0 (Table 20).
    const fn bits(self) -> u8 {
        self.0 & pll_p_r::R_MASK
    }

    /// The multiplier a value of register 5 holds: the field's 1 to 15 are themselves and 0 is
    /// 16 (Table 20), so every value decodes.
    pub(crate) const fn from_bits(p_r: u8) -> Self {
        match p_r & pll_p_r::R_MASK {
            0 => Self(16),
            r => Self(r),
        }
    }
}

/// The PLL's multiplier J, the integer part of K = J.D: 1 to 63 (Table 21, p. 47).
///
/// The register accepts 1 to 63, but the PLL only meets its specification for 4 to 55 with D = 0
/// and 4 to 11 otherwise (pp. 22 to 23).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PllJ(u8);

impl PllJ {
    /// J = 4, the reset value (Table 21).
    pub const RESET: Self = Self(4);

    /// `None` unless `j` is 1 to 63; 0 is reserved (Table 21).
    pub const fn new(j: u8) -> Option<Self> {
        if j >= 1 && j <= 63 {
            Some(Self(j))
        } else {
            None
        }
    }

    /// The multiplier, 1 to 63.
    pub const fn get(self) -> u8 {
        self.0
    }

    /// The field's bits (Table 21).
    const fn bits(self) -> u8 {
        self.0 & pll_j::J_MASK
    }

    /// The multiplier a value of register 6 holds. `None` for the reserved 0 (Table 21).
    pub(crate) const fn from_bits(j: u8) -> Option<Self> {
        Self::new(j & pll_j::J_MASK)
    }
}

/// The PLL's fractional multiplier D, the four decimal digits after the point of K = J.D: 0 to
/// 9999 (p. 22; Tables 22 and 23, p. 47). K = 8.5 is J = 8 and D = 5000.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PllD(u16);

impl PllD {
    /// D = 0, the reset value (Tables 22 and 23): an integer multiplier.
    pub const RESET: Self = Self(0);

    /// `None` unless `d` is 0 to 9999 (p. 22).
    pub const fn new(d: u16) -> Option<Self> {
        if d <= 9999 { Some(Self(d)) } else { None }
    }

    /// The fraction in ten-thousandths, 0 to 9999.
    pub const fn get(self) -> u16 {
        self.0
    }

    /// The fraction the values of registers 7 and 8 hold: a 14-bit number, high six bits first
    /// (Tables 22 and 23). `None` above 9999, which the 14 bits can hold but the PLL does not
    /// define (p. 22).
    pub(crate) const fn from_bits(msb: u8, lsb: u8) -> Option<Self> {
        Self::new(u16::from_be_bytes([msb & pll_d_msb::D_HIGH_MASK, lsb]))
    }
}

/// The PLL: its power bit and the four values of `PLL_CLKIN * R * J.D / P` (p. 22).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pll {
    /// Whether the PLL is powered: page 0, register 5, bit D7 (Table 20, p. 47).
    pub powered: bool,
    /// The input divider.
    pub p: PllP,
    /// The multiplier R.
    pub r: PllR,
    /// The integer part of the multiplier K.
    pub j: PllJ,
    /// The fractional part of the multiplier K.
    pub d: PllD,
}

impl Pll {
    /// The PLL after a reset: powered down, P = 1, R = 1, J = 4, D = 0 (Tables 20 to 23, p. 47).
    pub const RESET: Self = Self {
        powered: false,
        p: PllP::RESET,
        r: PllR::RESET,
        j: PllJ::RESET,
        d: PllD::RESET,
    };

    /// The value of register 5: power, P and R (Table 20).
    const fn p_r_bits(self) -> u8 {
        let power = if self.powered { pll_p_r::POWER_UP } else { 0 };
        power | self.p.bits() | self.r.bits()
    }

    /// The PLL the values of registers 5 to 8 describe (Tables 20 to 23). `None` when J or D
    /// holds a value the datasheet does not define.
    pub(crate) const fn from_bits(p_r: u8, j: u8, d_msb: u8, d_lsb: u8) -> Option<Self> {
        let (Some(j), Some(d)) = (PllJ::from_bits(j), PllD::from_bits(d_msb, d_lsb)) else {
            return None;
        };
        Some(Self {
            powered: p_r & pll_p_r::POWER_UP != 0,
            p: PllP::from_bits(p_r),
            r: PllR::from_bits(p_r),
            j,
            d,
        })
    }
}

/// What the NADC, MADC and BCLK N dividers divide by: 1 to 128 (Tables 25, 26 and 36, pp. 48 and
/// 50).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Divisor(u8);

impl Divisor {
    /// Divide by 1, the reset value of all three dividers (Tables 25, 26 and 36).
    pub const RESET: Self = Self(1);

    /// `None` unless `divisor` is 1 to 128, the values the seven-bit field encodes (Tables 25, 26
    /// and 36).
    pub const fn new(divisor: u8) -> Option<Self> {
        if divisor >= 1 && divisor <= 128 {
            Some(Self(divisor))
        } else {
            None
        }
    }

    /// The divisor, 1 to 128.
    pub const fn get(self) -> u8 {
        self.0
    }

    /// The field's bits: 1 to 127 are themselves and 128 is written as 0 (Tables 25, 26 and 36).
    const fn bits(self) -> u8 {
        self.0 & divider::VALUE_MASK
    }

    /// The divisor a divider register holds: the field's 1 to 127 are themselves and 0 is 128
    /// (Tables 25, 26 and 36), so every value decodes.
    pub(crate) const fn from_bits(register: u8) -> Self {
        match register & divider::VALUE_MASK {
            0 => Self(128),
            divisor => Self(divisor),
        }
    }
}

/// One of the NADC, MADC and BCLK N dividers: its power bit and its divisor, which share a
/// register (Tables 25, 26 and 36, pp. 48 and 50).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DividerConfig {
    /// Whether the divider is powered: bit D7. A divider that is powered down passes no clock on.
    pub powered: bool,
    /// What it divides by.
    pub divisor: Divisor,
}

impl DividerConfig {
    /// A divider after a reset: powered down, dividing by 1 (Tables 25, 26 and 36).
    pub const RESET: Self = Self {
        powered: false,
        divisor: Divisor::RESET,
    };

    /// The register value (Tables 25, 26 and 36).
    const fn bits(self) -> u8 {
        let power = if self.powered { divider::POWER_UP } else { 0 };
        power | self.divisor.bits()
    }

    /// The power bit and the divisor a divider register holds (Tables 25, 26 and 36).
    pub(crate) const fn from_bits(register: u8) -> Self {
        Self {
            powered: register & divider::POWER_UP != 0,
            divisor: Divisor::from_bits(register),
        }
    }
}

/// The oversampling ratio AOSR, from ADC_MOD_CLK to the sample rate: 1 to 256 (Table 27, p. 48).
///
/// It must be a multiple of the decimation the processing block uses (Table 27 note), and each
/// processing block lists the ratios it works with (Table 6, p. 31).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Aosr(u16);

impl Aosr {
    /// AOSR = 128, the reset value (Table 27).
    pub const RESET: Self = Self(128);

    /// `None` unless `aosr` is 1 to 256, the values the eight-bit register encodes (Table 27).
    pub const fn new(aosr: u16) -> Option<Self> {
        if aosr >= 1 && aosr <= 256 {
            Some(Self(aosr))
        } else {
            None
        }
    }

    /// The ratio, 1 to 256.
    pub const fn get(self) -> u16 {
        self.0
    }

    /// The register value: 1 to 255 are themselves and 256 is written as 0 (Table 27).
    const fn bits(self) -> u8 {
        self.0.to_le_bytes()[0]
    }

    /// The ratio register 20 holds: 1 to 255 are themselves and 0 is 256 (Table 27), so every
    /// value decodes.
    pub(crate) const fn from_bits(register: u8) -> Self {
        match register {
            0 => Self(256),
            aosr => Self(u16::from_be_bytes([0, aosr])),
        }
    }
}

/// The whole clock tree of one chip: registers 4 to 8, 18 to 20 and 30 of page 0.
///
/// The fields mirror the registers, power bits included, so a configuration can describe an
/// intermediate state too. Several chips on one TDM line need that: the chip that drives the bit
/// clock for the others gets its NADC divider programmed but not powered, and powered as the
/// very last step ([`Tlv320adc3101::set_divider_power`]), so that all of them start at once.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockConfig {
    /// The PLL's input.
    pub pll_input: PllInput,
    /// The codec's input, the clock NADC divides.
    pub codec_input: CodecInput,
    /// The PLL. Its values are written even when it stays powered down.
    pub pll: Pll,
    /// NADC, from CODEC_CLKIN to ADC_CLK.
    pub nadc: DividerConfig,
    /// MADC, from ADC_CLK to ADC_MOD_CLK.
    pub madc: DividerConfig,
    /// AOSR, from ADC_MOD_CLK to the sample rate.
    pub aosr: Aosr,
    /// BCLK N, from ADC_CLK or ADC_MOD_CLK ([`crate::InterfaceConfig::bclk_divider_input`]) to
    /// the BCLK pin. It only matters on a chip that drives BCLK.
    pub bclk_n: DividerConfig,
}

impl ClockConfig {
    /// The clock tree after a reset: everything from the MCLK pin, the PLL and all dividers
    /// powered down, AOSR 128 (Tables 19 to 27 and 36, pp. 47 to 50).
    pub const RESET: Self = Self {
        pll_input: PllInput::Mclk,
        codec_input: CodecInput::Mclk,
        pll: Pll::RESET,
        nadc: DividerConfig::RESET,
        madc: DividerConfig::RESET,
        aosr: Aosr::RESET,
        bclk_n: DividerConfig::RESET,
    };
}

/// Names one of the three dividers for [`Tlv320adc3101::set_divider_power`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Divider {
    /// NADC: page 0, register 18 (Table 25, p. 48).
    Nadc,
    /// MADC: page 0, register 19 (Table 26, p. 48).
    Madc,
    /// BCLK N: page 0, register 30 (Table 36, p. 50).
    BclkN,
}

impl Divider {
    /// The divider's register.
    const fn register(self) -> crate::Register {
        match self {
            Divider::Nadc => register::NADC,
            Divider::Madc => register::MADC,
            Divider::BclkN => register::BCLK_N,
        }
    }
}

impl<I2C: I2c> Tlv320adc3101<I2C> {
    /// Programs the clock tree: registers 4 to 8, 18 to 20 and 30 of page 0.
    ///
    /// The order is the one the datasheet's design procedure gives (section 11.2.2.2, step 2,
    /// p. 80): the clock sources, then the PLL's J and D, then P and R together with the PLL's
    /// power bit, then NADC, MADC, AOSR and the BCLK divider. If the configuration powers the PLL,
    /// the method waits [`PLL_STARTUP_MS`] after the power bit before it touches the dividers, so
    /// a divider is never powered from a PLL that has not started. J and D go before the power
    /// bit so that the PLL starts once, with its final multiplier.
    ///
    /// Meant for a chip that is not converting: call it after [`reset`](Self::reset), before the
    /// ADC channels are powered.
    pub fn configure_clocks<D: DelayNs>(
        &mut self,
        clocks: &ClockConfig,
        delay: &mut D,
    ) -> Result<(), Error<I2C::Error>> {
        self.write(
            register::CLOCK_GEN_MUX,
            clocks.pll_input.bits() | clocks.codec_input.bits(),
        )?;
        self.write(register::PLL_J, clocks.pll.j.bits())?;
        // D is a 14-bit number split over two registers. The chip only takes the high part when the
        // low part is written immediately after it (Tables 22 and 23, notes), so nothing may come
        // between these two writes.
        let [d_high, d_low] = clocks.pll.d.get().to_be_bytes();
        self.write(register::PLL_D_MSB, d_high)?;
        self.write(register::PLL_D_LSB, d_low)?;
        self.write(register::PLL_P_R, clocks.pll.p_r_bits())?;
        if clocks.pll.powered {
            delay.delay_ms(PLL_STARTUP_MS);
        }
        self.write(register::NADC, clocks.nadc.bits())?;
        self.write(register::MADC, clocks.madc.bits())?;
        self.write(register::AOSR, clocks.aosr.bits())?;
        self.write(register::BCLK_N, clocks.bclk_n.bits())
    }

    /// Powers one divider up or down and leaves its divisor as it is.
    ///
    /// This is the step for a divider that has to start after everything else, such as the NADC
    /// divider of a chip that drives the bit clock for others: powering it starts BCLK and WCLK
    /// for all of them at once.
    ///
    /// It reads the register and writes it back with the power bit changed, so it needs a chip
    /// this driver owns.
    pub fn set_divider_power(
        &mut self,
        which: Divider,
        powered: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let power = if powered { divider::POWER_UP } else { 0 };
        self.modify(which.register(), divider::POWER_UP, power)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pure decoding, no bus: the fields whose register value 0 stands for the largest setting
    /// (Tables 20, 25 to 27 and 36), and the values the datasheet leaves undefined.
    #[test]
    fn register_values_decode_with_their_wrap_arounds_and_reserved_codes() {
        // Register 5 = 0x80: PLL powered, P field 0 (8), R field 0 (16).
        let pll = Pll::from_bits(0x80, 63, 0x27, 0x0F).expect("J 63 and D 9999 are defined");
        assert!(pll.powered);
        assert_eq!((pll.p.get(), pll.r.get()), (8, 16));
        assert_eq!((pll.j.get(), pll.d.get()), (63, 9999));
        // J = 0 is reserved (Table 21); D = 10000 (0x2710) is outside 0 to 9999 (p. 22).
        assert_eq!(Pll::from_bits(0x11, 0, 0, 0), None);
        assert_eq!(Pll::from_bits(0x11, 4, 0x27, 0x10), None);

        assert_eq!(DividerConfig::from_bits(0x80).divisor.get(), 128);
        assert!(DividerConfig::from_bits(0x80).powered);
        assert_eq!(DividerConfig::from_bits(0x7F).divisor.get(), 127);
        assert!(!DividerConfig::from_bits(0x7F).powered);
        assert_eq!(Aosr::from_bits(0).get(), 256);
        assert_eq!(Aosr::from_bits(255).get(), 255);

        // Register 4: the code 10 is reserved in both fields (Table 19).
        assert_eq!(PllInput::from_bits(0b1000), None);
        assert_eq!(CodecInput::from_bits(0b0010), None);
        assert_eq!(PllInput::from_bits(0b1101), Some(PllInput::Low));
        assert_eq!(CodecInput::from_bits(0b1101), Some(CodecInput::Bclk));
    }
}
