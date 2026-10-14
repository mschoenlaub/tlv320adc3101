//! Reading a chip's state back without writing anything to it: the status API for a chip that
//! somebody else owns (crate docs, "Reading next to another owner").
//!
//! # What can be read depends on the page the owner left selected
//!
//! The chip shows one page at a time, and selecting another one is a register write that the
//! owner would not know about. So nothing here selects a page. Every value is fetched
//! with [`Tlv320adc3101::read_passive`], which reads the page-select register and the wanted
//! register in one bus transaction and keeps the value only if the page is the register's own.
//! What a call returns is therefore one of two things:
//!
//! - the registers of the page that happens to be selected, as raw bytes with typed decoders
//!   ([`AnalogRegisters`] for page 1; [`ClockRegisters`] and [`DigitalRegisters`] for page 0);
//! - [`Error::PageMismatch`], naming the page the chip is on, when the registers asked for live
//!   in another page. That is an answer and not a fault: ask for the other page's registers, or
//!   call [`Tlv320adc3101::status`], which looks at the page first and reads what it offers.
//!
//! Which page that is depends on the owner's last write. An owner whose capture start ends with
//! the microphone bias leaves the chip on page 1 while a capture runs, so [`AnalogRegisters`] is
//! what a read next to it normally gets, and the page 0 registers only when the chip happens to
//! be there.
//!
//! # What goes on the bus
//!
//! Per register one transaction of four messages: the register number 0 (one byte written), one
//! byte read (the page), the register number (one byte written), one byte read (the value). The
//! only bytes ever written are register numbers, which move the chip's address pointer and change
//! no register (Figure 47, p. 43). [`Tlv320adc3101::status`] starts with one more transaction of
//! two messages, the page read of [`Tlv320adc3101::page`].
//!
//! The registers of one call are read one transaction after the other, so they are not one
//! atomic snapshot: the owner can write between two of them. Each value is still tied to the page
//! it came from, and if the owner changes the page in the middle, the call ends with
//! [`Error::PageMismatch`] instead of returning bytes from two pages.
//!
//! # What is left out
//!
//! - The ADC flag register (page 0, register 36), which holds the only "ADC is powered up"
//!   status bits (Table 42, p. 52): two of its bits are sticky and cleared by a read, so the
//!   driver refuses it ([`crate::register::CLEAR_ON_READ`]). Its "gain applied" bits exist a
//!   second time in page 1, register 62, which [`AnalogRegisters::flags`] decodes.
//! - The sticky interrupt flag registers 42 and 45, for the same reason. The overflow flags of
//!   register 42 are read from their non-sticky copy, register 43 ([`OverflowFlags`]). The copy
//!   of register 45, register 47, holds AGC and miniDSP flags, which this driver does not cover.
//! - The serial interface registers (27 to 30, 38, 53) and the processing block (61): they have
//!   setters but no typed read-back yet. [`Tlv320adc3101::read_passive`] reads them as bytes.
//! - The software reset register (page 0, register 1): its one bit is write-only and clears
//!   itself (Table 16, p. 46), so there is nothing to read back.

use embedded_hal::i2c::I2c;

use crate::analog::{LeftInputs, MicBias, PgaGain, RightInputs};
use crate::clock::{Aosr, ClockConfig, CodecInput, DividerConfig, Pll, PllInput};
use crate::digital::{DigitalVolume, FineGain, SoftStepping};
use crate::filter::Channel;
use crate::register::{
    self, adc_digital, adc_fine_volume, micbias_control, overflow_flags, pga, pga_flags,
};
use crate::{Error, Register, Tlv320adc3101};

/// The registers [`Tlv320adc3101::analog_registers`] reads, in the order it reads them: page 1,
/// registers 51, 52, 54, 55, 57, 59, 60 and 62 (Tables 93, 94, 96, 97, 99, 101, 102 and 104,
/// pp. 65 to 68). The reserved registers between them (53, 56, 58) and the low-current mode
/// register (61) are not read.
pub const ANALOG_REGISTERS: [Register; 8] = [
    register::MICBIAS_CONTROL,
    register::LEFT_PGA_INPUT_1,
    register::LEFT_PGA_INPUT_2,
    register::RIGHT_PGA_INPUT_1,
    register::RIGHT_PGA_INPUT_2,
    register::LEFT_PGA,
    register::RIGHT_PGA,
    register::PGA_FLAGS,
];

/// The registers [`Tlv320adc3101::clock_registers`] reads, in the order it reads them: page 0,
/// registers 4 to 8, 18 to 20 and 30 (Tables 19 to 23, 25 to 27 and 36, pp. 47 to 50).
pub const CLOCK_REGISTERS: [Register; 9] = [
    register::CLOCK_GEN_MUX,
    register::PLL_P_R,
    register::PLL_J,
    register::PLL_D_MSB,
    register::PLL_D_LSB,
    register::NADC,
    register::MADC,
    register::AOSR,
    register::BCLK_N,
];

/// The registers [`Tlv320adc3101::digital_registers`] reads, in the order it reads them: page 0,
/// registers 81 to 84 and 43 (Tables 67 to 70 and 47, pp. 53 and 58 to 59). Register 43 is the
/// non-sticky copy of the overflow flags; the sticky one, 42, is never read.
pub const DIGITAL_REGISTERS: [Register; 5] = [
    register::ADC_DIGITAL,
    register::ADC_FINE_VOLUME,
    register::LEFT_ADC_VOLUME,
    register::RIGHT_ADC_VOLUME,
    register::INTERRUPT_FLAGS_OVERFLOW,
];

/// Whether any register of `registers` has the address of a clear-on-read register, in whatever
/// page. For the compile-time check below.
// Its only use is in the anonymous `const _` assertion below, which rustc before 1.89 does not
// count as a use, so the MSRV build would report it as dead code.
#[allow(dead_code)]
const fn touches_a_clear_on_read_address(registers: &[Register]) -> bool {
    let mut i = 0;
    while i < registers.len() {
        if register::is_clear_on_read_in_any_page(registers[i].addr()) {
            return true;
        }
        i += 1;
    }
    false
}

// A passive read learns the selected page only together with the value. If the owner switches the
// page just before the read, the address lands in the other page, so none of the addresses read
// here may be that of a sticky register in any page. `read_selected_page` refuses such an address
// at run time; this makes a list that contains one fail to build.
const _: () = assert!(
    !touches_a_clear_on_read_address(&ANALOG_REGISTERS)
        && !touches_a_clear_on_read_address(&CLOCK_REGISTERS)
        && !touches_a_clear_on_read_address(&DIGITAL_REGISTERS)
);

/// What a PGA register holds: page 1, register 59 (left) or 60 (right) (Tables 101 and 102,
/// p. 68).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PgaState {
    /// Bit D7: the PGA is muted. Set after a reset.
    pub muted: bool,
    /// Bits D6 to D0 as the register holds them: the gain in steps of 0.5 dB, 0 to 127. The
    /// datasheet defines 0 to 80 (0 dB to 40 dB) and calls the rest reserved, but the register
    /// takes all seven bits and a mixer control may let larger values through, so a read-back has
    /// to be able to show them.
    pub half_db: u8,
}

impl PgaState {
    /// Decodes a value of register 59 or 60 (Tables 101 and 102).
    pub const fn from_bits(register: u8) -> Self {
        Self {
            muted: register & pga::MUTE != 0,
            half_db: register & pga::GAIN_MASK,
        }
    }

    /// The gain as the type the setter takes. `None` when the register holds one of the codes
    /// the datasheet calls reserved, 81 to 127 (Tables 101 and 102).
    pub const fn gain(self) -> Option<PgaGain> {
        PgaGain::from_half_db(self.half_db)
    }
}

/// The PGA flags: page 1, register 62 (Table 104, p. 68). Live status bits, not sticky; reading
/// them changes nothing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PgaFlags {
    /// Bit D1: the left PGA's applied gain equals its programmed gain, so a soft-stepped gain
    /// change has finished.
    pub left_gain_applied: bool,
    /// Bit D0: the same for the right PGA.
    pub right_gain_applied: bool,
}

impl PgaFlags {
    /// Decodes a value of register 62 (Table 104).
    pub const fn from_bits(register: u8) -> Self {
        Self {
            left_gain_applied: register & pga_flags::LEFT_GAIN_APPLIED != 0,
            right_gain_applied: register & pga_flags::RIGHT_GAIN_APPLIED != 0,
        }
    }
}

/// The analog side of a chip as read back: the values of the page 1 registers in
/// [`ANALOG_REGISTERS`], one field per register, and what they decode to.
///
/// The fields are the bytes as the chip returned them, reserved bits included, so that a caller
/// can print or compare exactly what was read; the methods decode them and involve no bus.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AnalogRegisters {
    /// Page 1, register 51: MICBIAS control (Table 93, p. 65).
    pub micbias_control: u8,
    /// Page 1, register 52: left PGA input selection, first register (Table 94, p. 66).
    pub left_pga_input_1: u8,
    /// Page 1, register 54: left PGA input selection, second register (Table 96, p. 66).
    pub left_pga_input_2: u8,
    /// Page 1, register 55: right PGA input selection, first register (Table 97, p. 67).
    pub right_pga_input_1: u8,
    /// Page 1, register 57: right PGA input selection, second register (Table 99, p. 67).
    pub right_pga_input_2: u8,
    /// Page 1, register 59: left PGA mute and gain (Table 101, p. 68).
    pub left_pga: u8,
    /// Page 1, register 60: right PGA mute and gain (Table 102, p. 68).
    pub right_pga: u8,
    /// Page 1, register 62: PGA flags (Table 104, p. 68).
    pub pga_flags: u8,
}

impl AnalogRegisters {
    /// The bytes in the order of [`ANALOG_REGISTERS`], for printing and comparing.
    pub const fn bytes(&self) -> [u8; 8] {
        [
            self.micbias_control,
            self.left_pga_input_1,
            self.left_pga_input_2,
            self.right_pga_input_1,
            self.right_pga_input_2,
            self.left_pga,
            self.right_pga,
            self.pga_flags,
        ]
    }

    /// The two microphone bias outputs, MICBIAS1 then MICBIAS2: register 51, bits D6 to D5 and
    /// D4 to D3 (Table 93). The order is the one [`Tlv320adc3101::set_mic_bias`] takes.
    pub const fn mic_bias(&self) -> (MicBias, MicBias) {
        (
            MicBias::from_code(self.micbias_control >> micbias_control::MICBIAS1_SHIFT),
            MicBias::from_code(self.micbias_control >> micbias_control::MICBIAS2_SHIFT),
        )
    }

    /// What is switched onto the left PGA: registers 52 and 54 (Tables 94 and 96).
    pub const fn left_inputs(&self) -> LeftInputs {
        LeftInputs::from_bits(self.left_pga_input_1, self.left_pga_input_2)
    }

    /// What is switched onto the right PGA: registers 55 and 57 (Tables 97 and 99).
    pub const fn right_inputs(&self) -> RightInputs {
        RightInputs::from_bits(self.right_pga_input_1, self.right_pga_input_2)
    }

    /// Mute and gain of one PGA: register 59 or 60 (Tables 101 and 102).
    pub const fn pga(&self, channel: Channel) -> PgaState {
        match channel {
            Channel::Left => PgaState::from_bits(self.left_pga),
            Channel::Right => PgaState::from_bits(self.right_pga),
        }
    }

    /// Whether the PGAs have reached their programmed gain: register 62 (Table 104).
    pub const fn flags(&self) -> PgaFlags {
        PgaFlags::from_bits(self.pga_flags)
    }
}

/// The clock tree of a chip as read back: the values of the page 0 registers in
/// [`CLOCK_REGISTERS`], one field per register, and what they decode to.
///
/// The fields are the bytes as the chip returned them; the methods decode them and involve no
/// bus. A decoder returns `None` where the register holds a code the datasheet calls reserved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockRegisters {
    /// Page 0, register 4: clock-gen multiplexing (Table 19, p. 47).
    pub clock_gen_mux: u8,
    /// Page 0, register 5: PLL power, P and R (Table 20, p. 47).
    pub pll_p_r: u8,
    /// Page 0, register 6: PLL J (Table 21, p. 47).
    pub pll_j: u8,
    /// Page 0, register 7: PLL D, bits 13 to 8 (Table 22, p. 47).
    pub pll_d_msb: u8,
    /// Page 0, register 8: PLL D, bits 7 to 0 (Table 23, p. 47).
    pub pll_d_lsb: u8,
    /// Page 0, register 18: NADC (Table 25, p. 48).
    pub nadc: u8,
    /// Page 0, register 19: MADC (Table 26, p. 48).
    pub madc: u8,
    /// Page 0, register 20: AOSR (Table 27, p. 48).
    pub aosr: u8,
    /// Page 0, register 30: BCLK N (Table 36, p. 50).
    pub bclk_n: u8,
}

impl ClockRegisters {
    /// The clock registers after a reset, hardware or software (Tables 19 to 23, 25 to 27 and 36,
    /// pp. 47 to 50): register 4 `0x00` (PLL and codec from MCLK), 5 `0x11` (PLL powered down,
    /// P = 1, R = 1), 6 `0x04` (J = 4), 7 and 8 `0x00` (D = 0), 18 and 19 `0x01` (NADC and MADC 1,
    /// powered down), 20 `0x80` ([`register::AOSR_RESET`]), 30 `0x01` (BCLK N 1, powered down).
    /// These are the raw bytes of [`ClockConfig::RESET`], reserved bits (all zero) included.
    pub const RESET: Self = Self {
        clock_gen_mux: 0x00,
        pll_p_r: 0x11,
        pll_j: 0x04,
        pll_d_msb: 0x00,
        pll_d_lsb: 0x00,
        nadc: 0x01,
        madc: 0x01,
        aosr: register::AOSR_RESET,
        bclk_n: 0x01,
    };

    /// Whether every register holds its byte of [`ClockRegisters::RESET`]: nothing has programmed
    /// the clock tree since the chip's last reset. The raw bytes are compared, so a reserved bit
    /// that reads set also counts as "not at reset". Any clock tree that produces a sample rate
    /// changes at least the divider registers, so a chip that has been set up never reads as at
    /// reset.
    pub fn at_reset(&self) -> bool {
        self.bytes() == Self::RESET.bytes()
    }

    /// The bytes in the order of [`CLOCK_REGISTERS`], for printing and comparing.
    pub const fn bytes(&self) -> [u8; 9] {
        [
            self.clock_gen_mux,
            self.pll_p_r,
            self.pll_j,
            self.pll_d_msb,
            self.pll_d_lsb,
            self.nadc,
            self.madc,
            self.aosr,
            self.bclk_n,
        ]
    }

    /// The PLL's input: register 4, bits D3 to D2. `None` for the reserved code `10` (Table 19).
    pub const fn pll_input(&self) -> Option<PllInput> {
        PllInput::from_bits(self.clock_gen_mux)
    }

    /// The codec's input: register 4, bits D1 to D0. `None` for the reserved code `10`
    /// (Table 19).
    pub const fn codec_input(&self) -> Option<CodecInput> {
        CodecInput::from_bits(self.clock_gen_mux)
    }

    /// The PLL: registers 5 to 8 (Tables 20 to 23). `None` when J is the reserved 0 or D is
    /// above 9999.
    pub const fn pll(&self) -> Option<Pll> {
        Pll::from_bits(self.pll_p_r, self.pll_j, self.pll_d_msb, self.pll_d_lsb)
    }

    /// The NADC divider: register 18 (Table 25).
    pub const fn nadc(&self) -> DividerConfig {
        DividerConfig::from_bits(self.nadc)
    }

    /// The MADC divider: register 19 (Table 26).
    pub const fn madc(&self) -> DividerConfig {
        DividerConfig::from_bits(self.madc)
    }

    /// The oversampling ratio: register 20 (Table 27).
    pub const fn aosr(&self) -> Aosr {
        Aosr::from_bits(self.aosr)
    }

    /// The BCLK N divider: register 30 (Table 36).
    pub const fn bclk_n(&self) -> DividerConfig {
        DividerConfig::from_bits(self.bclk_n)
    }

    /// The whole clock tree as the value [`Tlv320adc3101::configure_clocks`] takes, so that a
    /// read-back can be compared with a configuration. `None` when a multiplexer field or the
    /// PLL holds a reserved value; the single decoders say which.
    pub const fn config(&self) -> Option<ClockConfig> {
        let (Some(pll_input), Some(codec_input), Some(pll)) =
            (self.pll_input(), self.codec_input(), self.pll())
        else {
            return None;
        };
        Some(ClockConfig {
            pll_input,
            codec_input,
            pll,
            nadc: self.nadc(),
            madc: self.madc(),
            aosr: self.aosr(),
            bclk_n: self.bclk_n(),
        })
    }
}

/// One ADC channel's digital settings as read back from page 0, registers 81 to 84 (Tables 67 to
/// 70, pp. 58 to 59).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChannelState {
    /// The channel's power bit in register 81 (D7 left, D6 right; Table 67). This is the
    /// setting, what the owner asked for. Whether the ADC has come up is a flag in register 36,
    /// which the driver does not read because a read clears other bits of it (module docs).
    pub power_up: bool,
    /// The channel's digital mute in register 82 (D7 left, D3 right; Table 68). Set after a
    /// reset.
    pub muted: bool,
    /// The channel's fine trim in register 82 (D6 to D4 left, D2 to D0 right; Table 68). `None`
    /// for the reserved codes 5 to 7.
    pub fine_gain: Option<FineGain>,
    /// The channel's digital volume, register 83 (left) or 84 (right) (Tables 69 and 70). `None`
    /// for a code outside -12 dB to +20 dB.
    pub volume: Option<DigitalVolume>,
}

/// The overflow flags: page 0, register 43 (Table 47, p. 53), the copy of the interrupt flags
/// that a read does not clear ("non-sticky" in Table 3, p. 28). Each bit shows the condition at
/// the moment of the read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OverflowFlags {
    /// Bit D3: the left ADC overflows.
    pub left_adc: bool,
    /// Bit D2: the right ADC overflows.
    pub right_adc: bool,
    /// Bit D1: the output of the ADC's barrel shifter overflows.
    pub barrel_shifter: bool,
}

impl OverflowFlags {
    /// Decodes a value of register 43 (Table 47).
    pub const fn from_bits(register: u8) -> Self {
        Self {
            left_adc: register & overflow_flags::LEFT_ADC_OVERFLOW != 0,
            right_adc: register & overflow_flags::RIGHT_ADC_OVERFLOW != 0,
            barrel_shifter: register & overflow_flags::BARREL_SHIFTER_OVERFLOW != 0,
        }
    }
}

/// The digital side of the two ADC channels as read back: the values of the page 0 registers in
/// [`DIGITAL_REGISTERS`], one field per register, and what they decode to.
///
/// The fields are the bytes as the chip returned them; the methods decode them and involve no
/// bus. The digital microphone bits of register 81 (D5 to D2) are not decoded, because the driver
/// does not cover the digital microphone inputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DigitalRegisters {
    /// Page 0, register 81: channel power and soft-stepping (Table 67, p. 58).
    pub adc_digital: u8,
    /// Page 0, register 82: mute and fine trim of both channels (Table 68, p. 58).
    pub adc_fine_volume: u8,
    /// Page 0, register 83: left digital volume (Table 69, p. 58).
    pub left_adc_volume: u8,
    /// Page 0, register 84: right digital volume (Table 70, p. 59).
    pub right_adc_volume: u8,
    /// Page 0, register 43: overflow flags, the non-sticky copy (Table 47, p. 53).
    pub overflow_flags: u8,
}

impl DigitalRegisters {
    /// The bytes in the order of [`DIGITAL_REGISTERS`], for printing and comparing.
    pub const fn bytes(&self) -> [u8; 5] {
        [
            self.adc_digital,
            self.adc_fine_volume,
            self.left_adc_volume,
            self.right_adc_volume,
            self.overflow_flags,
        ]
    }

    /// How gain changes are applied: register 81, bits D1 to D0. `None` for the reserved code
    /// `11` (Table 67).
    pub const fn soft_stepping(&self) -> Option<SoftStepping> {
        SoftStepping::from_bits(self.adc_digital)
    }

    /// Power, mute, fine trim and volume of one channel: registers 81 to 84 (Tables 67 to 70).
    pub const fn channel(&self, channel: Channel) -> ChannelState {
        match channel {
            Channel::Left => ChannelState {
                power_up: self.adc_digital & adc_digital::LEFT_POWER_UP != 0,
                muted: self.adc_fine_volume & adc_fine_volume::LEFT_MUTE != 0,
                fine_gain: FineGain::from_tenths_db_down(
                    (self.adc_fine_volume & adc_fine_volume::LEFT_FINE_GAIN_MASK)
                        >> adc_fine_volume::LEFT_FINE_GAIN_SHIFT,
                ),
                volume: DigitalVolume::from_bits(self.left_adc_volume),
            },
            Channel::Right => ChannelState {
                power_up: self.adc_digital & adc_digital::RIGHT_POWER_UP != 0,
                muted: self.adc_fine_volume & adc_fine_volume::RIGHT_MUTE != 0,
                fine_gain: FineGain::from_tenths_db_down(
                    self.adc_fine_volume & adc_fine_volume::RIGHT_FINE_GAIN_MASK,
                ),
                volume: DigitalVolume::from_bits(self.right_adc_volume),
            },
        }
    }

    /// The overflow flags at the moment of the read: register 43 (Table 47).
    pub const fn overflow(&self) -> OverflowFlags {
        OverflowFlags::from_bits(self.overflow_flags)
    }
}

/// What [`Tlv320adc3101::status`] found: the registers of the page the chip had selected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    /// Page 0 was selected: the clock tree and the digital side of the channels.
    Page0 {
        /// Registers 4 to 8, 18 to 20 and 30.
        clocks: ClockRegisters,
        /// Registers 81 to 84 and 43.
        digital: DigitalRegisters,
    },
    /// Page 1 was selected: microphone bias, input routing, PGA gain and flags.
    Page1(AnalogRegisters),
    /// Another page was selected, and nothing but the page-select register was read. Pages 4 and
    /// 5 hold filter coefficients and 32 to 47 miniDSP instructions (Table 15 note, p. 46), none
    /// of which this API decodes. An owner usually selects these pages only while it writes
    /// coefficients during a set-up, so asking again a moment later is the thing to do.
    OtherPage(u8),
}

impl Status {
    /// The page the chip had selected.
    pub const fn page(&self) -> u8 {
        match self {
            Status::Page0 { .. } => 0,
            Status::Page1(_) => 1,
            Status::OtherPage(page) => *page,
        }
    }
}

impl<I2C: I2c> Tlv320adc3101<I2C> {
    /// Reads each of `registers` with [`read_passive`](Self::read_passive), in order, and stops
    /// at the first one that fails, so that after a page mismatch nothing more is read.
    fn read_all_passive<const N: usize>(
        &mut self,
        registers: &[Register; N],
    ) -> Result<[u8; N], Error<I2C::Error>> {
        let mut values = [0u8; N];
        for (value, register) in values.iter_mut().zip(registers) {
            *value = self.read_passive(*register)?;
        }
        Ok(values)
    }

    /// Reads the analog side: the page 1 registers in [`ANALOG_REGISTERS`].
    ///
    /// Writes no register, so it is safe next to another owner of the chip: eight passive reads, each one
    /// bus transaction (module docs, "What goes on the bus"). Fails with
    /// [`Error::PageMismatch`] when the chip is not on page 1, at the first read that finds
    /// another page; nothing is read after that.
    pub fn analog_registers(&mut self) -> Result<AnalogRegisters, Error<I2C::Error>> {
        let [
            micbias_control,
            left_pga_input_1,
            left_pga_input_2,
            right_pga_input_1,
            right_pga_input_2,
            left_pga,
            right_pga,
            pga_flags,
        ] = self.read_all_passive(&ANALOG_REGISTERS)?;
        Ok(AnalogRegisters {
            micbias_control,
            left_pga_input_1,
            left_pga_input_2,
            right_pga_input_1,
            right_pga_input_2,
            left_pga,
            right_pga,
            pga_flags,
        })
    }

    /// Reads the clock tree: the page 0 registers in [`CLOCK_REGISTERS`].
    ///
    /// Writes no register, so it is safe next to another owner of the chip: nine passive reads, each one
    /// bus transaction (module docs, "What goes on the bus"). Fails with
    /// [`Error::PageMismatch`] when the chip is not on page 0, at the first read that finds
    /// another page; nothing is read after that.
    pub fn clock_registers(&mut self) -> Result<ClockRegisters, Error<I2C::Error>> {
        let [
            clock_gen_mux,
            pll_p_r,
            pll_j,
            pll_d_msb,
            pll_d_lsb,
            nadc,
            madc,
            aosr,
            bclk_n,
        ] = self.read_all_passive(&CLOCK_REGISTERS)?;
        Ok(ClockRegisters {
            clock_gen_mux,
            pll_p_r,
            pll_j,
            pll_d_msb,
            pll_d_lsb,
            nadc,
            madc,
            aosr,
            bclk_n,
        })
    }

    /// Reads the digital side of the channels: the page 0 registers in [`DIGITAL_REGISTERS`].
    ///
    /// Writes no register, so it is safe next to another owner of the chip: five passive reads, each one
    /// bus transaction (module docs, "What goes on the bus"). Fails with
    /// [`Error::PageMismatch`] when the chip is not on page 0, at the first read that finds
    /// another page; nothing is read after that.
    pub fn digital_registers(&mut self) -> Result<DigitalRegisters, Error<I2C::Error>> {
        let [
            adc_digital,
            adc_fine_volume,
            left_adc_volume,
            right_adc_volume,
            overflow_flags,
        ] = self.read_all_passive(&DIGITAL_REGISTERS)?;
        Ok(DigitalRegisters {
            adc_digital,
            adc_fine_volume,
            left_adc_volume,
            right_adc_volume,
            overflow_flags,
        })
    }

    /// Reads what the selected page offers: looks at the page-select register first and then
    /// reads the registers of that page, the clock tree and the digital side on page 0, the
    /// analog side on page 1, and nothing on any other page.
    ///
    /// Writes no register, so it is safe next to another owner of the chip. The page is read first so
    /// that only addresses that mean something in the selected page are read. If the owner
    /// changes the page after that first look, the call fails with [`Error::PageMismatch`];
    /// asking again then reads the new page.
    pub fn status(&mut self) -> Result<Status, Error<I2C::Error>> {
        match self.page()? {
            0 => Ok(Status::Page0 {
                clocks: self.clock_registers()?,
                digital: self.digital_registers()?,
            }),
            1 => Ok(Status::Page1(self.analog_registers()?)),
            other => Ok(Status::OtherPage(other)),
        }
    }
}

// Pure decoding only: these tests build register values by hand and involve no bus. The read
// methods above have no host tests by project policy; they are tried on the device.
#[cfg(test)]
mod tests {
    use super::*;
    use crate::{InputLevel, LeftInputs, MicBias, RightInputs};

    /// Page 1 of a capturing chip: both bias outputs at 2.5 V, the differential pairs IN2/IN3 at
    /// 0 dB on both PGAs, 20 dB of gain, unmuted, the left gain applied.
    #[test]
    fn analog_values_decode_per_field() {
        let analog = AnalogRegisters {
            micbias_control: 0x50,
            left_pga_input_1: 0x3F,
            left_pga_input_2: 0x3F,
            right_pga_input_1: 0x3F,
            right_pga_input_2: 0x3F,
            left_pga: 0x28,
            right_pga: 0x28,
            pga_flags: 0x02,
        };
        assert_eq!(analog.mic_bias(), (MicBias::V2_5, MicBias::V2_5));
        assert_eq!(
            analog.left_inputs(),
            LeftInputs {
                diff_in2l_in3l: InputLevel::ZeroDb,
                ..LeftInputs::NONE
            }
        );
        assert_eq!(
            analog.right_inputs(),
            RightInputs {
                diff_in2r_in3r: InputLevel::ZeroDb,
                ..RightInputs::NONE
            }
        );
        for channel in [Channel::Left, Channel::Right] {
            let pga = analog.pga(channel);
            assert!(!pga.muted);
            assert_eq!(pga.gain(), PgaGain::from_half_db(40));
        }
        assert_eq!(
            analog.flags(),
            PgaFlags {
                left_gain_applied: true,
                right_gain_applied: false,
            }
        );
    }

    /// A PGA register after a reset (muted, 0 dB; Tables 101 and 102) and with a gain the
    /// datasheet calls reserved, which the raw field still reports.
    #[test]
    fn pga_state_keeps_a_reserved_gain_as_its_raw_step_count() {
        let reset = PgaState::from_bits(0x80);
        assert!(reset.muted);
        assert_eq!(reset.gain(), Some(PgaGain::MIN));
        // 119 half decibels, the largest value rising gain has been observed for on a board.
        let beyond = PgaState::from_bits(119);
        assert_eq!(beyond.half_db, 119);
        assert_eq!(beyond.gain(), None);
    }

    /// A clock tree through the PLL decodes field by field; a reserved multiplexer code makes
    /// the whole tree undecodable, and the single decoder says which field it is.
    #[test]
    fn clock_values_decode_and_a_reserved_mux_code_is_reported() {
        let through_pll = ClockRegisters {
            clock_gen_mux: 0x03,
            pll_p_r: 0x91,
            pll_j: 0x08,
            pll_d_msb: 0x07,
            pll_d_lsb: 0x80,
            nadc: 0x88,
            madc: 0x82,
            aosr: 0x80,
            bclk_n: 0x84,
        };
        let config = through_pll.config().unwrap();
        assert_eq!(config.pll_input, PllInput::Mclk);
        assert_eq!(config.codec_input, CodecInput::Pll);
        assert!(config.pll.powered);
        assert_eq!(
            (config.pll.p.get(), config.pll.r.get(), config.pll.j.get()),
            (1, 1, 8)
        );
        assert_eq!(config.pll.d.get(), 1920);
        assert!(config.nadc.powered);
        assert_eq!(config.nadc.divisor.get(), 8);
        assert_eq!(config.madc.divisor.get(), 2);
        assert_eq!(config.aosr.get(), 128);
        assert_eq!(config.bclk_n.divisor.get(), 4);

        let reserved = ClockRegisters {
            clock_gen_mux: 0x02,
            ..through_pll
        };
        assert_eq!(reserved.config(), None);
        assert_eq!(reserved.codec_input(), None);
        assert_eq!(reserved.pll_input(), Some(PllInput::Mclk));
    }

    /// The page 0 digital values of a capturing chip (81 = `0xC2`, 82 to 84 = `0x00`), and the
    /// reset values of the same registers (Tables 67 to 70), with one overflow flag set.
    #[test]
    fn digital_values_decode_per_channel() {
        let capturing = DigitalRegisters {
            adc_digital: 0xC2,
            adc_fine_volume: 0x00,
            left_adc_volume: 0x00,
            right_adc_volume: 0x00,
            overflow_flags: 0x04,
        };
        assert_eq!(capturing.soft_stepping(), Some(SoftStepping::Disabled));
        let on = ChannelState {
            power_up: true,
            muted: false,
            fine_gain: Some(FineGain::NONE),
            volume: Some(DigitalVolume::ZERO_DB),
        };
        assert_eq!(capturing.channel(Channel::Left), on);
        assert_eq!(capturing.channel(Channel::Right), on);
        assert_eq!(
            capturing.overflow(),
            OverflowFlags {
                left_adc: false,
                right_adc: true,
                barrel_shifter: false,
            }
        );

        // Left trim -0.4 dB (D6 to D4 = 100), right channel muted with the reserved trim code 5.
        let mixed = DigitalRegisters {
            adc_digital: 0x80,
            adc_fine_volume: 0x4D,
            left_adc_volume: 0x68,
            right_adc_volume: 0x28,
            overflow_flags: 0x00,
        };
        let left = mixed.channel(Channel::Left);
        assert!(left.power_up && !left.muted);
        assert_eq!(left.fine_gain, Some(FineGain::MAX));
        assert_eq!(left.volume, Some(DigitalVolume::MIN));
        let right = mixed.channel(Channel::Right);
        assert!(!right.power_up && right.muted);
        assert_eq!(right.fine_gain, None);
        assert_eq!(right.volume, Some(DigitalVolume::MAX));
    }

    /// The reset bytes are the typed reset clock tree of the `clock` module.
    #[test]
    fn the_reset_clock_registers_decode_to_the_reset_clock_tree() {
        assert_eq!(ClockRegisters::RESET.config(), Some(ClockConfig::RESET));
        assert!(ClockRegisters::RESET.at_reset());
    }

    /// One changed byte in any of the nine registers is "not at reset", and so are programmed
    /// clock trees (one through the PLL, one from BCLK).
    #[test]
    fn at_reset_needs_every_clock_register_at_its_reset_value() {
        let from = |b: [u8; 9]| ClockRegisters {
            clock_gen_mux: b[0],
            pll_p_r: b[1],
            pll_j: b[2],
            pll_d_msb: b[3],
            pll_d_lsb: b[4],
            nadc: b[5],
            madc: b[6],
            aosr: b[7],
            bclk_n: b[8],
        };
        let reset = ClockRegisters::RESET.bytes();
        assert_eq!(from(reset), ClockRegisters::RESET);
        for i in 0..reset.len() {
            let mut changed = reset;
            changed[i] ^= 0x80;
            assert!(!from(changed).at_reset(), "register {i} of CLOCK_REGISTERS");
        }
        let through_pll = from([0x03, 0xD1, 0x30, 0x00, 0x00, 0x8F, 0x83, 0x80, 0x81]);
        let from_bclk = from([0x01, 0x51, 0x04, 0x00, 0x00, 0x81, 0x83, 0x80, 0x01]);
        assert!(!through_pll.at_reset());
        assert!(!from_bclk.at_reset());
    }
}
