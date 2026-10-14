#![doc = include_str!("../README.md")]
//!
//! ## What the driver covers
//!
//! - the paged register access layer, the typed error and the register map ([`register`]);
//! - reset, and the read path that is safe next to another owner of the chip (below);
//! - a read-only status API on top of that read path: the registers of the selected page as
//!   bytes, with typed decoders ([`status`]);
//! - the clock tree ([`clock`]) and the serial interface with its slot offset and DOUT control
//!   ([`interface`]);
//! - the processing block and the coefficients of its first biquad ([`filter`]);
//! - microphone bias, input routing and PGA gain ([`analog`]);
//! - channel power, mute, digital volume and fine trim ([`digital`]), with
//!   [`Tlv320adc3101::power_up`] and [`Tlv320adc3101::power_down`] for the start and the stop of
//!   a capture;
//! - a no-write identity and state check ([`Tlv320adc3101::probe`], [`probe`]).
//!
//! Gains are integers in the chip's step size (0.5 dB for the PGA and the digital volume, 0.1 dB
//! for the fine trim); the driver uses no floating point and does not allocate.
//!
//! ## Paged register model
//!
//! The chip has more registers than an 8-bit address reaches, so it groups them into pages of 128
//! (section 10.6.1, p. 43). Address 0 of *every* page is the same page-select register: writing
//! `N` there makes the following reads and writes address page `N`. Pages 0 (clocks, interface,
//! flags, digital gain), 1 (bias, routing, analog gain), 4 and 5 (filter coefficients) and 32 to
//! 47 (miniDSP instructions) exist; the rest is reserved (Table 15, p. 46).
//!
//! A register is therefore a pair, [`Register`]`{ page, addr }`, and the map in [`register`] is a
//! list of such constants with their bit fields. The driver keeps the page it selected last and
//! writes the page-select register only when the next access needs another page. That cache is
//! *unknown* after [`Tlv320adc3101::new`], after [`Tlv320adc3101::software_reset`] (and
//! [`Tlv320adc3101::reset`], which calls it) and after [`Tlv320adc3101::forget_page`], so the
//! first access after any of them selects its page explicitly. A failed page-select write also
//! leaves the cache unknown.
//!
//! The cached path is for a chip this driver owns. It must not be used next to another owner of
//! the chip, because both sides would cache a page the other one changes.
//!
//! ## Reading next to another owner
//!
//! A chip may belong to another driver, typically one that remembers the page it selected,
//! compares against that memory before each write and never reads the chip back. If anything
//! else wrote the page-select register, that owner's next register write would land in the wrong
//! page without anyone noticing.
//!
//! So beside such an owner the only methods to use are the three that never write register 0 or
//! any other register:
//!
//! - [`Tlv320adc3101::page`] reads the page-select register;
//! - [`Tlv320adc3101::read_selected_page`] reads one address in whichever page is selected and
//!   reports that page with the value;
//! - [`Tlv320adc3101::read_passive`] does the same for a [`Register`] and fails with
//!   [`Error::PageMismatch`] when another page is selected, instead of selecting its own.
//!
//! Each of them sends the register address (that is how an I2C register read works, Figure 47,
//! p. 43) but no register data. `read_selected_page` fetches the page and the value in one bus
//! transaction, so on a HAL that maps one `embedded-hal` transaction to one bus transfer the other
//! owner cannot change the page between the two.
//!
//! The status API ([`status`]) is built from these three and from nothing else, so its methods
//! are safe there as well: [`Tlv320adc3101::status`], [`Tlv320adc3101::analog_registers`],
//! [`Tlv320adc3101::clock_registers`] and [`Tlv320adc3101::digital_registers`]. They return the
//! registers of the selected page with typed decoders, and [`Error::PageMismatch`] for registers
//! of a page that is not selected.
//!
//! The other owner leaves the chip on whichever page it wrote last, so a passive read of a page 0
//! register may well report `PageMismatch`; take what the selected page offers rather than
//! forcing the page.
//!
//! ## Clear-on-read registers
//!
//! Four registers in page 0 hold sticky flags that a read clears: 34, 36, 42 and 45
//! ([`register::CLEAR_ON_READ`] has the list and the datasheet notes). Every read method of this
//! driver refuses them with [`Error::ClearOnRead`], so nothing reads one implicitly; registers 43
//! and 47 hold the same flags without the side effect.
#![cfg_attr(not(test), no_std)]

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{I2c, Operation};

pub mod analog;
pub mod clock;
pub mod digital;
pub mod filter;
pub mod interface;
pub mod probe;
pub mod register;
pub mod status;
#[cfg(test)]
mod test_support;

pub use analog::{InputLevel, LeftInputs, MicBias, PgaGain, RightInputs};
pub use clock::{
    Aosr, ClockConfig, CodecInput, Divider, DividerConfig, Divisor, PLL_STARTUP_MS, Pll, PllD,
    PllInput, PllJ, PllP, PllR,
};
pub use digital::{DigitalVolume, FineGain, SoftStepping};
pub use filter::{Biquad, Channel, ProcessingBlock};
pub use interface::{
    BclkDividerInput, ChannelEnable, ClockDirection, Format, InterfaceConfig, WordLength,
};
pub use probe::{Page0, Probe};
pub use register::Register;
pub use status::{
    AnalogRegisters, ChannelState, ClockRegisters, DigitalRegisters, OverflowFlags, PgaFlags,
    PgaState, Status,
};

use register::PAGE_SELECT;

/// How long [`Tlv320adc3101::reset`] waits after the software reset, in milliseconds.
///
/// The datasheet gives no settle time for the control registers. Its only figure is for the
/// coefficient memory in pages 4 and 5, whose defaults "only become valid 100 μs following a
/// hardware or software reset" (section 10.6.4, p. 68). 20 ms is used here: a conservative
/// default with a wide margin over the datasheet's 100 microseconds, and a reset happens once per
/// configuration.
pub const RESET_SETTLE_MS: u32 = 20;

/// The mute bits of both channels in page 0, register 82 (Table 68, p. 58), which
/// [`Tlv320adc3101::power_up`] and [`Tlv320adc3101::power_down`] change together in one write.
const DIGITAL_MUTES: u8 =
    register::adc_fine_volume::LEFT_MUTE | register::adc_fine_volume::RIGHT_MUTE;

/// The lowest 7-bit I2C address the chip answers to. Its address is `0011 0` followed by the
/// levels of its two address pins (Table 13, p. 42), so a chip is at one of `0x18` to
/// [`ADDRESS_MAX`]; which one is the board's strapping.
pub const ADDRESS_MIN: u8 = 0x18;

/// The highest 7-bit I2C address the chip answers to: `0011 0` with both address pins high
/// (Table 13, p. 42).
pub const ADDRESS_MAX: u8 = 0x1b;

/// Errors from the driver. `Clone` and `Copy` follow the bus error's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// The I2C transaction failed.
    Bus(E),
    /// The read was refused because reading this register clears its sticky flags
    /// ([`register::CLEAR_ON_READ`]). Nothing was sent to the chip.
    ClearOnRead(Register),
    /// A passive read found another page selected than the register lives in. Nothing was written
    /// and the value read from the other page was discarded.
    PageMismatch {
        /// The page the register lives in.
        wanted: u8,
        /// The page the chip had selected.
        selected: u8,
    },
}

/// Names the failure. A bus error is shown with its `Debug` form, the one representation every
/// `embedded-hal` bus error has; registers by page and address, as the datasheet names them.
impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Bus(error) => write!(f, "TLV320ADC3101 I2C error: {error:?}"),
            Self::ClearOnRead(r) => write!(
                f,
                "TLV320ADC3101 read of clear-on-read register page {} / {} refused",
                r.page(),
                r.addr()
            ),
            Self::PageMismatch { wanted, selected } => write!(
                f,
                "TLV320ADC3101 page {selected} selected, register lives in page {wanted}"
            ),
        }
    }
}

impl<E: core::fmt::Debug> core::error::Error for Error<E> {}

/// A TLV320ADC3101 on an `embedded-hal` I2C bus.
pub struct Tlv320adc3101<I2C> {
    i2c: I2C,
    address: u8,
    /// The page this driver selected last, or `None` while that is unknown (see the crate docs).
    page: Option<u8>,
}

impl<I2C: I2c> Tlv320adc3101<I2C> {
    /// Wraps the bus; talks to nothing until a method is called.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            page: None,
        }
    }

    /// Gives the bus back.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// The configured 7-bit address.
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Reads the page-select register: the page the chip has selected right now.
    ///
    /// Writes no register, so it is safe next to another owner of the chip. It does not touch this
    /// driver's page cache either: next to another owner the page is that owner's to change.
    pub fn page(&mut self) -> Result<u8, Error<I2C::Error>> {
        let mut page = [0u8];
        self.i2c
            .write_read(self.address, &[PAGE_SELECT], &mut page)
            .map_err(Error::Bus)?;
        Ok(page[0])
    }

    /// Reads the register at `addr` in whichever page is selected, and returns `(page, value)`.
    ///
    /// Writes no register, so it is safe next to another owner of the chip. The page and the value are
    /// fetched in one bus transaction so that they belong together.
    ///
    /// The address of a clear-on-read register is refused in every page: the selected page is only
    /// known after the read, and by then a sticky flag would be gone.
    pub fn read_selected_page(&mut self, addr: u8) -> Result<(u8, u8), Error<I2C::Error>> {
        if register::is_clear_on_read_in_any_page(addr) {
            return Err(Error::ClearOnRead(Register::new(0, addr)));
        }
        let mut page = [0u8];
        let mut value = [0u8];
        self.i2c
            .transaction(
                self.address,
                &mut [
                    Operation::Write(&[PAGE_SELECT]),
                    Operation::Read(&mut page),
                    Operation::Write(&[addr]),
                    Operation::Read(&mut value),
                ],
            )
            .map_err(Error::Bus)?;
        Ok((page[0], value[0]))
    }

    /// Reads `reg` if its page is the selected one, and fails with [`Error::PageMismatch`] if it
    /// is not.
    ///
    /// Writes no register, so it is safe next to another owner of the chip: it never selects a page.
    pub fn read_passive(&mut self, reg: Register) -> Result<u8, Error<I2C::Error>> {
        if reg.is_clear_on_read() {
            return Err(Error::ClearOnRead(reg));
        }
        let (selected, value) = self.read_selected_page(reg.addr())?;
        if selected != reg.page() {
            return Err(Error::PageMismatch {
                wanted: reg.page(),
                selected,
            });
        }
        Ok(value)
    }

    /// Resets all control registers to their defaults (Table 16, p. 46), waits
    /// [`RESET_SETTLE_MS`] and forgets the cached page.
    ///
    /// This is the first step of the datasheet's design procedure (section 11.2.2.2, step 1,
    /// p. 80). After it everything is powered down, the PGAs and the digital volume are muted and
    /// no input is connected.
    ///
    /// Never call this beside another owner of the chip: it writes the page-select register and
    /// wipes that owner's configuration.
    pub fn reset<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I2C::Error>> {
        self.software_reset()?;
        delay.delay_ms(RESET_SETTLE_MS);
        Ok(())
    }

    /// Starts conversion on both channels of a configured chip: unmutes the two PGAs, powers the
    /// two ADC channels and unmutes their digital outputs.
    ///
    /// The order is the last three steps of the datasheet's design procedure: unmute the analog
    /// PGAs, power up the ADC channels, unmute the digital volume (section 11.2.2.2, steps 3(e),
    /// 4(b) and 4(c), p. 80). There is nothing to wait for between the steps: the datasheet has
    /// no delay here.
    ///
    /// Clocks, interface, routing and gains have to be programmed before. The microphone bias
    /// ([`set_mic_bias`](Self::set_mic_bias)) is not part of this method: the datasheet programs
    /// it before the PGAs, but it is independent of these steps, so its place is the caller's
    /// choice.
    ///
    /// Each step reads a register and writes it back with one or two bits changed, so gains, fine
    /// trims and the soft-stepping mode stay as they were configured.
    pub fn power_up(&mut self) -> Result<(), Error<I2C::Error>> {
        self.set_pga_mute(Channel::Left, false)?;
        self.set_pga_mute(Channel::Right, false)?;
        self.set_channel_power(true, true)?;
        self.modify(register::ADC_FINE_VOLUME, DIGITAL_MUTES, 0)
    }

    /// Stops conversion on both channels and leaves the configuration in place: powers the two
    /// ADC channels down, mutes their digital outputs and mutes the two PGAs. It is
    /// [`power_up`](Self::power_up) undone in reverse order.
    ///
    /// The channels go first because the chip ramps the PGA and the digital volume down by itself
    /// when a channel is powered down (pp. 25 to 26). There is nothing to wait for. The digital
    /// mute is set so that the chip is back in the state `power_up` started from.
    ///
    /// The microphone bias, the clocks and the DOUT pin are left alone.
    pub fn power_down(&mut self) -> Result<(), Error<I2C::Error>> {
        self.set_channel_power(false, false)?;
        self.modify(register::ADC_FINE_VOLUME, DIGITAL_MUTES, DIGITAL_MUTES)?;
        self.set_pga_mute(Channel::Right, true)?;
        self.set_pga_mute(Channel::Left, true)
    }

    /// Resets all control registers to their defaults (Table 16, p. 46) and forgets the cached
    /// page, without waiting. [`reset`](Self::reset) is this plus the wait.
    ///
    /// The reset also puts the page-select register back to 0, but the cache is cleared rather
    /// than set to 0: the next access then states its page again, which costs one short write and
    /// does not depend on the reset having finished. The caller has to wait before the next
    /// access: see [`RESET_SETTLE_MS`] for what is known about how long.
    ///
    /// Never call this beside another owner of the chip: it writes the page-select register and
    /// wipes that owner's configuration.
    pub fn software_reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write(register::SOFTWARE_RESET, register::software_reset::RESET)?;
        self.page = None;
        Ok(())
    }

    /// Forgets the cached page, so the next cached access selects its page explicitly.
    ///
    /// Call it whenever the chip may have changed page behind this driver's back: after a hardware
    /// reset through the reset pin, or after another owner of the chip was active.
    pub fn forget_page(&mut self) {
        self.page = None;
    }

    /// Writes the page-select register, whatever page this driver selected last.
    ///
    /// This is for handing a chip back to another owner that keeps its own idea of the selected
    /// page and cannot be told otherwise. A caller that borrows such a chip notes
    /// [`page`](Self::page) before its first write and calls this as its last one. `page` is
    /// written as given, so that whatever was read can be put back; valid pages are 0, 1, 4, 5
    /// and 32 to 47 (Table 15 note, p. 46).
    pub fn restore_page(&mut self, page: u8) -> Result<(), Error<I2C::Error>> {
        self.page = None;
        self.select_page(page)
    }

    /// Makes `page` the selected one, writing the page-select register only if the cache says
    /// another page (or nothing) is selected.
    fn select_page(&mut self, page: u8) -> Result<(), Error<I2C::Error>> {
        if self.page == Some(page) {
            return Ok(());
        }
        // If the write fails, the chip may or may not have taken the new page, so the cache must
        // not keep claiming the old one.
        self.page = None;
        self.i2c
            .write(self.address, &[PAGE_SELECT, page])
            .map_err(Error::Bus)?;
        self.page = Some(page);
        Ok(())
    }

    /// Writes `value` to `reg`, selecting its page first if needed.
    fn write(&mut self, reg: Register, value: u8) -> Result<(), Error<I2C::Error>> {
        self.select_page(reg.page())?;
        self.i2c
            .write(self.address, &[reg.addr(), value])
            .map_err(Error::Bus)?;
        if reg.addr() == PAGE_SELECT {
            // Address 0 is the page select in every page, so this write changed the page.
            self.page = Some(value);
        }
        Ok(())
    }

    /// Reads `reg`, selecting its page first if needed. Refuses clear-on-read registers.
    fn read(&mut self, reg: Register) -> Result<u8, Error<I2C::Error>> {
        if reg.is_clear_on_read() {
            return Err(Error::ClearOnRead(reg));
        }
        self.select_page(reg.page())?;
        let mut value = [0u8];
        self.i2c
            .write_read(self.address, &[reg.addr()], &mut value)
            .map_err(Error::Bus)?;
        Ok(value[0])
    }

    /// Replaces the bits of `reg` selected by `mask` with the same bits of `value` and leaves the
    /// others as they are. Always writes, so the bus traffic does not depend on the old value.
    fn modify(&mut self, reg: Register, mask: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        let old = self.read(reg)?;
        self.write(reg, (old & !mask) | (value & mask))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    /// Pins the page-select write sequence: the first write into a page selects it, the next write
    /// in the same page does not, and a software reset forgets the page.
    #[test]
    fn page_select_is_written_only_when_the_page_changes() {
        const ADDRESS: u8 = 0x18;
        let expectations = [
            // Nothing is known after new(): select page 1, then write the register.
            Transaction::write(ADDRESS, vec![0x00, 0x01]),
            Transaction::write(ADDRESS, vec![59, 40]),
            // Same page: the register write alone.
            Transaction::write(ADDRESS, vec![60, 40]),
            // The reset register is in page 0: select it, then reset.
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
            Transaction::write(ADDRESS, vec![1, 0x01]),
            // The reset forgot the page, so page 0 is selected again although it was selected.
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
            Transaction::write(ADDRESS, vec![4, 0x03]),
            // Same page: the register write alone.
            Transaction::write(ADDRESS, vec![18, 0x8f]),
        ];
        let mut adc = Tlv320adc3101::new(Mock::new(&expectations), ADDRESS);
        adc.write(register::LEFT_PGA, 40).unwrap();
        adc.write(register::RIGHT_PGA, 40).unwrap();
        adc.software_reset().unwrap();
        adc.write(register::CLOCK_GEN_MUX, 0x03).unwrap();
        adc.write(register::NADC, 0x8f).unwrap();
        adc.release().done();
    }

    /// Pins the order of a capture stop: channels off, digital mutes on, then the PGAs muted, and
    /// that gains, trims and the soft-stepping mode pass through unchanged.
    #[test]
    fn power_down_stops_the_channels_before_it_mutes() {
        const ADDRESS: u8 = 0x18;
        let expectations = [
            Transaction::write(ADDRESS, vec![0x00, 0x00]),
            // Both channels on with soft-stepping disabled: only D7 and D6 go.
            Transaction::write_read(ADDRESS, vec![81], vec![0xC2]),
            Transaction::write(ADDRESS, vec![81, 0x02]),
            // Both digital mutes in one write; the trims (here -0.1 dB on the right) stay.
            Transaction::write_read(ADDRESS, vec![82], vec![0x01]),
            Transaction::write(ADDRESS, vec![82, 0x89]),
            // Page 1: the PGAs keep their 20 dB and get the mute bit, right then left.
            Transaction::write(ADDRESS, vec![0x00, 0x01]),
            Transaction::write_read(ADDRESS, vec![60], vec![0x28]),
            Transaction::write(ADDRESS, vec![60, 0xA8]),
            Transaction::write_read(ADDRESS, vec![59], vec![0x28]),
            Transaction::write(ADDRESS, vec![59, 0xA8]),
        ];
        let mut adc = Tlv320adc3101::new(Mock::new(&expectations), ADDRESS);
        adc.power_down().unwrap();
        adc.release().done();
    }

    /// Pins that restoring a page writes the page-select register even when the cache already
    /// holds that page: the other owner's idea of the page is what counts, not this driver's.
    #[test]
    fn restore_page_always_writes_the_page_select() {
        const ADDRESS: u8 = 0x18;
        let expectations = [
            Transaction::write(ADDRESS, vec![0x00, 0x01]),
            Transaction::write(ADDRESS, vec![51, 0x50]),
            // Page 1 is cached, and is written all the same.
            Transaction::write(ADDRESS, vec![0x00, 0x01]),
        ];
        let mut adc = Tlv320adc3101::new(Mock::new(&expectations), ADDRESS);
        adc.write(register::MICBIAS_CONTROL, 0x50).unwrap();
        adc.restore_page(1).unwrap();
        adc.release().done();
    }

    /// Pins every write path of a capture start on one chip, in the order a caller keeps: reset,
    /// DOUT off, clocks (with the PLL wait), processing block, biquad A on both channels,
    /// interface, soft-stepping, routing, PGA gain, power-up, then the NADC divider powered last,
    /// DOUT on and the microphone bias. The chip is its own clock master: 12 MHz MCLK through the
    /// PLL (P 1, R 1, J 8, D 1920: 98.304 MHz), NADC 8, MADC 2, AOSR 128 for 48 kHz, BCLK = ADC_CLK
    /// / 4, I2S with 16-bit words; the differential pairs IN2/IN3 at 0 dB, 10 dB of PGA gain and a
    /// 2.0 V bias.
    #[test]
    fn a_capture_start_writes_each_step_in_order() {
        use crate::test_support::{BusDelay, wait_ms};
        const ADDRESS: u8 = 0x1b;
        const fn divider(powered: bool, divisor: u8) -> DividerConfig {
            DividerConfig {
                powered,
                divisor: Divisor::new(divisor).expect("a divisor is 1 to 128"),
            }
        }
        const CLOCKS: ClockConfig = ClockConfig {
            pll_input: PllInput::Mclk,
            codec_input: CodecInput::Pll,
            pll: Pll {
                powered: true,
                p: PllP::new(1).expect("P is 1 to 8"),
                r: PllR::new(1).expect("R is 1 to 16"),
                j: PllJ::new(8).expect("J is 1 to 63"),
                d: PllD::new(1920).expect("D is 0 to 9999"),
            },
            // Programmed without its power bit; powered once everything else is set up.
            nadc: divider(false, 8),
            madc: divider(true, 2),
            aosr: Aosr::RESET,
            bclk_n: divider(true, 4),
        };
        const INTERFACE: InterfaceConfig = InterfaceConfig {
            bclk: ClockDirection::Output,
            wclk: ClockDirection::Output,
            bclk_divider_input: BclkDividerInput::AdcClk,
            ..InterfaceConfig::RESET
        };
        const FILTER: Biquad = Biquad {
            n0: 0x4000,
            n1: 0x0001,
            n2: -2,
            d1: 0x1234,
            d2: -0x1234,
        };
        let write = |register: u8, value: u8| Transaction::write(ADDRESS, vec![register, value]);
        let read =
            |register: u8, value: u8| Transaction::write_read(ADDRESS, vec![register], vec![value]);
        let mut expected = vec![
            // Nothing is known about the page after new(): select page 0, reset, wait 20 ms.
            write(0, 0),
            write(1, 0x01),
            wait_ms(20),
            // The reset forgot the page. DOUT off, bus keeper off.
            write(0, 0),
            write(53, 0x10),
            // Clocks: mux, J, D, then P and R with the power bit, the PLL wait, the dividers.
            write(4, 0x03),
            write(6, 0x08),
            write(7, 0x07),
            write(8, 0x80),
            write(5, 0x91),
            wait_ms(15),
            write(18, 0x08),
            write(19, 0x82),
            write(20, 0x80),
            write(30, 0x84),
            // PRB_R1.
            write(61, 0x01),
            // Page 4: biquad A, left then right, N0 N1 N2 D1 D2 = 4000 0001 FFFE 1234 EDCC.
            write(0, 4),
        ];
        for first_register in [14, 78] {
            let bytes = [0x40, 0x00, 0x00, 0x01, 0xFF, 0xFE, 0x12, 0x34, 0xED, 0xCC];
            expected.extend((first_register..).zip(bytes).map(|(r, b)| write(r, b)));
        }
        expected.extend([
            write(0, 0),
            // I2S, 16 bits, BCLK and WCLK outputs; no offset; BCLK divider from ADC_CLK; both
            // channels, DOUT not 3-stated, the early 3-state bit at its reset value.
            write(27, 0x0C),
            write(28, 0x00),
            write(29, 0x02),
            write(38, 0x02),
            // Soft-stepping off. The values read back are the reset values (Tables 67, 68, 101
            // and 102).
            read(81, 0x00),
            write(81, 0x02),
            // Page 1: the differential pairs at 0 dB on both PGAs, nothing else.
            write(0, 1),
            write(52, 0x3F),
            write(54, 0x3F),
            write(55, 0x3F),
            write(57, 0x3F),
            // PGA gain 10 dB, still muted.
            read(59, 0x80),
            write(59, 0x94),
            read(60, 0x80),
            write(60, 0x94),
            // power_up: PGAs unmuted, then in page 0 both channels on and both digital mutes off.
            read(59, 0x94),
            write(59, 0x14),
            read(60, 0x94),
            write(60, 0x14),
            write(0, 0),
            read(81, 0x02),
            write(81, 0xC2),
            read(82, 0x88),
            write(82, 0x00),
            // NADC powered: read it, write it back with the power bit.
            read(18, 0x08),
            write(18, 0x88),
            // DOUT on: primary data output, bus keeper off.
            write(53, 0x12),
            // Page 1: both microphone bias outputs at 2.0 V.
            write(0, 1),
            write(51, 0x28),
        ]);

        let mut bus = Mock::new(&expected);
        let mut delay = BusDelay(bus.clone());
        let mut adc = Tlv320adc3101::new(&mut bus, ADDRESS);
        adc.reset(&mut delay).unwrap();
        adc.set_data_output(false).unwrap();
        adc.configure_clocks(&CLOCKS, &mut delay).unwrap();
        adc.set_processing_block(ProcessingBlock::PRB_R1).unwrap();
        adc.set_biquad_a(Channel::Left, &FILTER).unwrap();
        adc.set_biquad_a(Channel::Right, &FILTER).unwrap();
        adc.configure_interface(&INTERFACE).unwrap();
        adc.set_soft_stepping(SoftStepping::Disabled).unwrap();
        let left = LeftInputs {
            diff_in2l_in3l: InputLevel::ZeroDb,
            ..LeftInputs::NONE
        };
        let right = RightInputs {
            diff_in2r_in3r: InputLevel::ZeroDb,
            ..RightInputs::NONE
        };
        adc.set_input_routing(&left, &right).unwrap();
        let gain = PgaGain::from_half_db(20).unwrap();
        adc.set_pga_gain(Channel::Left, gain).unwrap();
        adc.set_pga_gain(Channel::Right, gain).unwrap();
        adc.power_up().unwrap();
        adc.set_divider_power(Divider::Nadc, true).unwrap();
        adc.set_data_output(true).unwrap();
        adc.set_mic_bias(MicBias::V2, MicBias::V2).unwrap();
        bus.done();
    }
}

/// Compiles the README's usage example as a doctest, so the example cannot drift from the API.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
struct ReadmeDoctests;
