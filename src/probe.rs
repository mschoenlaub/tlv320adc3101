//! A no-write check of whatever answers at a chip's address ([`Tlv320adc3101::probe`]): which
//! page it has selected, whether register 20 holds the value a TLV320ADC3101 resets it to, and
//! whether its clock registers are still at their reset values.
//!
//! It writes no register, the page-select register least of all, so it can run beside another
//! owner of the chip and reports the page it finds instead of choosing one. Every byte it puts on
//! the bus after the chip's address is a register number: the page read is a one-byte
//! register-pointer write and a one-byte read joined by a repeated start, every other read is
//! [`Tlv320adc3101::read_passive`] (page-register pointer, read, register pointer, read, in one
//! transaction). None of the four clear-on-read registers (page 0: 34, 36, 42, 45) is read; the
//! driver refuses them, and the addresses read here (0, 4 to 8, 18 to 20, 30) are none of them.

use embedded_hal::i2c::I2c;

use crate::{Error, Tlv320adc3101, register};

/// What the page 0 reads of [`Tlv320adc3101::probe`] found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Page0 {
    /// Register 20 (AOSR) did not read [`register::AOSR_RESET`]: not the chip, or one set up for
    /// another oversampling ratio, and nothing more was read. Holds the value read.
    Unidentified {
        /// What register 20 read.
        aosr: u8,
    },
    /// Register 20 read [`register::AOSR_RESET`], and the clock registers were read. The chip is
    /// plausible, not proven: AOSR is writable, and `0x80` is not an ID.
    Identified {
        /// Whether every clock register held its reset value
        /// ([`ClockRegisters::at_reset`](crate::ClockRegisters::at_reset)): nothing has set the
        /// chip up since its last reset.
        clocks_at_reset: bool,
    },
}

/// What [`Tlv320adc3101::probe`] found on one chip.
#[derive(Debug, PartialEq, Eq)]
pub enum Probe<E> {
    /// The first read, of the page-select register, failed: a NACK (nobody answers at the
    /// address) or another bus error. Nothing else was tried.
    NoAnswer(Error<E>),
    /// The chip answered with another page than 0 selected. Nothing more was read: page 0 would
    /// need a page write.
    OtherPage(u8),
    /// The chip answered with page 0 selected; the outcome of the page 0 reads, or the error that
    /// ended them.
    Page0(Result<Page0, Error<E>>),
}

impl<I2C: I2c> Tlv320adc3101<I2C> {
    /// Reads, without writing a register: the page-select register; then, only if page 0 is
    /// selected, register 20 (AOSR, the identity check); then, only if that read
    /// [`register::AOSR_RESET`], the nine clock registers ([`Self::clock_registers`]: 4 to 8,
    /// 18 to 20, 30) to see whether they are at their reset values.
    ///
    /// Bus transactions: one write-read of the page register, then per register one combined
    /// transaction (page-register pointer, read, register pointer, read); 11 in all on a page 0
    /// chip that identifies, 2 on one that does not, 1 on another page or when the first read
    /// fails. The first failure ends the probe and is part of the result: [`Probe::NoAnswer`] for
    /// the page read, [`Probe::Page0`]`(Err(..))` for a later one.
    pub fn probe(&mut self) -> Probe<I2C::Error> {
        match self.page() {
            Err(e) => Probe::NoAnswer(e),
            Ok(page) if page != register::AOSR.page() => Probe::OtherPage(page),
            Ok(_) => Probe::Page0(self.probe_page_0()),
        }
    }

    /// [`Self::probe`]'s page 0 part: AOSR, then the clock registers if AOSR identified the chip.
    fn probe_page_0(&mut self) -> Result<Page0, Error<I2C::Error>> {
        let aosr = self.read_passive(register::AOSR)?;
        if aosr != register::AOSR_RESET {
            return Ok(Page0::Unidentified { aosr });
        }
        Ok(Page0::Identified {
            clocks_at_reset: self.clock_registers()?.at_reset(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::status::{CLOCK_REGISTERS, ClockRegisters};
    use crate::{ADDRESS_MAX, ADDRESS_MIN};
    use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
    use embedded_hal_mock::eh1::i2c::{Mock, Transaction};

    /// The page register alone, as `Tlv320adc3101::page` reads it.
    fn page_read(address: u8, page: u8) -> Transaction {
        Transaction::write_read(address, vec![register::PAGE_SELECT], vec![page])
    }

    /// One `read_passive`: the page register and `addr` in one transaction, only register numbers
    /// written.
    fn passive_read(address: u8, page: u8, addr: u8, value: u8) -> Vec<Transaction> {
        vec![
            Transaction::transaction_start(address),
            Transaction::write(address, vec![register::PAGE_SELECT]),
            Transaction::read(address, vec![page]),
            Transaction::write(address, vec![addr]),
            Transaction::read(address, vec![value]),
            Transaction::transaction_end(address),
        ]
    }

    /// Page 0, AOSR, then the nine clock registers holding `clocks`.
    fn page_0_reads(address: u8, clocks: ClockRegisters) -> Vec<Transaction> {
        let mut expected = vec![page_read(address, 0)];
        expected.extend(passive_read(address, 0, 20, clocks.aosr));
        for (register, value) in CLOCK_REGISTERS.iter().zip(clocks.bytes()) {
            expected.extend(passive_read(address, 0, register.addr(), value));
        }
        expected
    }

    /// A chip at reset: 11 transactions, identified, clocks at reset.
    #[test]
    fn a_reset_chip_on_page_0_is_identified_and_at_reset() {
        let expected = page_0_reads(ADDRESS_MIN, ClockRegisters::RESET);
        assert_eq!(expected.len(), 1 + 10 * 6);
        let mut i2c = Mock::new(&expected);
        let mut adc = Tlv320adc3101::new(&mut i2c, ADDRESS_MIN);
        assert_eq!(
            adc.probe(),
            Probe::Page0(Ok(Page0::Identified {
                clocks_at_reset: true
            }))
        );
        i2c.done();
    }

    /// A chip set up to run from another chip's bit clock, with AOSR still 128: identified, but
    /// the clocks are set up.
    #[test]
    fn a_chip_that_was_set_up_is_identified_but_not_at_reset() {
        let set_up = ClockRegisters {
            clock_gen_mux: 0x01,
            pll_p_r: 0x51,
            nadc: 0x81,
            madc: 0x83,
            ..ClockRegisters::RESET
        };
        let mut i2c = Mock::new(&page_0_reads(ADDRESS_MAX, set_up));
        let mut adc = Tlv320adc3101::new(&mut i2c, ADDRESS_MAX);
        assert_eq!(
            adc.probe(),
            Probe::Page0(Ok(Page0::Identified {
                clocks_at_reset: false
            }))
        );
        i2c.done();
    }

    /// A chip whose register 20 is not 128 is not read any further.
    #[test]
    fn a_wrong_aosr_ends_the_probe() {
        let mut expected = vec![page_read(ADDRESS_MIN, 0)];
        expected.extend(passive_read(ADDRESS_MIN, 0, 20, 0x40));
        let mut i2c = Mock::new(&expected);
        let mut adc = Tlv320adc3101::new(&mut i2c, ADDRESS_MIN);
        assert_eq!(
            adc.probe(),
            Probe::Page0(Ok(Page0::Unidentified { aosr: 0x40 }))
        );
        i2c.done();
    }

    /// On page 1 only the page register is read: no page write to get to page 0.
    #[test]
    fn on_another_page_only_the_page_register_is_read() {
        let mut i2c = Mock::new(&[page_read(ADDRESS_MIN, 1)]);
        let mut adc = Tlv320adc3101::new(&mut i2c, ADDRESS_MIN);
        assert_eq!(adc.probe(), Probe::OtherPage(1));
        i2c.done();
    }

    /// A bus that answers the page read (two operations) with page 0 and fails every combined
    /// read (four operations) with a timeout-like error: `embedded-hal-mock` cannot fail an
    /// operation inside a transaction.
    struct FailsAfterThePageRead;

    impl embedded_hal::i2c::ErrorType for FailsAfterThePageRead {
        type Error = ErrorKind;
    }

    impl I2c for FailsAfterThePageRead {
        fn transaction(
            &mut self,
            _address: u8,
            operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), ErrorKind> {
            match operations {
                [_, embedded_hal::i2c::Operation::Read(page)] => {
                    page.fill(0);
                    Ok(())
                }
                _ => Err(ErrorKind::Other),
            }
        }
    }

    /// A bus that answers the page read with page 0, the AOSR read with `0x80` and then the clock
    /// reads with zeros, until the `fail_at`-th four-operation transaction (0 is the AOSR read, 1
    /// the first of the nine clock reads), which it fails.
    struct FailsOnCombinedRead {
        fail_at: usize,
        seen: usize,
    }

    impl embedded_hal::i2c::ErrorType for FailsOnCombinedRead {
        type Error = ErrorKind;
    }

    impl I2c for FailsOnCombinedRead {
        fn transaction(
            &mut self,
            _address: u8,
            operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), ErrorKind> {
            use embedded_hal::i2c::Operation::Read;
            match operations {
                [_, Read(page)] => page.fill(0),
                [_, Read(_), _, Read(value)] => {
                    if self.seen == self.fail_at {
                        return Err(ErrorKind::Other);
                    }
                    value.fill(if self.seen == 0 { 0x80 } else { 0 });
                    self.seen += 1;
                }
                _ => return Err(ErrorKind::Other),
            }
            Ok(())
        }
    }

    /// A failure partway through the nine clock reads ends the probe with the error, never a
    /// partial [`Page0::Identified`].
    #[test]
    fn a_failure_partway_through_the_clock_reads_is_a_page_0_error() {
        for fail_at in 1..=9 {
            let mut adc = Tlv320adc3101::new(FailsOnCombinedRead { fail_at, seen: 0 }, ADDRESS_MIN);
            assert_eq!(
                adc.probe(),
                Probe::Page0(Err(Error::Bus(ErrorKind::Other))),
                "failing at combined read {fail_at}"
            );
        }
    }

    /// A NACK on the page read is no answer; a failure after it is kept as the page 0 result.
    /// Either ends the probe.
    #[test]
    fn the_first_failure_ends_the_probe_and_says_where_it_happened() {
        let nack = ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address);
        let mut i2c = Mock::new(&[page_read(ADDRESS_MIN, 0).with_error(nack)]);
        let mut adc = Tlv320adc3101::new(&mut i2c, ADDRESS_MIN);
        assert_eq!(adc.probe(), Probe::NoAnswer(Error::Bus(nack)));
        i2c.done();

        let mut adc = Tlv320adc3101::new(FailsAfterThePageRead, ADDRESS_MIN);
        assert_eq!(adc.probe(), Probe::Page0(Err(Error::Bus(ErrorKind::Other))));
    }
}
