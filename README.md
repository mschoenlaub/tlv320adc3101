# tlv320adc3101

[![crates.io](https://img.shields.io/crates/v/tlv320adc3101.svg)](https://crates.io/crates/tlv320adc3101)
[![docs.rs](https://docs.rs/tlv320adc3101/badge.svg)](https://docs.rs/tlv320adc3101)
[![CI](https://github.com/mschoenlaub/tlv320adc3101/actions/workflows/ci.yml/badge.svg)](https://github.com/mschoenlaub/tlv320adc3101/actions/workflows/ci.yml)

`no_std` driver for the TI **TLV320ADC3101**, a stereo audio ADC with two microphone
preamplifiers (0 dB to 40 dB), two microphone bias outputs, a PLL, a serial audio output (I2S,
left- and right-justified, DSP, with time-division multiplexing for several chips on one data
line) and fixed digital filter chains, controlled over I2C. It depends only on `embedded-hal` 1.0,
allocates nothing and uses no floating point.

- **Typed configuration.** Values with a range are checked when they are built, so in a `const`
  they are checked at compile time. Reserved codes read back from the chip are `None`, not panics.
- **Capture in the datasheet's order.** `configure_clocks` waits for the PLL where one is used,
  `power_up` and `power_down` are the design procedure's last steps, in order.
- **A read path that writes nothing.** The status API never writes a register, not even the page
  select, so it can read a chip another driver owns and configures.
- **Every register and bit cited** to TI's datasheet.

## Usage

The example is the datasheet's own (section 11.2.2.3): a stereo recording at 44.1 kHz from an
11.2896 MHz master clock, with the host driving the bit clock and the word clock, from the pins
IN1L(P) and IN1R(M). With that clock no PLL is needed: 11.2896 MHz / (NADC 1 x MADC 2 x AOSR
128) = 44.1 kHz.

```rust
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use tlv320adc3101::{
    Channel, ClockConfig, DividerConfig, Divisor, Error, InputLevel, InterfaceConfig, LeftInputs,
    PgaGain, ProcessingBlock, RightInputs, SoftStepping, Tlv320adc3101,
};

// Values with a range are checked when they are built. Building them in constants lets the
// compiler do that check, so nothing can fail at run time.
const MADC: Divisor = Divisor::new(2).expect("a divisor is 1 to 128");

// The codec clock straight from the MCLK pin, the PLL off, AOSR 128: the reset values, with the
// two dividers set and powered.
const CLOCKS: ClockConfig = ClockConfig {
    nadc: DividerConfig { powered: true, divisor: Divisor::RESET },
    madc: DividerConfig { powered: true, divisor: MADC },
    ..ClockConfig::RESET
};

fn record<I2C: I2c, D: DelayNs>(i2c: I2C, delay: &mut D) -> Result<I2C, Error<I2C::Error>> {
    // 0x18 is the address with both address pins low.
    let mut adc = Tlv320adc3101::new(i2c, 0x18);

    // Start from a known state. After the reset everything is powered down, the PGAs and the
    // digital volume are muted and no input is connected.
    adc.reset(delay)?;

    // Clocks first: the dividers are written and powered in the order the datasheet gives.
    adc.configure_clocks(&CLOCKS, delay)?;
    // I2S, 16-bit words, BCLK and WCLK as inputs: the reset values, written out.
    adc.configure_interface(&InterfaceConfig::RESET)?;
    adc.set_processing_block(ProcessingBlock::PRB_R1)?;

    // One single-ended pin per channel at 0 dB, PGA gain 0 dB, gain changes without ramping.
    adc.set_input_routing(
        &LeftInputs { in1l: InputLevel::ZeroDb, ..LeftInputs::NONE },
        &RightInputs { in1r: InputLevel::ZeroDb, ..RightInputs::NONE },
    )?;
    adc.set_pga_gain(Channel::Left, PgaGain::MIN)?;
    adc.set_pga_gain(Channel::Right, PgaGain::MIN)?;
    adc.set_soft_stepping(SoftStepping::Disabled)?;

    // Unmute the PGAs, power both ADC channels, unmute their digital outputs. From here on the
    // chip sends samples on the serial interface.
    adc.power_up()?;

    // Stop: power the channels down, mute. The configuration stays, so power_up can follow again.
    adc.power_down()?;

    Ok(adc.release())
}
```

Things to know:

- **`power_up` only does its own part.** The clocks, the interface, the routing and the gains have
  to be set before it, and the master clock must run. The microphone bias (`set_mic_bias`) is not
  part of it: the datasheet programs it before the PGAs, but it is independent of them, so its
  place is yours.
- **Three gains.** `set_pga_gain` is the analog input gain (0 dB to 40 dB in 0.5 dB steps),
  `set_digital_volume` the digital one (-12 dB to +20 dB in 0.5 dB steps) and `set_fine_gain` a
  trim (0 dB to -0.4 dB in 0.1 dB steps). Each counts in the chip's own steps, so nothing needs a
  float.
- **Values with a range are newtypes** whose constructors return `None` outside it (`Divisor`,
  `Aosr`, `PllP`, `PllR`, `PllJ`, `PllD`, `PgaGain`, `DigitalVolume`, `FineGain`,
  `ProcessingBlock`). What the types cannot know is the clock frequency you feed the chip, so the
  frequency limits of the PLL and the dividers stay with you; the `clock` module's documentation
  lists them.
- **Several chips on one data line**: `InterfaceConfig` has the data offset and the channel
  enable bits for time-division multiplexing, and `set_data_output` controls the DOUT pin.
- **The page is cached.** The driver remembers the page it selected last and writes the page
  select only when an access needs another one. That memory is only valid while nothing else
  talks to the chip; `forget_page` clears it.

## Addressing

The 7-bit address is `0011 0` followed by the levels of the two address pins (datasheet Table
13): `0x18` with both low, up to `0x1b` with both high (`ADDRESS_MIN` to `ADDRESS_MAX`). Up to
four chips share one bus.

## Reading the chip's state

The status API reads a chip without writing a register, so it also works on a chip that somebody
else owns and configures. It never selects a page. It reads what the selected page offers and says
which page that was:

```rust
use embedded_hal::i2c::I2c;
use tlv320adc3101::{Channel, Error, MicBias, Status, Tlv320adc3101};

/// Whether the left microphone path is live, as far as the selected page can tell.
fn left_path_is_live<I2C: I2c>(
    adc: &mut Tlv320adc3101<I2C>,
) -> Result<Option<bool>, Error<I2C::Error>> {
    Ok(match adc.status()? {
        // Page 1: microphone bias, input routing, PGA mute and gain, "gain applied" flags.
        Status::Page1(analog) => {
            let (micbias1, _micbias2) = analog.mic_bias();
            Some(micbias1 != MicBias::Off && !analog.pga(Channel::Left).muted)
        }
        // Page 0: the clock tree, channel power, digital mute and volume, overflow flags.
        Status::Page0 { clocks: _, digital } => {
            let left = digital.channel(Channel::Left);
            Some(left.power_up && !left.muted)
        }
        // A coefficient or miniDSP page: nothing was read beyond the page register.
        Status::OtherPage(_page) => None,
    })
}
```

`analog_registers()`, `clock_registers()` and `digital_registers()` read one group each and answer
`Error::PageMismatch { wanted, selected }` when the chip is on another page. What comes back is the
register bytes as the chip returned them (`AnalogRegisters`, `ClockRegisters`, `DigitalRegisters`)
with methods that decode them into the types the setters take; a decoder returns `None` where a
register holds a code the datasheet calls reserved. On the bus, each register costs one transaction
of four messages (register number 0, the page, the register number, the value), so every value is
tied to the page it came from, and the only bytes written are register numbers.

Not readable this way: the "ADC is powered up" flags, which exist only in the clear-on-read register
36. The serial interface registers have no typed read-back yet; `read_passive()` returns their bytes.

`probe()` is an identity check in the same spirit: it reads the page and, on page 0 only, register
20 (AOSR, reset value 128) as a plausibility check and the clock registers to see whether anything
has set the chip up since its reset.

## Pages

The chip has more registers than one address byte reaches, so it groups them into pages, and
address 0 of every page selects the page for all following accesses. The driver remembers the page
it selected last and writes the page-select register only when an access needs another one; that
memory is only valid while nothing else talks to the chip.

**Clear-on-read registers.** Registers 34, 36, 42 and 45 of page 0 hold sticky flags that a read
clears. Every read method of the driver refuses them (`Error::ClearOnRead`), so nothing reads one by
accident; registers 43 and 47 carry the same interrupt flags as 42 and 45 without the side effect.

## Ownership

The driver's write path assumes it is the only one writing to the chip while it holds it. Its
page cache and its read-modify-writes span several bus transactions, and another writer in between
would break them.

- **Beside another driver of the same chip**, such as a Linux codec driver, do not write. Such a
  driver keeps its own idea of the selected page and of every register it wrote. One
  write from outside, even a page select, makes its next write land in the wrong place without
  anyone noticing. Next to such a driver use only the methods that write nothing: `page()`,
  `read_selected_page()`, `read_passive()`, the status API built from them (`status()`,
  `analog_registers()`, `clock_registers()`, `digital_registers()`) and `probe()`.
- **Borrowing the chip** from an owner that caches the page is possible with `restore_page()`:
  note `page()` before the first write and select that page again as the last one, since the
  other owner cannot be told which page the chip is on.
- **Taking the chip over** from a previous owner, such as a bootloader, is fine: start with `new`
  and `reset`.
- **One instance per chip.** To use the chip from several threads, tasks or cores, share one
  `Tlv320adc3101` behind a lock (a `Mutex` around the driver, held for a whole method call), not
  the bus: a shared-bus wrapper locks each transaction, and two instances would each cache a page
  the other one changes.
- **Other devices on the same bus** are no problem: the page register is inside this chip.

## Status

The driver covers what a capture needs. Its read path has been checked on hardware: the status API
read four of these chips on one TDM line beside another driver that owned them. Its write path
has run only against `embedded-hal-mock`; every write sequence is pinned by a host test, and a
first capture on a chip is still to come.

Not covered: the AGC, the digital microphone inputs, the programmable miniDSP mode, the filters
other than biquad A, time-slot mode, the secondary audio interface, CLKOUT, interrupts and the
GPIO pins. The reset pin is the board's.

## Testing

`cargo test` runs one `embedded-hal-mock` test per write path, pinning the bytes on the bus with
the page selects, the read-backs and the waits in order: the page-select logic, a capture start
on one chip, the power-down sequence, the digital gain encodings, `restore_page()` and `probe()`'s
reads, plus unit tests of the status decoders, which build register values by hand and involve no
bus, and this README's examples as doctests. The status read paths were checked on chips, not on
the host.

## Related crates

- [`tlv320dac3203`](https://crates.io/crates/tlv320dac3203): the same author's driver for the
  TLV320DAC3203, the DAC of the same TI family, in the same API style.
- [`tlv320aic3204`](https://crates.io/crates/tlv320aic3204): a driver for the TLV320AIC3204 codec,
  whose ADC half resembles this chip (paged registers, MICPGA, AGC); the register map differs and
  the chips are not interchangeable.
- [`tlv320dac3100`](https://crates.io/crates/tlv320dac3100) and
  [`tlv320aic23`](https://crates.io/crates/tlv320aic23): drivers for other TI audio chips.

## Sources

- TI **SLAS553B**, [TLV320ADC3101 datasheet](https://www.ti.com/lit/ds/symlink/tlv320adc3101.pdf),
  "Low-Power Stereo ADC With Embedded miniDSP for Wireless Handsets and Portable Audio" (November
  2008, revised August 2015): section 10.3 (reset, PLL start-up, serial interface, clock
  generation, inputs, gain, processing blocks), section 10.6 (register maps, Tables 14 to 109),
  section 11.2.2 (design procedure and the example above). "Table N", "section N" and "p. N" in
  this crate refer to it.

## Minimum supported Rust version

Rust 1.87, tested in CI. Raising it is a minor version bump.

## License

Licensed under either of [Apache License, Version 2.0](https://github.com/mschoenlaub/tlv320adc3101/blob/main/LICENSE-APACHE) or
[MIT license](https://github.com/mschoenlaub/tlv320adc3101/blob/main/LICENSE-MIT) at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
