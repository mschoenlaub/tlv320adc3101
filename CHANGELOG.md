# Changelog

All notable changes to this crate are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the crate follows
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-10-14

First release.

- Paged register access with a cached page select, the register map with its bit fields, and a
  typed error.
- Reset with its settle time.
- The clock tree: the PLL and codec clock multiplexers, the PLL (P, R, J, D) with its start-up
  wait, NADC, MADC, AOSR and the BCLK divider.
- The serial audio interface: I2S, DSP, left- and right-justified, word length, clock directions,
  the data offset for time-division multiplexing, the channel enable bits and the DOUT pin.
- The processing block and the coefficients of biquad A.
- Microphone bias, input routing per pin and level, and the PGA gain.
- Channel power and mute, digital volume, fine gain and soft-stepping, with `power_up` and
  `power_down` for the start and the stop of a capture.
- A passive read path that writes nothing, not even the page select: `page`,
  `read_selected_page`, `read_passive` and the status API on top of them, with decoders into
  the write side's types; `restore_page` to hand a chip back to an owner that caches the page.
- `probe`, an identity and state check that writes nothing.

[Unreleased]: https://github.com/mschoenlaub/tlv320adc3101/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/mschoenlaub/tlv320adc3101/releases/tag/v0.1.0
