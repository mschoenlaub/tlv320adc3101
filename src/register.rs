//! The register map: addresses and bit fields, as far as this driver covers the chip so far.
//!
//! Every constant cites the TI TLV320ADC3101 datasheet, SLAS553B (November 2008, revised August
//! 2015). "Table N" and "p. N" below always mean that document. Register numbers are written in
//! decimal because the datasheet's tables number them that way.
//!
//! Multi-bit field values are already shifted into place, so they can be OR-ed together and used
//! with their `*_MASK` directly. Numeric fields (dividers, gains, offsets) come as a mask plus a
//! shift where the field does not start at bit 0.
//!
//! What is left out so far (gaps of this driver, not of the chip): the AGC (page 0, registers 86 to 101), the miniDSP instruction
//! count and decimation (21, 22), the secondary audio interface (31 to 33), the interrupt and GPIO
//! pin controls (48, 49, 51, 52), the sync and CIC controls (57 to 59), the digital microphone
//! polarity (80), dither and low-current mode (page 1, registers 26 and 61), and every coefficient
//! except biquad A.

/// One register: the page it lives in and its address inside that page.
///
/// The chip has 8-bit addresses 0 to 127 in each page and selects the page through register 0,
/// which exists in every page (p. 43, section 10.6.1). Valid pages are 0, 1, 4, 5 and 32 to 47;
/// every other page is reserved and must not be read or written (Table 15 note, p. 46). This type
/// does not enforce that, so a `Register` built by hand is the caller's responsibility.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Register {
    page: u8,
    addr: u8,
}

impl Register {
    /// A register at `addr` in `page`.
    pub const fn new(page: u8, addr: u8) -> Self {
        Self { page, addr }
    }

    /// The page this register lives in.
    pub const fn page(self) -> u8 {
        self.page
    }

    /// The address inside the page.
    pub const fn addr(self) -> u8 {
        self.addr
    }

    /// Whether reading this register changes it (see [`CLEAR_ON_READ`]).
    pub const fn is_clear_on_read(self) -> bool {
        let mut i = 0;
        while i < CLEAR_ON_READ.len() {
            if CLEAR_ON_READ[i].page == self.page && CLEAR_ON_READ[i].addr == self.addr {
                return true;
            }
            i += 1;
        }
        false
    }
}

/// Whether `addr` is a clear-on-read register in any page. The read path that does not know the
/// selected page before it reads uses this to refuse the address outright.
pub(crate) const fn is_clear_on_read_in_any_page(addr: u8) -> bool {
    let mut i = 0;
    while i < CLEAR_ON_READ.len() {
        if CLEAR_ON_READ[i].addr == addr {
            return true;
        }
        i += 1;
    }
    false
}

/// The page-select register. It sits at address 0 of every page, so it is an address and not a
/// [`Register`]: writing `N` to it selects page `N` whatever page was selected before (Table 15,
/// p. 46; the same register is described again as Table 89 for page 1 and Table 106 for page 4).
/// It resets to page 0.
pub const PAGE_SELECT: u8 = 0;

/// Every register with sticky flag bits: a read returns them and clears them, so the next reader
/// no longer sees the event. No method of the driver reads one of these unless its name says so.
///
/// The list is the datasheet's, complete for the pages that hold control registers:
///
/// - page 0, register 34, bit D6, the "I2C hang detected" flag ("once set get cleared only after
///   reading this register", Table 40, p. 51);
/// - page 0, register 36, bits D5 and D1, the AGC saturation flags (Table 42 note 2, p. 52);
/// - page 0, register 42, bits D3 to D1, the ADC and barrel-shifter overflow flags (Table 46
///   note 1, p. 53);
/// - page 0, register 45, bits D6 to D3, the AGC noise and miniDSP interrupt flags (Table 49
///   note 1, p. 53).
///
/// Table 3 (p. 28) names the same split from the other side: registers 43 and 47 carry the same
/// flags as 42 and 45 but are "non-sticky", so those two are safe to read. The PGA flags in page 1,
/// register 62, are plain status bits as well (Table 104, p. 68).
pub const CLEAR_ON_READ: [Register; 4] = [
    I2S_SYNC,
    ADC_FLAGS,
    INTERRUPT_FLAGS_OVERFLOW_STICKY,
    INTERRUPT_FLAGS_ADC_STICKY,
];

// ---------------------------------------------------------------------------------------------
// Page 0: clocks, serial interface, flags, processing block, digital gain (section 10.6.2).
// ---------------------------------------------------------------------------------------------

/// Software reset (Table 16, p. 46). Write-only; reads as zero.
pub const SOFTWARE_RESET: Register = Register::new(0, 1);
/// Bits of [`SOFTWARE_RESET`].
pub mod software_reset {
    /// D0: writing 1 resets the control registers; the bit clears itself (Table 16).
    pub const RESET: u8 = 0b0000_0001;
}

/// Clock-gen multiplexing: where the PLL and the codec take their clock from (Table 19, p. 47).
pub const CLOCK_GEN_MUX: Register = Register::new(0, 4);
/// Bits of [`CLOCK_GEN_MUX`].
pub mod clock_gen_mux {
    /// D3 to D2: the PLL's input (Table 19).
    pub const PLL_CLKIN_MASK: u8 = 0b0000_1100;
    /// D3 to D2 = 00: PLL_CLKIN is the MCLK pin (Table 19).
    pub const PLL_CLKIN_MCLK: u8 = 0b0000_0000;
    /// D3 to D2 = 01: PLL_CLKIN is the BCLK pin (Table 19).
    pub const PLL_CLKIN_BCLK: u8 = 0b0000_0100;
    /// D3 to D2 = 11: PLL_CLKIN is tied to logic 0 (Table 19). 10 is reserved.
    pub const PLL_CLKIN_ZERO: u8 = 0b0000_1100;
    /// D1 to D0: the codec's input, the clock NADC divides (Table 19 and Figure 28, p. 24).
    pub const CODEC_CLKIN_MASK: u8 = 0b0000_0011;
    /// D1 to D0 = 00: CODEC_CLKIN is the MCLK pin (Table 19).
    pub const CODEC_CLKIN_MCLK: u8 = 0b0000_0000;
    /// D1 to D0 = 01: CODEC_CLKIN is the BCLK pin (Table 19).
    pub const CODEC_CLKIN_BCLK: u8 = 0b0000_0001;
    /// D1 to D0 = 11: CODEC_CLKIN is the PLL's output (Table 19). 10 is reserved.
    pub const CODEC_CLKIN_PLL: u8 = 0b0000_0011;
}

/// PLL power, P divider and R multiplier (Table 20, p. 47). Resets to P = 1, R = 1, powered down.
pub const PLL_P_R: Register = Register::new(0, 5);
/// Bits of [`PLL_P_R`].
pub mod pll_p_r {
    /// D7: the PLL is powered up (Table 20). It needs about 10 ms to start (section 10.3.2, p. 13).
    pub const POWER_UP: u8 = 0b1000_0000;
    /// D6 to D4: the divider P; 1 to 7 mean themselves and 0 means 8 (Table 20).
    pub const P_MASK: u8 = 0b0111_0000;
    /// Bit position of P (Table 20).
    pub const P_SHIFT: u8 = 4;
    /// D3 to D0: the multiplier R; 1 to 15 mean themselves and 0 means 16 (Table 20).
    pub const R_MASK: u8 = 0b0000_1111;
}

/// PLL multiplier J, the integer part of K (Table 21, p. 47). Resets to 4.
pub const PLL_J: Register = Register::new(0, 6);
/// Bits of [`PLL_J`].
pub mod pll_j {
    /// D5 to D0: J = 1 to 63; 0 is reserved (Table 21).
    pub const J_MASK: u8 = 0b0011_1111;
}

/// PLL fractional multiplier D, bits 13 to 8 (Table 22, p. 47). It only takes effect when
/// [`PLL_D_LSB`] is written immediately afterwards (Table 22 note 1).
pub const PLL_D_MSB: Register = Register::new(0, 7);
/// Bits of [`PLL_D_MSB`].
pub mod pll_d_msb {
    /// D5 to D0: D\[13:8\], of a D that ranges over 0 to 9999 (Table 22; range on p. 22).
    pub const D_HIGH_MASK: u8 = 0b0011_1111;
}

/// PLL fractional multiplier D, bits 7 to 0 (Table 23, p. 47). Must be written immediately after
/// [`PLL_D_MSB`] (Table 23 note 1).
pub const PLL_D_LSB: Register = Register::new(0, 8);

/// NADC, the divider from CODEC_CLKIN to ADC_CLK (Table 25, p. 48). Resets to 1, powered down.
pub const NADC: Register = Register::new(0, 18);
/// MADC, the divider from ADC_CLK to ADC_MOD_CLK (Table 26, p. 48). Resets to 1, powered down.
pub const MADC: Register = Register::new(0, 19);
/// BCLK N, the divider from BDIV_CLKIN to the BCLK output (Table 36, p. 50). Resets to 1, powered
/// down.
pub const BCLK_N: Register = Register::new(0, 30);
/// CLKOUT M, the divider in front of the CLKOUT output (Table 32, p. 49). Resets to 1, powered
/// down.
pub const CLKOUT_M: Register = Register::new(0, 26);
/// The layout [`NADC`], [`MADC`], [`BCLK_N`] and [`CLKOUT_M`] share.
pub mod divider {
    /// D7: the divider is powered up (Tables 25, 26, 32 and 36).
    pub const POWER_UP: u8 = 0b1000_0000;
    /// D6 to D0: the divisor; 1 to 127 mean themselves and 0 means 128 (Tables 25, 26, 32 and 36).
    pub const VALUE_MASK: u8 = 0b0111_1111;
}

/// AOSR, the oversampling ratio from ADC_MOD_CLK to the sample rate (Table 27, p. 48). All eight
/// bits are the value; 1 to 255 mean themselves and 0 means 256. Resets to 128. It must be a
/// multiple of the decimation the processing block uses (Table 27 note 1; Table 6, p. 31).
pub const AOSR: Register = Register::new(0, 20);

/// [`AOSR`]'s reset value, 128 (Table 27, p. 48). The register is writable, unlike the
/// TLV320DAC3203's read-only AOSR, so reading this value only makes the chip plausible;
/// [`Tlv320adc3101::probe`](crate::Tlv320adc3101::probe) uses it as its identity check because it
/// holds at reset and in every configuration that keeps the default oversampling.
pub const AOSR_RESET: u8 = 0x80;

/// CLKOUT multiplexer: which clock feeds the CLKOUT divider (Table 31, p. 49).
pub const CLKOUT_MUX: Register = Register::new(0, 25);
/// Bits of [`CLKOUT_MUX`].
pub mod clkout_mux {
    /// D2 to D0: the source, CDIV_CLKIN (Table 31). 010, 100 and 101 are reserved.
    pub const SOURCE_MASK: u8 = 0b0000_0111;
    /// D2 to D0 = 000: the MCLK pin (Table 31).
    pub const SOURCE_MCLK: u8 = 0b0000_0000;
    /// D2 to D0 = 001: the BCLK pin (Table 31).
    pub const SOURCE_BCLK: u8 = 0b0000_0001;
    /// D2 to D0 = 011: the PLL's output (Table 31).
    pub const SOURCE_PLL: u8 = 0b0000_0011;
    /// D2 to D0 = 110: ADC_CLK (Table 31).
    pub const SOURCE_ADC_CLK: u8 = 0b0000_0110;
    /// D2 to D0 = 111: ADC_MOD_CLK (Table 31).
    pub const SOURCE_ADC_MOD_CLK: u8 = 0b0000_0111;
}

/// ADC audio interface control 1: format, word length, clock directions (Table 33, p. 49).
pub const INTERFACE_CONTROL_1: Register = Register::new(0, 27);
/// Bits of [`INTERFACE_CONTROL_1`].
pub mod interface_control_1 {
    /// D7 to D6: the serial format (Table 33).
    pub const FORMAT_MASK: u8 = 0b1100_0000;
    /// D7 to D6 = 00: I2S (Table 33).
    pub const FORMAT_I2S: u8 = 0b0000_0000;
    /// D7 to D6 = 01: DSP, left channel first from the rising edge of WCLK and the right channel
    /// immediately after it (Table 33; timing in section 10.3.6.4, p. 20).
    pub const FORMAT_DSP: u8 = 0b0100_0000;
    /// D7 to D6 = 10: right-justified (Table 33).
    pub const FORMAT_RIGHT_JUSTIFIED: u8 = 0b1000_0000;
    /// D7 to D6 = 11: left-justified (Table 33).
    pub const FORMAT_LEFT_JUSTIFIED: u8 = 0b1100_0000;
    /// D5 to D4: the word length (Table 33).
    pub const WORD_LENGTH_MASK: u8 = 0b0011_0000;
    /// D5 to D4 = 00: 16 bits (Table 33).
    pub const WORD_LENGTH_16: u8 = 0b0000_0000;
    /// D5 to D4 = 01: 20 bits (Table 33).
    pub const WORD_LENGTH_20: u8 = 0b0001_0000;
    /// D5 to D4 = 10: 24 bits (Table 33).
    pub const WORD_LENGTH_24: u8 = 0b0010_0000;
    /// D5 to D4 = 11: 32 bits (Table 33).
    pub const WORD_LENGTH_32: u8 = 0b0011_0000;
    /// D3: BCLK is an output, so this chip is the bit-clock master (Table 33).
    pub const BCLK_OUTPUT: u8 = 0b0000_1000;
    /// D2: WCLK is an output, so this chip is the word-clock master (Table 33).
    pub const WCLK_OUTPUT: u8 = 0b0000_0100;
    /// D0: DOUT goes high-impedance outside this chip's own slots, which is what lets several
    /// chips share one data line (Table 33).
    pub const DOUT_TRISTATE: u8 = 0b0000_0001;
}

/// Data slot offset 1, Ch_Offset_1: how many BCLKs after the rising edge of WCLK the first channel
/// starts, 0 to 255 (Table 34, p. 50). Unless time-slot mode is on, the second channel follows the
/// first directly (Table 44, bit D0).
pub const CH_OFFSET_1: Register = Register::new(0, 28);

/// ADC interface control 2: BCLK polarity and what the BCLK divider is fed from (Table 35, p. 50).
pub const INTERFACE_CONTROL_2: Register = Register::new(0, 29);
/// Bits of [`INTERFACE_CONTROL_2`].
pub mod interface_control_2 {
    /// D3: BCLK is inverted (Table 35).
    pub const BCLK_INVERTED: u8 = 0b0000_1000;
    /// D2: BCLK and WCLK keep running while the codec is powered down (Table 35).
    pub const CLOCKS_ACTIVE_WHEN_POWERED_DOWN: u8 = 0b0000_0100;
    /// D1 to D0: the input of the BCLK N divider (Table 35). 00 and 01 are reserved.
    pub const BDIV_CLKIN_MASK: u8 = 0b0000_0011;
    /// D1 to D0 = 10: BDIV_CLKIN is ADC_CLK, the reset value (Table 35).
    pub const BDIV_CLKIN_ADC_CLK: u8 = 0b0000_0010;
    /// D1 to D0 = 11: BDIV_CLKIN is ADC_MOD_CLK (Table 35).
    pub const BDIV_CLKIN_ADC_MOD_CLK: u8 = 0b0000_0011;
}

/// I2S sync, which also holds the I2C hang detector (Table 40, p. 51).
///
/// **Clear-on-read**: bit D6 is cleared by reading this register. Listed in [`CLEAR_ON_READ`].
pub const I2S_SYNC: Register = Register::new(0, 34);
/// Bits of [`I2S_SYNC`].
pub mod i2s_sync {
    /// D7: the logic that detects a hung I2C bus is disabled (Table 40).
    pub const I2C_HANG_DETECTION_DISABLED: u8 = 0b1000_0000;
    /// D6: an I2C hang was detected. Sticky: once set it is cleared only by reading this register
    /// (Table 40).
    pub const I2C_HANG_DETECTED: u8 = 0b0100_0000;
    /// D5: the chip answers the I2C general-call address (Table 40).
    pub const GENERAL_CALL_ACCEPTED: u8 = 0b0010_0000;
    /// D1: re-synchronise the ADC with the interface when the group delay drifts by more than a
    /// quarter of a sample period (Table 40).
    pub const RESYNC: u8 = 0b0000_0010;
    /// D0: soft-mute the channel while re-synchronising (Table 40).
    pub const RESYNC_SOFT_MUTE: u8 = 0b0000_0001;
}

/// ADC flag register: power and gain status, plus the AGC saturation flags (Table 42, p. 52).
///
/// **Clear-on-read**: bits D5 and D1 are sticky and cleared by reading this register (Table 42
/// note 2). The other bits are live status. Listed in [`CLEAR_ON_READ`].
pub const ADC_FLAGS: Register = Register::new(0, 36);
/// Bits of [`ADC_FLAGS`].
pub mod adc_flags {
    /// D7: the left PGA has reached its programmed gain (Table 42).
    pub const LEFT_PGA_GAIN_APPLIED: u8 = 0b1000_0000;
    /// D6: the left ADC is powered up (Table 42).
    pub const LEFT_ADC_POWERED: u8 = 0b0100_0000;
    /// D5: the left AGC applied its maximum gain. Sticky (Table 42 note 2).
    pub const LEFT_AGC_SATURATED: u8 = 0b0010_0000;
    /// D3: the right PGA has reached its programmed gain (Table 42).
    pub const RIGHT_PGA_GAIN_APPLIED: u8 = 0b0000_1000;
    /// D2: the right ADC is powered up (Table 42).
    pub const RIGHT_ADC_POWERED: u8 = 0b0000_0100;
    /// D1: the right AGC applied its maximum gain. Sticky (Table 42 note 2).
    pub const RIGHT_AGC_SATURATED: u8 = 0b0000_0010;
}

/// Data slot offset 2, Ch_Offset_2: how many BCLKs after the end of the first channel the second
/// one starts, 0 to 255 (Table 43, p. 52). Only used in time-slot mode (Table 44, bit D0).
pub const CH_OFFSET_2: Register = Register::new(0, 37);

/// I2S TDM control: which channels are sent and how the slot offsets apply (Table 44, p. 52).
pub const I2S_TDM_CONTROL: Register = Register::new(0, 38);
/// Bits of [`I2S_TDM_CONTROL`].
pub mod i2s_tdm_control {
    /// D4: left and right channel swap places on the bus (Table 44).
    pub const CHANNEL_SWAP: u8 = 0b0001_0000;
    /// D3 to D2: which channels are sent (Table 44).
    pub const CHANNELS_MASK: u8 = 0b0000_1100;
    /// D3 to D2 = 00: both channels (Table 44).
    pub const CHANNELS_BOTH: u8 = 0b0000_0000;
    /// D3 to D2 = 01: "left channel enabled", in the words of Table 44. The datasheet's own
    /// description of the register reads it the other way round: setting bit D2 disables the
    /// first channel and the second is still sent (p. 15 and Figure 12). Which is right on the
    /// chip is an open question; the constant keeps the table's name.
    pub const CHANNELS_LEFT: u8 = 0b0000_0100;
    /// D3 to D2 = 10: "right channel enabled", in the datasheet's words (Table 44). See
    /// [`CHANNELS_LEFT`] for the doubt about it.
    pub const CHANNELS_RIGHT: u8 = 0b0000_1000;
    /// D3 to D2 = 11: neither channel (Table 44).
    pub const CHANNELS_NONE: u8 = 0b0000_1100;
    /// D1: early 3-state, the reset value (Table 44).
    pub const EARLY_TRISTATE: u8 = 0b0000_0010;
    /// D0: time-slot mode. Off, Ch_Offset_1 places both channels; on, Ch_Offset_1 places the first
    /// and Ch_Offset_2 the second (Table 44).
    pub const TIME_SLOT_MODE: u8 = 0b0000_0001;
}

/// Interrupt flags, overflow, sticky copy (Table 46, p. 53). Bits as in [`overflow_flags`].
///
/// **Clear-on-read** (Table 46 note 1). Listed in [`CLEAR_ON_READ`]. Read
/// [`INTERRUPT_FLAGS_OVERFLOW`] for the same flags without the side effect.
pub const INTERRUPT_FLAGS_OVERFLOW_STICKY: Register = Register::new(0, 42);
/// Interrupt flags, overflow, live copy (Table 47, p. 53; "non-sticky" in Table 3, p. 28). Bits as
/// in [`overflow_flags`].
pub const INTERRUPT_FLAGS_OVERFLOW: Register = Register::new(0, 43);
/// Bits of [`INTERRUPT_FLAGS_OVERFLOW_STICKY`] and [`INTERRUPT_FLAGS_OVERFLOW`].
pub mod overflow_flags {
    /// D3: the left ADC overflowed (Tables 46 and 47).
    pub const LEFT_ADC_OVERFLOW: u8 = 0b0000_1000;
    /// D2: the right ADC overflowed (Tables 46 and 47).
    pub const RIGHT_ADC_OVERFLOW: u8 = 0b0000_0100;
    /// D1: the ADC barrel shifter's output overflowed (Tables 46 and 47).
    pub const BARREL_SHIFTER_OVERFLOW: u8 = 0b0000_0010;
}

/// Interrupt flags, ADC, sticky copy (Table 49, p. 53). Bits as in [`adc_interrupt_flags`].
///
/// **Clear-on-read** (Table 49 note 1). Listed in [`CLEAR_ON_READ`]. Read [`INTERRUPT_FLAGS_ADC`]
/// for the same flags without the side effect.
pub const INTERRUPT_FLAGS_ADC_STICKY: Register = Register::new(0, 45);
/// Interrupt flags, ADC, live copy (Table 51, p. 54; "non-sticky" in Table 3, p. 28). Bits as in
/// [`adc_interrupt_flags`].
pub const INTERRUPT_FLAGS_ADC: Register = Register::new(0, 47);
/// Bits of [`INTERRUPT_FLAGS_ADC_STICKY`] and [`INTERRUPT_FLAGS_ADC`].
pub mod adc_interrupt_flags {
    /// D6: the left signal is below the left AGC's noise threshold (Tables 49 and 51).
    pub const LEFT_AGC_NOISE: u8 = 0b0100_0000;
    /// D5: the right signal is below the right AGC's noise threshold (Tables 49 and 51).
    pub const RIGHT_AGC_NOISE: u8 = 0b0010_0000;
    /// D4: the miniDSP engine's standard interrupt port (Tables 49 and 51).
    pub const MINIDSP_STANDARD: u8 = 0b0001_0000;
    /// D3: the miniDSP engine's auxiliary interrupt port (Tables 49 and 51).
    pub const MINIDSP_AUXILIARY: u8 = 0b0000_1000;
}

/// DOUT pin control (Table 57, p. 55). Resets to the primary data output with the bus keeper off.
pub const DOUT_CONTROL: Register = Register::new(0, 53);
/// Bits of [`DOUT_CONTROL`].
pub mod dout_control {
    /// D4: the bus keeper on DOUT is disabled, the reset value; the bit is the disable (Table 57).
    pub const BUS_KEEPER_DISABLED: u8 = 0b0001_0000;
    /// D3 to D1: what the pin carries (Table 57).
    pub const FUNCTION_MASK: u8 = 0b0000_1110;
    /// D3 to D1 = 000: the output buffer is powered down (Table 57).
    pub const FUNCTION_DISABLED: u8 = 0b0000_0000;
    /// D3 to D1 = 001: the primary audio data output (Table 57).
    pub const FUNCTION_PRIMARY_DOUT: u8 = 0b0000_0010;
    /// D3 to D1 = 010: a general-purpose output driven by bit D0 (Table 57).
    pub const FUNCTION_GENERAL_PURPOSE: u8 = 0b0000_0100;
    /// D3 to D1 = 011: the CLKOUT output (Table 57).
    pub const FUNCTION_CLKOUT: u8 = 0b0000_0110;
    /// D3 to D1 = 100: the INT1 output (Table 57).
    pub const FUNCTION_INT1: u8 = 0b0000_1000;
    /// D3 to D1 = 101: the INT2 output (Table 57).
    pub const FUNCTION_INT2: u8 = 0b0000_1010;
    /// D3 to D1 = 110: the secondary BCLK output (Table 57).
    pub const FUNCTION_SECONDARY_BCLK: u8 = 0b0000_1100;
    /// D3 to D1 = 111: the secondary WCLK output (Table 57).
    pub const FUNCTION_SECONDARY_WCLK: u8 = 0b0000_1110;
    /// D0: the pin's level as a general-purpose output (Table 57).
    pub const GENERAL_PURPOSE_VALUE: u8 = 0b0000_0001;
}

/// ADC processing block selection (Table 63, p. 57). Resets to PRB_R1.
pub const PROCESSING_BLOCK: Register = Register::new(0, 61);
/// Bits of [`PROCESSING_BLOCK`].
pub mod processing_block {
    /// D4 to D0: 1 to 18 select PRB_R1 to PRB_R18, 0 selects the programmable miniDSP mode and the
    /// rest is reserved (Table 63). What each block contains is in Table 6 (p. 31).
    pub const SELECT_MASK: u8 = 0b0001_1111;
    /// PRB_R1: stereo, decimation filter A, first-order IIR, no biquads (Table 6). The reset value.
    pub const PRB_R1: u8 = 1;
    /// PRB_R2: stereo, decimation filter A, first-order IIR and five biquads, AOSR 128 or 64
    /// (Table 6).
    pub const PRB_R2: u8 = 2;
}

/// ADC digital: channel power, digital microphone selection, soft-stepping (Table 67, p. 58).
pub const ADC_DIGITAL: Register = Register::new(0, 81);
/// Bits of [`ADC_DIGITAL`].
pub mod adc_digital {
    /// D7: the left ADC channel is powered up (Table 67).
    pub const LEFT_POWER_UP: u8 = 0b1000_0000;
    /// D6: the right ADC channel is powered up (Table 67).
    pub const RIGHT_POWER_UP: u8 = 0b0100_0000;
    /// D5: the left digital microphone is read from the DMCLK pin instead of DMDIN (Table 67).
    pub const LEFT_DMIC_FROM_DMCLK: u8 = 0b0010_0000;
    /// D4: the right digital microphone is read from the DMCLK pin instead of DMDIN (Table 67).
    pub const RIGHT_DMIC_FROM_DMCLK: u8 = 0b0001_0000;
    /// D3: the left channel takes a digital microphone instead of the analog input (Table 67).
    pub const LEFT_DMIC_ENABLE: u8 = 0b0000_1000;
    /// D2: the right channel takes a digital microphone instead of the analog input (Table 67).
    pub const RIGHT_DMIC_ENABLE: u8 = 0b0000_0100;
    /// D1 to D0: soft-stepping of the volume, shared with the analog PGA (Table 67; p. 26).
    pub const SOFT_STEP_MASK: u8 = 0b0000_0011;
    /// D1 to D0 = 00: one gain step per sample, the reset value (Table 67).
    pub const SOFT_STEP_PER_SAMPLE: u8 = 0b0000_0000;
    /// D1 to D0 = 01: one gain step per two samples (Table 67).
    pub const SOFT_STEP_PER_TWO_SAMPLES: u8 = 0b0000_0001;
    /// D1 to D0 = 10: soft-stepping is off and gain changes apply at once (Table 67). 11 is
    /// reserved.
    pub const SOFT_STEP_DISABLED: u8 = 0b0000_0010;
}

/// ADC fine volume control: mute and the 0.1 dB trim, per channel (Table 68, p. 58). Both channels
/// reset to muted.
pub const ADC_FINE_VOLUME: Register = Register::new(0, 82);
/// Bits of [`ADC_FINE_VOLUME`].
pub mod adc_fine_volume {
    /// D7: the left channel is muted (Table 68).
    pub const LEFT_MUTE: u8 = 0b1000_0000;
    /// D6 to D4: the left fine gain, 0 to 4 for 0 to -0.4 dB in 0.1 dB steps; 5 to 7 are reserved
    /// (Table 68; section 10.3.9.2, p. 26).
    pub const LEFT_FINE_GAIN_MASK: u8 = 0b0111_0000;
    /// Bit position of the left fine gain (Table 68).
    pub const LEFT_FINE_GAIN_SHIFT: u8 = 4;
    /// D3: the right channel is muted (Table 68).
    pub const RIGHT_MUTE: u8 = 0b0000_1000;
    /// D2 to D0: the right fine gain, encoded like the left. Table 68 labels these rows "Left ADC
    /// channel" a second time, which is a slip: they sit under the right channel's mute bit and
    /// section 10.3.9.2 (p. 26) describes one trim per channel.
    pub const RIGHT_FINE_GAIN_MASK: u8 = 0b0000_0111;
}

/// Left ADC volume control (Table 69, p. 58; Table 2, p. 25). Resets to 0 dB.
pub const LEFT_ADC_VOLUME: Register = Register::new(0, 83);
/// Right ADC volume control (Table 70, p. 59; Table 2, p. 25). Resets to 0 dB.
pub const RIGHT_ADC_VOLUME: Register = Register::new(0, 84);
/// The layout [`LEFT_ADC_VOLUME`] and [`RIGHT_ADC_VOLUME`] share.
pub mod adc_volume {
    /// D6 to D0: the gain in half decibels as a 7-bit two's complement number (Tables 69 and 70,
    /// note 1).
    pub const VALUE_MASK: u8 = 0b0111_1111;
    /// The lowest setting, -12 dB: -24 half decibels in 7 bits (Table 2).
    pub const MIN_MINUS_12_DB: u8 = 0b110_1000;
    /// 0 dB, the reset value (Table 2).
    pub const ZERO_DB: u8 = 0b000_0000;
    /// The highest setting, +20 dB: 40 half decibels. Everything from 41 up to 63 is reserved
    /// (Tables 69 and 70).
    pub const MAX_PLUS_20_DB: u8 = 0b010_1000;
}

// ---------------------------------------------------------------------------------------------
// Page 1: microphone bias, input routing, analog gain (section 10.6.3).
// ---------------------------------------------------------------------------------------------

/// MICBIAS control (Table 93, p. 65). Both outputs reset to powered down.
pub const MICBIAS_CONTROL: Register = Register::new(1, 51);
/// Bits of [`MICBIAS_CONTROL`].
pub mod micbias_control {
    /// D6 to D5: MICBIAS1 (Table 93).
    pub const MICBIAS1_MASK: u8 = 0b0110_0000;
    /// Bit position of the MICBIAS1 field (Table 93).
    pub const MICBIAS1_SHIFT: u8 = 5;
    /// D6 to D5 = 00: MICBIAS1 is powered down (Table 93).
    pub const MICBIAS1_OFF: u8 = 0b0000_0000;
    /// D6 to D5 = 01: MICBIAS1 is 2 V (Table 93).
    pub const MICBIAS1_2V: u8 = 0b0010_0000;
    /// D6 to D5 = 10: MICBIAS1 is 2.5 V (Table 93).
    pub const MICBIAS1_2V5: u8 = 0b0100_0000;
    /// D6 to D5 = 11: MICBIAS1 is switched to AVDD (Table 93).
    pub const MICBIAS1_AVDD: u8 = 0b0110_0000;
    /// D4 to D3: MICBIAS2 (Table 93).
    pub const MICBIAS2_MASK: u8 = 0b0001_1000;
    /// Bit position of the MICBIAS2 field (Table 93).
    pub const MICBIAS2_SHIFT: u8 = 3;
    /// D4 to D3 = 00: MICBIAS2 is powered down (Table 93).
    pub const MICBIAS2_OFF: u8 = 0b0000_0000;
    /// D4 to D3 = 01: MICBIAS2 is 2 V (Table 93).
    pub const MICBIAS2_2V: u8 = 0b0000_1000;
    /// D4 to D3 = 10: MICBIAS2 is 2.5 V (Table 93).
    pub const MICBIAS2_2V5: u8 = 0b0001_0000;
    /// D4 to D3 = 11: MICBIAS2 is switched to AVDD (Table 93).
    pub const MICBIAS2_AVDD: u8 = 0b0001_1000;
}

/// How one input reaches a PGA: the two-bit code every field of the four input-selection registers
/// uses (Tables 94, 96, 97 and 99, pp. 66 to 67). Shift it to the field's position.
pub mod input_level {
    /// 00: connected at 0 dB (Tables 94, 96, 97 and 99).
    pub const ZERO_DB: u8 = 0b00;
    /// 01: connected at -6 dB (Tables 94, 96, 97 and 99).
    pub const MINUS_6_DB: u8 = 0b01;
    /// 11: not connected, the reset value. 10 means the same (Tables 94, 96, 97 and 99).
    pub const NOT_CONNECTED: u8 = 0b11;
    /// The width of one field, before shifting (Tables 94, 96, 97 and 99).
    pub const MASK: u8 = 0b11;
}

/// Left PGA input selection, first register (Table 94, p. 66). Resets to nothing connected.
pub const LEFT_PGA_INPUT_1: Register = Register::new(1, 52);
/// Field positions in [`LEFT_PGA_INPUT_1`]; each field holds an [`input_level`] code.
pub mod left_pga_input_1 {
    /// D7 to D6, LCH_SEL4: the differential pair IN2L(P) plus, IN3L(M) minus (Table 94).
    pub const DIFF_IN2L_IN3L_SHIFT: u8 = 6;
    /// D5 to D4, LCH_SEL3: the IN3L(M) pin, single-ended (Table 94).
    pub const IN3L_SHIFT: u8 = 4;
    /// D3 to D2, LCH_SEL2: the IN2L(P) pin, single-ended (Table 94).
    pub const IN2L_SHIFT: u8 = 2;
    /// D1 to D0, LCH_SEL1: the IN1L(P) pin, single-ended (Table 94).
    pub const IN1L_SHIFT: u8 = 0;
}

/// Left PGA input selection, second register (Table 96, p. 66).
pub const LEFT_PGA_INPUT_2: Register = Register::new(1, 54);
/// Bits and field positions in [`LEFT_PGA_INPUT_2`]; each field holds an [`input_level`] code.
pub mod left_pga_input_2 {
    /// D7: bypass the left PGA and feed the IN2L(P)/IN3L(M) pair to the ADC unbuffered (Table 96).
    pub const PGA_BYPASS: u8 = 0b1000_0000;
    /// D6, LCH_SELCM: unselected left inputs are weakly biased to the ADC's common-mode voltage
    /// (Table 96).
    pub const UNSELECTED_INPUTS_BIASED: u8 = 0b0100_0000;
    /// D5 to D4, LCH_SEL3X: the differential pair IN1L(P) plus, IN1R(M) minus (Table 96).
    pub const DIFF_IN1L_IN1R_SHIFT: u8 = 4;
    /// D3 to D2, LCH_SEL2X: the differential pair IN2R(P) plus, IN3R(M) minus (Table 96).
    pub const DIFF_IN2R_IN3R_SHIFT: u8 = 2;
    /// D1 to D0, LCH_SEL1X: the IN1R(M) pin, single-ended (Table 96).
    pub const IN1R_SHIFT: u8 = 0;
}

/// Right PGA input selection, first register (Table 97, p. 67). Resets to nothing connected.
pub const RIGHT_PGA_INPUT_1: Register = Register::new(1, 55);
/// Field positions in [`RIGHT_PGA_INPUT_1`]; each field holds an [`input_level`] code.
pub mod right_pga_input_1 {
    /// D7 to D6, RCH_SEL4: the differential pair IN2R(P) plus, IN3R(M) minus (Table 97).
    pub const DIFF_IN2R_IN3R_SHIFT: u8 = 6;
    /// D5 to D4, RCH_SEL3: the IN3R(M) pin, single-ended (Table 97).
    pub const IN3R_SHIFT: u8 = 4;
    /// D3 to D2, RCH_SEL2: the IN2R(P) pin, single-ended (Table 97).
    pub const IN2R_SHIFT: u8 = 2;
    /// D1 to D0, RCH_SEL1: the IN1R(M) pin, single-ended (Table 97).
    pub const IN1R_SHIFT: u8 = 0;
}

/// Right PGA input selection, second register (Table 99, p. 67).
pub const RIGHT_PGA_INPUT_2: Register = Register::new(1, 57);
/// Bits and field positions in [`RIGHT_PGA_INPUT_2`]; each field holds an [`input_level`] code.
pub mod right_pga_input_2 {
    /// D7: bypass the right PGA and feed the IN2R(P)/IN3R(M) pair to the ADC unbuffered
    /// (Table 99).
    pub const PGA_BYPASS: u8 = 0b1000_0000;
    /// D6, RCH_SELCM: unselected right inputs are weakly biased to the ADC's common-mode voltage
    /// (Table 99).
    pub const UNSELECTED_INPUTS_BIASED: u8 = 0b0100_0000;
    /// D5 to D4, RCH_SEL3X: the differential pair IN1L(P) plus, IN1R(M) minus (Table 99).
    pub const DIFF_IN1L_IN1R_SHIFT: u8 = 4;
    /// D3 to D2, RCH_SEL2X: the differential pair IN2L(P) plus, IN3L(M) minus (Table 99).
    pub const DIFF_IN2L_IN3L_SHIFT: u8 = 2;
    /// D1 to D0, RCH_SEL1X: the IN1L(P) pin, single-ended (Table 99).
    pub const IN1L_SHIFT: u8 = 0;
}

/// Left analog PGA setting (Table 101, p. 68). Resets to muted, 0 dB.
pub const LEFT_PGA: Register = Register::new(1, 59);
/// Right analog PGA setting (Table 102, p. 68). Resets to muted, 0 dB.
pub const RIGHT_PGA: Register = Register::new(1, 60);
/// The layout [`LEFT_PGA`] and [`RIGHT_PGA`] share.
pub mod pga {
    /// D7: the PGA is muted (Tables 101 and 102).
    pub const MUTE: u8 = 0b1000_0000;
    /// D6 to D0: the gain in half decibels (Tables 101 and 102).
    pub const GAIN_MASK: u8 = 0b0111_1111;
    /// The highest gain the datasheet defines, 40 dB: 80 half decibels. It calls 81 to 127
    /// reserved (Tables 101 and 102), although rising gain up to 119 has been observed on a production
    /// board ([`crate::PgaGain`]).
    pub const GAIN_MAX_40_DB: u8 = 80;
}

/// ADC analog PGA flags (Table 104, p. 68). Plain status, not sticky: the safe way to ask whether
/// a gain change has finished soft-stepping.
pub const PGA_FLAGS: Register = Register::new(1, 62);
/// Bits of [`PGA_FLAGS`].
pub mod pga_flags {
    /// D1: the left PGA has reached its programmed gain (Table 104).
    pub const LEFT_GAIN_APPLIED: u8 = 0b0000_0010;
    /// D0: the right PGA has reached its programmed gain (Table 104).
    pub const RIGHT_GAIN_APPLIED: u8 = 0b0000_0001;
}

// ---------------------------------------------------------------------------------------------
// Page 4: coefficients of biquad A (section 10.6.4).
// ---------------------------------------------------------------------------------------------

/// Coefficient registers of biquad A, the first of the five biquads in PRB_R2 (Table 8, p. 35;
/// Table 107, pp. 69 to 72).
///
/// Each coefficient is a 16-bit two's complement number in two adjacent registers. The constant
/// names the high byte; the low byte is the next address. The high byte must be written first and
/// the low byte immediately after it, even when only one of them changes (p. 69). The transfer
/// function is `(N0 + 2*N1*z^-1 + N2*z^-2) / (2^15 - 2*D1*z^-1 - D2*z^-2)` (Equation 4, p. 35).
pub mod biquad_a {
    use super::Register;

    /// Left channel, N0: page 4, registers 14 and 15 (Table 8).
    pub const LEFT_N0_MSB: Register = Register::new(4, 14);
    /// Left channel, N1: page 4, registers 16 and 17 (Table 8).
    pub const LEFT_N1_MSB: Register = Register::new(4, 16);
    /// Left channel, N2: page 4, registers 18 and 19 (Table 8).
    pub const LEFT_N2_MSB: Register = Register::new(4, 18);
    /// Left channel, D1: page 4, registers 20 and 21 (Table 8).
    pub const LEFT_D1_MSB: Register = Register::new(4, 20);
    /// Left channel, D2: page 4, registers 22 and 23 (Table 8).
    pub const LEFT_D2_MSB: Register = Register::new(4, 22);
    /// Right channel, N0: page 4, registers 78 and 79 (Table 8).
    pub const RIGHT_N0_MSB: Register = Register::new(4, 78);
    /// Right channel, N1: page 4, registers 80 and 81 (Table 8).
    pub const RIGHT_N1_MSB: Register = Register::new(4, 80);
    /// Right channel, N2: page 4, registers 82 and 83 (Table 8).
    pub const RIGHT_N2_MSB: Register = Register::new(4, 82);
    /// Right channel, D1: page 4, registers 84 and 85 (Table 8).
    pub const RIGHT_D1_MSB: Register = Register::new(4, 84);
    /// Right channel, D2: page 4, registers 86 and 87 (Table 8).
    pub const RIGHT_D2_MSB: Register = Register::new(4, 86);
}
