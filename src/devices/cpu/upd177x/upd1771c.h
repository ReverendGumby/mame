// license:BSD-3-Clause
// copyright-holders:David Hunter
#ifndef MAME_CPU_UPD177X_UPD1771_H
#define MAME_CPU_UPD177X_UPD1771_H

#pragma once


enum
{
	UPD1771C_PC=1, UPD1771C_A, UPD1771C_H, UPD1771C_SP, UPD1771C_X, UPD1771C_Y,
    UPD1771C_MD, UPD1771C_N, UPD1771C_NC, UPD1771C_DA
};

class upd1771c_device : public cpu_device,
						public device_sound_interface
{
public:
	// construction/destruction
	upd1771c_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// configuration helpers
	auto pb_out_cb() { return m_pb_out_cb.bind(); }

	u8 pa_r();
	void pa_w(u8 data);

protected:
	// Mode (md) flags
	enum
	{
		MD_64_32   = 1 << 0,
		MD_TONE_IE = 1 << 1,
		MD_NS_IE   = 1 << 2,
		MD_NSS     = 1 << 3,
		MD_TIME_IE = 1 << 4,
		MD_EXT_IE  = 1 << 5,
		MD_OUT     = 1 << 6,
		MD_IF      = 1 << 7,
		// Bits 8-9 set NS interrupt rate
	};

	// IRR flags
	enum
	{
		INT_TONE = 1 << 0,
		INT_NS   = 1 << 1,
		INT_EXT  = 1 << 2,
		INT_TIME = 1 << 3,
	};

	upd1771c_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// internal memory maps
	void internal_512x16(address_map &map) ATTR_COLD;
	void internal_64x8(address_map &map) ATTR_COLD;

	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_execute_interface overrides
	virtual uint64_t execute_clocks_to_cycles(uint64_t clocks) const noexcept override { return (clocks + 8 - 1) / 8; }
	virtual uint64_t execute_cycles_to_clocks(uint64_t cycles) const noexcept override { return (cycles * 8); }
	virtual uint32_t execute_min_cycles() const noexcept override { return 1; }
	virtual uint32_t execute_max_cycles() const noexcept override { return 40; }
	virtual bool execute_input_edge_triggered(int inputnum) const noexcept override { return true; }
	virtual void execute_run() override;
	//virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_state_interface overrides
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

	// sound stream generation
	virtual void sound_stream_update(sound_stream &stream, std::vector<read_stream_view> const &inputs, std::vector<write_stream_view> &outputs) override;

	// built-in peripherals
	void handle_timers(int cycles);

	devcb_write8      m_pb_out_cb;

	// opcode map and execution
	u16 take_irq();
	void init_ops();
	int op_cycles(u16 op);
	typedef void (upd1771c_device::*opcode_func)(u16 op);
	opcode_func m_op_funcs[65536];

	// address spacess
	address_space_config m_program_config;
	address_space_config m_sram_config;
	required_shared_ptr<u8> m_r;

    // internal state
	int m_cycles;						// cycles since reset
	PAIR m_ppc;							// previous program counter
	PAIR m_pc;							// program counter
	u8 m_a;								// Accumulator (A)
	u8 m_a_shadow;						// A' (interrupt save slot)
	u8 m_h;								// Data Pointer, 6 bits
	u8 m_sp;							// stack pointer, 3 bits
	u8 m_x;								// X Multiplier, 7 bits
	u8 m_y;								// Y Multiplier, 5 bits
	bool m_sk;							// skip flag
	bool m_sk_shadow;					// shadow skip flag (interrupt save slot)
	bool m_ts;							// Tone Sign (TS)
	bool m_ns;							// Noise Sign (NS)
	bool m_ss;							// Sample Sign (SS) = DAC out neg (-)
	u16 m_md;							// Mode flags

	u8 m_ma;							// port A input or output mask
	u8 m_mb;							// port B input or output mask
	u8 m_pa_io;							// port A,B in/out buffers
	u8 m_pb_io;

	u8 m_nc;							// "NC": Down-counter
	bool m_nc_uf;						// NC underflow
	u8 m_n;								// "N": Reload value for NC
	u8 m_nuc;							// NC relaod / tone counter, 3 bits
	bool m_int_tone_cond;				// tone int. condition
	bool m_int_tone_trig;				// tone int. trigger

	u8 m_int_pending;					// pending interrupt requests
	u8 m_int_active;					// active interrupt
	u8 m_int_clr_active;				// clear active interrupt

	sound_stream *m_stream;
	u8 m_dac_pcm;						// DAC output level
	bool m_dac_neg;						// invert DAC output (swing negative)

	memory_access<12, 1, -1, ENDIANNESS_LITTLE>::cache m_opcodes;
	memory_access<12, 1, -1, ENDIANNESS_LITTLE>::specific m_program;
	int m_icount;

	void illegal(u16 op);
	void MVI_Rr(u16 op);
	void MVI_Hp(u16 op);
	void MVI_A(u16 op);
	void MVI_H(u16 op);
	void MVI_MD1(u16 op);
	void MVI_MD0(u16 op);
	void MOV_Rr_A(u16 op);
	void MOV_A_Rr(u16 op);
	void MOV_Y_Rr(u16 op);
	void MOV_X_RG(u16 op);
	void RAR(u16 op);
	void RAL(u16 op);
	void IN_PA(u16 op);
	void IN_PB(u16 op);
	void OUT_PA(u16 op);
	void OUT_PB(u16 op);
	void OUT_DA(u16 op);
	void MUL2(u16 op);
	void MIX(u16 op);
	void TBL0_A(u16 op);
	void TBL0_X(u16 op);
	void JMP_n12(u16 op);
	void CALL(u16 op);
	void RET(u16 op);
	void RETI(u16 op);
	void NOP(u16 op);
	void ADI_Rr(u16 op);
	void ADIS_Rr(u16 op);
	void SBI_Rr(u16 op);
	void SBIS_Rr(u16 op);
	void TADINC_Rr(u16 op);
	void TADIC_Rr(u16 op);
	void TSBINC_Rr(u16 op);
	void TSBIC_Rr(u16 op);
	void ADIMS_Rr(u16 op);
	void ADI_A(u16 op);
	void ANDI_A(u16 op);
	void SBI_A(u16 op);
	void ORI_A(u16 op);
	void ADIS_A(u16 op);
	void ANDIS_A(u16 op);
	void SBIS_A(u16 op);
	void XORI_A(u16 op);
	void TADINC_A(u16 op);
	void TANDINZ_A(u16 op);
	void TSBINC_A(u16 op);
	void TSBINZ_A(u16 op);
	void TADIC_A(u16 op);
	void TANDIZ_A(u16 op);
	void TSBIC_A(u16 op);
	void TSBIZ_A(u16 op);
	void ADI_Hp(u16 op);
	void ANDI_Hp(u16 op);
	void SBI_Hp(u16 op);
	void ORI_Hp(u16 op);
	void ADIS_Hp(u16 op);
	void ANDIS_Hp(u16 op);
	void SBIS_Hp(u16 op);
	void XORI_Hp(u16 op);
	void TADINC_Hp(u16 op);
	void TANDINZ_Hp(u16 op);
	void TSBINC_Hp(u16 op);
	void TSBINZ_Hp(u16 op);
	void TADIC_Hp(u16 op);
	void TANDIZ_Hp(u16 op);
	void TSBIC_Hp(u16 op);
	void TSBIZ_Hp(u16 op);
	void ADI_H(u16 op);
	void ANDI_H(u16 op);
	void SBI_H(u16 op);
	void ORI_H(u16 op);
	void ADIS_H(u16 op);
	void ANDIS_H(u16 op);
	void SBIS_H(u16 op);
	void XORI_H(u16 op);
	void TADINC_H(u16 op);
	void TANDINZ_H(u16 op);
	void TSBINC_H(u16 op);
	void TSBINZ_H(u16 op);
	void TADIC_H(u16 op);
	void TANDIZ_H(u16 op);
	void TSBIC_H(u16 op);
	void TSBIZ_H(u16 op);
	void MOV_N_A(u16 op);
	void MOV_X_A(u16 op);
	void MOV_Hp_A(u16 op);
	void AD_A_Rr(u16 op);
	void AND_A_Rr(u16 op);
	void SB_A_Rr(u16 op);
	void OR_A_Rr(u16 op);
	void ADS_A_Rr(u16 op);
	void ANDS_A_Rr(u16 op);
	void SBS_A_Rr(u16 op);
	void XOR_A_Rr(u16 op);
	void TADNC_A_Rr(u16 op);
	void TANDNZ_A_Rr(u16 op);
	void TSBNC_A_Rr(u16 op);
	void TSBNZ_A_Rr(u16 op);
	void TADC_A_Rr(u16 op);
	void TANDZ_A_Rr(u16 op);
	void TSBC_A_Rr(u16 op);
	void TSBZ_A_Rr(u16 op);
	void AD_Rr_A(u16 op);
	void AND_Rr_A(u16 op);
	void SB_Rr_A(u16 op);
	void OR_Rr_A(u16 op);
	void ADS_Rr_A(u16 op);
	void ANDS_Rr_A(u16 op);
	void SBS_Rr_A(u16 op);
	void XOR_Rr_A(u16 op);
	void TADNC_Rr_A(u16 op);
	void TANDNZ_Rr_A(u16 op);
	void TSBNC_Rr_A(u16 op);
	void TSBNZ_Rr_A(u16 op);
	void TADC_Rr_A(u16 op);
	void TANDZ_Rr_A(u16 op);
	void TSBC_Rr_A(u16 op);
	void TSBZ_Rr_A(u16 op);
};


DECLARE_DEVICE_TYPE(UPD1771C, upd1771c_device)

#endif // MAME_CPU_UPD177X_UPD1771_H
