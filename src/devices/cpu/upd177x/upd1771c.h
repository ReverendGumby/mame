// license:BSD-3-Clause
// copyright-holders:David Hunter
#ifndef MAME_CPU_UPD177X_UPD1771_H
#define MAME_CPU_UPD177X_UPD1771_H

#pragma once


enum
{
	UPD1771C_PC=1,
};

class upd1771c_device : public cpu_device
{
public:
	// construction/destruction
	upd1771c_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

protected:
	upd1771c_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// internal memory maps
	void internal_512x16(address_map &map) ATTR_COLD;

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

	// device_disasm_interface overrides
	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

	void init_ops();
	int op_cycles(u16 op);

	typedef void (upd1771c_device::*opcode_func)(u16 op);
	opcode_func m_op_funcs[65536];

	address_space_config m_program_config;
	memory_view m_ram_view;

	u16 m_ppc;							// previous program counter
	u16 m_pc;							// program counter
	u8 m_sp;							// stack pointer
	bool m_sk;							// skip flag

	memory_access<12, 1, -1, ENDIANNESS_LITTLE>::cache m_opcodes;
	memory_access<12, 1, -1, ENDIANNESS_LITTLE>::specific m_program;
	int m_icount;

	void illegal(u16 op);
	void NOP(u16 op);
	void MVI_Rr(u16 op);
};


DECLARE_DEVICE_TYPE(UPD1771C, upd1771c_device)

#endif // MAME_CPU_UPD177X_UPD1771_H
