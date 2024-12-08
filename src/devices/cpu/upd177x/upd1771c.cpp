// license:BSD-3-Clause
// copyright-holders:David Hunter
/*****************************************************************************
 *
 *   upd1771c.cpp
 *   uPD1771C emulator
 *
 *  This work is based on
 *  - @src/devices/sound/upd1771c_017.cpp
 *  - https://oura.oguchi-rd.com - instruction reference
 *  - https://siliconpr0n.org/map/nec/d1771c-017/mcmaster_mz_mit20x/ - photomicrograph of D1771C-017 die
 *  - http://reverendgumby.gitlab.io/visuald1771c - JavaScript simulator derived from above die shot
 *  - https://gitlab.com/ReverendGumby/SuperCassetteVision_MiSTer/-/blob/main/rtl/scv/upd1771c/upd1771c.sv - Verilog emulation
 *
 *  This driver emulates 'slave' mode only.
 *
 *  TODO:
 *  - Implement all opcodes (not just those used in -017 ROM)
 *  - PNC1 (RG), PNC2 (NS), NS + Time int.
 *
 *****************************************************************************/
/*
  (TODO: Copy upd1771c_017.cpp docs here)
 */

#include "emu.h"
#include "upd1771c.h"

#include "upd177xd.h"


DEFINE_DEVICE_TYPE(UPD1771C, upd1771c_device, "upd1771c",  "NEC uPD1771C")


upd1771c_device::upd1771c_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: upd1771c_device(mconfig, UPD1771C, tag, owner, clock)
{
}

upd1771c_device::upd1771c_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: cpu_device(mconfig, type, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_pb_out_cb(*this)
	, m_program_config("program", ENDIANNESS_LITTLE, 16, 12, -1, address_map_constructor(FUNC(upd1771c_device::internal_512x16), this))
	, m_sram_config("sram", ENDIANNESS_LITTLE, 8, 6, 0, address_map_constructor(FUNC(upd1771c_device::internal_64x8), this))
	, m_r(*this, "sram") // pointer to SRAM
    , m_stream(nullptr)
{
}

void upd1771c_device::internal_512x16(address_map &map)
{
	// Internal 512-word program ROM.
    map(0x000, 0x1ff).rom();
}

void upd1771c_device::internal_64x8(address_map &map)
{
	// 64-byte internal SRAM. Supports both 8- and 16-bit accesses. Used for
	// direct Rr access, indirect (H) access, and stack (PC). 16-bit data is
	// stored little-endian.
    map(0x000, 0x3f).ram().share("sram");
}

device_memory_interface::space_config_vector upd1771c_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &m_program_config),
		std::make_pair(AS_DATA, &m_sram_config)
	};
}

std::unique_ptr<util::disasm_interface> upd1771c_device::create_disassembler()
{
	return std::make_unique<upd177x_disassembler>();
}

// Emulate external reads and writes at a high level. Simpler than modelling
// wiring CE/RD to PB6-7 and the data bus to PA.
u8 upd1771c_device::pa_r()
{
	u8 data = m_pa_io;
    //logerror("pa_r(): %02x\n", data);
    return data;
}

void upd1771c_device::pa_w(u8 data)
{
    //logerror("pa_w(%02x)\n", data);
    m_pa_io = data;
}

void upd1771c_device::handle_timers(int cycles)
{
    // NC down-counter, tone counter
    for (int i = 0; i < cycles; i++)
    {
        // NC decrementes every cycle. When NC reaches 1, it resets to N and
        // sets the underflow flag.
        bool nc_uf = false;
        if (m_nc != 1)
            m_nc --;
        else
        {
            nc_uf = true;
            m_nc = m_n;
        }

        // One cycle after the underflow flag is set, nuc (NC underflow counter)
        // goes up.
        if (m_nc_uf)
        {
            if (++m_nuc == 8)
                m_nuc = 0;
        }

        // N controls the tone interrupt condition.
        bool int_tone_cond;
        if (m_n < 0x08)
            int_tone_cond = false;
        else if (m_n < 0x10)
            int_tone_cond = m_nuc & (1 << 2);
        else if (m_n < 0x20)
            int_tone_cond = m_nuc & (1 << 1);
        else if (m_n < 0x40)
            int_tone_cond = m_nuc & (1 << 0);
        else
            int_tone_cond = m_nc_uf;

        // Interrupt is triggered one cycle after int_tone_cond negedge.
        if (m_int_tone_trig)
        {
            if ((m_md & MD_TONE_IE) && !(m_int_active & INT_TONE))
            {
                m_int_pending |= INT_TONE;
            }
        }
        m_int_tone_trig = !int_tone_cond && m_int_tone_cond;

        m_int_tone_cond = int_tone_cond;
        m_nc_uf = nc_uf;
    }
}

void upd1771c_device::device_start()
{
    init_ops();

	// large stream buffer to favour emu/sound.cpp resample quality
	m_stream = stream_alloc(0, 1, 48000 * 32);

	space(AS_PROGRAM).specific(m_program);
	space(AS_PROGRAM).cache(m_opcodes);

	state_add( UPD1771C_PC,  "PC",  m_pc.w.l ).formatstr("%03X");
	state_add( UPD1771C_A,   "A",   m_a ).formatstr("%02X");
	state_add( UPD1771C_H,   "H",   m_h ).formatstr("%02X");
	state_add( UPD1771C_SP,  "SP",  m_sp ).formatstr("%1X");
	state_add( UPD1771C_X,   "X",   m_x ).formatstr("%02X");
	state_add( UPD1771C_Y,   "Y",   m_y ).formatstr("%02X");
	state_add( UPD1771C_MD,  "MD",  m_md ).formatstr("%03X");
	state_add( UPD1771C_N,   "N",   m_n ).formatstr("%02X");
	state_add( UPD1771C_NC,  "NC",  m_nc ).formatstr("%02X");
	state_add( UPD1771C_DA,  "DA",  m_dac_pcm ).formatstr("%9s");

	state_add( STATE_GENPC, "GENPC", m_pc.w.l ).formatstr("%03X").noshow();
	state_add( STATE_GENPCBASE, "CURPC", m_ppc.w.l ).formatstr("%03X").noshow();
	state_add( STATE_GENFLAGS, "GENFLAGS", m_sk ).formatstr("%11s").noshow();

	save_item(NAME(m_ppc.w.l));
	save_item(NAME(m_pc.w.l));
	save_item(NAME(m_a));
	save_item(NAME(m_a_shadow));
	save_item(NAME(m_h));
	save_item(NAME(m_sp));
	save_item(NAME(m_x));
	save_item(NAME(m_y));
	save_item(NAME(m_sk));
	save_item(NAME(m_sk_shadow));
	save_item(NAME(m_ts));
	save_item(NAME(m_ns));
	save_item(NAME(m_ss));
	save_item(NAME(m_md));

	save_item(NAME(m_ma));
	save_item(NAME(m_mb));
	save_item(NAME(m_pa_io));
	save_item(NAME(m_pb_io));

	save_item(NAME(m_nc));
	save_item(NAME(m_nc_uf));
	save_item(NAME(m_n));
	save_item(NAME(m_nuc));
	save_item(NAME(m_int_tone_cond));
	save_item(NAME(m_int_tone_trig));

	save_item(NAME(m_int_pending));
	save_item(NAME(m_int_active));
	save_item(NAME(m_int_clr_active));

	save_item(NAME(m_dac_pcm));
	save_item(NAME(m_dac_neg));

	set_icountptr(m_icount);
}

void upd1771c_device::state_string_export(const device_state_entry &entry, std::string &str) const
{
	switch (entry.index())
	{
		case UPD1771C_DA:
			str = string_format("%c%02X",
				m_dac_neg ? '-' : '+',
				m_dac_pcm);
			break;
		case STATE_GENFLAGS:
			str = string_format("%s:%s:%s:%s",
				m_sk ? "SK":"--",
				m_ts ? "TS":"--",
				m_ns ? "NS":"--",
				m_ss ? "SS":"--");
			break;
	}
}

void upd1771c_device::device_reset()
{
    m_cycles = 0;
    m_ppc.d = 0;
    m_pc.d = 0;
    m_sp = 0;
    m_sk = false;
    m_ns = false;
    m_md = MD_IF;

    m_ma = 0xff;                    // PA[7:0] are inputs
    m_mb = 0xf8;                    // PB[7:3] are inputs, PB[2:0] are outputs
    m_pa_io = 0;
    m_pb_io = 0;

    m_nc = 0;
    m_nc_uf = false;
    m_nuc = 0;
    m_int_tone_cond = false;
    m_int_tone_trig = false;

    m_int_pending = 0;
    m_int_active = 0;
    m_int_clr_active = 0;

    m_dac_pcm = 0;
    m_dac_neg = false;
}

void upd1771c_device::execute_run()
{
    do
    {
		u16 irq_vec = take_irq();

		m_ppc = m_pc;
		debugger_instruction_hook(m_pc.w.l);

        u16 op = m_opcodes.read_word(m_pc.w.l);
        m_pc.d ++;

        if (irq_vec)
        {
            op = 0x7000 | irq_vec;		// CALL opcode
            irq_vec = 0;
            m_sk = false;
            m_pc.d --;                  // RETI to the ins. replaced by CALL
        }
		/* skip flag set? */
        if (m_sk)
        {
            op = 0x0000; // NOP
            m_sk = false;
        }
        int cc = op_cycles(op);
        handle_timers(cc);
        (this->*m_op_funcs[op])(op);

        m_icount -= cc;
        m_cycles += cc;
	} while (m_icount > 0);
}

u16 upd1771c_device::take_irq()
{
    // Clear active interrupt in the cycle after RETI.
    m_int_active &= ~m_int_clr_active;
    m_int_clr_active = 0;

    if (!m_int_pending)
        return 0;						// no pending interrupt

    // Select highest priority pending interrupt, make it active
    if (m_int_pending & INT_TONE)
        m_int_active = INT_TONE;
    else if (m_int_pending & INT_NS)
        m_int_active = INT_NS;
    else if (m_int_pending & INT_EXT)
        m_int_active = INT_EXT;
    else if (m_int_pending & INT_TIME)
        m_int_active = INT_TIME;
    m_int_pending &= ~m_int_active;     // clear pending request

    // Compute interrupt vector
	u16 irq_vec = 0;
    switch (m_int_active)
    {
    case INT_TONE:
        irq_vec = 0x20;
        if (m_n >= 0x10 && m_n < 0x20)
            irq_vec |= 0x04;
        if (m_n >= 0x20 && m_n < 0x40)
            irq_vec |= 0x08;
        if (m_n >= 0x40)
            irq_vec |= 0x0c;
        break;
    case INT_NS:
        irq_vec = 0x48;
        break;
    case INT_EXT:
        irq_vec = 0x60;
        break;
    case INT_TIME:
        irq_vec = 0x80;
        break;
    }

    // Copy accumulator and skip flag to shadow registers
    m_sk_shadow = m_sk;
    m_a_shadow = m_a;

    // The next instruction is still fetched, but effectively replaced with a
    // CALL to the interrupt vector.
	return irq_vec;
}

/***********************************************************************
 * sound stream updates
 ***********************************************************************/

void upd1771c_device::sound_stream_update(sound_stream &stream, std::vector<read_stream_view> const &inputs, std::vector<write_stream_view> &outputs)
{
    stream_buffer::sample_t curval;
    curval = m_dac_pcm / 255.0;
    if (m_dac_neg)
        curval = -curval;

	auto &out = outputs[0];
    out.fill(curval);
}

/***********************************************************************
 * opcodes
 ***********************************************************************/

void upd1771c_device::init_ops()
{
    // Note: Only opcodes used in ROM mask -017 are implemented.
    static const struct {
        u16 code;
        u16 mask;
        opcode_func fn;
    } instructions[] {
        { 0x4000, 0xe000, &upd1771c_device::MVI_Rr },

        { 0x3200, 0xff00, &upd1771c_device::MVI_Hp },
        { 0x3400, 0xff00, &upd1771c_device::MVI_A },
        { 0x3800, 0xffc0, &upd1771c_device::MVI_H },
        { 0x3100, 0xff1f, &upd1771c_device::MVI_MD1 },
        { 0x2100, 0xff80, &upd1771c_device::MVI_MD0 },

        { 0x1201, 0xfe0f, &upd1771c_device::MOV_Rr_A },
        { 0x1005, 0xfe0f, &upd1771c_device::MOV_A_Rr },
//      { 0x1205, 0xfe0f, &upd1771c_device::XCHG_Rr_A },
//      { 0x1202, 0xfe0f, &upd1771c_device::MOV_Rr_Hp },
//      { 0x100a, 0xfe0f, &upd1771c_device::MOV_Hp_Rr },
//      { 0x120a, 0xfe0f, &upd1771c_device::XCHG_Rr_A },
        { 0x1000, 0xfe0f, &upd1771c_device::MOV_Y_Rr },
        { 0x0008, 0xffff, &upd1771c_device::MOV_X_RG },

        { 0x0404, 0xffff, &upd1771c_device::RAR },
        { 0x0408, 0xffff, &upd1771c_device::RAL },

        { 0x0401, 0xffff, &upd1771c_device::IN_PA },
        { 0x0402, 0xffff, &upd1771c_device::IN_PB },
        { 0x0002, 0xffff, &upd1771c_device::OUT_PA },
        { 0x0004, 0xffff, &upd1771c_device::OUT_PB },
        { 0x0502, 0xffff, &upd1771c_device::OUT_DA },

//      { 0x0504, 0xffff, &upd1771c_device::MUL1 },
        { 0x050c, 0xffff, &upd1771c_device::MUL2 },

        { 0x1409, 0xfe0f, &upd1771c_device::MIX },

        { 0x1801, 0xfe0f, &upd1771c_device::TBL0_A },
        { 0x1802, 0xfe0f, &upd1771c_device::TBL0_X },
//      { 0x1804, 0xfe0f, &upd1771c_device::TBL0_Y },
//      { 0x1808, 0xfe0f, &upd1771c_device::CALL0 },

        { 0x6000, 0xf000, &upd1771c_device::JMP_n12 },
//      { 0x2000, 0xff0f, &upd1771c_device::JMP_n4 },
//      { 0x0501, 0xffff, &upd1771c_device::JMPA },
//      { 0x2400, 0xff00, &upd1771c_device::JMPFZ },

        { 0x7000, 0xf000, &upd1771c_device::CALL },

        { 0x0800, 0xffff, &upd1771c_device::RET },
//      { 0x0801, 0xffff, &upd1771c_device::RETS },
        { 0x090f, 0xffff, &upd1771c_device::RETI },

//      { 0x0005, 0xffff, &upd1771c_device::STF },
//      { 0x0602, 0xffff, &upd1771c_device::OFF },
        { 0x0000, 0xffff, &upd1771c_device::NOP },

        { 0xe000, 0xfe00, &upd1771c_device::ADI_Rr },
        { 0xe200, 0xfe00, &upd1771c_device::ADIS_Rr },
        { 0xe400, 0xfe00, &upd1771c_device::SBI_Rr },
        { 0xe600, 0xfe00, &upd1771c_device::SBIS_Rr },
        { 0xe800, 0xfe00, &upd1771c_device::TADINC_Rr },
        { 0xea00, 0xfe00, &upd1771c_device::TADIC_Rr },
        { 0xec00, 0xfe00, &upd1771c_device::TSBINC_Rr },
        { 0xee00, 0xfe00, &upd1771c_device::TSBIC_Rr },
//      { 0xf000, 0xfe00, &upd1771c_device::ADI5_Rr },
        { 0xf200, 0xfe00, &upd1771c_device::ADIMS_Rr },
//      { 0xf800, 0xfe00, &upd1771c_device::TADI5_Rr },

        { 0x8000, 0xff00, &upd1771c_device::ADI_A },
        { 0x8200, 0xff00, &upd1771c_device::ANDI_A },
        { 0x8400, 0xff00, &upd1771c_device::SBI_A },
        { 0x8600, 0xff00, &upd1771c_device::ORI_A },
        { 0x8800, 0xff00, &upd1771c_device::ADIS_A },
        { 0x8a00, 0xff00, &upd1771c_device::ANDIS_A },
        { 0x8c00, 0xff00, &upd1771c_device::SBIS_A },
        { 0x8e00, 0xff00, &upd1771c_device::XORI_A },
        { 0x9000, 0xff00, &upd1771c_device::TADINC_A },
        { 0x9200, 0xff00, &upd1771c_device::TANDINZ_A },
        { 0x9400, 0xff00, &upd1771c_device::TSBINC_A },
        { 0x9600, 0xff00, &upd1771c_device::TSBINZ_A },
        { 0x9800, 0xff00, &upd1771c_device::TADIC_A },
        { 0x9a00, 0xff00, &upd1771c_device::TANDIZ_A },
        { 0x9c00, 0xff00, &upd1771c_device::TSBIC_A },
        { 0x9e00, 0xff00, &upd1771c_device::TSBIZ_A },

//      { 0x8100, 0xff1f, &upd1771c_device::ADI_MD1 },
//      { 0x8300, 0xff1f, &upd1771c_device::ANDI_MD1 },
//      { 0x8500, 0xff1f, &upd1771c_device::SBI_MD1 },
//      { 0x8700, 0xff1f, &upd1771c_device::ORI_MD1 },
//      { 0x8900, 0xff1f, &upd1771c_device::ADIS_MD1 },
//      { 0x8b00, 0xff1f, &upd1771c_device::ANDIS_MD1 },
//      { 0x8d00, 0xff1f, &upd1771c_device::SBIS_MD1 },
//      { 0x8f00, 0xff1f, &upd1771c_device::XORI_MD1 },
//      { 0x9100, 0xff00, &upd1771c_device::TADINC_MD },
//      { 0x9300, 0xff00, &upd1771c_device::TANDINZ_MD },
//      { 0x9500, 0xff00, &upd1771c_device::TSBINC_MD },
//      { 0x9700, 0xff00, &upd1771c_device::TSBINZ_MD },
//      { 0x9900, 0xff00, &upd1771c_device::TADIC_MD },
//      { 0x9b00, 0xff00, &upd1771c_device::TANDIZ_MD },
//      { 0x9d00, 0xff00, &upd1771c_device::TSBIC_MD },
//      { 0x9f00, 0xff00, &upd1771c_device::TSBIZ_MD },

        { 0xa000, 0xff00, &upd1771c_device::ADI_Hp },
        { 0xa200, 0xff00, &upd1771c_device::ANDI_Hp },
        { 0xa400, 0xff00, &upd1771c_device::SBI_Hp },
        { 0xa600, 0xff00, &upd1771c_device::ORI_Hp },
        { 0xa800, 0xff00, &upd1771c_device::ADIS_Hp },
        { 0xaa00, 0xff00, &upd1771c_device::ANDIS_Hp },
        { 0xac00, 0xff00, &upd1771c_device::SBIS_Hp },
        { 0xae00, 0xff00, &upd1771c_device::XORI_Hp },
        { 0xb000, 0xff00, &upd1771c_device::TADINC_Hp },
        { 0xb200, 0xff00, &upd1771c_device::TANDINZ_Hp },
        { 0xb400, 0xff00, &upd1771c_device::TSBINC_Hp },
        { 0xb600, 0xff00, &upd1771c_device::TSBINZ_Hp },
        { 0xb800, 0xff00, &upd1771c_device::TADIC_Hp },
        { 0xba00, 0xff00, &upd1771c_device::TANDIZ_Hp },
        { 0xbc00, 0xff00, &upd1771c_device::TSBIC_Hp },
        { 0xbe00, 0xff00, &upd1771c_device::TSBIZ_Hp },

        { 0xa100, 0xffc0, &upd1771c_device::ADI_H },
        { 0xa300, 0xffc0, &upd1771c_device::ANDI_H },
        { 0xa500, 0xffc0, &upd1771c_device::SBI_H },
        { 0xa700, 0xffc0, &upd1771c_device::ORI_H },
        { 0xa900, 0xffc0, &upd1771c_device::ADIS_H },
        { 0xabc0, 0xffc0, &upd1771c_device::ANDIS_H },
        { 0xad00, 0xffc0, &upd1771c_device::SBIS_H },
        { 0xaf00, 0xffc0, &upd1771c_device::XORI_H },
        { 0xb1c0, 0xffc0, &upd1771c_device::TADINC_H },
        { 0xb300, 0xffc0, &upd1771c_device::TANDINZ_H },
        { 0xb500, 0xffc0, &upd1771c_device::TSBINC_H },
        { 0xb700, 0xffc0, &upd1771c_device::TSBINZ_H },
        { 0xb9c0, 0xffc0, &upd1771c_device::TADIC_H },
        { 0xbb00, 0xffc0, &upd1771c_device::TANDIZ_H },
        { 0xbd00, 0xffc0, &upd1771c_device::TSBIC_H },
        { 0xbf00, 0xffc0, &upd1771c_device::TSBIZ_H },

        { 0x0201, 0xffff, &upd1771c_device::MOV_N_A },
        { 0x0208, 0xffff, &upd1771c_device::MOV_X_A },
        { 0x1601, 0xffff, &upd1771c_device::MOV_Hp_A },
//      { 0x1405, 0xffff, &upd1771c_device::MOV_A_Hp },
//      { 0x1605, 0xffff, &upd1771c_device::XCHG_Hp_A },

        { 0xc000, 0xfe0f, &upd1771c_device::AD_A_Rr },
        { 0xc200, 0xfe0f, &upd1771c_device::AND_A_Rr },
        { 0xc400, 0xfe0f, &upd1771c_device::SB_A_Rr },
        { 0xc600, 0xfe0f, &upd1771c_device::OR_A_Rr },
        { 0xc800, 0xfe0f, &upd1771c_device::ADS_A_Rr },
        { 0xca00, 0xfe0f, &upd1771c_device::ANDS_A_Rr },
        { 0xcc00, 0xfe0f, &upd1771c_device::SBS_A_Rr },
        { 0xce00, 0xfe0f, &upd1771c_device::XOR_A_Rr },
        { 0xd000, 0xfe0f, &upd1771c_device::TADNC_A_Rr },
        { 0xd200, 0xfe0f, &upd1771c_device::TANDNZ_A_Rr },
        { 0xd400, 0xfe0f, &upd1771c_device::TSBNC_A_Rr },
        { 0xd600, 0xfe0f, &upd1771c_device::TSBNZ_A_Rr },
        { 0xd800, 0xfe0f, &upd1771c_device::TADC_A_Rr },
        { 0xda00, 0xfe0f, &upd1771c_device::TANDZ_A_Rr },
        { 0xdc00, 0xfe0f, &upd1771c_device::TSBC_A_Rr },
        { 0xde00, 0xfe0f, &upd1771c_device::TSBZ_A_Rr },

        { 0xc008, 0xfe0f, &upd1771c_device::AD_Rr_A },
        { 0xc208, 0xfe0f, &upd1771c_device::AND_Rr_A },
        { 0xc408, 0xfe0f, &upd1771c_device::SB_Rr_A },
        { 0xc608, 0xfe0f, &upd1771c_device::OR_Rr_A },
        { 0xc808, 0xfe0f, &upd1771c_device::ADS_Rr_A },
        { 0xca08, 0xfe0f, &upd1771c_device::ANDS_Rr_A },
        { 0xcc08, 0xfe0f, &upd1771c_device::SBS_Rr_A },
        { 0xce08, 0xfe0f, &upd1771c_device::XOR_Rr_A },
        { 0xd008, 0xfe0f, &upd1771c_device::TADNC_Rr_A },
        { 0xd208, 0xfe0f, &upd1771c_device::TANDNZ_Rr_A },
        { 0xd408, 0xfe0f, &upd1771c_device::TSBNC_Rr_A },
        { 0xd608, 0xfe0f, &upd1771c_device::TSBNZ_Rr_A },
        { 0xd808, 0xfe0f, &upd1771c_device::TADC_Rr_A },
        { 0xda08, 0xfe0f, &upd1771c_device::TANDZ_Rr_A },
        { 0xdc08, 0xfe0f, &upd1771c_device::TSBC_Rr_A },
        { 0xde08, 0xfe0f, &upd1771c_device::TSBZ_Rr_A },

//      { 0xc001, 0xffff, &upd1771c_device::AD_A_Hp },
//      { 0xc201, 0xffff, &upd1771c_device::AND_A_Hp },
//      { 0xc401, 0xffff, &upd1771c_device::SB_A_Hp },
//      { 0xc601, 0xffff, &upd1771c_device::OR_A_Hp },
//      { 0xc801, 0xffff, &upd1771c_device::ADS_A_Hp },
//      { 0xca01, 0xffff, &upd1771c_device::ANDS_A_Hp },
//      { 0xcc01, 0xffff, &upd1771c_device::SBS_A_Hp },
//      { 0xce01, 0xffff, &upd1771c_device::XOR_A_Hp },
//      { 0xd001, 0xffff, &upd1771c_device::TADNC_A_Hp },
//      { 0xd201, 0xffff, &upd1771c_device::TANDNZ_A_Hp },
//      { 0xd401, 0xffff, &upd1771c_device::TSBNC_A_Hp },
//      { 0xd601, 0xffff, &upd1771c_device::TSBNZ_A_Hp },
//      { 0xd801, 0xffff, &upd1771c_device::TADC_A_Hp },
//      { 0xda01, 0xffff, &upd1771c_device::TANDZ_A_Hp },
//      { 0xdc01, 0xffff, &upd1771c_device::TSBC_A_Hp },
//      { 0xde01, 0xffff, &upd1771c_device::TSBZ_A_Hp },

//      { 0xc009, 0xffff, &upd1771c_device::AD_Hp_A },
//      { 0xc209, 0xffff, &upd1771c_device::AND_Hp_A },
//      { 0xc409, 0xffff, &upd1771c_device::SB_Hp_A },
//      { 0xc609, 0xffff, &upd1771c_device::OR_Hp_A },
//      { 0xc809, 0xffff, &upd1771c_device::ADS_Hp_A },
//      { 0xca09, 0xffff, &upd1771c_device::ANDS_Hp_A },
//      { 0xcc09, 0xffff, &upd1771c_device::SBS_Hp_A },
//      { 0xce09, 0xffff, &upd1771c_device::XOR_Hp_A },
//      { 0xd009, 0xffff, &upd1771c_device::TADNC_Hp_A },
//      { 0xd209, 0xffff, &upd1771c_device::TANDNZ_Hp_A },
//      { 0xd409, 0xffff, &upd1771c_device::TSBNC_Hp_A },
//      { 0xd609, 0xffff, &upd1771c_device::TSBNZ_Hp_A },
//      { 0xd809, 0xffff, &upd1771c_device::TADC_Hp_A },
//      { 0xda09, 0xffff, &upd1771c_device::TANDZ_Hp_A },
//      { 0xdc09, 0xffff, &upd1771c_device::TSBC_Hp_A },
//      { 0xde09, 0xffff, &upd1771c_device::TSBZ_Hp_A },

//      { 0x1a01, 0xfe0f, &upd1771c_device::TBL1_A },
//      { 0x1a02, 0xfe0f, &upd1771c_device::TBL1_X },
//      { 0x1a04, 0xfe0f, &upd1771c_device::TBL1_Y },
//      { 0x1a08, 0xfe0f, &upd1771c_device::CALL1 },

//      { 0x0101, 0xffff, &upd1771c_device::MON },
    };

    for (int op = 0x0000; op < 65536; op++)
    {
        m_op_funcs[op] = &upd1771c_device::illegal;

        for (const auto &i : instructions)
        {
            if ((op & i.mask) == i.code)
            {
                m_op_funcs[op] = i.fn;
                break;
            }
        }
    }
}

int upd1771c_device::op_cycles(u16 op)
{
    if ((op & 0xfc00) == 0x1800)                // TBL0/1, CALL0/1
        return 2;
    return 1;
}

void upd1771c_device::illegal(u16 op)
{
	logerror("illegal opcode %04x at PC:%03x\n", op, m_ppc.w.l);
}

void upd1771c_device::MVI_Rr(u16 op)    // 010r rrrr nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u8 r = (op >> 8) & 0x1f;
    m_r[r] = n;
}

void upd1771c_device::MVI_Hp(u16 op)    // 0011 0010 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    m_r[m_h] = n;
}

void upd1771c_device::MVI_A(u16 op)     // 0011 0100 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    m_a = n;
}

void upd1771c_device::MVI_H(u16 op)     // 0011 1000 00nn nnnn
{
    u8 n = (op >> 0) & 0xff;
    m_h = n;
}

void upd1771c_device::MVI_MD1(u16 op)   // 0011 0001 nnn0 0000
{
    // Sets MD5-7
    m_md = (m_md & ~0x0e0) | (op & 0x0e0);
}

void upd1771c_device::MVI_MD0(u16 op)   // 0010 0001 0nnn nnnn
{
    // Sets MD0-4,8,9
    m_md = (m_md & ~0x31f) | (op & 0x01f) | ((op & 0x060) << 3);
}

void upd1771c_device::MOV_Rr_A(u16 op)  // 0001 001r rrrr 0001
{
    u8 r = (op >> 4) & 0x1f;
    m_r[r] = m_a;
}

void upd1771c_device::MOV_A_Rr(u16 op)  // 0001 000r rrrr 0101
{
    u8 r = (op >> 4) & 0x1f;
    m_a = m_r[r];
}

void upd1771c_device::MOV_Y_Rr(u16 op)  // 0001 000r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    m_y = m_r[r] & 0x1f;
}

void upd1771c_device::MOV_X_RG(u16 op)  // 0000 0000 0000 1000
{
    // TODO: Implement RG
}

void upd1771c_device::RAR(u16 op)       // 0000 0100 0000 0100
{
    m_a = (m_a >> 1) | ((m_a << 7) & 0x80);
}

void upd1771c_device::RAL(u16 op)       // 0000 0100 0000 1000
{
    m_a = (m_a << 1) | ((m_a >> 7) & 0x01);
}

void upd1771c_device::IN_PA(u16 op)     // 0000 0100 0000 0001
{
    m_a = m_pa_io;
}

void upd1771c_device::IN_PB(u16 op)     // 0000 0100 0000 0010
{
    m_a = m_pb_io;
}

void upd1771c_device::OUT_PA(u16 op)    // 0000 0000 0000 0010
{
    u8 data = m_a;
    //logerror("OUT_PA(%02x)\n", data);
    m_pa_io = data;
}

void upd1771c_device::OUT_PB(u16 op)    // 0000 0000 0000 0100
{
    u8 data = m_a;
    m_pb_io = data;
    data = (data & ~m_mb) | m_mb;
    m_pb_out_cb(data);
    //logerror("OUT_PB: %02x\n", data);
}

void upd1771c_device::OUT_DA(u16 op)    // 0000 0101 0000 0010
{
    m_stream->update();

    m_dac_pcm = m_a;
    if (m_ss ^ m_ts)                    // invert sample input to DAC
        m_dac_pcm = ~m_dac_pcm;
    m_dac_neg = m_ss;                   // DAC out neg (-)
    //logerror("OUT_DA: %c%02x\n", m_dac_neg ? '-' : '+', m_dac_pcm);
}

void upd1771c_device::MUL2(u16 op)      // 0000 0101 0000 1100
{
    if (m_y & 1)
    {
        m_a += m_x;
    }
    m_a >>= 1;
    m_y >>= 1;
}

void upd1771c_device::MIX(u16 op)       // 0001 010r rrrr 1001
{
    u8 r = (op >> 4) & 0x1f;
    u8 rr = m_r[r];
    u16 tmp = (m_ts == m_ns) ? rr + m_a : rr - m_a;
    m_ss = m_ts ^ (tmp > 0xff);
    m_a = tmp;
}

void upd1771c_device::TBL0_A(u16 op)    // 0001 100r rrrr 0010
{
    u8 r = (op >> 4) & 0x1f;
    PAIR prr;
    prr.b.l = m_r[r + 0];
    prr.b.h = m_r[r + 1];
    PAIR pw;
    pw.d = m_program.read_word(prr.w.l >> 1);
    u8 data = (prr.w.l & 1) ? pw.b.h : pw.b.l;
    m_a = data;
}

void upd1771c_device::TBL0_X(u16 op)    // 0001 100r rrrr 0001
{
    u8 r = (op >> 4) & 0x1f;
    PAIR prr;
    prr.b.l = m_r[r + 0];
    prr.b.h = m_r[r + 1];
    PAIR pw;
    pw.d = m_program.read_word(prr.w.l >> 1);
    u8 data = (prr.w.l & 1) ? pw.b.h : pw.b.l;
    m_x = data & 0x7f;
    m_ts = data & 0x80;
}

void upd1771c_device::JMP_n12(u16 op)   // 0110 nnnn nnnn nnnn
{
    u16 n = (op >> 0) & 0xfff;
    m_pc.w.l = n;
}

void upd1771c_device::CALL(u16 op)      // 0111 nnnn nnnn nnnn
{
    u8 rs = m_sp * 2 + 0x20;
    m_r[rs + 0] = m_pc.b.l;
    m_r[rs + 1] = m_pc.b.h;
    m_sp ++;

    u16 n = (op >> 0) & 0xfff;
    m_pc.w.l = n;
}

void upd1771c_device::RET(u16 op)       // 0000 1000 0000 0000
{
    m_sp --;
    u8 rs = m_sp * 2 + 0x20;
    m_pc.b.l = m_r[rs + 0];
    m_pc.b.h = m_r[rs + 1];
}

void upd1771c_device::RETI(u16 op)      // 0000 1001 0000 1111
{
    RET(op);
    m_sk = m_sk_shadow;
    m_a = m_a_shadow;
    m_int_clr_active = m_int_active;
}

void upd1771c_device::ADI_Rr(u16 op)    // 1110 000r rrrr nnnn
{
    u8 n = (op >> 0) & 0x0f;
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] + n;
    m_r[r] = tmp;
}

void upd1771c_device::ADIS_Rr(u16 op)   // 1110 001r rrrr nnnn
{
    u8 n = (op >> 0) & 0x0f;
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] + n;
    m_r[r] = tmp;
    m_sk = tmp > 0xff;
}

void upd1771c_device::SBI_Rr(u16 op)    // 1110 010r rrrr nnnn
{
    u8 n = (op >> 0) & 0x0f;
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] - n;
    m_r[r] = tmp;
}

void upd1771c_device::SBIS_Rr(u16 op)   // 1110 011r rrrr nnnn
{
    u8 n = (op >> 0) & 0x0f;
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] - n;
    m_r[r] = tmp;
    m_sk = tmp > 0xff;
}

void upd1771c_device::TADINC_Rr(u16 op) // 1110 100r rrrr nnnn
{
    u8 n = (op >> 0) & 0x0f;
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] + n;
    m_sk = tmp <= 0xff;
}

void upd1771c_device::TADIC_Rr(u16 op)  // 1110 101r rrrr nnnn
{
    u8 n = (op >> 0) & 0x0f;
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] + n;
    m_sk = tmp > 0xff;
}

void upd1771c_device::TSBINC_Rr(u16 op) // 1110 110r rrrr nnnn
{
    u8 n = (op >> 0) & 0x0f;
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] - n;
    m_sk = tmp <= 0xff;
}

void upd1771c_device::TSBIC_Rr(u16 op)  // 1110 111r rrrr nnnn
{
    u8 n = (op >> 0) & 0x0f;
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] - n;
    m_sk = tmp > 0xff;
}

void upd1771c_device::ADIMS_Rr(u16 op)  // 1111 001r rrrr nnnn
{
    u8 n = (op >> 0) & 0x0f;
    u8 r = (op >> 4) & 0x1f;
    u8 tmp;
    if (m_md & MD_64_32)
    {
        tmp = (m_r[r] & 0x3f) + (n & 0x3f);   // Sum bits 0-5
        m_sk = tmp > 0x3f;                    // Test if 6th carry C6
        tmp = (tmp & 0x3f) | (m_r[r] & 0xc0); // Copy bits 6-7
    }
    else
    {
        tmp = (m_r[r] & 0x1f) + (n & 0x1f);   // Sum bits 0-4
        m_sk = tmp > 0x1f;                    // Test if 5th carry C5
        tmp = (tmp & 0x1f) | (m_r[r] & 0xe0); // Copy bits 5-7
    }
    m_r[r] = tmp;
}

void upd1771c_device::ADI_A(u16 op)     // 1000 0000 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a + n;
    m_a = tmp;
}

void upd1771c_device::ANDI_A(u16 op)    // 1000 0010 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a & n;
    m_a = tmp;
}

void upd1771c_device::SBI_A(u16 op)     // 1000 0100 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a - n;
    m_a = tmp;
}

void upd1771c_device::ORI_A(u16 op)     // 1000 0110 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a | n;
    m_a = tmp;
}

void upd1771c_device::ADIS_A(u16 op)    // 1000 1000 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a + n;
    m_a = tmp;
    m_sk = tmp > 0xff;
}

void upd1771c_device::ANDIS_A(u16 op)   // 1000 1010 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a & n;
    m_a = tmp;
    m_sk = tmp == 0;
}

void upd1771c_device::SBIS_A(u16 op)    // 1000 1100 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a - n;
    m_a = tmp;
    m_sk = tmp > 0xff;
}

void upd1771c_device::XORI_A(u16 op)    // 1000 1110 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a ^ n;
    m_a = tmp;
}

void upd1771c_device::TADINC_A(u16 op)  // 1001 0000 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a + n;
    m_sk = tmp <= 0xff;
}

void upd1771c_device::TANDINZ_A(u16 op) // 1001 0010 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a & n;
    m_sk = tmp != 0;
}

void upd1771c_device::TSBINC_A(u16 op)  // 1001 0100 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a - n;
    m_sk = tmp <= 0xff;
}

void upd1771c_device::TSBINZ_A(u16 op)  // 1001 0110 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a - n;
    m_sk = tmp != 0;
}

void upd1771c_device::TADIC_A(u16 op)   // 1001 1000 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a + n;
    m_sk = tmp > 0xff;
}

void upd1771c_device::TANDIZ_A(u16 op)  // 1001 1010 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a & n;
    m_sk = tmp == 0;
}

void upd1771c_device::TSBIC_A(u16 op)   // 1001 1100 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a - n;
    m_sk = tmp > 0xff;
}

void upd1771c_device::TSBIZ_A(u16 op)   // 1001 1110 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_a - n;
    m_sk = tmp == 0;
}

void upd1771c_device::ADI_Hp(u16 op)    // 1010 0000 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] + n;
    m_r[m_h] = tmp;
}

void upd1771c_device::ANDI_Hp(u16 op)   // 1010 0010 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] & n;
    m_r[m_h] = tmp;
}

void upd1771c_device::SBI_Hp(u16 op)    // 1010 0100 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] - n;
    m_r[m_h] = tmp;
}

void upd1771c_device::ORI_Hp(u16 op)    // 1010 0110 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] | n;
    m_r[m_h] = tmp;
}

void upd1771c_device::ADIS_Hp(u16 op)   // 1010 1000 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] + n;
    m_r[m_h] = tmp;
    m_sk = tmp > 0xff;
}

void upd1771c_device::ANDIS_Hp(u16 op)  // 1010 1010 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] & n;
    m_r[m_h] = tmp;
    m_sk = tmp == 0;
}

void upd1771c_device::SBIS_Hp(u16 op)   // 1010 1100 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] - n;
    m_r[m_h] = tmp;
    m_sk = tmp > 0xff;
}

void upd1771c_device::XORI_Hp(u16 op)   // 1010 1110 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] ^ n;
    m_r[m_h] = tmp;
}

void upd1771c_device::TADINC_Hp(u16 op) // 1011 0000 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] + n;
    m_sk = tmp <= 0xff;
}

void upd1771c_device::TANDINZ_Hp(u16 op) // 1011 0010 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] & n;
    m_sk = tmp != 0;
}

void upd1771c_device::TSBINC_Hp(u16 op) // 1011 0100 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] - n;
    m_sk = tmp <= 0xff;
}

void upd1771c_device::TSBINZ_Hp(u16 op) // 1011 0110 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] - n;
    m_sk = tmp != 0;
}

void upd1771c_device::TADIC_Hp(u16 op)  // 1011 1000 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] + n;
    m_sk = tmp > 0xff;
}

void upd1771c_device::TANDIZ_Hp(u16 op) // 1011 1010 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] & n;
    m_sk = tmp == 0;
}

void upd1771c_device::TSBIC_Hp(u16 op)  // 1011 1100 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] - n;
    m_sk = tmp > 0xff;
}

void upd1771c_device::TSBIZ_Hp(u16 op)  // 1011 1110 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_r[m_h] - n;
    m_sk = tmp == 0;
}

void upd1771c_device::ADI_H(u16 op)     // 1010 0001 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h + n;
    m_h = tmp;
}

void upd1771c_device::ANDI_H(u16 op)    // 1010 0011 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h & n;
    m_h = tmp;
}

void upd1771c_device::SBI_H(u16 op)     // 1010 0101 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h - n;
    m_h = tmp;
}

void upd1771c_device::ORI_H(u16 op)     // 1010 0111 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h | n;
    m_h = tmp;
}

void upd1771c_device::ADIS_H(u16 op)    // 1010 1001 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h + n;
    m_h = tmp;
    m_sk = tmp > 0xff;
}

void upd1771c_device::ANDIS_H(u16 op)   // 1010 1011 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h & n;
    m_h = tmp;
    m_sk = tmp == 0;
}

void upd1771c_device::SBIS_H(u16 op)    // 1010 1101 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h - n;
    m_h = tmp;
    m_sk = tmp > 0xff;
}

void upd1771c_device::XORI_H(u16 op)    // 1010 1111 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h ^ n;
    m_h = tmp;
}

void upd1771c_device::TADINC_H(u16 op)  // 1010 0001 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h + n;
    m_sk = tmp <= 0xff;
}

void upd1771c_device::TANDINZ_H(u16 op) // 1011 0011 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h & n;
    m_sk = tmp != 0;
}

void upd1771c_device::TSBINC_H(u16 op)  // 1011 0101 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h - n;
    m_sk = tmp <= 0xff;
}

void upd1771c_device::TSBINZ_H(u16 op)  // 1011 0111 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h - n;
    m_sk = tmp != 0;
}

void upd1771c_device::TADIC_H(u16 op)   // 1011 1001 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h + n;
    m_sk = tmp > 0xff;
}

void upd1771c_device::TANDIZ_H(u16 op)  // 1011 1011 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h & n;
    m_sk = tmp == 0;
}

void upd1771c_device::TSBIC_H(u16 op)   // 1011 1101 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h - n;
    m_sk = tmp > 0xff;
}

void upd1771c_device::TSBIZ_H(u16 op)   // 1011 1111 nnnn nnnn
{
    u8 n = (op >> 0) & 0xff;
    u16 tmp = m_h - n;
    m_sk = tmp == 0;
}
void upd1771c_device::NOP(u16 op)       // 0000 0000 0000 0000
{
}

void upd1771c_device::MOV_N_A(u16 op)   // 0000 0010 0000 0001
{
    m_n = m_a;
}

void upd1771c_device::MOV_X_A(u16 op)   // 0000 0010 0000 1000
{
    m_x = m_a & 0x7f;
}

void upd1771c_device::MOV_Hp_A(u16 op)  // 0001 0110 0000 0001
{
    m_r[m_h] = m_a;
}

void upd1771c_device::AD_A_Rr(u16 op)   // 1100 000r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a + m_r[r];
    m_a = tmp;
}

void upd1771c_device::AND_A_Rr(u16 op)  // 1100 001r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a & m_r[r];
    m_a = tmp;
}

void upd1771c_device::SB_A_Rr(u16 op)   // 1100 010r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a - m_r[r];
    m_a = tmp;
}

void upd1771c_device::OR_A_Rr(u16 op)   // 1100 011r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a | m_r[r];
    m_a = tmp;
}

void upd1771c_device::ADS_A_Rr(u16 op)  // 1100 100r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a + m_r[r];
    m_a = tmp;
    m_sk = tmp > 0xff;
}

void upd1771c_device::ANDS_A_Rr(u16 op) // 1100 101r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a & m_r[r];
    m_a = tmp;
    m_sk = tmp == 0;
}

void upd1771c_device::SBS_A_Rr(u16 op)  // 1100 110r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a - m_r[r];
    m_a = tmp;
    m_sk = tmp > 0xff;
}

void upd1771c_device::XOR_A_Rr(u16 op)  // 1100 111r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a ^ m_r[r];
    m_a = tmp;
}

void upd1771c_device::TADNC_A_Rr(u16 op) // 1101 000r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a + m_r[r];
    m_sk = tmp <= 0xff;
}

void upd1771c_device::TANDNZ_A_Rr(u16 op) // 1101 001r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a & m_r[r];
    m_sk = tmp != 0;
}

void upd1771c_device::TSBNC_A_Rr(u16 op) // 1101 010r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a - m_r[r];
    m_sk = tmp <= 0xff;
}

void upd1771c_device::TSBNZ_A_Rr(u16 op) // 1101 011r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a - m_r[r];
    m_sk = tmp != 0;
}

void upd1771c_device::TADC_A_Rr(u16 op) // 1101 100r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a + m_r[r];
    m_sk = tmp > 0xff;
}

void upd1771c_device::TANDZ_A_Rr(u16 op) // 1101 101r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a & m_r[r];
    m_sk = tmp == 0;
}

void upd1771c_device::TSBC_A_Rr(u16 op) // 1101 110r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a - m_r[r];
    m_sk = tmp > 0xff;
}

void upd1771c_device::TSBZ_A_Rr(u16 op) // 1101 111r rrrr 0000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_a - m_r[r];
    m_sk = tmp == 0;
}

void upd1771c_device::AD_Rr_A(u16 op)   // 1100 000r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] + m_a;
    m_r[r] = tmp;
}

void upd1771c_device::AND_Rr_A(u16 op)  // 1100 001r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] & m_a;
    m_r[r] = tmp;
}

void upd1771c_device::SB_Rr_A(u16 op)   // 1100 010r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] - m_a;
    m_r[r] = tmp;
}

void upd1771c_device::OR_Rr_A(u16 op)   // 1100 011r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] | m_a;
    m_r[r] = tmp;
}

void upd1771c_device::ADS_Rr_A(u16 op)  // 1100 100r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] + m_a;
    m_r[r] = tmp;
    m_sk = tmp > 0xff;
}

void upd1771c_device::ANDS_Rr_A(u16 op) // 1100 101r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] & m_a;
    m_r[r] = tmp;
    m_sk = tmp == 0;
}

void upd1771c_device::SBS_Rr_A(u16 op)  // 1100 110r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] - m_a;
    m_r[r] = tmp;
    m_sk = tmp > 0xff;
}

void upd1771c_device::XOR_Rr_A(u16 op)  // 1100 111r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] ^ m_a;
    m_r[r] = tmp;
}

void upd1771c_device::TADNC_Rr_A(u16 op) // 1101 000r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] + m_a;
    m_sk = tmp <= 0xff;
}

void upd1771c_device::TANDNZ_Rr_A(u16 op) // 1101 001r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] & m_a;
    m_sk = tmp != 0;
}

void upd1771c_device::TSBNC_Rr_A(u16 op) // 1101 010r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] - m_a;
    m_sk = tmp <= 0xff;
}

void upd1771c_device::TSBNZ_Rr_A(u16 op) // 1101 011r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] - m_a;
    m_sk = tmp != 0;
}

void upd1771c_device::TADC_Rr_A(u16 op) // 1101 100r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] + m_a;
    m_sk = tmp > 0xff;
}

void upd1771c_device::TANDZ_Rr_A(u16 op) // 1101 101r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] & m_a;
    m_sk = tmp == 0;
}

void upd1771c_device::TSBC_Rr_A(u16 op) // 1101 110r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] - m_a;
    m_sk = tmp > 0xff;
}

void upd1771c_device::TSBZ_Rr_A(u16 op) // 1101 111r rrrr 1000
{
    u8 r = (op >> 4) & 0x1f;
    u16 tmp = m_r[r] - m_a;
    m_sk = tmp == 0;
}
