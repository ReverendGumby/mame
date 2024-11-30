// license:BSD-3-Clause
// copyright-holders:David Hunter
/*****************************************************************************
 *
 *   upd1771c.cpp
 *   uPD1771C emulator
 *
 *  This work is based on
 *  - @src/devices/sound/upd1771c_017.cpp
 *  - https://siliconpr0n.org/map/nec/d1771c-017/mcmaster_mz_mit20x/ - photomicrograph of D1771C-017 die
 *  - http://reverendgumby.gitlab.io/visuald1771c - JavaScript simulator derived from above die shot
 *  - https://gitlab.com/ReverendGumby/SuperCassetteVision_MiSTer/-/blob/main/rtl/scv/upd1771c/upd1771c.sv - Verilog emulation
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
	, m_program_config("program", ENDIANNESS_LITTLE, 16, 12, -1, address_map_constructor(FUNC(upd1771c_device::internal_512x16), this))
	, m_ram_view(*this, "ram_view")
{
}

void upd1771c_device::internal_512x16(address_map &map)
{
    map(0x000, 0x1ff).rom();
}

device_memory_interface::space_config_vector upd1771c_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &m_program_config)
	};
}

std::unique_ptr<util::disasm_interface> upd1771c_device::create_disassembler()
{
	return std::make_unique<upd177x_disassembler>();
}

void upd1771c_device::device_start()
{
    init_ops();

	space(AS_PROGRAM).specific(m_program);
	space(AS_PROGRAM).cache(m_opcodes);

	set_icountptr(m_icount);

	state_add(UPD1771C_PC,  "PC",   m_pc).formatstr("%04X");

	save_item(NAME(m_pc));
}

void upd1771c_device::device_reset()
{
}

void upd1771c_device::execute_run()
{
    do
    {
		m_ppc = m_pc;
		debugger_instruction_hook(m_pc);

        u16 op = m_opcodes.read_word(m_pc);

		/* skip flag set? */
        if (m_sk)
        {
            op = 0x0000; // NOP
            m_sk = false;
        }
        int cc = op_cycles(op);
        //handle_timers( cc );
        (this->*m_op_funcs[op])(op);
        m_pc ++;

        m_icount -= cc;
		//take_irq();
	} while (m_icount > 0);
}

/***********************************************************************
 * opcodes
 ***********************************************************************/

void upd1771c_device::init_ops()
{
    static const struct {
        u16 code;
        u16 mask;
        opcode_func fn;
    } instructions[] {
        { 0x4000, 0xe000, &upd1771c_device::MVI_Rr },

//      { 0x3200, 0xff00, &upd1771c_device::MVI_Hp },
//      { 0x3400, 0xff00, &upd1771c_device::MVI_A },
//      { 0x3800, 0xffc0, &upd1771c_device::MVI_H },
//      { 0x3100, 0xff1f, &upd1771c_device::MVI_MD1 },
//      { 0x2100, 0xff80, &upd1771c_device::MVI_MDO },

//      { 0x1201, 0xfe0f, &upd1771c_device::MOV_Rr_A },
//      { 0x1005, 0xfe0f, &upd1771c_device::MOV_A_Rr },
//      { 0x1205, 0xfe0f, &upd1771c_device::XCHG_Rr_A },
//      { 0x1202, 0xfe0f, &upd1771c_device::MOV_Rr_Hp },
//      { 0x100a, 0xfe0f, &upd1771c_device::MOV_Hp_Rr },
//      { 0x120a, 0xfe0f, &upd1771c_device::XCHG_Rr_A },
//      { 0x1000, 0xfe0f, &upd1771c_device::MOV_Y_Rr },
//      { 0x0008, 0xffff, &upd1771c_device::MOV_X_RG },

//      { 0x0404, 0xffff, &upd1771c_device::RAR },
//      { 0x0408, 0xffff, &upd1771c_device::RAL },

//      { 0x0401, 0xffff, &upd1771c_device::IN_PA },
//      { 0x0402, 0xffff, &upd1771c_device::IN_PB },
//      { 0x0002, 0xffff, &upd1771c_device::OUT_PA },
//      { 0x0004, 0xffff, &upd1771c_device::OUT_PB },
//      { 0x0502, 0xffff, &upd1771c_device::OUT_DA },

//      { 0x0504, 0xffff, &upd1771c_device::MUL1 },
//      { 0x050c, 0xffff, &upd1771c_device::MUL2 },

//      { 0x1409, 0xfe0f, &upd1771c_device::MIX },

//      { 0x1801, 0xfe0f, &upd1771c_device::TBL0_A },
//      { 0x1802, 0xfe0f, &upd1771c_device::TBL0_X },
//      { 0x1804, 0xfe0f, &upd1771c_device::TBL0_Y },
//      { 0x1808, 0xfe0f, &upd1771c_device::CALL0 },

//      { 0x6000, 0xf000, &upd1771c_device::JMP_n12 },
//      { 0x2000, 0xff0f, &upd1771c_device::JMP_n4 },
//      { 0x0501, 0xffff, &upd1771c_device::JMPA },
//      { 0x2400, 0xff00, &upd1771c_device::JMPFZ },

//      { 0x7000, 0xf000, &upd1771c_device::CALL },

//      { 0x0800, 0xffff, &upd1771c_device::RET },
//      { 0x0801, 0xffff, &upd1771c_device::RETS },
//      { 0x090f, 0xffff, &upd1771c_device::RETI },

//      { 0x0005, 0xffff, &upd1771c_device::STF },
//      { 0x0602, 0xffff, &upd1771c_device::OFF },
        { 0x0000, 0xffff, &upd1771c_device::NOP },

//      { 0xe000, 0xfe00, &upd1771c_device::ADI_Rr },
//      { 0xe200, 0xfe00, &upd1771c_device::ADIS_Rr },
//      { 0xe400, 0xfe00, &upd1771c_device::SBI_Rr },
//      { 0xe600, 0xfe00, &upd1771c_device::SBIS_Rr },
//      { 0xe800, 0xfe00, &upd1771c_device::TADINC_Rr },
//      { 0xea00, 0xfe00, &upd1771c_device::TADIC_Rr },
//      { 0xec00, 0xfe00, &upd1771c_device::TSBINC_Rr },
//      { 0xee00, 0xfe00, &upd1771c_device::TSBIC_Rr },
//      { 0xf000, 0xfe00, &upd1771c_device::ADI5_Rr },
//      { 0xf200, 0xfe00, &upd1771c_device::ADIMS_Rr },
//      { 0xf800, 0xfe00, &upd1771c_device::TADI5_Rr },

//      { 0x8000, 0xff00, &upd1771c_device::ADI_A },
//      { 0x8200, 0xff00, &upd1771c_device::ANDI_A },
//      { 0x8400, 0xff00, &upd1771c_device::SBI_A },
//      { 0x8600, 0xff00, &upd1771c_device::ORI_A },
//      { 0x8800, 0xff00, &upd1771c_device::ADIS_A },
//      { 0x8a00, 0xff00, &upd1771c_device::ANDIS_A },
//      { 0x8c00, 0xff00, &upd1771c_device::SBIS_A },
//      { 0x8e00, 0xff00, &upd1771c_device::XORI_A },
//      { 0x9000, 0xff00, &upd1771c_device::TADINC_A },
//      { 0x9200, 0xff00, &upd1771c_device::TANDINZ_A },
//      { 0x9400, 0xff00, &upd1771c_device::TSBINC_A },
//      { 0x9600, 0xff00, &upd1771c_device::TSBINZ_A },
//      { 0x9800, 0xff00, &upd1771c_device::TADIC_A },
//      { 0x9a00, 0xff00, &upd1771c_device::TANDIZ_A },
//      { 0x9c00, 0xff00, &upd1771c_device::TSBIC_A },
//      { 0x9e00, 0xff00, &upd1771c_device::TSBIZ_A },

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

//      { 0xa000, 0xff00, &upd1771c_device::ADI_Hp },
//      { 0xa200, 0xff00, &upd1771c_device::ANDI_Hp },
//      { 0xa400, 0xff00, &upd1771c_device::SBI_Hp },
//      { 0xa600, 0xff00, &upd1771c_device::ORI_Hp },
//      { 0xa800, 0xff00, &upd1771c_device::ADIS_Hp },
//      { 0xaa00, 0xff00, &upd1771c_device::ANDIS_Hp },
//      { 0xac00, 0xff00, &upd1771c_device::SBIS_Hp },
//      { 0xae00, 0xff00, &upd1771c_device::XORI_Hp },
//      { 0xb000, 0xff00, &upd1771c_device::TADINC_Hp },
//      { 0xb200, 0xff00, &upd1771c_device::TANDINZ_Hp },
//      { 0xb400, 0xff00, &upd1771c_device::TSBINC_Hp },
//      { 0xb600, 0xff00, &upd1771c_device::TSBINZ_Hp },
//      { 0xb800, 0xff00, &upd1771c_device::TADIC_Hp },
//      { 0xba00, 0xff00, &upd1771c_device::TANDIZ_Hp },
//      { 0xbc00, 0xff00, &upd1771c_device::TSBIC_Hp },
//      { 0xbe00, 0xff00, &upd1771c_device::TSBIZ_Hp },

//      { 0xa100, 0xffc0, &upd1771c_device::ADI_H },
//      { 0xa300, 0xffc0, &upd1771c_device::ANDI_H },
//      { 0xa500, 0xffc0, &upd1771c_device::SBI_H },
//      { 0xa700, 0xffc0, &upd1771c_device::ORI_H },
//      { 0xa900, 0xffc0, &upd1771c_device::ADIS_H },
//      { 0xabc0, 0xffc0, &upd1771c_device::ANDIS_H },
//      { 0xad00, 0xffc0, &upd1771c_device::SBIS_H },
//      { 0xaf00, 0xffc0, &upd1771c_device::XORI_H },
//      { 0xb1c0, 0xffc0, &upd1771c_device::TADINC_H },
//      { 0xb300, 0xffc0, &upd1771c_device::TANDINZ_H },
//      { 0xb500, 0xffc0, &upd1771c_device::TSBINC_H },
//      { 0xb700, 0xffc0, &upd1771c_device::TSBINZ_H },
//      { 0xb9c0, 0xffc0, &upd1771c_device::TADIC_H },
//      { 0xbb00, 0xffc0, &upd1771c_device::TANDIZ_H },
//      { 0xbd00, 0xffc0, &upd1771c_device::TSBIC_H },
//      { 0xbf00, 0xffc0, &upd1771c_device::TSBIZ_H },

//      { 0x0201, 0xffff, &upd1771c_device::MOV_N_A },
//      { 0x0208, 0xffff, &upd1771c_device::MOV_X_A },
//      { 0x1601, 0xffff, &upd1771c_device::MOV_Hp_A },
//      { 0x1405, 0xffff, &upd1771c_device::MOV_A_Hp },
//      { 0x1605, 0xffff, &upd1771c_device::XCHG_Hp_A },

//      { 0xc000, 0xfe0f, &upd1771c_device::AD_A_Rr },
//      { 0xc200, 0xfe0f, &upd1771c_device::AND_A_Rr },
//      { 0xc400, 0xfe0f, &upd1771c_device::SB_A_Rr },
//      { 0xc600, 0xfe0f, &upd1771c_device::AOR_A_Rr },
//      { 0xc800, 0xfe0f, &upd1771c_device::ADS_A_Rr },
//      { 0xca00, 0xfe0f, &upd1771c_device::ANDS_A_Rr },
//      { 0xcc00, 0xfe0f, &upd1771c_device::SBS_A_Rr },
//      { 0xce00, 0xfe0f, &upd1771c_device::XOR_A_Rr },
//      { 0xd000, 0xfe0f, &upd1771c_device::TADNC_A_Rr },
//      { 0xd200, 0xfe0f, &upd1771c_device::TANDNZ_A_Rr },
//      { 0xd400, 0xfe0f, &upd1771c_device::TSBNC_A_Rr },
//      { 0xd600, 0xfe0f, &upd1771c_device::TSBNZ_A_Rr },
//      { 0xd800, 0xfe0f, &upd1771c_device::TADC_A_Rr },
//      { 0xda00, 0xfe0f, &upd1771c_device::TANDZ_A_Rr },
//      { 0xdc00, 0xfe0f, &upd1771c_device::TSBC_A_Rr },
//      { 0xde00, 0xfe0f, &upd1771c_device::TSBZ_A_Rr },

//      { 0xc008, 0xfe0f, &upd1771c_device::AD_Rr_A },
//      { 0xc208, 0xfe0f, &upd1771c_device::AND_Rr_A },
//      { 0xc408, 0xfe0f, &upd1771c_device::SB_Rr_A },
//      { 0xc608, 0xfe0f, &upd1771c_device::AOR_Rr_A },
//      { 0xc808, 0xfe0f, &upd1771c_device::ADS_Rr_A },
//      { 0xca08, 0xfe0f, &upd1771c_device::ANDS_Rr_A },
//      { 0xcc08, 0xfe0f, &upd1771c_device::SBS_Rr_A },
//      { 0xce08, 0xfe0f, &upd1771c_device::XOR_Rr_A },
//      { 0xd008, 0xfe0f, &upd1771c_device::TADNC_Rr_A },
//      { 0xd208, 0xfe0f, &upd1771c_device::TANDNZ_Rr_A },
//      { 0xd408, 0xfe0f, &upd1771c_device::TSBNC_Rr_A },
//      { 0xd608, 0xfe0f, &upd1771c_device::TSBNZ_Rr_A },
//      { 0xd808, 0xfe0f, &upd1771c_device::TADC_Rr_A },
//      { 0xda08, 0xfe0f, &upd1771c_device::TANDZ_Rr_A },
//      { 0xdc08, 0xfe0f, &upd1771c_device::TSBC_Rr_A },
//      { 0xde08, 0xfe0f, &upd1771c_device::TSBZ_Rr_A },

//      { 0xc001, 0xffff, &upd1771c_device::AD_A_Hp },
//      { 0xc201, 0xffff, &upd1771c_device::AND_A_Hp },
//      { 0xc401, 0xffff, &upd1771c_device::SB_A_Hp },
//      { 0xc601, 0xffff, &upd1771c_device::AOR_A_Hp },
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
//      { 0xc609, 0xffff, &upd1771c_device::AOR_Hp_A },
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
	logerror("uPD1771C '%s': illegal opcode %04x at PC:%04x\n", tag(), op, m_pc);
}

void upd1771c_device::NOP(u16 op)       // 0000 0000 0000 0000
{
}

void upd1771c_device::MVI_Rr(u16 op)    // 010r rrrr nnnn nnnn
{
}

