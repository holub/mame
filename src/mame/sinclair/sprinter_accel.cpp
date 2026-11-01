// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
/**********************************************************************
    Sprinter Sp2000 RAM Operations Accelerator

The accelerator has no memory of its own: it snoops the z80 bus and is
programmed with nop-like z80 opcodes. This device exposes a read-only
mirror of the cpu program space window the accelerator watches, so the
debugger can disassemble z80 code with the accelerator command set, eg
    dasm accel.lst,c000,40,,accel
**********************************************************************/

#include "emu.h"
#include "sprinter_accel.h"
#include "sprinter_accel_dasm.h"


// device type definition
DEFINE_DEVICE_TYPE(SPRINTER_ACCEL, sprinter_accel_device, "sprinter_accel", "Sprinter Sp2000 RAM Operations Accelerator")

sprinter_accel_device::sprinter_accel_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SPRINTER_ACCEL, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, device_disasm_interface(mconfig, *this)
	, device_state_interface(mconfig, *this)
	, m_space_config("program", ENDIANNESS_LITTLE, 8, 16, 0, address_map_constructor(FUNC(sprinter_accel_device::accel_map), this))
	, m_cpu_prg(*this, finder_base::DUMMY_TAG, -1)
{
}

void sprinter_accel_device::device_start()
{
	m_cpu_prg->device().interface(m_cpu_state);

	// the currently executing command is the one at the snooped cpu pc
	state_add<u16>(STATE_GENPCBASE, "CURPC", [this]() { return m_cpu_state ? m_cpu_state->pcbase() & 0xffff : 0; });
}

u8 sprinter_accel_device::bus_r(offs_t offset)
{
	return m_cpu_prg->read_byte(0x10000 | offset);
}

void sprinter_accel_device::accel_map(address_map &map)
{
	map(0x0000, 0xffff).r(FUNC(sprinter_accel_device::bus_r));
}

device_memory_interface::space_config_vector sprinter_accel_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &m_space_config)
	};
}

std::unique_ptr<util::disasm_interface> sprinter_accel_device::create_disassembler()
{
	return std::make_unique<sprinter_accel_disassembler>();
}
