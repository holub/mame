// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
#ifndef MAME_SINCLAIR_SPRINTER_ACCEL_H
#define MAME_SINCLAIR_SPRINTER_ACCEL_H

#pragma once

#include "emu.h"

class sprinter_accel_device : public device_t,
								public device_memory_interface,
								public device_disasm_interface,
								public device_state_interface
{
public:
	sprinter_accel_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	template <typename T> void set_bus(T &&tag, int spacenum = AS_PROGRAM)
	{
		m_cpu_prg.set_tag(std::forward<T>(tag), spacenum);
	}

protected:
	virtual void device_start() override ATTR_COLD;

	virtual space_config_vector memory_space_config() const override;
	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

private:
	u8 bus_r(offs_t offset);

	void accel_map(address_map &map) ATTR_COLD;

	const address_space_config m_space_config;
	required_address_space m_cpu_prg;
	device_state_interface *m_cpu_state = nullptr;
};

DECLARE_DEVICE_TYPE(SPRINTER_ACCEL, sprinter_accel_device)

#endif // MAME_SINCLAIR_SPRINTER_ACCEL_H
