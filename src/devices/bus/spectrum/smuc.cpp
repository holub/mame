// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub

#include "emu.h"
#include "smuc.h"

#include "machine/ds17x85.h"

namespace bus::spectrum::zxbus {

namespace {

class smuc_device : public device_t, public device_zxbus_card_interface
{
public:
	smuc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: device_t(mconfig, ZXBUS_SMUC, tag, owner, clock)
		, device_zxbus_card_interface(mconfig, *this)
		, m_rtc(*this, "rtc")
    { }

protected:
	void device_start() override;
	void device_add_mconfig(machine_config &config) override;

private:
	void dos_w(int state);
	void map_io(address_map &map);
	void port_ffba_w(offs_t offset, u8 data);

	required_device<ds17x85_device> m_rtc;
	u8 m_port_ffba_data;
	u8 m_port_7fba_data;
};

void smuc_device::port_ffba_w(offs_t offset, u8 data)
{
	m_port_ffba_data = data;
}

void smuc_device::dos_w(int state)
{
	printf("%d ", state);
	//machine().debug_break();
}

void smuc_device::map_io(address_map &map)
{
	map(0x18a2, 0x18a2).mirror(0x4718) // 5fba | 0x011xxx101xx010 | Version
		.lr8(NAME([]() { return 0x3f; }));
	map(0x18a6, 0x18a6).mirror(0x4718) // 5fbe | 0x011xxx101xx110 | Revision
		.lr8(NAME([]() { return 0x57; }));
	map(0x38a6, 0x38a6).mirror(0x4618) // 7ebe | 0x111xx0101xx110 | ?8259
		.lr8(NAME([]() { return 0x57; }));
	map(0x38a2, 0x38a2).mirror(0x4718) // 7fba | 0x111xxx101xx010 | VirtualFDD
		.lrw8(NAME([this]() { return m_port_7fba_data & 0x3f; })
			, NAME([this](offs_t offset, u8 data) { m_port_7fba_data = data; }));
	map(0x39a6, 0x39a6).mirror(0x4618) // 7fbe | 0x111xx1101xx110 | ?8259
		.lr8(NAME([]() { return 0x57; }));
	map(0x98a6, 0x98a6).mirror(0x4718) // d8be | 1x011xxx101xx110 | IDE-Hi
		.noprw();
	map(0x98a2, 0x98a2).mirror(0x4718) // dfba | 1x011xxx101xx010 | DS1685RTC
		.lrw8(NAME([this]() { return m_rtc->read(1); })
			, NAME([this](offs_t offset, u8 data) { m_rtc->write(BIT(m_port_ffba_data, 7), data); }));
	//map(0x, 0x).mirror(0x4718) // f8be-ffbe | 1x111NNN101xx110  | IDE#1Fx/#3F6
//		.noprw();
	map(0xb8a2, 0xb8a2).mirror(0x4718) // ffba | 1x111xxx101xx010 | SYS
		.lr8(NAME([this]() { return m_port_ffba_data; })).w(FUNC(smuc_device::port_ffba_w));
}

void smuc_device::device_add_mconfig(machine_config &config)
{
	DS1685(config, m_rtc, XTAL(32'768));
}

void smuc_device::device_start()
{
	m_zxbus->install_device(0x0000, 0xffff, *this, &smuc_device::map_io);
	//m_zxbus->dos_cb().set(FUNC(smuc_device::dos_w));  // !!!!!!! m_functions.empty()
}

} // anonymous namespace

} // namespace bus::spectrum::zxbus

DEFINE_DEVICE_TYPE_PRIVATE(ZXBUS_SMUC, device_zxbus_card_interface, bus::spectrum::zxbus::smuc_device, "smuc", "SMUC")
