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

	required_device<ds17x85_device> m_rtc;

private:
	void map_io(address_map &map);
};

void smuc_device::map_io(address_map &map)
{
	map(0x00ba, 0x00ba).mirror(0xff00).noprw();
	map(0x00be, 0x00be).mirror(0xff00).noprw();

	// 0x011xxx101xx010 - Version
	map(0x18a2, 0x18a2).mirror(0x4718).lr8(NAME([]() { return 0x77; }));
	// 0x011xxx101xx110 - Revision
	map(0x18a6, 0x18a6).mirror(0x4718).lr8(NAME([]() { return 0x77; }));
	// 1x011xxx101xx010
	map(0x98a2, 0x98a2).mirror(0x4718).rw(m_rtc, FUNC(ds17x85_device::read), FUNC(ds17x85_device::write));
}

void smuc_device::device_add_mconfig(machine_config &config)
{
	DS1685(config, m_rtc, XTAL(32'768));
}

void smuc_device::device_start()
{
	m_zxbus->install_device(0x0000, 0xffff, *this, &smuc_device::map_io);
}

} // anonymous namespace

} // namespace bus::spectrum::zxbus

DEFINE_DEVICE_TYPE_PRIVATE(ZXBUS_SMUC, device_zxbus_card_interface, bus::spectrum::zxbus::smuc_device, "smuc", "SMUC")
