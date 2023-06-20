// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub

#include "emu.h"
#include "smuc.h"

namespace bus::spectrum::zxbus {

namespace {

class smuc_device : public device_t, public device_zxbus_card_interface
{
public:
	smuc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: device_t(mconfig, ZXBUS_SMUC, tag, owner, clock)
		, device_zxbus_card_interface(mconfig, *this)
    { }

protected:
	void device_start() override;

};

void smuc_device::device_start()
{
}

} // anonymous namespace

} // namespace bus::spectrum::zxbus

DEFINE_DEVICE_TYPE_PRIVATE(ZXBUS_SMUC, device_zxbus_card_interface, bus::spectrum::zxbus::smuc_device, "smuc", "SMUC")
