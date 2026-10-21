// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
#include "emu.h"
#include "tsconf_vout.h"

#include "emupal.h"


DEFINE_DEVICE_TYPE(VOUT_SLOT, vout_device, "vout_slot", "TS-Conf Video Output Slot")

device_vout_interface::device_vout_interface(const machine_config &mconfig, device_t &device)
	: device_interface(device, "vout")
{
}

device_vout_interface::~device_vout_interface()
{
}

vout_device::vout_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, VOUT_SLOT, tag, owner, clock)
	, device_single_card_slot_interface<device_vout_interface>(mconfig, *this)
	, m_ts_update(*this)
	, m_screen(*this, "screen")
	, m_out_miso(*this)
	, m_out_int(*this)
{
}

void vout_device::device_add_mconfig(machine_config &config)
{
	SCREEN(config, m_screen);
	m_screen->set_screen_update(FUNC(vout_device::dispatch_update));
	PALETTE(config, "palette", palette_device::BLACK, 256);
}

void vout_device::set_ts_raw(u32 pxclock, u16 htotal, u16 hbend, u16 hbstart, u16 vtotal, u16 vbend, u16 vbstart)
{
	m_ts_pxclock = pxclock;
	m_ts_htotal = htotal;
	m_ts_hbend = hbend;
	m_ts_hbstart = hbstart;
	m_ts_vtotal = vtotal;
	m_ts_vbend = vbend;
	m_ts_vbstart = vbstart;
}

void vout_device::device_resolve_objects()
{
	device_t::device_resolve_objects();
	m_ts_update.resolve();
}

void vout_device::device_start()
{
	// the TS raster is pushed via configure(), which needs the screen fully
	// started (it reschedules the screen's vblank/scanline timers)
	if (!m_screen->started())
		throw device_missing_dependencies();
	m_card = get_card_device();
	save_item(NAME(m_output_ft));
	if (m_ts_pxclock)
		configure_ts();
}

void vout_device::select_output(bool ft)
{
	if (ft)
	{
		if (!m_output_ft && m_card && m_card->ft_output())
		{
			m_output_ft = true;
			m_card->output_claim();
		}
	}
	else if (m_output_ft)
	{
		m_output_ft = false;
		configure_ts();
	}
}

void vout_device::configure_ts()
{
	m_screen->configure(m_ts_htotal, m_ts_vtotal,
			rectangle(m_ts_hbend, m_ts_hbstart - 1, m_ts_vbend, m_ts_vbstart - 1),
			attotime(0, HZ_TO_ATTOSECONDS(m_ts_pxclock) * m_ts_htotal * m_ts_vtotal));
}

u32 vout_device::dispatch_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	if (m_output_ft && m_card)
		return m_card->update_output(screen, bitmap, cliprect);
	if (!m_ts_update.isnull())
		return m_ts_update(screen, bitmap, cliprect);
	bitmap.fill(rgb_t::black(), cliprect);
	return 0;
}
