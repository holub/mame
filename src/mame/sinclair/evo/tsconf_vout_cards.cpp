// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
#include "emu.h"
#include "tsconf_vout_cards.h"

#include "drawgfx.h"


DEFINE_DEVICE_TYPE(VOUT_RGB, vout_rgb_device, "vout_rgb", "TS-Conf Video Output (RGB)")
DEFINE_DEVICE_TYPE(VOUT_VDAC2, vout_vdac2_device, "vout_vdac2", "TS-Conf Video Output (VDAC2)")

vout_rgb_device::vout_rgb_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, VOUT_RGB, tag, owner, clock)
	, device_vout_interface(mconfig, *this)
{
}

// The FPGA (video_out.v) drives a 2-bit DAC: base c = level[4:3], while the
// low 3 bits index an 8-sub-phase duty pattern pwm[i] = {0,1,3,4,5,6,7,7}/8
// that bumps the output to c+1 on set phases (saturating at 3). The table is
// the resulting time average per 5-bit level, c + duty/8, scaled to 8 bits.
static constexpr u8 level5_to_rgb[32] = {
	0, 11, 32, 43, 53, 64, 74, 74,
	85, 96, 117, 128, 138, 149, 159, 159,
	170, 181, 202, 213, 223, 234, 244, 244,
	255, 255, 255, 255, 255, 255, 255, 255};

rgb_t vout_rgb_device::palette_color(u16 pwm15) const
{
	return rgb_t(level5_to_rgb[BIT(pwm15, 10, 5)], level5_to_rgb[BIT(pwm15, 5, 5)], level5_to_rgb[BIT(pwm15, 0, 5)]);
}

void vout_rgb_device::device_start()
{
}

vout_vdac2_device::vout_vdac2_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, VOUT_VDAC2, tag, owner, clock)
	, device_vout_interface(mconfig, *this)
	, m_ft812(*this, "ft812")
{
}

rgb_t vout_vdac2_device::palette_color(u16 pwm15) const
{
	return rgb_t(pal5bit(BIT(pwm15, 10, 5)), pal5bit(BIT(pwm15, 5, 5)), pal5bit(BIT(pwm15, 0, 5)));
}

void vout_vdac2_device::device_add_mconfig(machine_config &config)
{
	FT812(config, m_ft812, 0);
	m_ft812->spi_miso_callback().set(FUNC(vout_vdac2_device::ft_miso_w));
	m_ft812->int_callback().set(FUNC(vout_vdac2_device::ft_int_w));
	m_ft812->set_timing_cb(FUNC(vout_vdac2_device::ft_timing_changed));
}

void vout_vdac2_device::device_start()
{
	// the board advances the FT812 frame state at the active output's vblank
	m_vout->screen().register_vblank_callback(vblank_state_delegate(
			[this](screen_device &screen, bool vblank_state)
			{
				if (vblank_state)
					m_ft812->vblank();
			}));
}

void vout_vdac2_device::spi_byte_w(u8 data)
{
	for (u8 m = 0x80; m; m >>= 1)
	{
		m_ft812->spi_mosi_w(data & m ? 1 : 0);
		m_ft812->spi_clock_w(CLEAR_LINE);
		m_ft812->spi_clock_w(ASSERT_LINE);
	}
}

void vout_vdac2_device::output_claim()
{
	if (m_ft812->timing().valid)
	{
		const ft812_device::ft812_timing &t = m_ft812->timing();
		m_vout->card_configure(t.width, t.height, t.vis, t.period);
	}
}

u32 vout_vdac2_device::update_output(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	// cliprect/bitmap are in total-raster coordinates once the timing has been
	// reconfigured; the FT812 framebuffer is visible-area relative
	const rectangle vis = screen.visible_area();
	copybitmap(bitmap, m_ft812->framebuffer(), 0, 0, vis.left(), vis.top(), cliprect);
	return 0;
}

void default_vout_devices(device_slot_interface &device)
{
	device.option_add("rgb", VOUT_RGB);
	device.option_add("vdac2", VOUT_VDAC2);
}
