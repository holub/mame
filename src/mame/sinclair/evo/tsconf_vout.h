// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
#ifndef MAME_SINCLAIR_EVO_TSCONF_VOUT_H
#define MAME_SINCLAIR_EVO_TSCONF_VOUT_H

#pragma once

#include "screen.h"

class vout_device;

// device interface for a video output card inserted into the vout slot
class device_vout_interface : public device_interface
{
public:
	device_vout_interface(const machine_config &mconfig, device_t &device);
	virtual ~device_vout_interface();

	// CRAM PWM value as provided by the inserted card (rgb: linear PWM DAC
	// expansion; vdac2: raw 555 for both outputs)
	virtual rgb_t palette_color(u16 pwm15) const = 0;
	// card can drive the FT812 output (V_CONFIG VID_FT812 switch makes sense)
	virtual bool ft_output() const { return false; }
	virtual bool spi_present() const { return false; }
	virtual void spi_ss_w(int state) { }
	virtual void spi_byte_w(u8 data) { }
	// gained output control: push the card raster now
	virtual void output_claim() { }
	virtual u32 update_output(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect) { return 0; }

protected:
	vout_device *m_vout = nullptr;
};


// TS-Conf video output slot: owns the machine screen (subdevice "screen") and
// switches it between the TS renderer and the card output (V_CONFIG VID_FT812)
class vout_device : public device_t, public device_single_card_slot_interface<device_vout_interface>
{
public:
	vout_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
	template <typename T>
	vout_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock, T &&opts, const char *dflt)
		: vout_device(mconfig, tag, owner, clock)
	{
		set_options(std::forward<T>(opts), dflt, false);
	}

	// driver config: TS raster params (applied initially and on FT release)
	void set_ts_raw(u32 pxclock, u16 htotal, u16 hbend, u16 hbstart, u16 vtotal, u16 vbend, u16 vbstart);
	// driver config: TS-side renderer, dispatched while the FT output is not active
	template <typename F>
	void set_ts_screen_update(F &&callback, const char *name) requires screen_update_rgb32_delegate::supports_callback<F>::value
	{
		m_ts_update.set(std::forward<F>(callback), name);
	}

	// driver runtime: V_CONFIG VID_FT812 → select output source
	void select_output(bool ft);
	// card runtime (vdac2): push FT812 raster; honored only while FT output is active
	void card_configure(int width, int height, const rectangle &visarea, attotime frame_period)
	{
		if (m_output_ft)
			m_screen->configure(width, height, visarea, frame_period);
	}

	screen_device &screen() { return *m_screen; }

	rgb_t ts_palette_color(u16 pwm15) const { return m_card->palette_color(pwm15); }
	bool spi_present() const { return m_card && m_card->spi_present(); }
	void spi_ss_w(int state) { if (m_card) m_card->spi_ss_w(state); }
	void spi_byte_w(u8 data) { if (m_card) m_card->spi_byte_w(data); }

	// card (vdac2) outbound FT812 lines
	void miso_w(int state) { m_out_miso(state); }
	void int_w(int state) { m_out_int(state); }

	auto spi_miso_callback() { return m_out_miso.bind(); }
	auto int_callback() { return m_out_int.bind(); }

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_resolve_objects() override;
	virtual void device_start() override ATTR_COLD;

private:
	u32 dispatch_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	void configure_ts();

	device_vout_interface *m_card = nullptr;
	screen_update_rgb32_delegate m_ts_update;
	required_device<screen_device> m_screen;
	u32 m_ts_pxclock = 0;
	u16 m_ts_htotal = 0, m_ts_hbend = 0, m_ts_hbstart = 0;
	u16 m_ts_vtotal = 0, m_ts_vbend = 0, m_ts_vbstart = 0;
	bool m_output_ft = false;
	devcb_write_line m_out_miso;
	devcb_write_line m_out_int;
};

DECLARE_DEVICE_TYPE(VOUT_SLOT, vout_device)

#endif // MAME_SINCLAIR_EVO_TSCONF_VOUT_H
