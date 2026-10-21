// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
#ifndef MAME_SINCLAIR_EVO_TSCONF_VOUT_CARDS_H
#define MAME_SINCLAIR_EVO_TSCONF_VOUT_CARDS_H

#pragma once

#include "ft812.h"
#include "tsconf_vout.h"


// baseboard analog RGB output: the FPGA's internal PWM DAC (linear palette)
class vout_rgb_device : public device_t, public device_vout_interface
{
public:
	vout_rgb_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	virtual rgb_t palette_color(u16 pwm15) const override;

protected:
	virtual void device_resolve_objects() override { device_t::device_resolve_objects(); m_vout = &downcast<vout_device &>(*owner()); }
	virtual void device_start() override ATTR_COLD;
};

DECLARE_DEVICE_TYPE(VOUT_RGB, vout_rgb_device)


// VDAC2 board: FT812 on the SPI port, routes the VGA output through the panel DAC
class vout_vdac2_device : public device_t, public device_vout_interface
{
public:
	vout_vdac2_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// the panel DAC carries the raw 555 palette value as-is
	virtual rgb_t palette_color(u16 pwm15) const override;
	virtual bool ft_output() const override { return true; }
	virtual bool spi_present() const override { return true; }
	virtual void spi_ss_w(int state) override { m_ft812->spi_ss_w(state); }
	virtual void spi_byte_w(u8 data) override;
	virtual void output_claim() override;
	virtual u32 update_output(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect) override;

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_resolve_objects() override { device_t::device_resolve_objects(); m_vout = &downcast<vout_device &>(*owner()); }
	virtual void device_start() override ATTR_COLD;

private:
	void ft_timing_changed(int width, int height, const rectangle &vis, attotime period) { m_vout->card_configure(width, height, vis, period); }
	void ft_miso_w(int state) { m_vout->miso_w(state); }
	void ft_int_w(int state) { m_vout->int_w(state); }

	required_device<ft812_device> m_ft812;
};

DECLARE_DEVICE_TYPE(VOUT_VDAC2, vout_vdac2_device)


void default_vout_devices(device_slot_interface &device);

#endif // MAME_SINCLAIR_EVO_TSCONF_VOUT_CARDS_H
