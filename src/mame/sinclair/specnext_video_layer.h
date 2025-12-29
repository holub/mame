// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub

#ifndef MAME_SINCLAIR_SPECNEXT_VIDEO_LAYER_H
#define MAME_SINCLAIR_SPECNEXT_VIDEO_LAYER_H

#pragma once

class specnext_video_layer_interface : public device_gfx_interface
{
public:
	specnext_video_layer_interface(const machine_config &mconfig, device_t &device, const gfx_decode_entry *gfxinfo = nullptr);
	virtual ~specnext_video_layer_interface();

	virtual specnext_video_layer_interface &set_raster_offset(u16 offset_h,  u16 offset_v) { m_offset_h = offset_h; m_offset_v = offset_v; return *this; }
	virtual specnext_video_layer_interface &set_palette(const char *tag, u16 base_offset, u16 alt_offset);

	virtual void set_global_transparent(u8 global_transparent) { m_global_transparent = global_transparent; }
	virtual void alt_palette_select_w(bool alt_palette_select) { m_alt_palette_select = alt_palette_select; }

protected:
	void interface_post_start() ATTR_COLD;
	void interface_post_reset() ATTR_COLD;

	u16 m_offset_h, m_offset_v;
	u8 m_global_transparent;
	u16 m_palette_base_offset;
	u16 m_palette_alt_offset;
	bool m_alt_palette_select;

};

#endif // MAME_SINCLAIR_SPECNEXT_VIDEO_LAYER_H
