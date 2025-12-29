// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
#ifndef MAME_SINCLAIR_SPECNEXT_LAYER2_H
#define MAME_SINCLAIR_SPECNEXT_LAYER2_H

#pragma once

#include "specnext_video_layer.h"

class specnext_layer2_device : public device_t, public specnext_video_layer_interface
{

public:
	specnext_layer2_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	specnext_layer2_device &set_host_ram_ptr(const u8 *host_ram_ptr) { m_host_ram_ptr = host_ram_ptr; return *this; }

	void pen_priority_w(u16 pen, bool priority) { m_pen_priority[pen] = priority; }

	void layer2_en_w(bool layer2_en) { m_layer2_en = layer2_en; }
	void resolution_w(u8 resolution) { m_resolution = resolution & 0x03; }
	void palette_offset_w(u8 palette_offset) { m_palette_offset = palette_offset & 0x0f; }
	void layer2_active_bank_w(u8 layer2_active_bank) { m_layer2_active_bank = layer2_active_bank & 0x7f; }

	void scroll_x_w(u16 scroll_x) { m_scroll_x = scroll_x & 0x1ff; }
	void scroll_y_w(u8 scroll_y) { m_scroll_y = scroll_y; }
	void clip_x1_w(u8 clip_x1) { m_clip_x1 = clip_x1; }
	void clip_x2_w(u8 clip_x2) { m_clip_x2 = clip_x2; }
	void clip_y1_w(u8 clip_y1) { m_clip_y1 = clip_y1; }
	void clip_y2_w(u8 clip_y2) { m_clip_y2 = clip_y2; }

	void draw(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect, u8 pcode = 0, u8 priority_mask = 0);
	void draw_mix(screen_device &screen, bitmap_rgb32 &bitmap, bitmap_rgb32 &blendprio, const rectangle &cliprect, u8 mixer);
	void copyprio(screen_device &screen, bitmap_rgb32 &bitmap, bitmap_rgb32 &blendprio, const rectangle &cliprect);

protected:
	static constexpr u16 LAYER2_INFO[2][5] =
	{
		//width  height  border  x-inc  y-inc
		{   256,    192,      0,     1,   256 },
		{   320,    256,     32,   256,     1 },
	};

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	template <typename FunctionClass> void draw_256(screen_device &screen, bitmap_rgb32 &bitmap, bitmap_rgb32 &blendprio, const rectangle &cliprect, FunctionClass blend_op);
	template <typename FunctionClass> void draw_16(screen_device &screen, bitmap_rgb32 &bitmap, bitmap_rgb32 &blendprio, const rectangle &cliprect, FunctionClass blend_op);

	const u8 *m_host_ram_ptr;
	bool m_pen_priority[512 * 4];

	bool m_layer2_en;
	u8 m_resolution; // u2: 00 = 256x192, 01 = 320x256, 1X = 640x256x4
	u8 m_palette_offset; // u4
	u8 m_layer2_active_bank; // u7

	u16 m_scroll_x; // u9
	u8 m_scroll_y;
	u8 m_clip_x1;
	u8 m_clip_x2;
	u8 m_clip_y1;
	u8 m_clip_y2;
};


DECLARE_DEVICE_TYPE(SPECNEXT_LAYER2, specnext_layer2_device)
#endif // MAME_SINCLAIR_SPECNEXT_LAYER2_H
