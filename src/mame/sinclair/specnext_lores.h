// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
#ifndef MAME_SINCLAIR_SPECNEXT_LORES_H
#define MAME_SINCLAIR_SPECNEXT_LORES_H

#pragma once

#include "specnext_video_layer.h"

class specnext_lores_device : public device_t, public specnext_video_layer_interface
{

public:
	specnext_lores_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	specnext_lores_device &set_host_ram_ptr(const u8 *host_ram_ptr) { m_host_ram_ptr = host_ram_ptr; return *this; }

	void mode_w(bool mode) { m_mode = mode; } // 0 = lores, 1 = radastan
	void dfile_w(bool dfile) { m_dfile = dfile; } // timex display file to use for radastan
	void ulap_en_w(bool ulap_en) { m_ulap_en = ulap_en; } // translate radastan pixel to ula+ palette

	void lores_palette_offset_w(u8 lores_palette_offset) { m_lores_palette_offset = lores_palette_offset & 0x0f; }
	void scroll_x_w(u8 scroll_x) { m_scroll_x = scroll_x; }
	void scroll_y_w(u8 scroll_y) { m_scroll_y = scroll_y; }
	void clip_x1_w(u8 clip_x1) { m_clip_x1 = clip_x1; }
	void clip_x2_w(u8 clip_x2) { m_clip_x2 = clip_x2; }
	void clip_y1_w(u8 clip_y1) { m_clip_y1 = clip_y1; }
	void clip_y2_w(u8 clip_y2) { m_clip_y2 = clip_y2; }

	void draw(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect, u8 pcode = 0);

protected:
	static constexpr rectangle SCREEN_AREA = { 0, 255, 0, 191 };

	virtual void device_start() override ATTR_COLD;

private:
	const u8 *m_host_ram_ptr;

	bool m_mode;
	bool m_dfile;
	bool m_ulap_en;

	u8 m_lores_palette_offset; // u4
	u8 m_clip_x1;
	u8 m_clip_x2;
	u8 m_clip_y1;
	u8 m_clip_y2;
	u8 m_scroll_x;
	u8 m_scroll_y;

};


DECLARE_DEVICE_TYPE(SPECNEXT_LORES, specnext_lores_device)
#endif // MAME_SINCLAIR_SPECNEXT_LORES_H
