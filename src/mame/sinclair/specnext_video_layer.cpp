// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
/**********************************************************************
    Spectrum Next Video Layer (Interface)
**********************************************************************/

#include "emu.h"
#include "specnext_video_layer.h"


specnext_video_layer_interface::specnext_video_layer_interface(const machine_config &mconfig, device_t &device, const gfx_decode_entry *gfxinfo)
	: device_gfx_interface(mconfig, device, gfxinfo)
{
}

specnext_video_layer_interface::~specnext_video_layer_interface()
{
}


specnext_video_layer_interface &specnext_video_layer_interface::set_palette(const char *tag, u16 base_offset, u16 alt_offset)
{
	device_gfx_interface::set_palette(tag);
	m_palette_base_offset = base_offset,
	m_palette_alt_offset = alt_offset;
	return *this;
}


void specnext_video_layer_interface::interface_post_start()
{
	device().save_item(NAME(m_offset_h));
	device().save_item(NAME(m_offset_v));
	device().save_item(NAME(m_global_transparent));
	device().save_item(NAME(m_alt_palette_select));
}

void specnext_video_layer_interface::interface_post_reset()
{
	m_offset_h = 0;
	m_offset_v = 0;

	m_global_transparent = 0xaa; // TODO feature toggle
	m_alt_palette_select = 0;
}
