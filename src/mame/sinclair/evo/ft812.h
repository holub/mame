// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
/***************************************************************************

    FTDI FT812 "Eve" Embedded Video Engine (VDAC2 board for ZX-Evolution)

    SPI-attached GPU with 1 MB graphics RAM (RAM_G), 8 KB display list
    memory (RAM_DL), register file (RAM_REG) and 4 KB co-processor fifo
    (RAM_CMD). Hosts issue 24-bit addressed read/write transactions plus
    3-byte host commands, all wrapped in chip select.

    References:
    * FT81x Series Programmer Guide (FTDI)
    * ZX-Evolution VDAC2 tutorial (Zuna Deluxe, tslabs)

    TODO:
    * remaining co-processor commands and full widget fidelity
    * streamed image loading (CMD_LOADIMAGE)
    * touch engine, audio
    * TEXT8X8/TEXTVGA and BARGRAPH bitmap formats

***************************************************************************/

#ifndef MAME_SINCLAIR_EVO_FT812_H
#define MAME_SINCLAIR_EVO_FT812_H

#pragma once

#include <stack>

#include "screen.h"


class ft812_device : public device_t, public device_memory_interface
{
public:
	ft812_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto spi_miso_callback() { return m_miso_cb.bind(); }
	auto int_callback() { return m_int_cb.bind(); }

	void spi_mosi_w(int state) { m_in_bit = state ? 1 : 0; }
	void spi_clock_w(int state);
	void spi_ss_w(int state);

	// current display list render target (for host video mixing)
	bitmap_rgb32 &framebuffer() { return m_bitmap; }

	static constexpr int AS_FT = AS_OPCODES + 1;

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

	void draw_rom_font(s32 x, s32 y, u8 handle, u8 cell);
	bool rom_font_desc(u32 font, u32 &fmt, u32 &stride, u32 &w, u32 &h, u32 &gfx) const;
	const u8 *m_font_rom = nullptr;      // optional font.bin, verbatim ROM window 0x1e0000-0x300000

	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	void postload_reconfig() ATTR_COLD;
	virtual void device_stop() override;
	virtual space_config_vector memory_space_config() const override;

private:
	enum spi_state : u8
	{
		SPI_HDR,
		SPI_ADDR_MID,
		SPI_ADDR_LO,
		SPI_READ,
		SPI_WRITE,
		SPI_HOSTCMD_1,
		SPI_HOSTCMD_2
	};

	// host commands
	static constexpr u8 CMD_ACTIVE  = 0x00;
	static constexpr u8 CMD_CLKEXT  = 0x44;
	static constexpr u8 CMD_CLKSEL  = 0x61;
	static constexpr u8 CMD_PWRDOWN = 0x50;

	// memory map
	static constexpr u32 RAM_G_SIZE   = 0x100000;                    // 0x000000
	static constexpr u32 RAM_DL_BASE  = 0x300000, RAM_DL_SIZE = 0x2000;
	static constexpr u32 RAM_REG_BASE = 0x302000, RAM_REG_SIZE = 0x1000;
	static constexpr u32 RAM_CMD_BASE = 0x308000, RAM_CMD_SIZE = 0x1000;
	// optional font ROM (user supplied dump of the FT812 font window 0x1e0000-0x300000)
	static constexpr u32 FONT_ROM_SIZE   = 0x120000;
	static constexpr u32 FONT_ROM_TABLE  = 0x021ee0; // 19 x 148 byte font metrics blocks (PG 5.5.1)


	// RAM_REG offsets (from 0x302000), per TSLib 81x Const.inc
	static constexpr u32 REG_ID          = 0x000;
	static constexpr u32 REG_FRAMES      = 0x004;
	static constexpr u32 REG_FREQUENCY   = 0x00c;
	static constexpr u32 REG_CPURESET    = 0x020;
	static constexpr u32 REG_HCYCLE      = 0x02c;
	static constexpr u32 REG_HOFFSET     = 0x030;
	static constexpr u32 REG_HSIZE       = 0x034;
	static constexpr u32 REG_HSYNC0      = 0x038;
	static constexpr u32 REG_HSYNC1      = 0x03c;
	static constexpr u32 REG_VCYCLE      = 0x040;
	static constexpr u32 REG_VOFFSET     = 0x044;
	static constexpr u32 REG_VSIZE       = 0x048;
	static constexpr u32 REG_VSYNC0      = 0x04c;
	static constexpr u32 REG_VSYNC1      = 0x050;
	static constexpr u32 REG_DLSWAP      = 0x054;
	static constexpr u32 REG_PCLK_POL    = 0x06c;
	static constexpr u32 REG_PCLK        = 0x070;
	static constexpr u32 REG_INT_FLAGS   = 0x0a8; // clear-on-read
	static constexpr u32 REG_INT_EN      = 0x0ac;
	static constexpr u32 REG_INT_MASK    = 0x0b0;
	static constexpr u32 REG_CMD_DL      = 0x100;
	static constexpr u32 REG_MACRO_0     = 0x0d8;
	static constexpr u32 REG_MACRO_1     = 0x0dc;
	static constexpr u32 REG_CMD_READ    = 0x0f8; // 16-bit
	static constexpr u32 REG_CMD_WRITE   = 0x0fc; // 16-bit
	static constexpr u32 REG_CMDB_SPACE  = 0x574; // free fifo bytes, 16-bit
	static constexpr u32 REG_CMDB_WRITE  = 0x578; // word write -> RAM_CMD

	static constexpr u32 INT_SWAP = 0x01;

	// display list opcodes (top byte)
	static constexpr u32 DL_BITMAP_SOURCE    = 0x01'000000;
	static constexpr u32 DL_CLEAR_COLOR_RGB  = 0x02'000000;
	static constexpr u32 DL_BITMAP_HANDLE    = 0x05'000000;
	static constexpr u32 DL_CELL             = 0x06'000000;
	static constexpr u32 DL_BITMAP_LAYOUT    = 0x07'000000;
	static constexpr u32 DL_BITMAP_SIZE      = 0x08'000000;
	static constexpr u32 DL_BITMAP_LAYOUT_H  = 0x28'000000;
	static constexpr u32 DL_BITMAP_SIZE_H    = 0x29'000000;
	static constexpr u32 DL_BITMAP_TRANSFORM_A = 0x15'000000;
	static constexpr u32 DL_BITMAP_TRANSFORM_B = 0x16'000000;
	static constexpr u32 DL_BITMAP_TRANSFORM_C = 0x17'000000;
	static constexpr u32 DL_BITMAP_TRANSFORM_D = 0x18'000000;
	static constexpr u32 DL_BITMAP_TRANSFORM_E = 0x19'000000;
	static constexpr u32 DL_BITMAP_TRANSFORM_F = 0x1a'000000;
	static constexpr u32 DL_STENCIL_FUNC      = 0x0a'000000;
	static constexpr u32 DL_CLEAR_STENCIL     = 0x11'000000;
	static constexpr u32 DL_TAG               = 0x03'000000;
	static constexpr u32 DL_STENCIL_MASK      = 0x13'000000;
	static constexpr u32 DL_TAG_MASK          = 0x14'000000;
	static constexpr u32 DL_COLOR_MASK        = 0x20'000000;
	static constexpr u32 DL_VERTEX_FORMAT     = 0x27'000000;
	static constexpr u32 DL_CLEAR_TAG         = 0x12'000000;
	static constexpr u32 DL_CLEAR_COLOR_A    = 0x0f'000000;
	static constexpr u32 DL_COLOR_RGB        = 0x04'000000;
	static constexpr u32 DL_POINT_SIZE       = 0x0d'000000;
	static constexpr u32 DL_LINE_WIDTH       = 0x0e'000000;
	static constexpr u32 DL_COLOR_A          = 0x10'000000;
	static constexpr u32 DL_BLEND_FUNC       = 0x0b'000000;
	static constexpr u32 DL_ALPHA_FUNC       = 0x09'000000;
	static constexpr u32 DL_STENCIL_OP       = 0x0c'000000;
	static constexpr u32 DL_SCISSOR_XY       = 0x1b'000000;
	static constexpr u32 DL_SCISSOR_SIZE     = 0x1c'000000;
	static constexpr u32 DL_BEGIN            = 0x1f'000000;
	static constexpr u32 DL_END              = 0x21'000000;
	static constexpr u32 DL_SAVE_CONTEXT     = 0x22'000000;
	static constexpr u32 DL_RESTORE_CONTEXT  = 0x23'000000;
	static constexpr u32 DL_CLEAR            = 0x26'000000;
	static constexpr u32 DL_MACRO            = 0x25'000000;
	static constexpr u32 DL_DISPLAY          = 0x00'000000;

	// primitives
	static constexpr u8 PRIM_BITMAPS     = 1;
	static constexpr u8 PRIM_POINTS      = 2;
	static constexpr u8 PRIM_LINES       = 3;
	static constexpr u8 PRIM_LINE_STRIP  = 4;
	static constexpr u8 PRIM_EDGE_STRIP_R = 5;
	static constexpr u8 PRIM_EDGE_STRIP_L = 6;
	static constexpr u8 PRIM_EDGE_STRIP_A = 7;
	static constexpr u8 PRIM_EDGE_STRIP_B = 8;
	static constexpr u8 PRIM_RECTS       = 9;

	struct bitmap_state
	{
		u32 source = 0;
		u32 layout = 0, size = 0;
		u32 layout_h = 0, size_h = 0;
		u8 filter = 0, wrap_x = 0, wrap_y = 0;
	};

	// bitmap formats (BITMAP_LAYOUT)
	// bitmap formats per FT81x PG table 7 / TSLib 81x Const.inc
	static constexpr u32 FMT_ARGB1555 = 0;
	static constexpr u32 FMT_L1       = 1;
	static constexpr u32 FMT_L4       = 2;
	static constexpr u32 FMT_L8       = 3;
	static constexpr u32 FMT_RGB332   = 4;
	static constexpr u32 FMT_ARGB2    = 5;
	static constexpr u32 FMT_ARGB4    = 6;
	static constexpr u32 FMT_RGB565   = 7;
	static constexpr u32 FMT_TEXT8X8  = 9;
	static constexpr u32 FMT_PAL565   = 14;
	static constexpr u32 FMT_PAL4444  = 15;
	static constexpr u32 FMT_PAL8     = 16;
	static constexpr u32 FMT_L2       = 17;
	struct dl_context
	{
		u32 color = 0xffffffff;
		u32 clear_color = 0;
		u32 clear_a = 0;
		u32 clear_stencil = 0, clear_tag = 0;
		u32 point_size = 16, line_width = 16;
		s32 bitmap_transform_a = 256, bitmap_transform_b = 0;
		s32 bitmap_transform_c = 0, bitmap_transform_d = 0;
		s32 bitmap_transform_e = 256, bitmap_transform_f = 0;
		u8 bitmap_handle = 0, cell = 0;
		s32 scissor_x = 0, scissor_y = 0, scissor_w = 2048, scissor_h = 2048;
		u8 alpha_func = 7, alpha_ref = 0; // ALWAYS
		u8 stencil_func = 7, stencil_ref = 0, stencil_read_mask = 0xff;
		u8 stencil_write_mask = 0xff, stencil_fail = 1, stencil_pass = 1;
		u8 tag = 0, tag_mask = 1, color_mask = 0x0f, vertex_format = 4;
		u8 blend_src = 2, blend_dst = 4; // SRC_ALPHA, ONE_MINUS_SRC_ALPHA
		s32 vtx_tx = 0, vtx_ty = 0; // VERTEX_TRANSLATE_X/Y, 1/16 pixel
		u32 palette_source = 0;
	};
	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	void byte_in(u8 data);
	void load_output(u8 data) { m_out_latch = data; }
	void host_command(u8 cmd, u8 arg);

	void ft_map(address_map &map) ATTR_COLD;
	u8 reg_r(offs_t reg);
	u8 cmd_r(offs_t off);
	void cmd_w(offs_t off, u8 data);
	void reg_w(offs_t reg, u8 data);
	u32 reg_r32(u32 reg);
	void reg_w32(u32 reg, u32 data);
	void update_int();
	void reconfigure_timing();

	void render_dl();
	void copro_execute();
	void draw_px(s32 x, s32 y, u32 argb);
	void draw_point(s32 x, s32 y);
	void draw_line(s32 x0, s32 y0, s32 x1, s32 y1);
	void draw_rect(s32 x0, s32 y0, s32 x1, s32 y1);
	void draw_edge_segment(s32 x0, s32 y0, s32 x1, s32 y1);
	void draw_bitmap(s32 x, s32 y, u8 handle, u8 cell);
	u32 bitmap_pixel(u32 byte_addr, u32 fmt, s32 u, s32 v, u32 stride, u32 lines);

	devcb_write_line m_miso_cb;
	devcb_write_line m_int_cb;

	screen_device *m_screen;
	address_space *m_space;
	address_space_config m_space_cfg;

	memory_share_creator<u8> m_ram_g;   // 0x000000, 1 MB
	memory_share_creator<u8> m_ram_dl;  // 0x300000, 8 KB
	memory_share_creator<u8> m_ram_cmd; // 0x308000, 4 KB
	std::vector<u8> m_ram_reg;          // 0x302000, 4 KB

	bitmap_rgb32 m_bitmap;
	bitmap_ind8 m_stencil;
	bitmap_ind8 m_tag_buffer;
	bitmap_state m_bitmap_state[32]{};

	// spi engine
	u8 m_in_bit, m_in_latch, m_out_latch, m_cur_bit, m_ss;
	spi_state m_spi_state;
	u32 m_addr;
	u8 m_write;
	u8 m_cmdb_burst;
	u8 m_hostcmd[3];
	u8 m_hostcmd_pos;

	// registers
	bool m_swap_pending;
	bool m_timing_dirty;
	u32 m_dl_ptr;
	u32 m_frames;
	u32 m_copro_fgcolor;
	double m_copro_matrix[6] = { 1.0, 0.0, 0.0, 1.0, 0.0, 0.0 };
	bool m_inflate_active;
	u32 m_inflate_addr;
	void *m_inflate_zs; // z_stream kept opaque in the header

	// render state
	dl_context m_ctx;
	std::stack<dl_context> m_ctx_stack;
	u8 m_prim;
	s32 m_last_x, m_last_y;
	bool m_have_vertex;
};

DECLARE_DEVICE_TYPE(FT812, ft812_device)

#endif // MAME_SINCLAIR_EVO_FT812_H
