// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
#include "emu.h"
#include "ft812.h"

#include "drawgfx.h"
#include "machine/ram.h"
#include "multibyte.h"
#include <charconv>
#include <cmath>
#include <string>
#include <zlib.h>

#define LOG_WARN    (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(FT812, ft812_device, "ft812", "FTDI FT812 Embedded Video Engine")

// optional user supplied dump of the FT812 ROM font window (0x1e0000-0x300000);
// layout: 19 x 148 byte font metrics blocks at 0x21ee0, ROM_FONTROOT pointer at 0xffffc (PG 5.5.1).
// The ROM font data is proprietary and is therefore not distributed with MAME.
ROM_START( ft812 )
	ROM_REGION( 0x120000, "fonts", ROMREGION_ERASE00 )
	ROM_LOAD_OPTIONAL( "font.bin", 0x000000, 0x120000, NO_DUMP )
ROM_END

const tiny_rom_entry *ft812_device::device_rom_region() const
{
	return ROM_NAME( ft812 );
}


ft812_device::ft812_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, FT812, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, m_miso_cb(*this)
	, m_int_cb(*this)
	, m_screen(nullptr)
	, m_space(nullptr)
	, m_space_cfg("ft", ENDIANNESS_LITTLE, 8, 22, 0, address_map_constructor(FUNC(ft812_device::ft_map), this))
	, m_ram_g(*this, "ram_g", RAM_G_SIZE, ENDIANNESS_LITTLE)
	, m_ram_dl(*this, "ram_dl", RAM_DL_SIZE, ENDIANNESS_LITTLE)
	, m_ram_cmd(*this, "ram_cmd", RAM_CMD_SIZE, ENDIANNESS_LITTLE)
	, m_swap_pending(false)
	, m_frames(0)
	, m_copro_fgcolor(0x00ffffff)
{
}

void ft812_device::ft_map(address_map &map)
{
	map(0x000000, RAM_G_SIZE - 1).ram().share("ram_g");
	map(RAM_DL_BASE, RAM_DL_BASE + RAM_DL_SIZE - 1).ram().share("ram_dl");
	map(RAM_REG_BASE, RAM_REG_BASE + RAM_REG_SIZE - 1).rw(FUNC(ft812_device::reg_r), FUNC(ft812_device::reg_w));
	map(RAM_CMD_BASE, RAM_CMD_BASE + RAM_CMD_SIZE - 1).rw(FUNC(ft812_device::cmd_r), FUNC(ft812_device::cmd_w));
}

ft812_device::space_config_vector ft812_device::memory_space_config() const
{
	return space_config_vector{ std::make_pair(AS_FT, &m_space_cfg) };
}

void ft812_device::device_add_mconfig(machine_config &config)
{
	screen_device &screen(SCREEN(config, "screen"));
	// default timing: 640x480 visible; guest reconfigures via RAM_REG later
	screen.set_raw(32_MHz_XTAL, 832, 0, 640, 520, 0, 480);
	screen.set_screen_update(FUNC(ft812_device::screen_update));
}

void ft812_device::device_start()
{
	m_screen = subdevice<screen_device>("screen");
	m_space = &space(AS_FT);
	m_inflate_zs = new z_stream;
	m_inflate_active = false;
	m_cmdb_burst = 0;

	memory_region *const region = memregion("fonts");
	if (region && region->bytes() >= FONT_ROM_SIZE && get_u32le(&region->base()[0x11fffc]) == 0x201ee0)
		m_font_rom = region->base();
	else if (region)
		logerror("%s: font.bin is not an FT812 ROM font dump (bad ROM_FONTROOT), using built-in font\n", tag());

	m_ram_reg.resize(RAM_REG_SIZE);
	m_bitmap.allocate(640, 480);
	m_stencil.allocate(640, 480);
	m_tag_buffer.allocate(640, 480);
	m_bitmap.fill(0);
	m_stencil.fill(0);
	m_tag_buffer.fill(0);

	save_item(NAME(m_ram_reg));
	save_item(NAME(m_in_bit));
	save_item(NAME(m_in_latch));
	save_item(NAME(m_out_latch));
	save_item(NAME(m_cur_bit));
	save_item(NAME(m_ss));
	save_item(NAME(m_addr));
	save_item(NAME(m_swap_pending));
	save_item(NAME(m_timing_dirty));
	save_item(NAME(m_write));
	save_item(NAME(m_frames));
	save_item(NAME(m_copro_fgcolor));
	save_item(NAME(m_copro_matrix));
	save_item(NAME(m_prim));
	save_item(NAME(m_last_x));
	save_item(NAME(m_last_y));
	save_item(NAME(m_have_vertex));

	m_screen->register_vblank_callback(vblank_state_delegate(
		[this](screen_device &screen, bool vblank_state)
		{
		if (vblank_state)
		{
			if (m_timing_dirty)
			{
				m_timing_dirty = false;
				reconfigure_timing();
			}
			m_frames++;
			reg_w32(REG_FRAMES, m_frames);
			if (m_swap_pending)
			{
				m_swap_pending = false;
				render_dl();
				reg_w32(REG_DLSWAP, 0);
				reg_w32(REG_INT_FLAGS, reg_r32(REG_INT_FLAGS) | INT_SWAP);
				update_int();
			}
		}
	}));
}

void ft812_device::device_stop()
{
	if (m_inflate_active)
		inflateEnd(reinterpret_cast<z_stream *>(m_inflate_zs));
	delete reinterpret_cast<z_stream *>(m_inflate_zs);
	m_inflate_zs = nullptr;
}


void ft812_device::device_reset()
{
	m_spi_state = SPI_HDR;
	m_addr = 0;
	m_cur_bit = 0;
	m_in_latch = 0;
	m_out_latch = 0xff;
	m_hostcmd_pos = 0;
	m_swap_pending = false;
	m_dl_ptr = 0;
	m_frames = 0;
	m_copro_fgcolor = 0x00ffffff;
	m_copro_matrix[0] = 1.0;
	m_copro_matrix[1] = 0.0;
	m_copro_matrix[2] = 0.0;
	m_copro_matrix[3] = 1.0;
	m_copro_matrix[4] = 0.0;
	m_copro_matrix[5] = 0.0;
	if (m_inflate_active)
	{
		inflateEnd(reinterpret_cast<z_stream *>(m_inflate_zs));
		m_inflate_active = false;
	}

	std::fill(m_ram_reg.begin(), m_ram_reg.end(), 0);
	// reset values per programmer guide / vdac2 tutorial
	reg_w32(REG_ID, 0x7c);
	reg_w32(REG_FREQUENCY, 60000000);
	reg_w32(REG_HCYCLE, 0x224);
	reg_w32(REG_HOFFSET, 0x02b);
	reg_w32(REG_HSIZE, 0x1e0);
	reg_w32(REG_HSYNC0, 0x000);
	reg_w32(REG_HSYNC1, 0x029);
	reg_w32(REG_VCYCLE, 0x124);
	reg_w32(REG_VOFFSET, 0x00c);
	reg_w32(REG_VSIZE, 0x110);
	reg_w32(REG_VSYNC0, 0x000);
	reg_w32(REG_VSYNC1, 0x00a);
	reg_w32(REG_CPURESET, 0x48); // per guide; software waits for 0 after ACTIVE

	m_timing_dirty = false;
	m_int_cb(CLEAR_LINE);
}

void ft812_device::update_int()
{
	const bool state = (reg_r32(REG_INT_FLAGS) & reg_r32(REG_INT_MASK)) && BIT(reg_r32(REG_INT_EN), 0);
	m_int_cb(state ? ASSERT_LINE : CLEAR_LINE);
}

void ft812_device::reconfigure_timing()
{
	const u32 hcycle = reg_r32(REG_HCYCLE), vcycle = reg_r32(REG_VCYCLE);
	const u32 hsize = reg_r32(REG_HSIZE), vsize = reg_r32(REG_VSIZE);
	const u32 hoffset = reg_r32(REG_HOFFSET), voffset = reg_r32(REG_VOFFSET);
	if (!hsize || !vsize || hsize > 2048 || vsize > 2048 || hcycle < hsize + hoffset || vcycle < vsize + voffset)
		return;

	const u32 pclk = std::max<u32>(1, reg_r32(REG_PCLK) & 0xffff);
	const double frame_hz = double(reg_r32(REG_FREQUENCY)) / pclk / (double(hcycle) * vcycle);
	if (!(frame_hz > 1) || frame_hz > 1000)
		return;

	m_screen->configure(hcycle, vcycle,
			rectangle(hoffset, hoffset + hsize - 1, voffset, voffset + vsize - 1),
			attotime::from_hz(frame_hz));
	m_bitmap.allocate(hsize, vsize);
	m_stencil.allocate(hsize, vsize);
	m_tag_buffer.allocate(hsize, vsize);
	m_bitmap.fill(0);
	m_stencil.fill(0);
	m_tag_buffer.fill(0);
}

void ft812_device::spi_ss_w(int state)
{
	if (m_ss && !state)
	{
		// transaction ends
		m_spi_state = SPI_HDR;
		m_cur_bit = 0;
		m_hostcmd_pos = 0;
		m_cmdb_burst = false;
		load_output(0xff);
	}
	m_ss = state ? 1 : 0;
}

void ft812_device::spi_clock_w(int state)
{
	if (!m_ss || !state)
		return;

	// sample MOSI on rising edge, shift MISO out MSB first
	m_in_latch <<= 1;
	m_in_latch |= m_in_bit;
	m_miso_cb(BIT(m_out_latch, 7));
	m_out_latch <<= 1;
	m_out_latch |= 1;

	if (++m_cur_bit == 8)
	{
		m_cur_bit = 0;
		byte_in(m_in_latch);
		m_in_latch = 0;
	}
}

void ft812_device::byte_in(u8 data)
{
	switch (m_spi_state)
	{
	case SPI_HDR:
		if ((data & 0xc0) == 0x40)
		{
			m_hostcmd[0] = data;
			m_spi_state = SPI_HOSTCMD_1;
		}
		else
		{
			m_write = BIT(data, 7); // 10b prefix = write, 00b = read
			m_addr = u32(data & 0x3f) << 16;
			m_spi_state = SPI_ADDR_MID;
		}
		load_output(0xff);
		break;

	case SPI_ADDR_MID:
		m_addr |= data << 8;
		m_spi_state = SPI_ADDR_LO;
		load_output(0xff);
		break;

	case SPI_ADDR_LO:
		m_addr = (m_addr | data) & 0x3fffff;
		if (m_write)
		{
			// silicon keeps the address frozen for co-processor fifo bursts:
			// every data byte of the transaction appends to RAM_CMD
			m_cmdb_burst = (m_addr == RAM_REG_BASE + REG_CMDB_WRITE);
			m_spi_state = SPI_WRITE;
		}
		else
		{
			// one dummy byte, then data
			load_output(0xff);
			m_spi_state = SPI_READ;
		}
		break;

	case SPI_READ:
		// guest clocks in junk; reply with data for the *next* byte
		load_output(m_space->read_byte(m_addr));
		m_addr = (m_addr + 1) & 0x3fffff;
		break;

	case SPI_WRITE:
		m_space->write_byte(m_addr, data);
		if (!m_cmdb_burst)
			m_addr = (m_addr + 1) & 0x3fffff;
		load_output(0xff);
		break;

	case SPI_HOSTCMD_1:
		m_hostcmd[1] = data;
		m_spi_state = SPI_HOSTCMD_2;
		load_output(0xff);
		break;

	case SPI_HOSTCMD_2:
		m_hostcmd[2] = data;
		host_command(m_hostcmd[0], m_hostcmd[1]);
		m_spi_state = SPI_HDR;
		load_output(0xff);
		break;
	}
}

void ft812_device::host_command(u8 cmd, u8 arg)
{
	switch (cmd)
	{
	case CMD_ACTIVE:
		reg_w32(REG_CPURESET, 0); // coprocessor ready
		break;
	case CMD_CLKSEL:
		// external 8 MHz clock, low 6 bits of arg = multiplier (F_MUL)
		reg_w32(REG_FREQUENCY, u32(8'000'000 * (arg & 0x3f)));
		[[fallthrough]];
	case CMD_CLKEXT:
		// clock applied: coprocessor comes out of reset (matches silicon:
		// guests send PWRDOWN, then CLKEXT/CLKSEL, then poll CPURESET == 0)
		reg_w32(REG_CPURESET, 0);
		break;
	case CMD_PWRDOWN:
		reg_w32(REG_CPURESET, 0x48);
		break;
	default:
		break;
	}
}

u32 ft812_device::reg_r32(u32 reg)
{
	if (reg + 3 >= RAM_REG_SIZE)
		return 0;
	return get_u32le(&m_ram_reg[reg]);
}

void ft812_device::reg_w32(u32 reg, u32 data)
{
	if (reg + 3 >= RAM_REG_SIZE)
		return;
	put_u32le(&m_ram_reg[reg], data);

	switch (reg)
	{
	case REG_DLSWAP:
			if (data & 2)
			m_swap_pending = true;
		break;
	case REG_INT_MASK:
	case REG_INT_EN:
		update_int();
		break;
	case REG_CMD_WRITE:
		copro_execute();
		break;
	case REG_CMDB_WRITE:
	{
		// byte-granular append: a write of any width adds its bytes to the fifo
		const u32 w = reg_r32(REG_CMD_WRITE) & 0xfff;
		for (u32 i = 0; i < 4; i++)
			m_ram_cmd[(w + i) & 0xfff] = (data >> (8 * i)) & 0xff;
		reg_w32(REG_CMD_WRITE, (w + 4) & 0xfff);
		break;
	}
	case REG_FREQUENCY:
	case REG_HCYCLE:
	case REG_HOFFSET:
	case REG_HSIZE:
	case REG_VCYCLE:
	case REG_VOFFSET:
	case REG_VSIZE:
	case REG_PCLK:
		m_timing_dirty = true;
		break;
	default:
		break;
	}
}

u8 ft812_device::cmd_r(offs_t off)
{
	return m_ram_cmd[off & 0xfff];
}

void ft812_device::cmd_w(offs_t off, u8 data)
{
	m_ram_cmd[off & 0xfff] = data;
}

u8 ft812_device::reg_r(offs_t reg)
{
	switch (reg)
	{
	case REG_INT_FLAGS:
	{
		const u32 val = reg_r32(REG_INT_FLAGS);
		reg_w32(REG_INT_FLAGS, 0); // clear on read
		update_int();
		return val; // byte reads only use offset % 4 == 0 in practice
	}
	case REG_CMDB_SPACE:
	case REG_CMDB_SPACE + 1:
	case REG_CMDB_SPACE + 2:
	case REG_CMDB_SPACE + 3:
	{
		// fifo always drained in this model: report full free space
		return (RAM_CMD_SIZE >> (8 * (reg & 3))) & 0xff;
	}
	default:
		return m_ram_reg[reg];
	}
}

void ft812_device::reg_w(offs_t reg, u8 data)
{
	if (reg == REG_INT_FLAGS)
		return; // writes ignored

	if (reg >= REG_CMDB_WRITE && reg < REG_CMDB_WRITE + 4)
	{
		// each byte written through the window appends to the fifo
		const u32 w = reg_r32(REG_CMD_WRITE) & 0xfff;
		m_ram_cmd[w] = data;
		reg_w32(REG_CMD_WRITE, (w + 1) & 0xfff);
		return;
	}

	const u32 shift = 8 * (reg & 3);
	const u32 old = reg_r32(reg & ~3);
	reg_w32(reg & ~3, (old & ~(0xffU << shift)) | (u32(data) << shift));
}

u32 ft812_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	// cliprect/bitmap are in total-raster coordinates once the timing has been
	// reconfigured; m_bitmap is visible-area relative
	const rectangle vis = screen.visible_area();
	copybitmap(bitmap, m_bitmap, 0, 0, vis.left(), vis.top(), cliprect);
	return 0;
}

void ft812_device::draw_px(s32 x, s32 y, u32 argb)
{
	if (x < m_ctx.scissor_x || y < m_ctx.scissor_y)
		return;
	if (x >= m_ctx.scissor_x + m_ctx.scissor_w || y >= m_ctx.scissor_y + m_ctx.scissor_h)
		return;
	if (x < 0 || y < 0 || x >= m_bitmap.width() || y >= m_bitmap.height())
		return;

	const u32 a = (argb >> 24) & 0xff;

	// alpha test (ALPHA_FUNC: 0 NEVER, 1 LESS, 2 LEQUAL, 3 GREATER, 4 GEQUAL, 5 EQUAL, 6 NOTEQUAL, 7 ALWAYS)
	switch (m_ctx.alpha_func)
	{
	case 0: return;
	case 1: if (!(a < m_ctx.alpha_ref)) return; break;
	case 2: if (!(a <= m_ctx.alpha_ref)) return; break;
	case 3: if (!(a > m_ctx.alpha_ref)) return; break;
	case 4: if (!(a >= m_ctx.alpha_ref)) return; break;
	case 5: if (!(a == m_ctx.alpha_ref)) return; break;
	case 6: if (!(a != m_ctx.alpha_ref)) return; break;
	default: break;
	}

	const u32 stencil = m_stencil.pix(y, x);
	const u32 ref = m_ctx.stencil_ref & m_ctx.stencil_read_mask;
	const u32 value = stencil & m_ctx.stencil_read_mask;
	bool stencil_pass = false;
	switch (m_ctx.stencil_func)
	{
	case 0: stencil_pass = false; break;
	case 1: stencil_pass = ref < value; break;
	case 2: stencil_pass = ref <= value; break;
	case 3: stencil_pass = ref > value; break;
	case 4: stencil_pass = ref >= value; break;
	case 5: stencil_pass = ref == value; break;
	case 6: stencil_pass = ref != value; break;
	default: stencil_pass = true; break;
	}

	auto stencil_op = [](u8 op, u8 old_value, u8 stencil_ref)
	{
		switch (op)
		{
		case 0: return u8(0);
		case 2: return stencil_ref;
		case 3: return u8(std::min<u32>(0xff, old_value + 1));
		case 4: return u8(old_value ? old_value - 1 : 0);
		case 5: return u8(~old_value);
		case 6: return u8(old_value + 1);
		case 7: return u8(old_value - 1);
		default: return old_value;
		}
	};
	const u8 selected_op = stencil_pass ? m_ctx.stencil_pass : m_ctx.stencil_fail;
	if (!stencil_pass || m_ctx.stencil_write_mask)
	{
		const u8 result = stencil_op(selected_op, u8(stencil), m_ctx.stencil_ref);
		m_stencil.pix(y, x) = (u8(stencil) & ~m_ctx.stencil_write_mask) | (result & m_ctx.stencil_write_mask);
	}
	if (!stencil_pass)
		return;

	if (m_ctx.tag_mask)
		m_tag_buffer.pix(y, x) = m_ctx.tag;

	const u32 d = m_bitmap.pix(y, x);
	auto blend_factor = [a, d](u8 factor)
	{
		switch (factor)
		{
		case 0: return 0U;
		case 1: return 255U;
		case 2: return a;
		case 3: return (d >> 24) & 0xff;
		case 4: return 255U - a;
		case 5: return 255U - ((d >> 24) & 0xff);
		default: return 0U;
		}
	};
	const u32 fs = blend_factor(m_ctx.blend_src);
	const u32 fd = blend_factor(m_ctx.blend_dst);
	u32 out = 0;
	for (u32 shift : { 0U, 8U, 16U, 24U })
	{
		const u32 src = (argb >> shift) & 0xff;
		const u32 dst = (d >> shift) & 0xff;
		const u32 component = std::min<u32>(255, (src * fs + dst * fd + 127) / 255);
		const u32 mask_bit = shift == 24 ? 0x01 : 1U << (shift / 8 + 1);
		const u32 selected = (m_ctx.color_mask & mask_bit) ? component : dst;
		out |= selected << shift;
	}
	m_bitmap.pix(y, x) = out;
}

u32 ft812_device::bitmap_pixel(u32 byte_addr, u32 fmt, s32 u, s32 v, u32 stride, u32 lines)
{
	const u32 idx = byte_addr + v * stride + u;
	const u32 idx16 = byte_addr + v * stride + u * 2;
	const u32 b = m_ram_g[idx & (RAM_G_SIZE - 1)];
	const u32 alpha = (m_ctx.color >> 24) & 0xff;
	const u32 color = m_ctx.color & 0x00ffffff;
	auto modulate = [color, alpha](u32 pixel)
	{
		const u32 a = ((pixel >> 24) & 0xff) * alpha / 255;
		const u32 r = ((pixel >> 16) & 0xff) * ((color >> 16) & 0xff) / 255;
		const u32 g = ((pixel >> 8) & 0xff) * ((color >> 8) & 0xff) / 255;
		const u32 bl = (pixel & 0xff) * (color & 0xff) / 255;
		return (a << 24) | (r << 16) | (g << 8) | bl;
	};
	auto luminance = [color, alpha](u32 level)
	{
		return ((level * alpha / 255) << 24)
			| (((color >> 16) & 0xff) * level / 255 << 16)
			| (((color >> 8) & 0xff) * level / 255 << 8)
			| ((color & 0xff) * level / 255);
	};
	switch (fmt)
	{
	case FMT_L8:
		return luminance(b);
	case FMT_RGB332: // r3 g3 b2
		return modulate(0xff000000
			| (u32(b >> 5) * 255 / 7 << 16)
			| (u32((b >> 2) & 7) * 255 / 7 << 8)
			| (u32(b & 3) * 255 / 3));
	case FMT_ARGB1555:
	{
		const u16 w = get_u16le(&m_ram_g[idx16 & (RAM_G_SIZE - 2)]);
		return modulate((BIT(w, 15) ? 0xff000000 : 0)
			| (u32((w >> 10) & 0x1f) * 255 / 31 << 16)
			| (u32((w >> 5) & 0x1f) * 255 / 31 << 8)
			| (u32(w & 0x1f) * 255 / 31));
	}
	case FMT_ARGB4: // a4 r4 g4 b4
	{
		const u16 w = get_u16le(&m_ram_g[idx16 & (RAM_G_SIZE - 2)]);
		return modulate(u32((w >> 12) & 0xf) * 17 << 24 | u32((w >> 8) & 0xf) * 17 << 16
			| u32((w >> 4) & 0xf) * 17 << 8 | u32(w & 0xf) * 17);
	}
	case FMT_RGB565:
	{
		const u16 w = get_u16le(&m_ram_g[idx16 & (RAM_G_SIZE - 2)]);
		return modulate(0xff000000
			| (u32((w >> 11) & 0x1f) * 255 / 31 << 16)
			| (u32((w >> 5) & 0x3f) * 255 / 63 << 8)
			| (u32(w & 0x1f) * 255 / 31));
	}
	case FMT_L4: // 2 px/byte, high nibble first; default palette = luminance ramp
	{
		const u32 by = m_ram_g[(byte_addr + v * stride + u / 2) & (RAM_G_SIZE - 1)];
		const u32 n = (u & 1) ? (by & 0xf) : (by >> 4);
		return luminance(n * 17);
	}
	case FMT_L2: // 4 px/byte
	{
		const u32 by = m_ram_g[(byte_addr + v * stride + u / 4) & (RAM_G_SIZE - 1)];
		const u32 n = (by >> ((3 - (u & 3)) * 2)) & 3;
		return luminance(n * 85);
	}
	case FMT_L1: // 8 px/byte
	{
		const u32 by = m_ram_g[(byte_addr + v * stride + u / 8) & (RAM_G_SIZE - 1)];
		const u32 n = (by >> (7 - (u & 7))) & 1;
		return luminance(n ? 0xff : 0);
	}
	case FMT_PAL8: // index into 256-entry RGBA table at PALETTE_SOURCE
	{
		const u32 w = get_u32le(&m_ram_g[(m_ctx.palette_source + 4 * b) & (RAM_G_SIZE - 4)]);
		return modulate(w);
	}
	case FMT_PAL4444: // 8bpp index into 256-entry ARGB4444 table
	{
		const u16 w = get_u16le(&m_ram_g[(m_ctx.palette_source + 2 * b) & (RAM_G_SIZE - 2)]);
		return modulate(u32((w >> 12) & 0xf) * 17 << 24 | u32((w >> 8) & 0xf) * 17 << 16
			| u32((w >> 4) & 0xf) * 17 << 8 | u32(w & 0xf) * 17);
	}
	case FMT_PAL565: // 8bpp index into 256-entry RGB565 table
	{
		const u16 w = get_u16le(&m_ram_g[(m_ctx.palette_source + 2 * b) & (RAM_G_SIZE - 2)]);
		return modulate(0xff000000
			| (u32((w >> 11) & 0x1f) * 255 / 31 << 16)
			| (u32((w >> 5) & 0x3f) * 255 / 63 << 8)
			| (u32(w & 0x1f) * 255 / 31));
	}
	case FMT_ARGB2: // 2:2:2:2
	{
		return modulate(u32((b >> 6) & 3) * 85 << 24
			| u32((b >> 4) & 3) * 85 << 16
			| u32((b >> 2) & 3) * 85 << 8
			| u32(b & 3) * 85);
	}
	default: // TEXT8X8, BARGRAPH and anything else: magenta stub
		return 0xff000000 | 0xff00ff;
	}
}

u32 ft812_device::bitmap_sample(u32 byte_addr, u32 fmt, double u, double v, u32 stride, u32 lines, u32 width, u32 height)
{
	auto texel = [this, byte_addr, fmt, stride, lines, width, height](s32 x, s32 y)
	{
		if (m_ctx.bitmap_wrap_x)
			x = (x % s32(width) + s32(width)) % s32(width);
		else if (x < 0 || x >= s32(width))
			return 0U;

		if (m_ctx.bitmap_wrap_y)
			y = (y % s32(height) + s32(height)) % s32(height);
		else if (y < 0 || y >= s32(height))
			return 0U;

		return bitmap_pixel(byte_addr, fmt, x, y, stride, lines);
	};

	if (!m_ctx.bitmap_filter)
		return texel(s32(std::floor(u + 0.5)), s32(std::floor(v + 0.5)));

	const s32 x0 = s32(std::floor(u)), y0 = s32(std::floor(v));
	const double fx = u - x0, fy = v - y0;
	const u32 p00 = texel(x0, y0), p10 = texel(x0 + 1, y0);
	const u32 p01 = texel(x0, y0 + 1), p11 = texel(x0 + 1, y0 + 1);
	u32 result = 0;
	for (u32 shift : { 0U, 8U, 16U, 24U })
	{
		const double top = ((p00 >> shift) & 0xff) * (1.0 - fx) + ((p10 >> shift) & 0xff) * fx;
		const double bottom = ((p01 >> shift) & 0xff) * (1.0 - fx) + ((p11 >> shift) & 0xff) * fx;
		result |= u32(std::clamp(top * (1.0 - fy) + bottom * fy + 0.5, 0.0, 255.0)) << shift;
	}
	return result;
}

void ft812_device::draw_point(s32 x, s32 y)
{
	const s32 radius = m_ctx.point_size;
	const s32 min_x = std::max<s32>({ 0, m_ctx.scissor_x, (x - radius) >> 4 });
	const s32 max_x = std::min<s32>({ m_bitmap.width() - 1, m_ctx.scissor_x + m_ctx.scissor_w - 1, (x + radius + 15) >> 4 });
	const s32 min_y = std::max<s32>({ 0, m_ctx.scissor_y, (y - radius) >> 4 });
	const s32 max_y = std::min<s32>({ m_bitmap.height() - 1, m_ctx.scissor_y + m_ctx.scissor_h - 1, (y + radius + 15) >> 4 });
	for (s32 py = min_y; py <= max_y; py++)
		for (s32 px = min_x; px <= max_x; px++)
		{
			const s64 dx = s64(px * 16 + 8) - x;
			const s64 dy = s64(py * 16 + 8) - y;
			if (dx * dx + dy * dy <= s64(radius) * radius)
				draw_px(px, py, m_ctx.color);
		}
}

void ft812_device::draw_line(s32 x0, s32 y0, s32 x1, s32 y1)
{
	const s32 radius = m_ctx.line_width;
	const s32 min_x = std::max<s32>({ 0, m_ctx.scissor_x, (std::min(x0, x1) - radius) >> 4 });
	const s32 max_x = std::min<s32>({ m_bitmap.width() - 1, m_ctx.scissor_x + m_ctx.scissor_w - 1, (std::max(x0, x1) + radius + 15) >> 4 });
	const s32 min_y = std::max<s32>({ 0, m_ctx.scissor_y, (std::min(y0, y1) - radius) >> 4 });
	const s32 max_y = std::min<s32>({ m_bitmap.height() - 1, m_ctx.scissor_y + m_ctx.scissor_h - 1, (std::max(y0, y1) + radius + 15) >> 4 });
	const double vx = x1 - x0, vy = y1 - y0;
	const double length2 = vx * vx + vy * vy;
	const double radius2 = double(radius) * radius;
	for (s32 py = min_y; py <= max_y; py++)
		for (s32 px = min_x; px <= max_x; px++)
		{
			const double dx = px * 16 + 8 - x0, dy = py * 16 + 8 - y0;
			const double t = length2 ? std::clamp((dx * vx + dy * vy) / length2, 0.0, 1.0) : 0.0;
			const double ex = dx - t * vx, ey = dy - t * vy;
			if (ex * ex + ey * ey <= radius2)
				draw_px(px, py, m_ctx.color);
		}
}

void ft812_device::draw_rect(s32 x0, s32 y0, s32 x1, s32 y1)
{
	const s32 radius = m_ctx.line_width;
	const s32 left = std::min(x0, x1), right = std::max(x0, x1);
	const s32 top = std::min(y0, y1), bottom = std::max(y0, y1);
	const s32 min_x = std::max<s32>({ 0, m_ctx.scissor_x, (left - radius) >> 4 });
	const s32 max_x = std::min<s32>({ m_bitmap.width() - 1, m_ctx.scissor_x + m_ctx.scissor_w - 1, (right + radius + 15) >> 4 });
	const s32 min_y = std::max<s32>({ 0, m_ctx.scissor_y, (top - radius) >> 4 });
	const s32 max_y = std::min<s32>({ m_bitmap.height() - 1, m_ctx.scissor_y + m_ctx.scissor_h - 1, (bottom + radius + 15) >> 4 });
	for (s32 py = min_y; py <= max_y; py++)
		for (s32 px = min_x; px <= max_x; px++)
		{
			const s32 cx = std::clamp(px * 16 + 8, left, right);
			const s32 cy = std::clamp(py * 16 + 8, top, bottom);
			const s64 dx = s64(px * 16 + 8) - cx, dy = s64(py * 16 + 8) - cy;
			if (dx * dx + dy * dy <= s64(radius) * radius)
				draw_px(px, py, m_ctx.color);
		}
}

void ft812_device::draw_edge_segment(s32 x0, s32 y0, s32 x1, s32 y1)
{
	switch (m_prim)
	{
	case PRIM_EDGE_STRIP_R:
	case PRIM_EDGE_STRIP_L:
	{
		if (y0 == y1)
			break;
		const s32 min_y = std::max<s32>(0, std::min(y0, y1) >> 4);
		const s32 max_y = std::min<s32>(m_bitmap.height() - 1, (std::max(y0, y1) + 15) >> 4);
		for (s32 py = min_y; py <= max_y; py++)
		{
			const double cy = py * 16 + 8;
			const double t = (cy - y0) / (y1 - y0);
			if (t < 0.0 || t > 1.0)
				continue;
			const s32 edge_x = s32(std::floor((x0 + t * (x1 - x0)) / 16.0));
			const s32 first = m_prim == PRIM_EDGE_STRIP_R ? std::max<s32>(0, edge_x) : 0;
			const s32 last = m_prim == PRIM_EDGE_STRIP_R ? m_bitmap.width() - 1 : std::min<s32>(m_bitmap.width() - 1, edge_x);
			for (s32 px = first; px <= last; px++)
				draw_px(px, py, m_ctx.color);
		}
		break;
	}
	case PRIM_EDGE_STRIP_A:
	case PRIM_EDGE_STRIP_B:
	{
		if (x0 == x1)
			break;
		const s32 min_x = std::max<s32>(0, std::min(x0, x1) >> 4);
		const s32 max_x = std::min<s32>(m_bitmap.width() - 1, (std::max(x0, x1) + 15) >> 4);
		for (s32 px = min_x; px <= max_x; px++)
		{
			const double cx = px * 16 + 8;
			const double t = (cx - x0) / (x1 - x0);
			if (t < 0.0 || t > 1.0)
				continue;
			const s32 edge_y = s32(std::floor((y0 + t * (y1 - y0)) / 16.0));
			const s32 first = m_prim == PRIM_EDGE_STRIP_A ? 0 : std::max<s32>(0, edge_y);
			const s32 last = m_prim == PRIM_EDGE_STRIP_A ? std::min<s32>(m_bitmap.height() - 1, edge_y) : m_bitmap.height() - 1;
			for (s32 py = first; py <= last; py++)
				draw_px(px, py, m_ctx.color);
		}
		break;
	}
	}
}

// 8x8 ROM font replacement (public domain, dhepper/font8x8, IBM VGA derived).
// FT812 built-in font handles 0-31 render L8 glyphs colored by COLOR_RGB;
// the real ROM bitmap data is proprietary, so a compatible 8x8 face is used.
static const u8 ROM_FONT[128][8] = {
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x18, 0x3c, 0x3c, 0x18, 0x18, 0x00, 0x18, 0x00 },
	{ 0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x36, 0x36, 0x7f, 0x36, 0x7f, 0x36, 0x36, 0x00 },
	{ 0x0c, 0x3e, 0x03, 0x1e, 0x30, 0x1f, 0x0c, 0x00 },
	{ 0x00, 0x63, 0x33, 0x18, 0x0c, 0x66, 0x63, 0x00 },
	{ 0x1c, 0x36, 0x1c, 0x6e, 0x3b, 0x33, 0x6e, 0x00 },
	{ 0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x18, 0x0c, 0x06, 0x06, 0x06, 0x0c, 0x18, 0x00 },
	{ 0x06, 0x0c, 0x18, 0x18, 0x18, 0x0c, 0x06, 0x00 },
	{ 0x00, 0x66, 0x3c, 0xff, 0x3c, 0x66, 0x00, 0x00 },
	{ 0x00, 0x0c, 0x0c, 0x3f, 0x0c, 0x0c, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c, 0x06 },
	{ 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x0c, 0x00 },
	{ 0x60, 0x30, 0x18, 0x0c, 0x06, 0x03, 0x01, 0x00 },
	{ 0x3e, 0x63, 0x73, 0x7b, 0x6f, 0x67, 0x3e, 0x00 },
	{ 0x0c, 0x0e, 0x0c, 0x0c, 0x0c, 0x0c, 0x3f, 0x00 },
	{ 0x1e, 0x33, 0x30, 0x1c, 0x06, 0x33, 0x3f, 0x00 },
	{ 0x1e, 0x33, 0x30, 0x1c, 0x30, 0x33, 0x1e, 0x00 },
	{ 0x38, 0x3c, 0x36, 0x33, 0x7f, 0x30, 0x78, 0x00 },
	{ 0x3f, 0x03, 0x1f, 0x30, 0x30, 0x33, 0x1e, 0x00 },
	{ 0x1c, 0x06, 0x03, 0x1f, 0x33, 0x33, 0x1e, 0x00 },
	{ 0x3f, 0x33, 0x30, 0x18, 0x0c, 0x0c, 0x0c, 0x00 },
	{ 0x1e, 0x33, 0x33, 0x1e, 0x33, 0x33, 0x1e, 0x00 },
	{ 0x1e, 0x33, 0x33, 0x3e, 0x30, 0x18, 0x0e, 0x00 },
	{ 0x00, 0x0c, 0x0c, 0x00, 0x00, 0x0c, 0x0c, 0x00 },
	{ 0x00, 0x0c, 0x0c, 0x00, 0x00, 0x0c, 0x0c, 0x06 },
	{ 0x18, 0x0c, 0x06, 0x03, 0x06, 0x0c, 0x18, 0x00 },
	{ 0x00, 0x00, 0x3f, 0x00, 0x00, 0x3f, 0x00, 0x00 },
	{ 0x06, 0x0c, 0x18, 0x30, 0x18, 0x0c, 0x06, 0x00 },
	{ 0x1e, 0x33, 0x30, 0x18, 0x0c, 0x00, 0x0c, 0x00 },
	{ 0x3e, 0x63, 0x7b, 0x7b, 0x7b, 0x03, 0x1e, 0x00 },
	{ 0x0c, 0x1e, 0x33, 0x33, 0x3f, 0x33, 0x33, 0x00 },
	{ 0x3f, 0x66, 0x66, 0x3e, 0x66, 0x66, 0x3f, 0x00 },
	{ 0x3c, 0x66, 0x03, 0x03, 0x03, 0x66, 0x3c, 0x00 },
	{ 0x1f, 0x36, 0x66, 0x66, 0x66, 0x36, 0x1f, 0x00 },
	{ 0x7f, 0x46, 0x16, 0x1e, 0x16, 0x46, 0x7f, 0x00 },
	{ 0x7f, 0x46, 0x16, 0x1e, 0x16, 0x06, 0x0f, 0x00 },
	{ 0x3c, 0x66, 0x03, 0x03, 0x73, 0x66, 0x7c, 0x00 },
	{ 0x33, 0x33, 0x33, 0x3f, 0x33, 0x33, 0x33, 0x00 },
	{ 0x1e, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x1e, 0x00 },
	{ 0x78, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1e, 0x00 },
	{ 0x67, 0x66, 0x36, 0x1e, 0x36, 0x66, 0x67, 0x00 },
	{ 0x0f, 0x06, 0x06, 0x06, 0x46, 0x66, 0x7f, 0x00 },
	{ 0x63, 0x77, 0x7f, 0x7f, 0x6b, 0x63, 0x63, 0x00 },
	{ 0x63, 0x67, 0x6f, 0x7b, 0x73, 0x63, 0x63, 0x00 },
	{ 0x1c, 0x36, 0x63, 0x63, 0x63, 0x36, 0x1c, 0x00 },
	{ 0x3f, 0x66, 0x66, 0x3e, 0x06, 0x06, 0x0f, 0x00 },
	{ 0x1e, 0x33, 0x33, 0x33, 0x3b, 0x1e, 0x38, 0x00 },
	{ 0x3f, 0x66, 0x66, 0x3e, 0x36, 0x66, 0x67, 0x00 },
	{ 0x1e, 0x33, 0x07, 0x0e, 0x38, 0x33, 0x1e, 0x00 },
	{ 0x3f, 0x2d, 0x0c, 0x0c, 0x0c, 0x0c, 0x1e, 0x00 },
	{ 0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0x3f, 0x00 },
	{ 0x33, 0x33, 0x33, 0x33, 0x33, 0x1e, 0x0c, 0x00 },
	{ 0x63, 0x63, 0x63, 0x6b, 0x7f, 0x77, 0x63, 0x00 },
	{ 0x63, 0x63, 0x36, 0x1c, 0x1c, 0x36, 0x63, 0x00 },
	{ 0x33, 0x33, 0x33, 0x1e, 0x0c, 0x0c, 0x1e, 0x00 },
	{ 0x7f, 0x63, 0x31, 0x18, 0x4c, 0x66, 0x7f, 0x00 },
	{ 0x1e, 0x06, 0x06, 0x06, 0x06, 0x06, 0x1e, 0x00 },
	{ 0x03, 0x06, 0x0c, 0x18, 0x30, 0x60, 0x40, 0x00 },
	{ 0x1e, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1e, 0x00 },
	{ 0x08, 0x1c, 0x36, 0x63, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff },
	{ 0x0c, 0x0c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x1e, 0x30, 0x3e, 0x33, 0x6e, 0x00 },
	{ 0x07, 0x06, 0x06, 0x3e, 0x66, 0x66, 0x3b, 0x00 },
	{ 0x00, 0x00, 0x1e, 0x33, 0x03, 0x33, 0x1e, 0x00 },
	{ 0x38, 0x30, 0x30, 0x3e, 0x33, 0x33, 0x6e, 0x00 },
	{ 0x00, 0x00, 0x1e, 0x33, 0x3f, 0x03, 0x1e, 0x00 },
	{ 0x1c, 0x36, 0x06, 0x0f, 0x06, 0x06, 0x0f, 0x00 },
	{ 0x00, 0x00, 0x6e, 0x33, 0x33, 0x3e, 0x30, 0x1f },
	{ 0x07, 0x06, 0x36, 0x6e, 0x66, 0x66, 0x67, 0x00 },
	{ 0x0c, 0x00, 0x0e, 0x0c, 0x0c, 0x0c, 0x1e, 0x00 },
	{ 0x30, 0x00, 0x30, 0x30, 0x30, 0x33, 0x33, 0x1e },
	{ 0x07, 0x06, 0x66, 0x36, 0x1e, 0x36, 0x67, 0x00 },
	{ 0x0e, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x1e, 0x00 },
	{ 0x00, 0x00, 0x33, 0x7f, 0x7f, 0x6b, 0x63, 0x00 },
	{ 0x00, 0x00, 0x1f, 0x33, 0x33, 0x33, 0x33, 0x00 },
	{ 0x00, 0x00, 0x1e, 0x33, 0x33, 0x33, 0x1e, 0x00 },
	{ 0x00, 0x00, 0x3b, 0x66, 0x66, 0x3e, 0x06, 0x0f },
	{ 0x00, 0x00, 0x6e, 0x33, 0x33, 0x3e, 0x30, 0x78 },
	{ 0x00, 0x00, 0x3b, 0x6e, 0x66, 0x06, 0x0f, 0x00 },
	{ 0x00, 0x00, 0x3e, 0x03, 0x1e, 0x30, 0x1f, 0x00 },
	{ 0x08, 0x0c, 0x3e, 0x0c, 0x0c, 0x2c, 0x18, 0x00 },
	{ 0x00, 0x00, 0x33, 0x33, 0x33, 0x33, 0x6e, 0x00 },
	{ 0x00, 0x00, 0x33, 0x33, 0x33, 0x1e, 0x0c, 0x00 },
	{ 0x00, 0x00, 0x63, 0x6b, 0x7f, 0x7f, 0x36, 0x00 },
	{ 0x00, 0x00, 0x63, 0x36, 0x1c, 0x36, 0x63, 0x00 },
	{ 0x00, 0x00, 0x33, 0x33, 0x33, 0x3e, 0x30, 0x1f },
	{ 0x00, 0x00, 0x3f, 0x19, 0x0c, 0x26, 0x3f, 0x00 },
	{ 0x38, 0x0c, 0x0c, 0x07, 0x0c, 0x0c, 0x38, 0x00 },
	{ 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x00 },
	{ 0x07, 0x0c, 0x0c, 0x38, 0x0c, 0x0c, 0x07, 0x00 },
	{ 0x6e, 0x3b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
};

bool ft812_device::rom_font_desc(u32 font, u32 &fmt, u32 &stride, u32 &w, u32 &h, u32 &gfx) const
{
	if (!m_font_rom || font < 16 || font > 34)
		return false;
	const u8 *const e = &m_font_rom[FONT_ROM_TABLE + (font - 16) * 148];
	fmt = get_u32le(&e[128]);
	stride = get_u32le(&e[132]);
	w = get_u32le(&e[136]);
	h = get_u32le(&e[140]);
	gfx = get_u32le(&e[144]);
	return (fmt == FMT_L1 || fmt == FMT_L4) && stride && w && h && gfx >= 0x1e0000 && gfx < 0x300000;
}

void ft812_device::draw_rom_font(s32 x, s32 y)
{
	u32 fmt, stride, w, h, gfx;
	if (!rom_font_desc(m_ctx.bitmap_handle, fmt, stride, w, h, gfx))
		return;
	const u32 cell = m_ctx.cell;
	if (cell < 0x20 || cell > 0x7e)
		return;
	// glyph data is indexed by ASCII code (stream includes codes 0x00-0x1f)
	const u32 glyph = cell * stride * h;
	const u64 src_off = u64(gfx) - 0x1e0000 + glyph;
	if (src_off + u64(stride) * h > FONT_ROM_SIZE)
		return;
	const u8 *const src = &m_font_rom[src_off];
	const u32 rgb = 0xff'000000 | (m_ctx.color & 0x00'ffffff);
	for (u32 v = 0; v < h; v++)
		for (u32 u = 0; u < w; u++)
		{
			if (fmt == FMT_L1)
			{
				if (BIT(src[v * stride + (u >> 3)], 7 - (u & 7)))
					draw_px(x + s32(u), y + s32(v), rgb);
			}
			else
			{
				const u8 byte = src[v * stride + (u >> 1)];
				const u32 level = (u & 1) ? (byte & 0x0f) : (byte >> 4);
				if (level)
					draw_px(x + s32(u), y + s32(v), (level * 0x11) << 24 | (m_ctx.color & 0x00'ffffff));
			}
		}
}

void ft812_device::draw_bitmap(s32 x, s32 y)
{
	// Handles 16-31 can also reference RAM_G bitmaps; ROM fonts use a source outside RAM_G.
	if (m_ctx.bitmap_handle >= 16 && m_ctx.bitmap_source >= RAM_G_SIZE)
	{
		if (m_font_rom)
		{
			draw_rom_font(x, y);
			return;
		}
		const u8 *glyph = ROM_FONT[m_ctx.cell & 0x7f]; // fallback: public domain 8x8
		for (u32 v = 0; v < 8; v++)
			for (u32 u = 0; u < 8; u++)
				if (glyph[v] & (0x80 >> u))
					draw_px(x + s32(u), y + s32(v), 0xff000000 | (m_ctx.color & 0x00'ffffff));
		return;
	}

	const u32 fmt = (m_ctx.bitmap_layout >> 19) & 0x1f;
	const u32 stride = ((m_ctx.bitmap_layout >> 9) & 0x3ff) | (((m_ctx.bitmap_layout_h >> 2) & 3) << 10);
	const u32 lines = (m_ctx.bitmap_layout & 0x1ff) | ((m_ctx.bitmap_layout_h & 3) << 9);
	u32 w = ((m_ctx.bitmap_size >> 9) & 0x1ff) | (((m_ctx.bitmap_size_h >> 2) & 3) << 9);
	u32 h = (m_ctx.bitmap_size & 0x1ff) | ((m_ctx.bitmap_size_h & 3) << 9);
	if (!stride || !lines || !w || !h)
		return;

	u32 row_width = stride;
	switch (fmt)
	{
	case FMT_ARGB1555:
	case FMT_ARGB4:
	case FMT_RGB565:
		row_width /= 2;
		break;
	case FMT_L4:
		row_width *= 2;
		break;
	case FMT_L2:
		row_width *= 4;
		break;
	case FMT_L1:
		row_width *= 8;
		break;
	}
	// Keep transformed draws within the source allocation when size exceeds stride/lines.
	w = std::min(w, row_width);
	h = std::min(h, lines);

	const u32 src = m_ctx.bitmap_source + u32(m_ctx.cell) * stride * lines;
	const double a = double(m_ctx.bitmap_transform_a) / 256.0;
	const double b = double(m_ctx.bitmap_transform_b) / 256.0;
	const double c = double(m_ctx.bitmap_transform_c) / 256.0;
	const double d = double(m_ctx.bitmap_transform_d) / 256.0;
	const double e = double(m_ctx.bitmap_transform_e) / 256.0;
	const double f = double(m_ctx.bitmap_transform_f) / 256.0;
	const double det = a * d - b * c;
	if (std::abs(det) < 1.0e-12)
		return;

	auto inverse = [a, b, c, d, e, f, det](double u, double v)
	{
		u -= e;
		v -= f;
		return std::pair<double, double>((d * u - c * v) / det, (a * v - b * u) / det);
	};

	auto transform = [a, b, c, d, e, f](double u, double v)
	{
		return std::pair<double, double>(a * u + c * v + e, b * u + d * v + f);
	};

	const std::pair<double, double> corners[] = {
		transform(0, 0), transform(w, 0), transform(0, h), transform(w, h)
	};
	double min_x = corners[0].first, max_x = corners[0].first;
	double min_y = corners[0].second, max_y = corners[0].second;
	for (const auto &[cx, cy] : corners)
	{
		min_x = std::min(min_x, cx);
		max_x = std::max(max_x, cx);
		min_y = std::min(min_y, cy);
		max_y = std::max(max_y, cy);
	}

	const double left = std::max({ std::ceil(x + min_x - 0.5), double(m_ctx.scissor_x), 0.0 });
	const double top = std::max({ std::ceil(y + min_y - 0.5), double(m_ctx.scissor_y), 0.0 });
	const double right = std::min({ std::ceil(x + max_x - 0.5) - 1.0,
			double(m_ctx.scissor_x + m_ctx.scissor_w - 1), double(m_bitmap.width() - 1) });
	const double bottom = std::min({ std::ceil(y + max_y - 0.5) - 1.0,
			double(m_ctx.scissor_y + m_ctx.scissor_h - 1), double(m_bitmap.height() - 1) });
	if (left > right || top > bottom)
		return;

	for (s32 dy = s32(top); dy <= s32(bottom); dy++)
	{
		for (s32 dx = s32(left); dx <= s32(right); dx++)
		{
			const double px = double(dx - x) + 0.5;
			const double py = double(dy - y) + 0.5;
			const auto [u, v] = inverse(px, py);
			draw_px(dx, dy, bitmap_sample(src, fmt, u - 0.5, v - 0.5, stride, lines, w, h));
		}
	}
}

void ft812_device::copro_execute()
{
	u32 r = reg_r32(REG_CMD_READ) & 0xffc;
	const u32 w = reg_r32(REG_CMD_WRITE) & 0xffc;
	auto available = [&r, w]() { return (w - r) & 0xfff; };
	auto byte_at = [this, &r](u32 offset) { return m_ram_cmd[(r + offset) & 0xfff]; };
	auto word_at = [&byte_at](u32 offset)
	{
		return u32(byte_at(offset))
			| (u32(byte_at(offset + 1)) << 8)
			| (u32(byte_at(offset + 2)) << 16)
			| (u32(byte_at(offset + 3)) << 24);
	};
	auto append_dl = [this](u32 word)
	{
		if (m_dl_ptr + 4 > RAM_DL_SIZE)
		{
			LOGMASKED(LOG_WARN, "%s: display list overflow while executing co-processor command\n", tag());
			return false;
		}
		put_u32le(&m_ram_dl[m_dl_ptr], word);
		m_dl_ptr += 4;
		return true;
	};
	auto emit_text = [this, &append_dl](s32 x, s32 y, u32 font, u32 options, const std::string &text)
	{
		u32 fmt, stride, fw, fh, gfx;
		const bool rom_font = (font >= 16) && rom_font_desc(font, fmt, stride, fw, fh, gfx);
		const u8 *const widths = rom_font ? &m_font_rom[FONT_ROM_TABLE + (font - 16) * 148] : nullptr;
		const s32 glyph_width = rom_font ? s32(fw) : 8;
		const s32 glyph_height = rom_font ? s32(fh) : 8;
		auto char_width = [&](u8 ch) -> s32
		{
			return (widths && ch >= 0x20 && ch <= 0x7e) ? widths[ch] : glyph_width;
		};
		s32 lines = 1, line_width = 0, max_width = 0;
		for (char ch : text)
		{
			if (ch == '\n')
			{
				max_width = std::max(max_width, line_width);
				line_width = 0;
				lines++;
			}
			else
				line_width += char_width(u8(ch));
		}
		max_width = std::max(max_width, line_width);
		if (options & 0x0200)
			x -= max_width / 2;
		else if (options & 0x0800)
			x -= max_width;
		if (options & 0x0400)
			y -= lines * glyph_height / 2;

		append_dl(0x22000000); // SAVE_CONTEXT
		append_dl(0x27000004); // VERTEX_FORMAT(4)
		append_dl(0x2b000000); // VERTEX_TRANSLATE_X(0)
		append_dl(0x2c000000); // VERTEX_TRANSLATE_Y(0)
		if (font >= 16)
			append_dl(DL_BITMAP_SOURCE | 0x1e0000); // select the FT812 ROM font window
		append_dl(0x1f000001); // BEGIN(BITMAPS)
		append_dl(0x05000000 | (font & 0x1f)); // BITMAP_HANDLE
		s32 pen_x = x, pen_y = y;
		for (u8 ch : text)
		{
			if (ch == '\n')
			{
				pen_x = x;
				pen_y += glyph_height;
				continue;
			}
			append_dl(0x06000000 | ch); // CELL
			const u32 vertex = 0x40000000
				| ((u32(pen_x * 16) & 0x7fff) << 15)
				| (u32(pen_y * 16) & 0x7fff);
			append_dl(vertex);
			pen_x += char_width(ch);
		}
		append_dl(0x21000000); // END
		append_dl(0x23000000); // RESTORE_CONTEXT
	};
	auto valid_write_range = [this](u32 address, u32 length)
	{
		const u64 end = u64(address) + length;
		const bool in_ram_g = address < RAM_G_SIZE && end <= RAM_G_SIZE;
		const bool in_ram_dl = address >= RAM_DL_BASE && end <= RAM_DL_BASE + RAM_DL_SIZE;
		const bool in_ram_reg = address >= RAM_REG_BASE && end <= RAM_REG_BASE + RAM_REG_SIZE;
		const bool touches_copro_regs = in_ram_reg
			&& address < RAM_REG_BASE + REG_CMD_WRITE + 4
			&& end > RAM_REG_BASE + REG_CMD_READ;
		if ((!in_ram_g && !in_ram_dl && !in_ram_reg) || touches_copro_regs)
		{
			LOGMASKED(LOG_WARN, "%s: unsupported co-processor memory write %06x-%06llx\n", tag(), address, end ? end - 1 : 0);
			return false;
		}
		return true;
	};
	auto write_memory = [this, &valid_write_range](u32 address, const u8 *data, u32 length)
	{
		if (!valid_write_range(address, length))
			return;
		for (u32 i = 0; i < length; i++)
			m_space->write_byte(address + i, data[i]);
	};
	auto fill_memory = [this, &valid_write_range](u32 address, u8 value, u32 length)
	{
		if (!valid_write_range(address, length))
			return;
		for (u32 i = 0; i < length; i++)
			m_space->write_byte(address + i, value);
	};
	auto command_name = [](u8 opcode) -> const char *
	{
		switch (opcode)
		{
		case 0x00: return "CMD_DLSTART";
		case 0x01: return "CMD_SWAP";
		case 0x0a: return "CMD_FGCOLOR";
		case 0x0c: return "CMD_TEXT";
		case 0x0d: return "CMD_BUTTON";
		case 0x1a: return "CMD_MEMWRITE";
		case 0x1b: return "CMD_MEMSET";
		case 0x1e: return "CMD_APPEND";
		case 0x26: return "CMD_LOADIDENTITY";
		case 0x27: return "CMD_TRANSLATE";
		case 0x28: return "CMD_SCALE";
		case 0x29: return "CMD_ROTATE";
		case 0x2a: return "CMD_SETMATRIX";
		case 0x22: return "CMD_INFLATE";
		case 0x2e: return "CMD_NUMBER";
		default: return "unknown";
		}
	};

	while (r != w)
	{
		if (m_inflate_active)
		{
			// CMD_INFLATE: the zlib stream follows the command in the fifo;
			// feed whatever is available and consume exactly what zlib used
			z_stream *const zs = reinterpret_cast<z_stream *>(m_inflate_zs);
			u8 in_buf[64];
			u32 in_avail = available();
			const u32 in_take = std::min<u32>(in_avail, sizeof(in_buf));
			for (u32 i = 0; i < in_take; i++)
				in_buf[i] = byte_at(i);
			zs->next_in = in_buf;
			zs->avail_in = in_take;
			u32 consumed = 0;
			bool done = false;
			while (zs->avail_in && !done)
			{
				u8 out_buf[512];
				if (m_inflate_addr >= RAM_G_SIZE)
				{
					done = true;
					break;
				}
				const u32 room = std::min<u32>(sizeof(out_buf), RAM_G_SIZE - m_inflate_addr);
				zs->next_out = out_buf;
				zs->avail_out = room;
				const int rc = inflate(zs, Z_NO_FLUSH);
				const u32 produced = room - zs->avail_out;
				if (produced)
				{
					std::copy(out_buf, out_buf + produced, &m_ram_g[m_inflate_addr]);
					m_inflate_addr += produced;
				}
				consumed = in_take - zs->avail_in;
				if (rc == Z_STREAM_END)
				{
					done = true;
					break;
				}
				if (rc != Z_OK)
				{
					logerror("%s: CMD_INFLATE zlib error %d\n", tag(), rc);
					done = true;
					break;
				}
			}
			if (done)
			{
				consumed = in_take - zs->avail_in;
				inflateEnd(zs);
				m_inflate_active = false;
				// next command starts on the following word boundary
				r = (r + consumed + 3) & ~3U;
				continue;
			}
			r = (r + consumed) & 0xfff;
			continue;
		}

		const u32 count = available();
		if (count < 4)
			break;

		const u32 cmd = word_at(0);
		if ((cmd & 0xffffff00) == 0xffffff00)
		{
			u32 command_size = 4;
			u32 string_offset = 0;
			u32 string_length = 0;
			bool string_terminated = false;
			bool complete = true;
			const u8 opcode = cmd & 0xff;
			switch (opcode)
			{
			case 0x0c: // CMD_TEXT
				string_offset = 12;
				[[fallthrough]];
			case 0x0d: // CMD_BUTTON
				if (opcode == 0x0d)
					string_offset = 16;
				for (u32 pos = string_offset; pos < count; pos++)
				{
					if (!byte_at(pos))
					{
						string_length = pos - string_offset;
						command_size = (pos + 4) & ~3U;
						string_terminated = true;
						break;
					}
				}
				if (!string_terminated)
				{
					command_size = count + 1;
					complete = false;
				}
				break;
			case 0x22: // CMD_INFLATE
				command_size = 8;
				complete = count >= command_size;
				break;
			case 0x1a: // CMD_MEMWRITE
				if (count < 12)
				{
					command_size = 12;
					complete = false;
				}
				else
				{
					const u32 payload_length = word_at(8);
					const u64 total = 12 + ((u64(payload_length) + 3) & ~u64(3));
					command_size = total > 0xffffffff ? 0xffffffff : u32(total);
					complete = command_size <= count;
				}
				break;
			case 0x0a: // CMD_FGCOLOR
				command_size = 8;
				complete = count >= command_size;
				break;
			case 0x27: // CMD_TRANSLATE
			case 0x28: // CMD_SCALE
				command_size = 12;
				complete = count >= command_size;
				break;
			case 0x29: // CMD_ROTATE
				command_size = 8;
				complete = count >= command_size;
				break;
			case 0x1b: // CMD_MEMSET
			case 0x2e: // CMD_NUMBER
				command_size = 16;
				complete = count >= command_size;
				break;
			case 0x26: // CMD_LOADIDENTITY
			case 0x2a: // CMD_SETMATRIX
				command_size = 4;
				complete = true;
				break;
			case 0x00: // CMD_DLSTART
			case 0x01: // CMD_SWAP
				break;
			default:
				LOGMASKED(LOG_WARN, "%s: unsupported co-processor command %08x (%s) at FIFO %03x, %u bytes available; args %08x %08x %08x\n",
						tag(), cmd, command_name(opcode), r, count,
						count >= 8 ? word_at(4) : 0,
						count >= 12 ? word_at(8) : 0,
						count >= 16 ? word_at(12) : 0);
				break;
			}
			if (!complete || count < command_size)
				break;

			switch (cmd & 0xff)
			{
			case 0x00: // CMD_DLSTART
				m_dl_ptr = 0;
				break;
			case 0x01: // CMD_SWAP
				m_swap_pending = true;
				break;
			case 0x0a: // CMD_FGCOLOR
				m_copro_fgcolor = word_at(4) & 0x00ffffff;
				break;
			case 0x0c: // CMD_TEXT
			case 0x0d: // CMD_BUTTON
			{
				const bool button = (cmd & 0xff) == 0x0d;
				const s32 x = s16(word_at(4) & 0xffff);
				const s32 y = s16(word_at(4) >> 16);
				const u32 dimensions = button ? word_at(8) : 0;
				const s32 width = s16(dimensions & 0xffff);
				const s32 height = s16(dimensions >> 16);
				const u32 font_options = button ? word_at(12) : word_at(8);
				const u32 font = font_options & 0xffff;
				const u32 options = font_options >> 16;
				std::string text;
				text.reserve(string_length);
				for (u32 i = 0; i < string_length; i++)
					text.push_back(char(byte_at(string_offset + i)));
				if (button)
				{
					append_dl(0x22000000); // SAVE_CONTEXT
					append_dl(0x04000000 | 0x00404040); // COLOR_RGB(default widget background)
					append_dl(0x1f000009); // BEGIN(RECTS)
					append_dl(0x0e000010); // LINE_WIDTH(1 pixel)
					append_dl(0x40000000 | ((u32(x * 16) & 0x7fff) << 15) | (u32(y * 16) & 0x7fff));
					append_dl(0x40000000 | ((u32((x + width) * 16) & 0x7fff) << 15) | (u32((y + height) * 16) & 0x7fff));
					append_dl(0x21000000); // END
					append_dl(0x04000000 | m_copro_fgcolor);
					emit_text(x + width / 2, y + height / 2, font, options | 0x0600, text);
					append_dl(0x23000000); // RESTORE_CONTEXT
				}
				else
					emit_text(x, y, font, options, text);
				break;
			}
			case 0x1a: // CMD_MEMWRITE
			{
				const u32 address = word_at(4);
				const u32 length = word_at(8);
				std::vector<u8> payload(length);
				for (u32 i = 0; i < length; i++)
					payload[i] = byte_at(12 + i);
				write_memory(address, payload.data(), length);
				break;
			}
			case 0x22: // CMD_INFLATE
			{
				m_inflate_addr = word_at(4);
				if (m_inflate_addr < RAM_G_SIZE)
				{
					z_stream *const zs = reinterpret_cast<z_stream *>(m_inflate_zs);
					std::memset(zs, 0, sizeof(*zs));
					if (inflateInit(zs) == Z_OK)
					{
						m_inflate_active = true;
					}
				}
				break;
			}
			case 0x1b: // CMD_MEMSET
			{
				const u32 address = word_at(4);
				const u8 value = word_at(8) & 0xff;
				const u32 length = word_at(12);
				fill_memory(address, value, length);
				break;
			}
			case 0x2e: // CMD_NUMBER
			{
				const s32 x = s16(word_at(4) & 0xffff);
				const s32 y = s16(word_at(4) >> 16);
				const u32 font_options = word_at(8);
				const u32 font = font_options & 0xffff;
				const u32 options = font_options >> 16;
				const u32 value = word_at(12);
				char buffer[40];
				std::to_chars_result result;
				if (options & 0x0100)
					result = std::to_chars(buffer, buffer + sizeof(buffer), s32(value), 10);
				else
					result = std::to_chars(buffer, buffer + sizeof(buffer), value, 10);
				std::string text(buffer, result.ptr);
				const u32 width = options & 0x0f;
				if (width > text.size())
				{
					const u32 insert = text.front() == '-' ? 1 : 0;
					text.insert(insert, width - text.size(), '0');
				}
				emit_text(x, y, font, options, text);
				break;
			}
			case 0x26: // CMD_LOADIDENTITY
				m_copro_matrix[0] = 1.0;
				m_copro_matrix[1] = 0.0;
				m_copro_matrix[2] = 0.0;
				m_copro_matrix[3] = 1.0;
				m_copro_matrix[4] = 0.0;
				m_copro_matrix[5] = 0.0;
				break;
			case 0x27: // CMD_TRANSLATE
			{
				const double x = s32(word_at(4)) / 65536.0;
				const double y = s32(word_at(8)) / 65536.0;
				m_copro_matrix[4] += m_copro_matrix[0] * x + m_copro_matrix[2] * y;
				m_copro_matrix[5] += m_copro_matrix[1] * x + m_copro_matrix[3] * y;
				break;
			}
			case 0x28: // CMD_SCALE
			{
				const double x = s32(word_at(4)) / 65536.0;
				const double y = s32(word_at(8)) / 65536.0;
				m_copro_matrix[0] *= x;
				m_copro_matrix[1] *= x;
				m_copro_matrix[2] *= y;
				m_copro_matrix[3] *= y;
				break;
			}
			case 0x29: // CMD_ROTATE
			{
				const double angle = s32(word_at(4)) * (2.0 * std::acos(-1.0) / 65536.0);
				const double cosine = std::cos(angle);
				const double sine = std::sin(angle);
				const double a = m_copro_matrix[0], b = m_copro_matrix[1];
				const double c = m_copro_matrix[2], d = m_copro_matrix[3];
				m_copro_matrix[0] = a * cosine + c * sine;
				m_copro_matrix[1] = b * cosine + d * sine;
				m_copro_matrix[2] = c * cosine - a * sine;
				m_copro_matrix[3] = d * cosine - b * sine;
				break;
			}
			case 0x2a: // CMD_SETMATRIX
			{
				auto encode = [](double value, s32 min_value, s32 max_value)
				{
					return s32(std::round(std::clamp(value * 256.0, double(min_value), double(max_value))));
				};
				append_dl(DL_BITMAP_TRANSFORM_A | (u32(encode(m_copro_matrix[0], -(1 << 16), (1 << 16) - 1)) & 0x1ffff));
				append_dl(DL_BITMAP_TRANSFORM_B | (u32(encode(m_copro_matrix[1], -(1 << 16), (1 << 16) - 1)) & 0x1ffff));
				append_dl(DL_BITMAP_TRANSFORM_C | (u32(encode(m_copro_matrix[2], -(1 << 16), (1 << 16) - 1)) & 0x1ffff));
				append_dl(DL_BITMAP_TRANSFORM_D | (u32(encode(m_copro_matrix[3], -(1 << 16), (1 << 16) - 1)) & 0x1ffff));
				append_dl(DL_BITMAP_TRANSFORM_E | (u32(encode(m_copro_matrix[4], -(1 << 23), (1 << 23) - 1)) & 0x00ffffff));
				append_dl(DL_BITMAP_TRANSFORM_F | (u32(encode(m_copro_matrix[5], -(1 << 23), (1 << 23) - 1)) & 0x00ffffff));
				break;
			}
			default:
				break;
			}
			r = (r + command_size) & 0xffc;
		}
		else
		{
			// plain display list word relayed into RAM_DL
			append_dl(cmd);
			r = (r + 4) & 0xffc;
		}
	}
	reg_w32(REG_CMD_READ, r);
	reg_w32(REG_CMD_DL, r);
}

void ft812_device::render_dl()
{
	m_ctx = dl_context();
	m_ctx_stack = {};
	m_prim = 0;
	m_have_vertex = false;

	for (u32 i = 0; i < RAM_DL_SIZE / 4; i++)
	{
		u32 cmd = get_u32le(&m_ram_dl[i * 4]);
		if ((cmd & 0xff'000000) == DL_MACRO)
		{
			cmd = reg_r32((cmd & 1) ? REG_MACRO_1 : REG_MACRO_0);
		}
		const u32 op = cmd & 0xff'000000;

		if (op == DL_DISPLAY && (cmd & 0xff'ffffff) == 0)
			break;

		switch (op)
		{
		case DL_CLEAR_COLOR_RGB:
			m_ctx.clear_color = (m_ctx.clear_color & 0xff000000) | (cmd & 0x00'ffffff);
			break;
		case DL_CLEAR_COLOR_A:
			m_ctx.clear_a = cmd & 0xff;
			break;
		case DL_CLEAR_STENCIL:
			m_ctx.clear_stencil = cmd & 0xff;
			break;
		case DL_CLEAR_TAG:
			m_ctx.clear_tag = cmd & 0xff;
			break;
		case DL_COLOR_RGB:
			m_ctx.color = (m_ctx.color & 0xff000000) | (cmd & 0x00'ffffff);
			break;
		case DL_TAG:
			m_ctx.tag = cmd & 0xff;
			break;
		case DL_COLOR_A:
			m_ctx.color = (m_ctx.color & 0x00'ffffff) | (u32(cmd & 0xff) << 24);
			break;
		case DL_POINT_SIZE:
			m_ctx.point_size = cmd & 0x00'ffffff;
			break;
		case DL_LINE_WIDTH:
			m_ctx.line_width = cmd & 0x00'ffffff;
			break;
		case DL_BITMAP_SOURCE:
			m_ctx.bitmap_source = cmd & 0x00'ffff'ff;
			break;
		case DL_BITMAP_HANDLE:
			m_ctx.bitmap_handle = cmd & 0x1f;
			break;
		case DL_BITMAP_LAYOUT:
			m_ctx.bitmap_layout = cmd & 0x00'ffffff;
			break;
		case DL_BITMAP_SIZE:
			m_ctx.bitmap_size = cmd & 0x00'ffffff;
			m_ctx.bitmap_filter = BIT(cmd, 20);
			m_ctx.bitmap_wrap_x = BIT(cmd, 19);
			m_ctx.bitmap_wrap_y = BIT(cmd, 18);
			break;
		case DL_BITMAP_LAYOUT_H: // linestride bits 11:10, height bits 10:9
			m_ctx.bitmap_layout_h = cmd & 0xf;
			break;
		case DL_BITMAP_SIZE_H: // width bits 10:9, height bits 10:9
			m_ctx.bitmap_size_h = cmd & 0xf;
			break;
		case DL_CELL:
			m_ctx.cell = cmd & 0x7f;
			break;
		case DL_BITMAP_TRANSFORM_A:
			m_ctx.bitmap_transform_a = util::sext(cmd & 0x1ffff, 17);
			break;
		case DL_BITMAP_TRANSFORM_B:
			m_ctx.bitmap_transform_b = util::sext(cmd & 0x1ffff, 17);
			break;
		case DL_BITMAP_TRANSFORM_C:
			m_ctx.bitmap_transform_c = util::sext(cmd & 0x1ffff, 17);
			break;
		case DL_BITMAP_TRANSFORM_D:
			m_ctx.bitmap_transform_d = util::sext(cmd & 0x1ffff, 17);
			break;
		case DL_BITMAP_TRANSFORM_E:
			m_ctx.bitmap_transform_e = util::sext(cmd & 0x00ffffff, 24);
			break;
		case DL_BITMAP_TRANSFORM_F:
			m_ctx.bitmap_transform_f = util::sext(cmd & 0x00ffffff, 24);
			break;
		case DL_SCISSOR_XY:
			m_ctx.scissor_x = s32((cmd >> 11) & 0x7ff);
			m_ctx.scissor_y = s32(cmd & 0x7ff);
			break;
		case DL_SCISSOR_SIZE:
			m_ctx.scissor_w = s32((cmd >> 12) & 0xfff);
			m_ctx.scissor_h = s32(cmd & 0xfff);
			break;
		case DL_VERTEX_FORMAT:
			m_ctx.vertex_format = cmd & 7;
			break;
		case DL_STENCIL_FUNC:
			m_ctx.stencil_func = (cmd >> 16) & 7;
			m_ctx.stencil_ref = (cmd >> 8) & 0xff;
			m_ctx.stencil_read_mask = cmd & 0xff;
			break;
		case DL_STENCIL_MASK:
			m_ctx.stencil_write_mask = cmd & 0xff;
			break;
		case DL_TAG_MASK:
			m_ctx.tag_mask = cmd & 1;
			break;
		case DL_COLOR_MASK:
			m_ctx.color_mask = cmd & 0x0f;
			break;
		case DL_BEGIN:
			m_prim = cmd & 0x0f;
			m_have_vertex = false;
			break;
		case DL_END:
			m_prim = 0;
			m_have_vertex = false;
			break;
		case DL_SAVE_CONTEXT:
			m_ctx_stack.push(m_ctx);
			break;
		case DL_RESTORE_CONTEXT:
			if (!m_ctx_stack.empty())
			{
				m_ctx = m_ctx_stack.top();
				m_ctx_stack.pop();
			}
			break;
		case DL_CLEAR:
		{
			const s32 min_x = std::max<s32>(0, m_ctx.scissor_x);
			const s32 min_y = std::max<s32>(0, m_ctx.scissor_y);
			const s32 max_x = std::min<s32>(m_bitmap.width(), m_ctx.scissor_x + m_ctx.scissor_w);
			const s32 max_y = std::min<s32>(m_bitmap.height(), m_ctx.scissor_y + m_ctx.scissor_h);
			for (s32 y = min_y; y < max_y; y++)
				for (s32 x = min_x; x < max_x; x++)
				{
					if (cmd & 4)
					{
						u32 &pixel = m_bitmap.pix(y, x);
						const u32 clear = (m_ctx.clear_a << 24) | (m_ctx.clear_color & 0x00'ffffff);
						if (m_ctx.color_mask & 1) pixel = (pixel & 0x00ffffff) | (clear & 0xff000000);
						if (m_ctx.color_mask & 2) pixel = (pixel & 0xffff00ff) | (clear & 0x0000ff00);
						if (m_ctx.color_mask & 4) pixel = (pixel & 0xff00ffff) | (clear & 0x00ff0000);
						if (m_ctx.color_mask & 8) pixel = (pixel & 0xff'ffff00) | (clear & 0x000000ff);
					}
					if (cmd & 2)
					{
						const u8 old = m_stencil.pix(y, x);
						m_stencil.pix(y, x) = (old & ~m_ctx.stencil_write_mask) | (m_ctx.clear_stencil & m_ctx.stencil_write_mask);
					}
					if ((cmd & 1) && m_ctx.tag_mask)
						m_tag_buffer.pix(y, x) = m_ctx.clear_tag;
				}
			break;
		}
		case 0x2a'000000: // PALETTE_SOURCE
			m_ctx.palette_source = cmd & 0x00'ffff'ff;
			break;
		case 0x2b'000000: // VERTEX_TRANSLATE_X, signed 17-bit, 1/16 pixel
			m_ctx.vtx_tx = util::sext(cmd & 0x1ffff, 17);
			break;
		case 0x2c'000000: // VERTEX_TRANSLATE_Y
			m_ctx.vtx_ty = util::sext(cmd & 0x1ffff, 17);
			break;
		case DL_BLEND_FUNC:
			m_ctx.blend_src = (cmd >> 3) & 7;
			m_ctx.blend_dst = cmd & 7;
			break;
		case DL_STENCIL_OP:
			m_ctx.stencil_fail = (cmd >> 3) & 7;
			m_ctx.stencil_pass = cmd & 7;
			break;
		case DL_ALPHA_FUNC:
			m_ctx.alpha_func = (cmd >> 8) & 7;
			m_ctx.alpha_ref = cmd & 0xff;
			break;
		default:
			// vertex opcodes have top bits 01/10 and fall through here
			if (cmd & 0x8000'0000) // VERTEX2II
			{
				const s32 x = s32((cmd >> 21) & 0x1ff) * 16 + m_ctx.vtx_tx;
				const s32 y = s32((cmd >> 12) & 0x1ff) * 16 + m_ctx.vtx_ty;
				const u8 handle = (cmd >> 7) & 0x1f;
				const u8 cell = cmd & 0x7f;
				m_ctx.cell = cell;
				m_ctx.bitmap_handle = handle;
				switch (m_prim)
				{
				case PRIM_BITMAPS:
					draw_bitmap(x >> 4, y >> 4);
					break;
				case PRIM_POINTS:
					draw_point(x, y);
					break;
				case PRIM_RECTS:
					if (m_have_vertex)
					{
						draw_rect(m_last_x, m_last_y, x, y);
						m_have_vertex = false;
					}
					else
					{
						m_last_x = x; m_last_y = y; m_have_vertex = true;
					}
					break;
				case PRIM_LINES:
					if (m_have_vertex)
					{
						draw_line(m_last_x, m_last_y, x, y);
						if (m_prim == PRIM_LINES)
							m_have_vertex = false;
					}
					else
					{
						m_last_x = x; m_last_y = y; m_have_vertex = true;
					}
					break;
				case PRIM_LINE_STRIP:
				case PRIM_EDGE_STRIP_R:
				case PRIM_EDGE_STRIP_L:
				case PRIM_EDGE_STRIP_A:
				case PRIM_EDGE_STRIP_B:
					if (m_have_vertex)
					{
						if (m_prim == PRIM_LINE_STRIP)
							draw_line(m_last_x, m_last_y, x, y);
						else
							draw_edge_segment(m_last_x, m_last_y, x, y);
					}
					m_last_x = x;
					m_last_y = y;
					m_have_vertex = true;
					break;
				default:
					break;
				}
			}
			else if (cmd & 0x4000'0000) // VERTEX2F
			{
				const s32 scale = 1 << (4 - std::min<u8>(m_ctx.vertex_format, 4));
				const s32 xs = (s32(cmd << 2) >> 17) * scale + m_ctx.vtx_tx;
				const s32 ys = (s32(cmd << 17) >> 17) * scale + m_ctx.vtx_ty;
				switch (m_prim)
				{
				case PRIM_BITMAPS:
					draw_bitmap(xs >> 4, ys >> 4);
					break;
				case PRIM_POINTS:
					draw_point(xs, ys);
					break;
				case PRIM_RECTS:
					if (m_have_vertex)
					{
						draw_rect(m_last_x, m_last_y, xs, ys);
						m_have_vertex = false;
					}
					else
					{
						m_last_x = xs; m_last_y = ys; m_have_vertex = true;
					}
					break;
				case PRIM_LINES:
					if (m_have_vertex)
					{
						draw_line(m_last_x, m_last_y, xs, ys);
						if (m_prim == PRIM_LINES)
							m_have_vertex = false;
					}
					else
					{
						m_last_x = xs; m_last_y = ys; m_have_vertex = true;
					}
					break;
				case PRIM_LINE_STRIP:
				case PRIM_EDGE_STRIP_R:
				case PRIM_EDGE_STRIP_L:
				case PRIM_EDGE_STRIP_A:
				case PRIM_EDGE_STRIP_B:
					if (m_have_vertex)
					{
						if (m_prim == PRIM_LINE_STRIP)
							draw_line(m_last_x, m_last_y, xs, ys);
						else
							draw_edge_segment(m_last_x, m_last_y, xs, ys);
					}
					m_last_x = xs;
					m_last_y = ys;
					m_have_vertex = true;
					break;
				default:
					break;
				}
			}
			else
				LOGMASKED(LOG_WARN, "%s: unsupported display-list word %08x at DL[%04x]\n",
						tag(), cmd, i * 4);
			break;
		}
	}
}
