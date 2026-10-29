// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
#include "emu.h"
#include "ft812.h"

#include "drawgfx.h"
#include "machine/ram.h"
#include "multibyte.h"
#include <charconv>
#include <cmath>
#include <vector>
#include <string>
#include <zlib.h>

#define LOG_WARN    (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(FT812, ft812_device, "ft812", "FTDI FT812 Embedded Video Engine")


// FT812 ROM font window (0x1e0000-0x300000);
// layout: 19 x 148 byte font metrics blocks at 0x21ee0, ROM_FONTROOT pointer at 0xffffc (PG 5.5.1).
ROM_START( ft812 )
	ROM_REGION( 0x120000, "fonts", ROMREGION_ERASE00)
	ROM_LOAD( "font.bin", 0x000000, 0x120000, CRC(2a84edf6) SHA1(5581c4780698392d5c5f2e9a9e4ab37e9e41e540))
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
	, m_timing_cb(*this)
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

void ft812_device::device_resolve_objects()
{
	device_t::device_resolve_objects();
	m_timing_cb.resolve();
}

void ft812_device::device_start()
{
	m_space = &space(AS_FT);
	m_inflate_zs = new z_stream;
	m_inflate_active = false;
	m_cmdb_burst = 0;

	m_font_rom = memregion("fonts")->base();
	if (get_u32le(&m_font_rom[0x11fffc]) != 0x201ee0)
		fatalerror("%s: font.bin is not an FT812 ROM font dump (bad ROM_FONTROOT)\n", tag());

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
	save_item(STRUCT_MEMBER(m_bitmap_state, source));
	save_item(STRUCT_MEMBER(m_bitmap_state, layout));
	save_item(STRUCT_MEMBER(m_bitmap_state, size));
	save_item(STRUCT_MEMBER(m_bitmap_state, layout_h));
	save_item(STRUCT_MEMBER(m_bitmap_state, size_h));
	save_item(STRUCT_MEMBER(m_bitmap_state, filter));
	save_item(STRUCT_MEMBER(m_bitmap_state, wrap_x));
	save_item(STRUCT_MEMBER(m_bitmap_state, wrap_y));
	save_item(NAME(m_prim));
	save_item(NAME(m_last_x));
	save_item(NAME(m_last_y));
	save_item(NAME(m_have_vertex));
}

void ft812_device::vblank()
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

void ft812_device::device_stop()
{
	if (m_inflate_active)
		inflateEnd(reinterpret_cast<z_stream *>(m_inflate_zs));
	delete reinterpret_cast<z_stream *>(m_inflate_zs);
	m_inflate_zs = nullptr;
}


void ft812_device::device_post_load()
{
	// timing re-applies at the next vblank (the raster owner pushes it)
	m_timing_dirty = true;
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

	m_timing = ft812_timing{ true, hcycle, vcycle,
			rectangle(hoffset, hoffset + hsize - 1, voffset, voffset + vsize - 1),
			attotime::from_hz(frame_hz) };
	if (!m_timing_cb.isnull())
		m_timing_cb(m_timing.width, m_timing.height, m_timing.vis, m_timing.period);
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
		if (alpha == 255 && color == 0x00'ff'ffff)
			return pixel;
		auto div255 = [](u32 x) { return (x * 0x8081) >> 23; }; // exact for x <= 65025
		const u32 a = div255(((pixel >> 24) & 0xff) * alpha);
		const u32 r = div255(((pixel >> 16) & 0xff) * ((color >> 16) & 0xff));
		const u32 g = div255(((pixel >> 8) & 0xff) * ((color >> 8) & 0xff));
		const u32 bl = div255((pixel & 0xff) * (color & 0xff));
		return (a << 24) | (r << 16) | (g << 8) | bl;
	};
	auto luminance = [color, alpha](u32 level)
	{
		if (alpha == 255 && color == 0x00'ff'ffff)
			return level << 24 | level * 0x01'01'01;
		auto div255 = [](u32 x) { return (x * 0x8081) >> 23; }; // exact for x <= 65025
		return (div255(level * alpha) << 24)
			| (div255(((color >> 16) & 0xff) * level) << 16)
			| (div255(((color >> 8) & 0xff) * level) << 8)
			| div255((color & 0xff) * level);
	};
	switch (fmt)
	{
	case FMT_L8:
		return luminance(b);
	case FMT_RGB332: // r3 g3 b2
		return modulate(0xff000000
			| (((b >> 5) * 73 >> 1) << 16)
			| ((((b >> 2) & 7) * 73 >> 1) << 8)
			| ((b & 3) * 85));
	case FMT_ARGB1555:
	{
		const u16 w = get_u16le(&m_ram_g[idx16 & (RAM_G_SIZE - 2)]);
		return modulate((BIT(w, 15) ? 0xff000000 : 0)
			| ((((w >> 10) & 0x1f) * 33 >> 2) << 16)
			| ((((w >> 5) & 0x1f) * 33 >> 2) << 8)
			| ((w & 0x1f) * 33 >> 2));
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
			| ((((w >> 11) & 0x1f) * 33 >> 2) << 16)
			| ((((w >> 5) & 0x3f) * 65 >> 4) << 8)
			| ((w & 0x1f) * 33 >> 2));
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
			| ((((w >> 11) & 0x1f) * 33 >> 2) << 16)
			| ((((w >> 5) & 0x3f) * 65 >> 4) << 8)
			| ((w & 0x1f) * 33 >> 2));
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


bool ft812_device::rom_font_desc(u32 font, u32 &fmt, u32 &stride, u32 &w, u32 &h, u32 &gfx) const
{
	if (font < 16 || font > 34)
		return false;
	const u8 *const e = &m_font_rom[FONT_ROM_TABLE + (font - 16) * 148];
	fmt = get_u32le(&e[128]);
	stride = get_u32le(&e[132]);
	w = get_u32le(&e[136]);
	h = get_u32le(&e[140]);
	gfx = get_u32le(&e[144]);
	return (fmt == FMT_L1 || fmt == FMT_L4) && stride && w && h && gfx >= 0x1e0000 && gfx < 0x300000;
}

void ft812_device::draw_rom_font(s32 x, s32 y, u8 handle, u8 cell)
{
	u32 fmt, stride, w, h, gfx;
	if (!rom_font_desc(handle, fmt, stride, w, h, gfx))
		return;
	// fonts 17/19 hold ASCII 0x80-0xFF indexed from code 0 (PG 5.5.2)
	if (cell > 0x7f || (handle != 17 && handle != 19 && (cell < 0x20 || cell > 0x7e)))
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

void ft812_device::draw_bitmap(s32 x, s32 y, u8 handle, u8 cell)
{
	const bitmap_state &bitmap = m_bitmap_state[handle];
	// Handles 16-31 address the built-in ROM fonts unless the guest overrides
	// them with BITMAP_LAYOUT (a raw VERTEX2II needs no BITMAP_SOURCE)
	if (handle >= 16 && bitmap.layout == 0)
	{
		draw_rom_font(x, y, handle, cell);
		return;
	}

	const u32 fmt = (bitmap.layout >> 19) & 0x1f;
	const u32 stride = ((bitmap.layout >> 9) & 0x3ff) | (((bitmap.layout_h >> 2) & 3) << 10);
	const u32 lines = (bitmap.layout & 0x1ff) | ((bitmap.layout_h & 3) << 9);
	const u32 w = ((bitmap.size >> 9) & 0x1ff) | (((bitmap.size_h >> 2) & 3) << 9);
	const u32 h = (bitmap.size & 0x1ff) | ((bitmap.size_h & 3) << 9);
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
	const u32 source_width = std::min(w, row_width);
	const u32 source_height = std::min(h, lines);
	if (!source_width || !source_height)
		return;

	const u32 src = bitmap.source + u32(cell) * stride * lines;
	const double a = double(m_ctx.bitmap_transform_a) / 256.0;
	const double b = double(m_ctx.bitmap_transform_b) / 256.0;
	const double c = double(m_ctx.bitmap_transform_c) / 256.0;
	const double d = double(m_ctx.bitmap_transform_d) / 256.0;
	const double e = double(m_ctx.bitmap_transform_e) / 256.0;
	const double f = double(m_ctx.bitmap_transform_f) / 256.0;
	const double det = a * e - b * d;
	if (std::abs(det) < 1.0e-12)
		return;

	// FT812 coefficients map destination coordinates to bitmap coordinates.
	auto inverse = [a, b, c, d, e, f, det](double u, double v)
	{
		u -= c;
		v -= f;
		return std::pair<double, double>((e * u - b * v) / det, (a * v - d * u) / det);
	};


	const std::pair<double, double> corners[] = {
		inverse(0, 0), inverse(source_width, 0), inverse(0, source_height), inverse(source_width, source_height)
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

	// when no alpha test, stencil effect, tag write or blending applies the
	// result replaces the destination directly; skip the draw_px machinery
	const bool direct = m_ctx.alpha_func == 7 && m_ctx.stencil_func == 7 && m_ctx.stencil_write_mask == 0 &&
		m_ctx.tag_mask == 0 && m_ctx.blend_src == 1 && m_ctx.blend_dst == 0 && m_ctx.color_mask == 0xf;

	// magnification: decode the sampled source rectangle once into argb texels,
	// replacing per-pixel format decode and modulation with a table lookup
	const double u_corners[] = {
		a * (left - x + 0.5) + b * (top - y + 0.5) + c, a * (right - x + 0.5) + b * (top - y + 0.5) + c,
		a * (left - x + 0.5) + b * (bottom - y + 0.5) + c, a * (right - x + 0.5) + b * (bottom - y + 0.5) + c };
	const double v_corners[] = {
		d * (left - x + 0.5) + e * (top - y + 0.5) + f, d * (right - x + 0.5) + e * (top - y + 0.5) + f,
		d * (left - x + 0.5) + e * (bottom - y + 0.5) + f, d * (right - x + 0.5) + e * (bottom - y + 0.5) + f };
	const s32 su_lo = std::clamp<s32>(s32(std::floor(*std::min_element(std::begin(u_corners), std::end(u_corners)))) - 1, 0, s32(source_width) - 1);
	const s32 su_hi = std::clamp<s32>(s32(std::ceil(*std::max_element(std::begin(u_corners), std::end(u_corners)))) + 1, 0, s32(source_width) - 1);
	const s32 sv_lo = std::clamp<s32>(s32(std::floor(*std::min_element(std::begin(v_corners), std::end(v_corners)))) - 1, 0, s32(source_height) - 1);
	const s32 sv_hi = std::clamp<s32>(s32(std::ceil(*std::max_element(std::begin(v_corners), std::end(v_corners)))) + 1, 0, s32(source_height) - 1);
	const u32 rw = u32(su_hi - su_lo + 1), rh = u32(sv_hi - sv_lo + 1);
	const u64 dest_count = u64(right - left + 1) * u64(bottom - top + 1);
	const bool decode_all = !bitmap.wrap_x && !bitmap.wrap_y && u64(rw) * rh < dest_count * (bitmap.filter ? 3 : 1);
	std::vector<u32> scratch;
	if (decode_all)
	{
		scratch.resize(rw * rh);
		for (u32 v = 0; v < rh; v++)
			for (u32 u = 0; u < rw; u++)
				scratch[v * rw + u] = bitmap_pixel(src, fmt, u32(su_lo) + u, u32(sv_lo) + v, stride, lines);
	}
	auto fetch = [&](s32 tu, s32 tv) -> u32
	{
		if (decode_all)
			return scratch[(tv - sv_lo) * rw + (tu - su_lo)];
		return bitmap_pixel(src, fmt, tu, tv, stride, lines);
	};

	// per-draw pixel writer: when the alpha and stencil tests always pass the
	// per-pixel decisions are constant, so only blending/tag/stencil-write remain
	u8 stencil_lut[256];
	if (m_ctx.alpha_func == 7 && m_ctx.stencil_func == 7 && m_ctx.stencil_write_mask)
	{
		for (u32 old = 0; old < 256; old++)
		{
			switch (m_ctx.stencil_pass)
			{
			case 0: stencil_lut[old] = 0; break;
			case 2: stencil_lut[old] = m_ctx.stencil_ref; break;
			case 3: stencil_lut[old] = u8(std::min<u32>(0xff, old + 1)); break;
			case 4: stencil_lut[old] = u8(old ? old - 1 : 0); break;
			case 5: stencil_lut[old] = u8(~old); break;
			case 6: stencil_lut[old] = u8(old + 1); break;
			case 7: stencil_lut[old] = u8(old - 1); break;
			default: stencil_lut[old] = u8(old); break;
			}
		}
	}
	const bool fast_px = m_ctx.alpha_func == 7 && m_ctx.stencil_func == 7;
	auto put_px = [&](s32 dx, s32 dy, u32 px)
	{
		if (direct)
		{
			m_bitmap.pix(dy, dx) = px;
			return;
		}
		if (!fast_px)
		{
			draw_px(dx, dy, px);
			return;
		}
		if (m_ctx.stencil_write_mask)
		{
			u8 &st = m_stencil.pix(dy, dx);
			st = u8(st & ~m_ctx.stencil_write_mask) | u8(stencil_lut[st] & m_ctx.stencil_write_mask);
		}
		if (m_ctx.tag_mask)
			m_tag_buffer.pix(dy, dx) = m_ctx.tag;
		if (!px && m_ctx.blend_src == 2 && m_ctx.blend_dst == 4)
			return; // fully transparent source leaves every channel at dst
		const u32 d = m_bitmap.pix(dy, dx);
		if (m_ctx.color_mask == 0x1 && m_ctx.blend_src == 1 && m_ctx.blend_dst == 0)
		{
			// only the alpha channel is written and passes through unmodulated
			m_bitmap.pix(dy, dx) = (d & 0x00'ff'ffff) | (px & 0xff'00'00'00);
			return;
		}
		const u32 a = (px >> 24) & 0xff;
		if (m_ctx.color_mask == 0xe && m_ctx.blend_dst == 0 && m_ctx.blend_src == 2)
		{
			// rgb = src * src_alpha, alpha keeps destination
			const u32 r = (((px >> 16) & 0xff) * a + 127) / 255;
			const u32 g = (((px >> 8) & 0xff) * a + 127) / 255;
			const u32 b = ((px & 0xff) * a + 127) / 255;
			m_bitmap.pix(dy, dx) = (d & 0xff'00'00'00) | r << 16 | g << 8 | b;
			return;
		}
		if (m_ctx.color_mask == 0xe && m_ctx.blend_src == 2 && m_ctx.blend_dst == 4)
		{
			// classic alpha blend of rgb, alpha keeps destination
			const u32 ia = 255 - a;
			auto channel = [a, ia](u32 s, u32 dst) { return std::min<u32>(255, (s * a + dst * ia + 127) / 255); };
			const u32 r = channel((px >> 16) & 0xff, (d >> 16) & 0xff);
			const u32 g = channel((px >> 8) & 0xff, (d >> 8) & 0xff);
			const u32 b = channel(px & 0xff, d & 0xff);
			m_bitmap.pix(dy, dx) = (d & 0xff'00'00'00) | r << 16 | g << 8 | b;
			return;
		}
		auto blend_factor = [a, d](u8 factor)
		{
			switch (factor)
			{
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
			const u32 src = (px >> shift) & 0xff;
			const u32 dst = (d >> shift) & 0xff;
			const u32 component = std::min<u32>(255, (src * fs + dst * fd + 127) / 255);
			const u32 mask_bit = shift == 24 ? 0x01 : 1U << (shift / 8 + 1);
			out |= ((m_ctx.color_mask & mask_bit) ? component : dst) << shift;
		}
		m_bitmap.pix(dy, dx) = out;
	};

	if (!bitmap.filter)
	{
		// nearest sampling with fixed-point stepping; out-of-range texels never
		// reach draw_px, which keeps the common unfiltered-scaled case cheap
		const s32 step_ux = s32(std::clamp(a * 65536.0, -2147483648.0, 2147483520.0));
		const s32 step_uy = s32(std::clamp(b * 65536.0, -2147483648.0, 2147483520.0));
		const s32 step_vx = s32(std::clamp(d * 65536.0, -2147483648.0, 2147483520.0));
		const s32 step_vy = s32(std::clamp(e * 65536.0, -2147483648.0, 2147483520.0));
		const double px0 = double(s32(left) - x) + 0.5;
		const double py0 = double(s32(top) - y) + 0.5;
		s64 su = s64((a * px0 + b * py0 + c - 0.5) * 65536.0);
		s64 sv = s64((d * px0 + e * py0 + f - 0.5) * 65536.0);

		const s32 y_lo = s32(top), y_hi = s32(bottom);
		auto rows = [&](s32 ra, s32 rb)
		{
			s64 ru = su + s64(step_uy) * (ra - y_lo), rv = sv + s64(step_vy) * (ra - y_lo);
			for (s32 dy = ra; dy <= rb; dy++)
			{
				s64 cu = ru, cv = rv;
				for (s32 dx = s32(left); dx <= s32(right); dx++)
				{
					s32 tu = s32((cu + 0x8000) >> 16);
					s32 tv = s32((cv + 0x8000) >> 16);
					cu += step_ux;
					cv += step_vx;

					if (bitmap.wrap_x)
						tu = (tu % s32(source_width) + s32(source_width)) % s32(source_width);
					else if (tu < 0 || tu >= s32(source_width))
					{
						// the old path still ran a transparent pixel through draw_px
						put_px(dx, dy, 0);
						continue;
					}
					if (bitmap.wrap_y)
						tv = (tv % s32(source_height) + s32(source_height)) % s32(source_height);
					else if (tv < 0 || tv >= s32(source_height))
					{
						put_px(dx, dy, 0);
						continue;
					}

					put_px(dx, dy, fetch(tu, tv));
				}
				ru += step_uy;
				rv += step_vy;
			}
		};
		rows(y_lo, y_hi);
		return;
	}

	// bilinear sampling with the same fixed-point stepping
	{
		const s32 step_ux = s32(std::clamp(a * 65536.0, -2147483648.0, 2147483520.0));
		const s32 step_uy = s32(std::clamp(b * 65536.0, -2147483648.0, 2147483520.0));
		const s32 step_vx = s32(std::clamp(d * 65536.0, -2147483648.0, 2147483520.0));
		const s32 step_vy = s32(std::clamp(e * 65536.0, -2147483648.0, 2147483520.0));
		const double px0 = double(s32(left) - x) + 0.5;
		const double py0 = double(s32(top) - y) + 0.5;
		s64 su = s64((a * px0 + b * py0 + c - 0.5) * 65536.0);
		s64 sv = s64((d * px0 + e * py0 + f - 0.5) * 65536.0);
		auto texel = [&fetch, &bitmap, w = source_width, h = source_height](s32 tx, s32 ty) -> u32
		{
			if (bitmap.wrap_x)
				tx = (tx % s32(w) + s32(w)) % s32(w);
			else if (tx < 0 || tx >= s32(w))
				return 0U;
			if (bitmap.wrap_y)
				ty = (ty % s32(h) + s32(h)) % s32(h);
			else if (ty < 0 || ty >= s32(h))
				return 0U;
			return fetch(tx, ty);
		};

		const s32 y_lo = s32(top), y_hi = s32(bottom);
		auto rows = [&](s32 ra, s32 rb)
		{
			s64 ru = su + s64(step_uy) * (ra - y_lo), rv = sv + s64(step_vy) * (ra - y_lo);
			for (s32 dy = ra; dy <= rb; dy++)
			{
				s64 cu = ru, cv = rv;
				for (s32 dx = s32(left); dx <= s32(right); dx++)
				{
					const s32 x0 = s32(cu >> 16), y0 = s32(cv >> 16);
				const u32 fx = u32(cu >> 8) & 0xff;
				const u32 fy = u32(cv >> 8) & 0xff;
				cu += step_ux;
				cv += step_vx;

				const u32 p00 = texel(x0, y0), p10 = texel(x0 + 1, y0);
				const u32 p01 = texel(x0, y0 + 1), p11 = texel(x0 + 1, y0 + 1);
				u32 out = 0;
				for (u32 shift : { 0U, 8U, 16U, 24U })
				{
					const u32 top = ((p00 >> shift) & 0xff) * (0x100 - fx) + ((p10 >> shift) & 0xff) * fx;
					const u32 bottom = ((p01 >> shift) & 0xff) * (0x100 - fx) + ((p11 >> shift) & 0xff) * fx;
					out |= ((top * (0x100 - fy) + bottom * fy) >> 16) << shift;
				}
					put_px(dx, dy, out);
				}
				ru += step_uy;
				rv += step_vy;
			}
		};
		rows(y_lo, y_hi);
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
	auto active_primitive = [this]()
	{
		u8 primitive = 0;
		for (u32 offset = 0; offset < m_dl_ptr; offset += 4)
		{
			const u32 command = get_u32le(&m_ram_dl[offset]);
			if ((command & 0xff'000000) == DL_BEGIN)
				primitive = command & 0x0f;
			else if ((command & 0xff'000000) == DL_END)
				primitive = 0;
		}
		return primitive;
	};
	auto emit_text = [this, &append_dl, &active_primitive](s32 x, s32 y, u32 font, u32 options, const std::string &text)
	{
		const u8 saved_primitive = active_primitive();
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
		append_dl(0x05000000 | (font & 0x1f)); // BITMAP_HANDLE
		if (font >= 16)
			append_dl(DL_BITMAP_SOURCE | 0x1e0000); // select the FT812 ROM font window
		append_dl(0x1f000001); // BEGIN(BITMAPS)
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
		if (saved_primitive)
			append_dl(DL_BEGIN | saved_primitive);
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
	auto fifo_put32 = [this, &r](u32 offset, u32 value)
	{
		for (u32 i = 0; i < 4; i++)
			m_ram_cmd[(r + offset + i) & 0xfff] = value >> (8 * i);
	};
	// total size in bytes of fixed-length FT81x commands (without string payloads)
	auto fixed_command_size = [](u8 opcode) -> u32
	{
		switch (opcode)
		{
		case 0x02: case 0x09: case 0x15: case 0x1f: case 0x23: case 0x36: case 0x38: case 0x3a: case 0x3c: case 0x34:
			return 8;
		case 0x16: case 0x1c: case 0x19: case 0x2b: case 0x39: case 0x3f: case 0x41:
			return 12;
		case 0x18: case 0x1d: case 0x25: case 0x2c: case 0x2d: case 0x3b: case 0x43:
			return 16;
		case 0x0b: case 0x0f: case 0x10: case 0x11: case 0x13: case 0x14: case 0x37:
			return 20;
		case 0x30: case 0x35:
			return 24;
		case 0x20: case 0x33:
			return 28;
		case 0x21:
			return 56;
		case 0x17: case 0x2f: case 0x31: case 0x32: case 0x40: case 0x42:
			return 4;
		default:
			return 0;
		}
	};
	auto handled_command = [](u8 opcode)
	{
		switch (opcode)
		{
		case 0x02: case 0x18: case 0x19: case 0x1c: case 0x1d: case 0x23: case 0x32: case 0x33:
			return true;
		default:
			return false;
		}
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
				r = ((r + consumed + 3) & ~3U) & 0xfff;
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
			case 0x0e: // CMD_KEYS
			case 0x12: // CMD_TOGGLE
				if (opcode != 0x0c)
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
			case 0x1e: // CMD_APPEND
				command_size = 12;
				complete = count >= command_size;
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
				if (const u32 size = fixed_command_size(opcode))
				{
					// recognised but not emulated (widgets, touch, media, ...): consume its
					// arguments so they are not relayed into the display list as plain words
					command_size = size;
					complete = count >= command_size;
					if (complete && !handled_command(opcode))
						LOGMASKED(LOG_WARN, "%s: co-processor command %08x is not emulated, %u bytes skipped\n", tag(), cmd, size);
					break;
				}
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
					const u8 saved_primitive = active_primitive();
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
					if (saved_primitive)
						append_dl(DL_BEGIN | saved_primitive);
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
			case 0x1e: // CMD_APPEND
			{
				const u32 address = word_at(4);
				const u32 length = word_at(8);
				if (length & 3)
				{
					logerror("%s: CMD_APPEND length %06x is not word-aligned\n", tag(), length);
					break;
				}
				if (address > RAM_G_SIZE || length > RAM_G_SIZE - address)
				{
					logerror("%s: CMD_APPEND source %06x+%06x exceeds RAM_G\n", tag(), address, length);
					break;
				}
				if (m_dl_ptr > RAM_DL_SIZE || length > RAM_DL_SIZE - m_dl_ptr)
				{
					logerror("%s: CMD_APPEND of %06x bytes exceeds remaining RAM_DL space\n", tag(), length);
					break;
				}
				for (u32 offset = 0; offset < length; offset += 4)
					append_dl(get_u32le(&m_ram_g[address + offset]));
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
				const double a = m_copro_matrix[0], b = m_copro_matrix[1];
				const double c = m_copro_matrix[2], d = m_copro_matrix[3];
				const double e = m_copro_matrix[4], f = m_copro_matrix[5];
				const double det = a * d - b * c;
				if (std::abs(det) < 1.0e-12)
				{
					LOGMASKED(LOG_WARN, "%s: CMD_SETMATRIX ignored for singular matrix\n", tag());
					break;
				}
				auto encode = [](double value, s32 min_value, s32 max_value)
				{
					return s32(std::clamp(value * 256.0, double(min_value), double(max_value)));
				};
				const double inv_det = 1.0 / det;
				const double bitmap_a = d * inv_det;
				const double bitmap_b = -c * inv_det;
				const double bitmap_c = (c * f - d * e) * inv_det;
				const double bitmap_d = -b * inv_det;
				const double bitmap_e = a * inv_det;
				const double bitmap_f = (b * e - a * f) * inv_det;
				append_dl(DL_BITMAP_TRANSFORM_A | (u32(encode(bitmap_a, -(1 << 16), (1 << 16) - 1)) & 0x1ffff));
				append_dl(DL_BITMAP_TRANSFORM_B | (u32(encode(bitmap_b, -(1 << 16), (1 << 16) - 1)) & 0x1ffff));
				append_dl(DL_BITMAP_TRANSFORM_C | (u32(encode(bitmap_c, -(1 << 23), (1 << 23) - 1)) & 0x00ffffff));
				append_dl(DL_BITMAP_TRANSFORM_D | (u32(encode(bitmap_d, -(1 << 16), (1 << 16) - 1)) & 0x1ffff));
				append_dl(DL_BITMAP_TRANSFORM_E | (u32(encode(bitmap_e, -(1 << 16), (1 << 16) - 1)) & 0x1ffff));
				append_dl(DL_BITMAP_TRANSFORM_F | (u32(encode(bitmap_f, -(1 << 23), (1 << 23) - 1)) & 0x00ffffff));
				break;
			}
			case 0x02: // CMD_INTERRUPT
				reg_w32(REG_INT_FLAGS, reg_r32(REG_INT_FLAGS) | INT_CMDFLAG);
				update_int();
				break;
			case 0x19: // CMD_REGREAD, result replaces the last argument in the fifo
			{
				const u32 address = word_at(4) & ~3U;
				u32 value = 0;
				if (address < RAM_REG_BASE + RAM_REG_SIZE)
					for (u32 i = 0; i < 4; i++)
						value |= u32(m_space->read_byte(address + i)) << (8 * i);
				fifo_put32(8, value);
				break;
			}
			case 0x18: // CMD_MEMCRC
			{
				const u32 address = word_at(4);
				const u32 length = word_at(8);
				u32 crc = 0;
				if (address < RAM_G_SIZE && length <= RAM_G_SIZE - address)
					crc = crc32(crc32(0, Z_NULL, 0), &m_ram_g[address], length);
				else
					LOGMASKED(LOG_WARN, "%s: CMD_MEMCRC range %06x+%06x exceeds RAM_G\n", tag(), address, length);
				fifo_put32(12, crc);
				break;
			}
			case 0x1c: // CMD_MEMZERO
				fill_memory(word_at(4), 0, word_at(8));
				break;
			case 0x1d: // CMD_MEMCPY
			{
				const u32 dest = word_at(4);
				const u32 src = word_at(8);
				const u32 length = word_at(12);
				if (dest < RAM_G_SIZE && src < RAM_G_SIZE && length <= RAM_G_SIZE - dest && length <= RAM_G_SIZE - src)
					std::memmove(&m_ram_g[dest], &m_ram_g[src], length);
				else
					LOGMASKED(LOG_WARN, "%s: CMD_MEMCPY %06x <- %06x, %06x bytes exceeds RAM_G\n", tag(), dest, src, length);
				break;
			}
			case 0x23: // CMD_GETPTR
				fifo_put32(4, m_inflate_addr);
				break;
			case 0x32: // CMD_COLDSTART
				m_copro_fgcolor = 0x00ffffff;
				m_copro_matrix[0] = 1.0;
				m_copro_matrix[1] = 0.0;
				m_copro_matrix[2] = 0.0;
				m_copro_matrix[3] = 1.0;
				m_copro_matrix[4] = 0.0;
				m_copro_matrix[5] = 0.0;
				break;
			case 0x33: // CMD_GETMATRIX, a..f as 16.16 in hardware order
			{
				static constexpr int order[6] = { 0, 2, 4, 1, 3, 5 };
				for (u32 i = 0; i < 6; i++)
					fifo_put32(4 + i * 4, u32(s32(std::clamp(m_copro_matrix[order[i]] * 65536.0, -2147483648.0, 2147483647.0))));
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
	reg_w32(REG_CMD_DL, m_dl_ptr);
}

void ft812_device::render_dl()
{
	m_ctx = dl_context();
	m_ctx_stack = {};
	m_have_vertex = false;

	u32 call_stack[4];
	u32 call_depth = 0;
	for (u32 i = 0, steps = 0; i < RAM_DL_SIZE / 4 && steps < RAM_DL_SIZE; i++, steps++)
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
			m_bitmap_state[m_ctx.bitmap_handle].source = cmd & 0x00'ffff'ff;
			break;
		case DL_BITMAP_HANDLE:
			m_ctx.bitmap_handle = cmd & 0x1f;
			break;
		case DL_BITMAP_LAYOUT:
			m_bitmap_state[m_ctx.bitmap_handle].layout = cmd & 0x00'ffffff;
			break;
		case DL_BITMAP_SIZE:
			m_bitmap_state[m_ctx.bitmap_handle].size = cmd & 0x00'ffffff;
			m_bitmap_state[m_ctx.bitmap_handle].filter = BIT(cmd, 20);
			m_bitmap_state[m_ctx.bitmap_handle].wrap_x = BIT(cmd, 19);
			m_bitmap_state[m_ctx.bitmap_handle].wrap_y = BIT(cmd, 18);
			break;
		case DL_BITMAP_LAYOUT_H: // linestride bits 11:10, height bits 10:9
			m_bitmap_state[m_ctx.bitmap_handle].layout_h = cmd & 0xf;
			break;
		case DL_BITMAP_SIZE_H: // width bits 10:9, height bits 10:9
			m_bitmap_state[m_ctx.bitmap_handle].size_h = cmd & 0xf;
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
			m_ctx.bitmap_transform_c = util::sext(cmd & 0x00ffffff, 24);
			break;
		case DL_BITMAP_TRANSFORM_D:
			m_ctx.bitmap_transform_d = util::sext(cmd & 0x1ffff, 17);
			break;
		case DL_BITMAP_TRANSFORM_E:
			m_ctx.bitmap_transform_e = util::sext(cmd & 0x1ffff, 17);
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
		case 0x1d'000000: // CALL, destination is a display list word index
			if (call_depth < 4)
				call_stack[call_depth++] = i;
			else
				LOGMASKED(LOG_WARN, "%s: display list CALL stack overflow at DL[%04x]\n", tag(), i * 4);
			i = (cmd & 0xffff) - 1;
			break;
		case 0x1e'000000: // JUMP
			i = (cmd & 0xffff) - 1;
			break;
		case 0x24'000000: // RETURN
			if (call_depth)
				i = call_stack[--call_depth];
			else
				LOGMASKED(LOG_WARN, "%s: display list RETURN without CALL at DL[%04x]\n", tag(), i * 4);
			break;
		case 0x2d'000000: // NOP
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
				switch (m_prim)
				{
				case PRIM_BITMAPS:
					draw_bitmap(x >> 4, y >> 4, handle, cell);
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
					draw_bitmap(xs >> 4, ys >> 4, m_ctx.bitmap_handle, m_ctx.cell);
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
