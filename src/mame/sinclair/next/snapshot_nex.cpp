// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
/**********************************************************************

    NEX file format loader for ZX Spectrum Next.

    Reference: https://wiki.specnext.dev/NEX_file_format
    Loader behaviour mirrors nexload.asm by Jim Bagley.

    The NEX file format loads self-contained applications into
    memory and starts them. The file consists of:
      - 512-byte header (magic "Next" + version + bank map + entry info)
      - optional palette (512 bytes) - for Layer2, LoRes or Tilemap screen
      - optional loading screens (Layer2 / ULA / LoRes / HiRes / HiCol)
      - optional Copper data (2048 bytes) [not applied]
      - 16K bank data in file order: 5,2,0,1,3,4,6,7,8,...,111

    Only the banks flagged in the header's 112-byte bank map are present.

    All memory writes go through the poke callback so the driver's MMU
    routing (including bram_bank5 / bram_bank7 display shares) is honoured.

**********************************************************************/

#include "snapshot_nex.h"

#include <algorithm>
#include <cstring>

nex_file::result nex_file::load(snapshot_image_device &image, const hooks &h)
{
	result res;

	u8 header[512];
	if (image.fread(header, 512) != 512)
		return res;

	// Validate magic "Next"
	if (std::memcmp(header, "Next", 4) != 0)
		return res;

	// Parse header fields
	const u8  screen_flags   = header[10];      // bit0=Layer2 bit1=ULA bit2=LoRes bit3=HiRes bit4=HiCol bit7=no-palette
	const u8  border_color   = header[11];
	const u16 sp             = header[12] | (header[13] << 8);
	const u16 pc             = header[14] | (header[15] << 8);
	const u8 *bank_map       = header + 18;     // 112 bytes, indexed by physical bank number
	const bool preserve_regs = true || header[134];     // 0 = reset machine state, 1 = preserve
	const u8  entry_bank     = header[139];     // V1.2: bank mapped to $C000 before jump

	auto read_to_bank = [&image, &h](u8 bank, u32 offset, u32 count)
	{
		h.reg_w(0x56, bank * 2);
		h.reg_w(0x57, bank * 2 + 1);

		u8 buf[0x4000];
		const u32 to_read = std::min(count, u32(0x4000));
		if (image.fread(buf, to_read) != to_read)
			return;
		for (u32 i = 0; i < to_read; i++)
			h.poke(0xc000 + offset + i, buf[i]);
	};

	static const u8 ula_default[16] = {
		0x00, 0x02, 0xa0, 0xa2, 0x14, 0x16, 0xb4, 0xb6,
		0x00, 0x03, 0xe0, 0xe7, 0x1c, 0x1f, 0xfc, 0xff
	};
	auto write_pal = [&](u8 nr43_val, int count, const u8 *values)
	{
		h.reg_w(0x43, nr43_val);
		h.reg_w(0x40, 0x00);
		for (int i = 0; i < count; i++)
			h.reg_w(0x41, values[i]);
	};
	u8 pal_buf[256];
	for (int i = 0; i < 256; i++) pal_buf[i] = ula_default[i & 15];
	write_pal(0x00, 256, pal_buf);
	write_pal(0x40, 256, pal_buf);

	for (int i = 0; i < 256; i++) pal_buf[i] = 0;
	write_pal(0x30, 256, pal_buf);
	write_pal(0x70, 256, pal_buf);

	for (int i = 0; i < 256; i++) pal_buf[i] = i;
	write_pal(0x10, 256, pal_buf);
	write_pal(0x50, 256, pal_buf);
	write_pal(0x20, 256, pal_buf);
	write_pal(0x60, 256, pal_buf);

	const bool has_palette = !(screen_flags & 0x80);
	if (has_palette)
	{
		u8 palette[512];
		if (image.fread(palette, 512) != 512)
			return res;
	}

	if (screen_flags & 0x01)
	{
		read_to_bank(9,  0, 0x4000);
		read_to_bank(10, 0, 0x4000);
		read_to_bank(11, 0, 0x4000);
	}

	// Classic ULA (6912 bytes) -> bank 5 ($C000+).
	if (screen_flags & 0x02)
	{
		read_to_bank(5, 0, 6912);
	}

	// LoRes / HiRes / HiCol (2 x 6144 bytes) -> bank 5, then bank 7.
	if (screen_flags & (0x04 | 0x08 | 0x10))
	{
		read_to_bank(5, 0, 6144);
		read_to_bank(7, 0, 6144);
	}

	// Layer2 320x256x8 or 640x256x4 (81920 bytes) -> banks 9, 10, 11, 12, 13.
	if (screen_flags & 0x20)
	{
		read_to_bank(9,  0, 0x4000);
		read_to_bank(10, 0, 0x4000);
		read_to_bank(11, 0, 0x4000);
		read_to_bank(12, 0, 0x4000);
		read_to_bank(13, 0, 0x4000);
	}

	static const u8 nex_bank_order[6] = { 5, 2, 0, 1, 3, 4 };
	for (int i = 0; i < 112; i++)
	{
		const u8 phys_bank = (i < 6) ? nex_bank_order[i] : u8(i);
		if (!bank_map[phys_bank])
			continue;

		read_to_bank(phys_bank, 0, 0x4000);
	}

	// nexload.asm resets these when preserve_regs=0 (HEADER_DONTRESETNEXTREGS)
	if (!preserve_regs)
	{
		h.reg_w(0x62, 0x00); h.reg_w(0x61, 0x00); // stop copper
		// nr08 (peripheral 3): preserve bit 4 (internal speaker = user setting),
		// set bits 7,6 (unlock paging, no contention), 5 (ABC stereo), 3 (specdrum),
		// 2 (timex port), 1 (turbosound), clear bit 0.
		const u8 nr08_old = h.reg_r ? h.reg_r(0x08) : u8(0);
		h.reg_w(0x08, (nr08_old & 0x10) | 0xce);
		h.reg_w(0x07, 0x03); // CPU speed 28MHz
		h.reg_w(0x12, 0x09); // layer2 active page
		h.reg_w(0x13, 0x0c); // layer2 shadow page
		h.reg_w(0x14, 0xe3); // global transparent RGB
		h.reg_w(0x15, 0x01); // sprite control: sprites on, over border, SLU priority
		h.reg_w(0x16, 0x00); h.reg_w(0x17, 0x00); // layer2 X/Y scroll
		h.reg_w(0x1c, 0x0f); // clip window index: reset all 4
		h.reg_w(0x18, 0x00); h.reg_w(0x18, 0xff); h.reg_w(0x18, 0x00); h.reg_w(0x18, 0xbf); // layer2 clip
		h.reg_w(0x19, 0x00); h.reg_w(0x19, 0xff); h.reg_w(0x19, 0x00); h.reg_w(0x19, 0xbf); // sprite clip
		h.reg_w(0x1a, 0x00); h.reg_w(0x1a, 0xff); h.reg_w(0x1a, 0x00); h.reg_w(0x1a, 0xbf); // ULA clip
		h.reg_w(0x1b, 0x00); h.reg_w(0x1b, 0x9f); h.reg_w(0x1b, 0x00); h.reg_w(0x1b, 0xff); // tilemap clip
		h.reg_w(0x32, 0x00); h.reg_w(0x33, 0x00); // lores X/Y scroll
		h.reg_w(0x43, 0x00); // ULA palette control (select primary palette)
		h.reg_w(0x42, 0x0f); // allow flashing
		h.reg_w(0x40, 0x00); // palette index
		h.reg_w(0x4a, 0x00); // transparency fallback value
		h.reg_w(0x4b, 0xe3); // sprite transparency index
		h.reg_w(0x50, 0xff); // MMU slot 0
		h.reg_w(0x51, 0xff); // MMU slot 1
	}

	h.reg_w(0x56, entry_bank * 2);
	h.reg_w(0x57, entry_bank * 2 + 1);

	res.loaded      = true;
	res.pc          = pc;
	res.sp          = sp;
	res.border_color = border_color;
	return res;
}
