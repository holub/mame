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

	// Palette is only present when there's a Layer2 or LoRes loading screen
	// (bits 0 or 2), the no-palette flag (bit 7) is clear, and no ULA/HiRes/HiCol
	// screen (bits 1/3/4) is set. Matches nexload.asm lines 424-428.
	const bool has_palette = (screen_flags & 0x05) && !(screen_flags & 0x9a);
	if (has_palette)
	{
		u8 palette[512];
		if (image.fread(palette, 512) != 512)
			return res;
	}

	// Loading screens - each type reads independently from the file,
	// in the same order as nexload.asm lines 440-511.
	// Layer2 320x256x8 (3 x 16K) -> banks 9, 10, 11.
	if (screen_flags & 0x01)
	{
		read_to_bank(9,  0, 0x4000);
		read_to_bank(10, 0, 0x4000);
		read_to_bank(11, 0, 0x4000);
	}

	// Classic ULA (6912 bytes) -> bank 5.
	if (screen_flags & 0x02)
		read_to_bank(5, 0, 6912);

	// LoRes / HiRes / HiCol: each set type reads 2 x 6144 bytes -> bank 5.
	for (u8 mask = 0x04; mask <= 0x10; mask <<= 1)
		if (screen_flags & mask)
		{
			read_to_bank(5, 0x0000, 0x1800);
			read_to_bank(5, 0x2000, 0x1800);
		}

	static const u8 nex_bank_order[6] = { 5, 2, 0, 1, 3, 4 };
	for (int i = 0; i < 112; i++)
	{
		const u8 phys_bank = (i < 6) ? nex_bank_order[i] : u8(i);
		if (!bank_map[phys_bank])
			continue;

		read_to_bank(phys_bank, 0, 0x4000);
	}

	h.reg_w(0x4a, 0x00); // transparency fallback value
	h.reg_w(0x56, entry_bank * 2);
	h.reg_w(0x57, entry_bank * 2 + 1);

	res.loaded      = true;
	res.pc          = pc;
	res.sp          = sp;
	res.border_color = border_color;
	return res;
}
