// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
/**********************************************************************

    NEX file format loader for ZX Spectrum Next.

    Reference: https://wiki.specnext.dev/NEX_file_format
    Loader behaviour mirrors nexload.asm by Jim Bagley.

**********************************************************************/

#ifndef MAME_SINCLAIR_NEXT_SNAPSHOT_NEX_H
#define MAME_SINCLAIR_NEXT_SNAPSHOT_NEX_H

#pragma once

#include "emu.h"
#include "imagedev/snapquik.h"

#include <functional>

class nex_file
{
public:
	struct result
	{
		bool   loaded = false;
		u16    pc = 0;
		u16    sp = 0;
		u8     border_color = 0;
	};

	// Callbacks that let the loader reach into the machine state.
	// reg_w writes a NextReg register; poke writes a byte to the CPU
	// memory space (the driver pages in the desired bank via MMU 6/7 first).
	struct hooks
	{
		std::function<void(u8 reg, u8 data)> reg_w;
		std::function<u8(u8 reg)> reg_r;
		std::function<void(u16 addr, u8 data)> poke;
	};

	// Load a .NEX file. Returns result with loaded=true on success.
	result load(snapshot_image_device &image, const hooks &h);
};

#endif // MAME_SINCLAIR_NEXT_SNAPSHOT_NEX_H
