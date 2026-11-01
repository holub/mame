// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
/**********************************************************************
    Sprinter Sp2000 RAM Operations Accelerator

The accelerator listens to the z80 bus cycles:
  m1 cycle    - the opcode byte can switch the accelerator mode.
  read cycle  - memory drives the data bus; the accelerator writes
                its buffer from the bus address.
  write cycle - the cpu drives the data bus; the accelerator reads
                its buffer out to the bus address.

The disassembler decodes ordinary z80 instructions (with the stock
z80 disassembler) and prints the accelerator command each of them
carries on its own, without tracking which mode is armed above. The
plain z80 view is available in its own disassembly window.

  on / off / copy / vcopy / fill / vfill - direction commands seen by
      the accelerator on the m1 fetch of an unprefixed opcode. Only
      on (LD D,D) and off (LD B,B) are documented; the rest comes
      from the emulation (LD H,H keeps its raw direction code). After
      `on`, every read cycle loads the block size register ($00
      meaning $100).
  read (s)      - read cycle: memory drives the data bus; the armed
                   mode loads the accelerator buffer (buf) from the
                   bus address.
  write (t)     - write cycle: the cpu drives the data bus; the armed
                   mode reads the buffer out to the bus address (the
                   cpu-driven byte only survives in fill mode, so no
                   value operand is shown).
  ldi/ldir      - read (hl) + write (de) pair, bc times for the ldir
                   family. Whether the transfer steps horizontally or
                   vertically is decided by the mode command above,
                   not by the instruction.
  di / ei        - interrupts must stay masked while the accelerator
                   is armed: an interrupt on the bus would corrupt the
                   armed mode, hence the commands are accel-critical.
  fn ...        - buffer function select.

Everything else (immediate fetches, stack traffic, inc/dec, bit
tests, ...) is bus activity the accelerator may react to by accident
when armed, but carries no command of its own and prints "-".

Refs:
    sp2000.pdf, section 6 "RAM operations accelerator" (Akselerator
    operatsiy s OZU)
**********************************************************************/

#include "emu.h"
#include "sprinter_accel_dasm.h"

u32 sprinter_accel_disassembler::opcode_alignment() const
{
	return m_z80.opcode_alignment();
}

namespace {


// commands of a plain (unprefixed) instruction
void plain_ops(std::ostream &c, u8 op)
{
	if ((op & 0xc0) == 0x40) // ld r,r'
	{
		switch (op)
		{
			case 0x40:
			case 0x76: c << "off"; return;        // ld b,b / halt
			case 0x52: c << "on"; return;         // ld d,d
			case 0x49: c << "fill"; return;       // ld c,c
			case 0x5b: c << "vfill"; return;      // ld e,e
			case 0x64: c << "dir $41"; return;    // ld h,h
			case 0x6d: c << "copy"; return;       // ld l,l
			case 0x7f: c << "vcopy"; return;      // ld a,a
			default: break;
		}
		const u8 dst = (op >> 3) & 7;
		const u8 src = op & 7;
		if (dst == 6 && src != 6)
			c << "write (hl)";
		else if (src == 6 && dst != 6)
			c << "read (hl)";
		return;
	}

	if ((op & 0xc0) == 0x80) // alu a,(hl)
	{
		switch (op)
		{
			case 0xa6: c << "fn and"; return;
			case 0xb6: c << "fn or";  return;
			case 0xae: c << "fn xor"; return;
			case 0xbe: c << "fn nop"; return;
			default: return;
		}
	}

	switch (op)
	{
		case 0xf3: c << "di"; break; // interrupts must stay off while the accel is armed
		case 0xfb: c << "ei"; break;
		case 0x02: c << "write (bc)"; break;
		case 0x0a: c << "read (bc)"; break;
		case 0x12: c << "write (de)"; break;
		case 0x1a: c << "read (de)"; break;

		case 0x22: c << "write (nn)"; break;    // ld (nn),hl
		case 0x2a: c << "read (nn)"; break;     // ld hl,(nn)
		case 0x32: c << "write (nn)"; break;    // ld (nn),a
		case 0x3a: c << "read (nn)"; break;     // ld a,(nn)
		case 0x34: case 0x35: c << "read (hl), write (hl)"; break; // inc/dec (hl)
		case 0x36: c << "read, write (hl)"; break; // ld (hl),n: immediate fetch + write
		default: break;
	}
}

// commands of an ed-prefixed instruction
void ed_ops(std::ostream &c, u8 op)
{
	switch (op)
	{
		case 0xa0: case 0xa8: c << "read (hl), write (de)"; break;     // ldi/ldd
		case 0xb0: case 0xb8: c << "read (hl), write (de), bc"; break; // ldir/lddr
		case 0xa1: case 0xa9: c << "read (hl)"; break;                 // cpi/cpd
		case 0xb1: case 0xb9: c << "read (hl), bc"; break;             // cpir/cpdr
		case 0xa2: case 0xaa: c << "read (hl)"; break;                 // outi/outd
		case 0xb2: case 0xba: c << "read (hl), bc"; break;             // otir/otdr
		case 0xa3: case 0xab: c << "write (hl)"; break;                // ini/ind
		case 0xb3: case 0xbb: c << "write (hl), bc"; break;            // inir/indr
		case 0x43: case 0x53: case 0x63: case 0x73: c << "write (nn)"; break; // ld (nn),rp
		case 0x4b: case 0x5b: case 0x6b: case 0x7b: c << "read (nn)"; break;  // ld rp,(nn)
		default: break;
	}
}

// commands of an xx-prefixed (ix/iy) instruction
void xy_ops(std::ostream &c, u8 op)
{
	if ((op & 0xc0) == 0x40) // ld r,r' via (xy+d)
	{
		const u8 dst = (op >> 3) & 7;
		const u8 src = op & 7;
		if (dst == 6 && src != 6)
			c << "write (xy)";
		else if (src == 6 && dst != 6)
			c << "read (xy)";
	}
	else if (op == 0x36) // ld (xy+d),n: immediate fetch + write
	{
		c << "read, write (xy)";
	}
}

} // anonymous namespace

offs_t sprinter_accel_disassembler::disassemble(std::ostream &stream, offs_t pc, const data_buffer &opcodes, const data_buffer &params)
{
	// instruction boundaries and flags come from the stock disassembler; discard its text
	std::ostringstream z80_text;
	const offs_t info = m_z80.disassemble(z80_text, pc, opcodes, params);

	std::ostringstream c;
	const u8 op0 = opcodes.r8(pc);
	const u8 op1 = opcodes.r8(pc + 1);
	if (op0 == 0x52 && op1 == 0x3e) // on followed by its simplest size carrier ld a,n; $00 means $100
	{
		const u16 size = opcodes.r8(pc + 2);
		util::stream_format(stream, "on $%x", size ? size : 0x100);
		return (info & ~util::disasm_interface::LENGTHMASK) | 3;
	}
	if (op0 == 0xcb)
	{
		if ((op1 & 7) == 6)
		{
			if ((op1 & 0xc0) == 0x40) // bit b,(hl)
				c << "read (hl)";
			else // rot/shift/res/set (hl)
				c << "read (hl), write (hl)";
		}
	}
	else if (op0 == 0xed)
	{
		ed_ops(c, op1);
	}
	else if (op0 == 0xdd || op0 == 0xfd)
	{
		if (op1 == 0xcb) // xx cb d op: rot/res/set style (xy) access
			c << "read (xy), write (xy)";
		else
			xy_ops(c, op1);
	}
	else
	{
		plain_ops(c, op0);
	}

	const std::string text = c.str();
	if (text.empty())
		stream << "-";
	else
		stream << text;

	return info;
}
