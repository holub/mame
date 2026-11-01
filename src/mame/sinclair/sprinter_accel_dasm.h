// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
#ifndef MAME_SINCLAIR_SPRINTER_ACCEL_DASM_H
#define MAME_SINCLAIR_SPRINTER_ACCEL_DASM_H

#pragma once

#include "cpu/z80/z80dasm.h"

class sprinter_accel_disassembler : public util::disasm_interface
{
public:
	sprinter_accel_disassembler() = default;
	virtual ~sprinter_accel_disassembler() = default;

	virtual u32 opcode_alignment() const override;
	virtual offs_t disassemble(std::ostream &stream, offs_t pc, const data_buffer &opcodes, const data_buffer &params) override;

private:
	z80_disassembler m_z80;
};

#endif // MAME_SINCLAIR_SPRINTER_ACCEL_DASM_H
