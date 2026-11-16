// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
/**********************************************************************
    Spectrum Next DMA

    Spectrum Next DMA operates in two mode: z80dma compatible and
    N-mode with Next specific fixes.

    z80dma mode is implemented there based on intensive testing of
    the device. Potentially any mismatches not related to N-mode
    covered here must be moved to the z80dma parent.

    zxnDDMA (core 3.03) extends N-mode with 24-bit SRAM addresses,
    2D (rectangle) transfers, transparent colour, fill and Layer 2
    X,Y destination.

**********************************************************************/

#include "emu.h"
#include "specnext_dma.h"

#define LOG_UNDOC (1U << 1)
#define LOG_DDMA  (1U << 2)

#define VERBOSE (LOG_UNDOC)
#include "logmacro.h"

#define LOGUNDOC(...) LOGMASKED(LOG_UNDOC, __VA_ARGS__)
#define LOGDDMA(...)  LOGMASKED(LOG_DDMA, __VA_ARGS__)


// device type definition
DEFINE_DEVICE_TYPE(SPECNEXT_DMA, specnext_dma_device, "specnext_dma", "Spectrum Next DMA")


// TODO: this stuff is copy/pasted from machine/z80dma.cpp - ideally it wouldn't need to be
#define GET_REGNUM(_r)          (&(_r) - &(WR0))
#define WR0                     REG(0, 0)
#define WR1                     REG(1, 0)
#define WR2                     REG(2, 0)
#define WR4                     REG(4, 0)
#define WR5                     REG(5, 0)

#define PORTB_ADDRESS_L         REG(4,1)
#define PORTB_ADDRESS_H         REG(4,2)

#define ZXN_PRESCALER           REG(2,2)

#define PORTA_INC               (WR1 & 0x10)
#define PORTB_INC               (WR2 & 0x10)
#define PORTA_FIXED             (((WR1 >> 4) & 0x02) == 0x02)
#define PORTB_FIXED             (((WR2 >> 4) & 0x02) == 0x02)
#define PORTA_MEMORY            (((WR1 >> 3) & 0x01) == 0x00)
#define PORTB_MEMORY            (((WR2 >> 3) & 0x01) == 0x00)

#define PORTA_IS_SOURCE         ((WR0 >> 2) & 0x01)

#define TRANSFER_MODE           (WR0 & 0x03)
#define OPERATING_MODE          ((WR4 >> 5) & 0x03) // 0b00: Byte; 0b01: Continuous; 0b10: Burst; 0b11: Do not program
#define AUTO_RESTART            ((WR5>>5) & 0x01)

#define DDMA_PHYSICAL           BIT(m_ddma_flags, 0)
#define DDMA_2D                 BIT(m_ddma_flags, 1)
#define DDMA_TRANSPARENT        BIT(m_ddma_flags, 2)
#define DDMA_FILL               BIT(m_ddma_flags, 3)
#define DDMA_LAYER2_XY          BIT(m_ddma_flags, 4)
#define DDMA_LAYER2_SHADOW      BIT(m_ddma_flags, 5)

enum : u8
{
	COMMAND_PORTA_EXT       = 0xd7,
	COMMAND_PORTB_EXT       = 0xdb,
	COMMAND_MODE            = 0xdf,
	COMMAND_ROW_LEN         = 0xe3,
	COMMAND_ROWS            = 0xe7,
	COMMAND_PORTA_STRIDE    = 0xeb,
	COMMAND_PORTB_STRIDE    = 0xef,
	COMMAND_TRANSPARENT     = 0xf3,
	COMMAND_FILL            = 0xf7
};


specnext_dma_device::specnext_dma_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: z80dma_device(mconfig, SPECNEXT_DMA, tag, owner, clock)
	, m_in_sram_cb(*this, 0)
	, m_out_sram_cb(*this)
	, m_ddma_en(false)
{
}

void specnext_dma_device::dma_delay_w(bool dma_delay)
{
	if (m_dma_delay && !dma_delay && (m_dma_seq == SEQ_WAIT_READY))
	{
		set_busrq(ASSERT_LINE);
		m_dma_seq = SEQ_WAITING_ACK;
	}
	m_dma_delay = dma_delay;
}

void specnext_dma_device::reset_byte_counter()
{
	m_byte_counter = m_dma_mode ? 0 : 1;
}

u32 specnext_dma_device::address_step(u32 addr, s32 delta) const
{
	// 64K wrap unless physical
	return DDMA_PHYSICAL
		? (addr + delta) & 0xffffff
		: (addr & 0xff0000) | ((addr + delta) & 0xffff);
}

void specnext_dma_device::ddma_reset()
{
	m_ddma_flags = 0;
	m_portA_ext_s = 0;
	m_portB_ext_s = 0;
	m_row_len = 0;
	m_rows = 0;
	m_portA_stride = 0;
	m_portB_stride = 0;
	m_transparent = 0;
	m_fill = 0;
}

void specnext_dma_device::ddma_load(bool reload)
{
	if (reload)
	{
		m_portA_ext = m_portA_ext_s;
		m_portB_ext = m_portB_ext_s;
	}

	m_portA_row = portA_address();
	m_portB_row = portB_address();
	m_row_counter = 0;
	m_rows_counter = 0;

	if (DDMA_2D)
	{
		// end of block is checked on the last row only
		m_count = (m_rows > 1) ? 0 : std::max<u16>(m_row_len, 1);
		m_byte_counter = 1;
	}
}

void specnext_dma_device::layer2_xy_to_portB()
{
	const u8 bank = DDMA_LAYER2_SHADOW ? m_layer2_shadow_bank : m_layer2_active_bank;
	const u16 x = m_ddma_follow[1] | (BIT(m_ddma_follow[2], 0) << 8);
	const u8 y = m_ddma_follow[3];
	const u32 addr = 0x40000 + (bank << 14) + (m_layer2_resolution ? ((x << 8) | y) : ((y << 8) | (x & 0xff)));

	PORTB_ADDRESS_L = addr & 0xff;
	PORTB_ADDRESS_H = (addr >> 8) & 0xff;
	m_portB_ext_s = addr >> 16;
	m_portB_stride = 0x100;
	LOGDDMA("Layer 2 X,Y: %d,%d -> %06x\n", x, y, addr);
}

void specnext_dma_device::ddma_write(u8 data)
{
	m_ddma_follow[m_ddma_cur_follow++] = data;
	if (m_ddma_cmd == COMMAND_MODE && m_ddma_cur_follow == 1 && BIT(data, 4))
		m_ddma_num_follow = 4;
	if (m_ddma_cur_follow < m_ddma_num_follow)
		return;

	m_ddma_num_follow = 0;
	const u16 data16 = m_ddma_follow[0] | (m_ddma_follow[1] << 8);
	switch (m_ddma_cmd)
	{
	case COMMAND_PORTA_EXT:
		m_portA_ext_s = data;
		break;
	case COMMAND_PORTB_EXT:
		m_portB_ext_s = data;
		break;
	case COMMAND_MODE:
		m_ddma_flags = m_ddma_follow[0] & 0x3f;
		if (DDMA_LAYER2_XY)
			layer2_xy_to_portB();
		break;
	case COMMAND_ROW_LEN:
		m_row_len = data16;
		break;
	case COMMAND_ROWS:
		m_rows = data16;
		break;
	case COMMAND_PORTA_STRIDE:
		m_portA_stride = data16;
		break;
	case COMMAND_PORTB_STRIDE:
		m_portB_stride = data16;
		break;
	case COMMAND_TRANSPARENT:
		m_transparent = data;
		break;
	case COMMAND_FILL:
		m_fill = data;
		break;
	}
	LOGDDMA("DDMA %02x: %02x %02x %02x %02x\n", m_ddma_cmd, m_ddma_follow[0], m_ddma_follow[1], m_ddma_follow[2], m_ddma_follow[3]);
}

void specnext_dma_device::write(u8 data)
{
	if (m_ddma_num_follow)
	{
		ddma_write(data);
		return;
	}

	if (num_follow() == 0)
	{
		if (m_ddma_en && !m_dma_mode)
		{
			switch (data)
			{
			case COMMAND_PORTA_EXT:
			case COMMAND_PORTB_EXT:
			case COMMAND_MODE:
			case COMMAND_TRANSPARENT:
			case COMMAND_FILL:
				m_ddma_num_follow = 1;
				break;
			case COMMAND_ROW_LEN:
			case COMMAND_ROWS:
			case COMMAND_PORTA_STRIDE:
			case COMMAND_PORTB_STRIDE:
				m_ddma_num_follow = 2;
				break;
			default:
				break;
			}

			if (m_ddma_num_follow)
			{
				m_ddma_cmd = data;
				m_ddma_cur_follow = 0;
				return;
			}
		}

		if ((data & 0x82) == 0x02)
		{
			LOGUNDOC("WR0 uses with not Transfer mode bits set: %02x. Forcing Transfer mode.\n", data);
			data = (data | 0x01) & ~0x02;
		}

		z80dma_device::write(data);
		if ((data & 0x83) == 0x83) // WR6
		{
			switch (data)
			{
			case COMMAND_ENABLE_DMA:
				reset_byte_counter();
				break;
			case COMMAND_RESET:
				m_r2_portB_preescaler_s = 0;
				ddma_reset();
				break;
			case COMMAND_LOAD:
				ddma_load(true);
				break;
			case COMMAND_CONTINUE:
				ddma_load(false);
				break;
			default:
				break;
			}
		}
	}
	else
	{
		const u8 nreg = m_regs_follow[m_cur_follow];
		if (nreg == REGNUM(2, 1))
		{
			if (data & 0x20)
				m_regs_follow[m_num_follow++] = GET_REGNUM(ZXN_PRESCALER);
		}
		else if (nreg == REGNUM(2, 2))
		{
			m_r2_portB_preescaler_s = data;
		}
		z80dma_device::write(data);
	}
}

void specnext_dma_device::do_read()
{
	if (DDMA_FILL)
		m_latch = m_fill;
	else if (DDMA_PHYSICAL && (PORTA_IS_SOURCE ? PORTA_MEMORY : PORTB_MEMORY))
		m_latch = m_in_sram_cb((PORTA_IS_SOURCE ? portA_address() : portB_address()) & 0x3fffff);
	else
		z80dma_device::do_read();
}

void specnext_dma_device::do_write()
{
	if ((m_ddma_flags & 0x0f) == 0)
	{
		z80dma_device::do_write();
		return;
	}

	if (!DDMA_TRANSPARENT || m_latch != m_transparent)
	{
		if (DDMA_PHYSICAL && (PORTA_IS_SOURCE ? PORTB_MEMORY : PORTA_MEMORY))
			m_out_sram_cb((PORTA_IS_SOURCE ? portB_address() : portA_address()) & 0x3fffff, m_latch);
		else
			do_transfer_write();
	}

	u32 portA = address_step(portA_address(), PORTA_FIXED ? 0 : PORTA_INC ? 1 : -1);
	u32 portB = address_step(portB_address(), PORTB_FIXED ? 0 : PORTB_INC ? 1 : -1);
	m_byte_counter++;

	if (DDMA_2D && ++m_row_counter >= std::max<u16>(m_row_len, 1))
	{
		m_row_counter = 0;
		if (++m_rows_counter < m_rows)
		{
			m_portA_row = portA = address_step(m_portA_row, m_portA_stride);
			m_portB_row = portB = address_step(m_portB_row, m_portB_stride);
			if (m_rows_counter == m_rows - 1)
			{
				m_count = std::max<u16>(m_row_len, 1);
				m_byte_counter = 1;
			}
		}
	}

	m_addressA = portA & 0xffff;
	m_portA_ext = portA >> 16;
	m_addressB = portB & 0xffff;
	m_portB_ext = portB >> 16;
}

TIMER_CALLBACK_MEMBER(specnext_dma_device::clock_w)
{
	if (m_dma_delay)
	{
		if (m_dma_seq == SEQ_WAIT_READY)
			return;

		if (m_dma_seq == SEQ_TRANS1_WRITE_DEST)
		{
			z80dma_device::clock_w(param);
			if (m_dma_seq == SEQ_TRANS1_SAMPLE_READY)
			{
				set_busrq(CLEAR_LINE);
				m_dma_seq = SEQ_WAIT_READY;
			}
			return;
		}
	}

	// the second and later bytes of a burst or block enter the transfer through SEQ_TRANS1_SAMPLE_READY
	if (m_dma_seq == SEQ_TRANS1_INC_DEC_SOURCE_ADDRESS || m_dma_seq == SEQ_TRANS1_SAMPLE_READY)
	{
		m_dma_timer_0 = machine().time().as_ticks(unscaled_clock()) * 8;
	}

	const bool may_prescaled = m_dma_seq == SEQ_TRANS1_WRITE_DEST && m_r2_portB_preescaler_s;
	const bool may_restart = m_dma_seq == SEQ_FINISH;
	if (m_dma_mode == 0 && m_dma_seq == SEQ_WAIT_READY && is_ready() && m_byte_counter == 1)
		m_dma_seq = SEQ_REQUEST_BUS;
	else
	{
		z80dma_device::clock_w(param);

		if (DDMA_FILL && m_dma_seq == SEQ_TRANS1_READ_SOURCE)
		{
			// fill has no read cycle
			do_read();
			m_dma_seq = SEQ_TRANS1_INC_DEC_DEST_ADDRESS;
			z80dma_device::clock_w(param);
		}
		else if (may_restart && AUTO_RESTART)
		{
			ddma_load(true);
		}
	}

	if (may_prescaled && m_dma_seq != SEQ_FINISH)
	{
		const u64 dma_timer_s = machine().time().as_ticks(unscaled_clock()) * 8 - m_dma_timer_0;
		if (m_r2_portB_preescaler_s > BIT(dma_timer_s, 5, 9))
		{
			if (OPERATING_MODE == 0b10) // Burst
				set_busrq(CLEAR_LINE);
			const u64 adj = (dma_timer_s & ~u16(0x3fff)) | (m_r2_portB_preescaler_s << 5);
			m_timer->adjust(attotime::from_ticks(adj - dma_timer_s, unscaled_clock() * 8), 0, clocks_to_attotime(1));
			return;
		}
	}
}

void specnext_dma_device::device_start()
{
	z80dma_device::device_start();

	save_item(NAME(m_dma_mode));
	save_item(NAME(m_dma_delay));
	save_item(NAME(m_r2_portB_preescaler_s));
	save_item(NAME(m_dma_timer_0));

	save_item(NAME(m_ddma_en));
	save_item(NAME(m_ddma_cmd));
	save_item(NAME(m_ddma_num_follow));
	save_item(NAME(m_ddma_cur_follow));
	save_item(NAME(m_ddma_follow));
	save_item(NAME(m_ddma_flags));
	save_item(NAME(m_portA_ext_s));
	save_item(NAME(m_portB_ext_s));
	save_item(NAME(m_portA_ext));
	save_item(NAME(m_portB_ext));
	save_item(NAME(m_row_len));
	save_item(NAME(m_rows));
	save_item(NAME(m_portA_stride));
	save_item(NAME(m_portB_stride));
	save_item(NAME(m_transparent));
	save_item(NAME(m_fill));
	save_item(NAME(m_portA_row));
	save_item(NAME(m_portB_row));
	save_item(NAME(m_row_counter));
	save_item(NAME(m_rows_counter));
	save_item(NAME(m_layer2_active_bank));
	save_item(NAME(m_layer2_shadow_bank));
	save_item(NAME(m_layer2_resolution));
}

void specnext_dma_device::device_reset()
{
	z80dma_device::device_reset();

	m_dma_mode = 0;
	m_dma_delay = 0;
	m_r2_portB_preescaler_s = 0;
	m_dma_timer_0 = 0;

	m_ddma_cmd = 0;
	m_ddma_num_follow = 0;
	m_ddma_cur_follow = 0;
	memset(m_ddma_follow, 0, sizeof(m_ddma_follow));
	ddma_reset();
	m_portA_ext = 0;
	m_portB_ext = 0;
	m_portA_row = 0;
	m_portB_row = 0;
	m_row_counter = 0;
	m_rows_counter = 0;
	m_layer2_active_bank = 0;
	m_layer2_shadow_bank = 0;
	m_layer2_resolution = 0;
}
