// license:BSD-3-Clause
// copyright-holders:Andrei I. Holub
#ifndef MAME_SINCLAIR_NEXT_SPECNEXT_DMA_H
#define MAME_SINCLAIR_NEXT_SPECNEXT_DMA_H

#pragma once

#include "machine/z80dma.h"

class specnext_dma_device : public z80dma_device
{
public:
	specnext_dma_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto in_sram_callback() { return m_in_sram_cb.bind(); }
	auto out_sram_callback() { return m_out_sram_cb.bind(); }

	void dma_mode_w(bool dma_mode) { m_dma_mode = dma_mode; }
	void dma_delay_w(bool dma_delay);
	void ddma_en_w(bool ddma_en) { m_ddma_en = ddma_en; }
	void layer2_active_bank_w(u8 bank) { m_layer2_active_bank = bank & 0x7f; }
	void layer2_shadow_bank_w(u8 bank) { m_layer2_shadow_bank = bank & 0x7f; }
	void layer2_resolution_w(u8 resolution) { m_layer2_resolution = resolution & 0x03; }

	virtual void write(u8 data) override;

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual void do_read() override;
	virtual void do_write() override;
	virtual void reset_byte_counter() override;

	virtual TIMER_CALLBACK_MEMBER(clock_w) override;

private:
	void ddma_write(u8 data);
	void ddma_reset();
	void ddma_load(bool reload);
	void layer2_xy_to_portB();
	u32 address_step(u32 addr, s32 delta) const;
	u32 portA_address() const { return (m_portA_ext << 16) | m_addressA; }
	u32 portB_address() const { return (m_portB_ext << 16) | m_addressB; }

	devcb_read8 m_in_sram_cb;
	devcb_write8 m_out_sram_cb;

	bool m_dma_mode; // 0 = zxn dma, 1 = z80 dma
	bool m_dma_delay;
	u8 m_r2_portB_preescaler_s;
	u64 m_dma_timer_0;

	// zxnDDMA (port 0x6b only)
	bool m_ddma_en;
	u8 m_ddma_cmd;
	u8 m_ddma_num_follow;
	u8 m_ddma_cur_follow;
	u8 m_ddma_follow[4];

	u8 m_ddma_flags;
	u8 m_portA_ext_s;
	u8 m_portB_ext_s;
	u8 m_portA_ext;
	u8 m_portB_ext;
	u16 m_row_len;
	u16 m_rows;
	u16 m_portA_stride;
	u16 m_portB_stride;
	u8 m_transparent;
	u8 m_fill;

	u32 m_portA_row;
	u32 m_portB_row;
	u16 m_row_counter;
	u16 m_rows_counter;

	u8 m_layer2_active_bank;
	u8 m_layer2_shadow_bank;
	u8 m_layer2_resolution;
};

DECLARE_DEVICE_TYPE(SPECNEXT_DMA, specnext_dma_device)

#endif // MAME_SINCLAIR_NEXT_SPECNEXT_DMA_H
