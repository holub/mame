// license:BSD-3-Clause
// copyright-holders:Dmitrii Mikhalchenkov
/***************************************************************************

    Sprinter-WiFi / Sprinter-ESP ISA card

    The card decodes COM3 (0x03e8-0x03ef) and uses a TL16C550C UART clocked
    by 14.7456 MHz (divisor 8 = 115200 bit/s).  The UART -OUT1 pin drives
    the RESET input of the ESP8266 module: MCR bit 2 set holds the module
    in reset.  -OUT2 drives the module's program-mode strap and is left
    unconnected here.

    The emulated 16550 does not implement the TL16C550C auto-flow (AFE)
    mode: RTS follows MCR bit 1 only, CTS is reported in MSR.

***************************************************************************/

#include "emu.h"
#include "sprinter_esp.h"

#include "bus/rs232/esp8266_at.h"
#include "bus/rs232/null_modem.h"
#include "bus/rs232/rs232.h"
#include "bus/rs232/terminal.h"
#include "machine/ins8250.h"


namespace {

static void sprinter_esp_rs232_devices(device_slot_interface &device)
{
	device.option_add("esp8266_at", ESP8266_AT);
	device.option_add("null_modem", NULL_MODEM);
	device.option_add("terminal", SERIAL_TERMINAL);
}

static DEVICE_INPUT_DEFAULTS_START(sprinter_esp_rs232_defaults)
	DEVICE_INPUT_DEFAULTS("RS232_TXBAUD", 0xff, RS232_BAUD_115200)
	DEVICE_INPUT_DEFAULTS("RS232_RXBAUD", 0xff, RS232_BAUD_115200)
	DEVICE_INPUT_DEFAULTS("RS232_DATABITS", 0xff, RS232_DATABITS_8)
	DEVICE_INPUT_DEFAULTS("RS232_PARITY", 0xff, RS232_PARITY_NONE)
	DEVICE_INPUT_DEFAULTS("RS232_STOPBITS", 0xff, RS232_STOPBITS_1)
	DEVICE_INPUT_DEFAULTS("FLOW_CONTROL", 0x07, 0x01) // honour UART RTS when feeding bytes from the ESP side
DEVICE_INPUT_DEFAULTS_END


class isa8_sprinter_esp_device : public device_t, public device_isa8_card_interface
{
public:
	isa8_sprinter_esp_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
		device_t(mconfig, ISA8_SPRINTER_ESP, tag, owner, clock),
		device_isa8_card_interface(mconfig, *this),
		m_uart(*this, "uart"),
		m_esp(*this, "esp")
	{
	}

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;

private:
	void out1_w(int state);

	required_device<ns16550_device> m_uart;
	required_device<rs232_port_device> m_esp;
};


void isa8_sprinter_esp_device::device_add_mconfig(machine_config &config)
{
	NS16550(config, m_uart, XTAL(14'745'600));
	m_uart->out_tx_callback().set(m_esp, FUNC(rs232_port_device::write_txd));
	m_uart->out_dtr_callback().set(m_esp, FUNC(rs232_port_device::write_dtr));
	m_uart->out_rts_callback().set(m_esp, FUNC(rs232_port_device::write_rts));
	m_uart->out_out1_callback().set(FUNC(isa8_sprinter_esp_device::out1_w));

	RS232_PORT(config, m_esp, sprinter_esp_rs232_devices, "esp8266_at");
	m_esp->set_option_device_input_defaults("null_modem", DEVICE_INPUT_DEFAULTS_NAME(sprinter_esp_rs232_defaults));
	m_esp->set_option_device_input_defaults("terminal", DEVICE_INPUT_DEFAULTS_NAME(sprinter_esp_rs232_defaults));
	m_esp->rxd_handler().set(m_uart, FUNC(ins8250_uart_device::rx_w));
	m_esp->dcd_handler().set(m_uart, FUNC(ins8250_uart_device::dcd_w));
	m_esp->dsr_handler().set(m_uart, FUNC(ins8250_uart_device::dsr_w));
	m_esp->ri_handler().set(m_uart, FUNC(ins8250_uart_device::ri_w));
	m_esp->cts_handler().set(m_uart, FUNC(ins8250_uart_device::cts_w));
}


// -OUT1 low (MCR bit 2 set) holds the module in reset.
void isa8_sprinter_esp_device::out1_w(int state)
{
	device_rs232_port_interface *const module = m_esp->get_card_device();
	device_esp8266_rst_interface *rst = nullptr;
	if (module && module->device().interface(rst))
		rst->rst_w(state ? CLEAR_LINE : ASSERT_LINE);
}


void isa8_sprinter_esp_device::device_start()
{
	set_isa_device();
	m_isa->install_device(0x03e8, 0x03ef,
			read8sm_delegate(*m_uart, FUNC(ins8250_device::ins8250_r)),
			write8sm_delegate(*m_uart, FUNC(ins8250_device::ins8250_w)));
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(ISA8_SPRINTER_ESP, device_isa8_card_interface, isa8_sprinter_esp_device, "sprinter_esp", "Sprinter-WiFi ISA card (TL16C550C + ESP8266)")
