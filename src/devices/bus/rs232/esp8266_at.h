// license:BSD-3-Clause
// copyright-holders:D. Rimron-Soutter
#ifndef MAME_BUS_RS232_ESP8266_AT_H
#define MAME_BUS_RS232_ESP8266_AT_H

#pragma once

#include "rs232.h"


// The module's RST pin, which RS-232 lacks; a host finds it on the slot card.
class device_esp8266_rst_interface : public device_interface
{
public:
	virtual void rst_w(int state) = 0;  // ASSERT_LINE holds the module in reset

protected:
	device_esp8266_rst_interface(machine_config const &mconfig, device_t &device) : device_interface(device, "esp8266_rst") { }
};


DECLARE_DEVICE_TYPE(ESP8266_AT, device_rs232_port_interface)

#endif // MAME_BUS_RS232_ESP8266_AT_H
