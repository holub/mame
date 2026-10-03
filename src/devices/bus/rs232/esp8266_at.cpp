// license:BSD-3-Clause
// copyright-holders:D. Rimron-Soutter
/***************************************************************************

    ESP8266 module running Espressif's AT 0.21.0.0, 0.40.0.0, 1.1.0.0,
    1.3.0.0, 2.1.0.0-dev or 2.2.0.0 firmware

    High-level emulation of the AT command interface.  Links are TCP and
    UDP sockets on the host; there is no WiFi radio.

***************************************************************************/

#include "emu.h"
#include "esp8266_at.h"

#include "multibyte.h"

#include "asio.h"

// asio brings the Windows headers back in after diserial.h
#undef PARITY_NONE
#undef PARITY_ODD
#undef PARITY_EVEN

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#define LOG_CMD     (1U << 1)
#define LOG_REPLY   (1U << 2)
#define LOG_LINK    (1U << 3)

#define VERBOSE (0)
#include "logmacro.h"

#define LOGCMD(...)     LOGMASKED(LOG_CMD, __VA_ARGS__)
#define LOGREPLY(...)   LOGMASKED(LOG_REPLY, __VA_ARGS__)
#define LOGLINK(...)    LOGMASKED(LOG_LINK, __VA_ARGS__)


namespace {

constexpr unsigned LINK_COUNT = 5;
constexpr unsigned MAX_SEND = 2048;
constexpr unsigned MAX_LINE = 128;
constexpr unsigned OUT_HIGH_WATER = 1024;
constexpr unsigned CONNECT_TIMEOUT_MS = 10'000;
constexpr unsigned RECONNECT_DELAY_MS = 100;
constexpr unsigned RX_BATCH_GAP_MS = 20;
constexpr unsigned RX_TOUT_BITS = 20;
constexpr u16 DEFAULT_SERVER_PORT = 333;
constexpr u16 DEFAULT_SERVER_TIMEOUT = 180;
constexpr unsigned BOOT_DELAY_MS = 300;

constexpr unsigned ESP_TCP_MSS = 1460;
constexpr unsigned ESP_TCP_TMR_MS = 125;
constexpr unsigned TCP_FIN_WAIT_MS = 20'000;
constexpr unsigned UART_FIFO = 128;
constexpr unsigned PASSTHROUGH_ACK_MS = 28;

constexpr unsigned SERVER_PORT_OFFSETS[] = { 0, 1'000, 10'000, 20'000, 30'000, 40'000, 0, 0 };

constexpr char const STATION_MAC[] = "1a:fe:34:00:00:01";

constexpr char const DEFAULT_SSID[] = "MAME";
constexpr char const AP_BSSID[] = "02:00:00:00:00:01";
constexpr int AP_ECN = 3;
constexpr int AP_RSSI = -40;
constexpr int AP_CHANNEL = 1;
constexpr unsigned MAX_SSID = 32;

constexpr unsigned CWJAP_TICK_MS = 3'000;
constexpr unsigned PING_COARSE_MS = 1'000;
constexpr unsigned PING_NO_DNS_MS = 6'770;
constexpr unsigned DNS_NO_IP_MS = 6'550;
constexpr unsigned CWLAP_SCAN_MS = 2'150;

constexpr u32 CARD_MAX_BAUD = 2'000'000;
constexpr u16 SSL_SIZE_DEFAULT = 2048;
constexpr u32 IPADDR_NONE = 0xffff'ffff;
constexpr unsigned TCP_SND_BUF = 2 * ESP_TCP_MSS;
constexpr unsigned TCP_SND_QUEUELEN = ((4 * TCP_SND_BUF) + (ESP_TCP_MSS - 1)) / ESP_TCP_MSS;
constexpr unsigned MAX_TRANSLINK_HOST = 63;

constexpr u8 STATUS_GOT_IP = 2;
constexpr u8 STATUS_CONNECTED = 3;
constexpr u8 STATUS_DISCONNECTED = 4;
constexpr u8 STATUS_NO_WIFI = 5;

constexpr unsigned NV_BLOCK = 130;
constexpr unsigned NV_EXTRA = 8;

constexpr unsigned RTOS_RX_TOUT_BITS = 100;
constexpr unsigned RTOS_RX_EVENT = 120;
constexpr unsigned RTOS_LINE_MAX = 256;
constexpr unsigned RTOS_TX_RING = 2048;
constexpr unsigned RTOS_IPD_MAX = 2'920;
constexpr unsigned RTOS_UDP_MBOX = 6;
constexpr unsigned RTOS_RECONNECT_MS = 100;
constexpr unsigned RTOS_PING_MS = 1'000;
constexpr unsigned RTOS_UART_CLOCK = 80'000'000;
constexpr int RTOS_HEAP_FREE = 49'216;
constexpr int RTOS_HEAP_LINK = 7'132;
constexpr int RTOS_HEAP_MORE_LINK = 380;
constexpr int RTOS_HEAP_SERVER = 1'396;
constexpr unsigned RTOS_USERRAM_CHUNK = 1'024;
constexpr unsigned MAX_PASSWORD = 64;
constexpr unsigned NV_RTOS2 = 81;

struct esp8266_firmware
{
	char const *boot_text;
	char const *gmr_text;
	char const *const *absent;
	unsigned tcp_wnd;
	unsigned tx_ring;
	unsigned block_free;
	unsigned recover_free;
	unsigned nv_offset;
	bool cwlap_cal;
	bool status_tcp_local;
	bool bufstatus_id;
	bool translink_names;
	bool drop_tcp_any_mode;
	bool udp_restore;
	bool keepalive_empty = false;
	unsigned passthrough_packet = 2'920;
	unsigned reconnect_step_us = 0;
	char const *link_type_error = "Link type ERROR\r\n";
	char const *unlink_text = "UNLINK\r\n";
	bool cwlap_five = false;
	bool quiet_wifi = false;
	bool status_commands = false;
	bool ap_ssid_only = false;
	bool cipsta_ip_only = false;
	bool cipstart_checks = false;
	bool udp_connect_id = false;
	bool send_checks = false;
	bool send_echo = false;
	bool send_busy_only = false;
	bool udp_ipd_ok = false;
	bool cipmode_stored = false;
	bool cipappup = false;
	bool close_waits = false;
	bool ring_passthrough_only = false;
	bool uart_stall = false;
	bool cipsto_fault = false;
	bool rtos = false;
	u16 server_timeout = DEFAULT_SERVER_TIMEOUT;
	unsigned nv_size = NV_BLOCK;
	unsigned scan_ms = CWLAP_SCAN_MS;
	unsigned rx_event = RTOS_RX_EVENT;
	bool cmd_list = false;
	bool sysstore = false;
	bool strict_params = false;
	bool sysram_low = false;
	bool sysmsg_bit2 = false;
	bool sleep_modes = false;
	bool uart_odd = false;
	u8 uart_flow = 0;
	bool cwmode_autoconn = false;
	bool cwjap_nine = false;
	bool cwlap_eleven = false;
	bool cipmux_clients = false;
	bool port_nonzero = false;
	bool sendex_escape = false;
	bool cipserver_port = false;
	bool sto_reset = false;
	bool recvmode_wake = false;
	bool udp_recvdata = false;
	bool reconnintv_wide = false;
	bool domain_quoted = false;
	bool ping_wait = false;
	bool translink_keepalive = false;
	bool status_closed = false;
};

constexpr char const *const AT021_ABSENT[] = {
		"SLEEP", "UART_CUR", "UART_DEF", "CWMODE_CUR", "CWMODE_DEF", "CWJAP_CUR", "CWJAP_DEF", "CWDHCP_CUR", "CWDHCP_DEF",
		"CIPSTA_CUR", "CIPSTA_DEF", "CIPSENDEX", "CIPDINFO", "CIPALIVE", "CIPBUFRESET", "CIPSENDBUF", "CIPCHECKSEQ",
		"CIPCHECKQUEUE", "CIPBUFSTATUS", "SAVETRANSLINK", "CIPSSLSIZE", "CIPDOMAIN", nullptr };
constexpr char const *const AT040_ABSENT[] = { "CIPSSLSIZE", "CIPDOMAIN", nullptr };
constexpr char const *const AT11_ABSENT[] = { nullptr };
constexpr char const *const AT13_ABSENT[] = { "CIPSTART=?", nullptr };
constexpr char const *const AT21_ABSENT[] = { nullptr };

constexpr esp8266_firmware FIRMWARE_021 = {
		.boot_text = "\r\nStale Pixels ESP8266 AT emulation, AT 0.21.0.0\r\n",
		.gmr_text =
				"AT version:0.21.0.0\r\n"
				"SDK version:0.9.5\r\n"
				"\r\n",
		.absent = AT021_ABSENT,
		.tcp_wnd = 4 * ESP_TCP_MSS,
		.tx_ring = 8'192,
		.block_free = 6'144,
		.recover_free = 7'999,
		.nv_offset = 2 * NV_BLOCK,
		.cwlap_cal = false,
		.status_tcp_local = false,
		.bufstatus_id = false,
		.translink_names = false,
		.drop_tcp_any_mode = false,
		.udp_restore = true,
		.passthrough_packet = MAX_SEND,
		.reconnect_step_us = 10'000,
		.link_type_error = "Link typ ERROR\r\n",
		.unlink_text = "link is not\r\n",
		.cwlap_five = true,
		.quiet_wifi = true,
		.status_commands = true,
		.ap_ssid_only = true,
		.cipsta_ip_only = true,
		.cipstart_checks = true,
		.udp_connect_id = true,
		.send_checks = true,
		.send_echo = true,
		.send_busy_only = true,
		.udp_ipd_ok = true,
		.cipmode_stored = true,
		.cipappup = true,
		.close_waits = true,
		.ring_passthrough_only = true,
		.uart_stall = true,
		.cipsto_fault = true };

constexpr esp8266_firmware FIRMWARE_040 = {
		"\r\nStale Pixels ESP8266 AT emulation, AT 0.40.0.0\r\n",
		"AT version:0.40.0.0(Aug  8 2015 14:45:58)\r\n"
		"SDK version:1.3.0\r\n"
		"compile time:Aug  8 2015 17:35:24\r\n",
		AT040_ABSENT,
		4 * ESP_TCP_MSS, 8'192, 7'168, 7'999, NV_BLOCK,
		false, false, true, false, true, true, false };

constexpr esp8266_firmware FIRMWARE_11 = {
		"\r\nStale Pixels ESP8266 AT emulation, AT 1.1.0.0\r\n",
		"AT version:1.1.0.0(May 11 2016 18:09:56)\r\n"
		"SDK version:1.5.4(baaeaebb)\r\n"
		"compile time:May 20 2016 15:08:19\r\n",
		AT11_ABSENT,
		2 * ESP_TCP_MSS, (2 * ESP_TCP_MSS) + 100, 0, 0, 0,
		true, true, false, true, false, false, true };

constexpr esp8266_firmware FIRMWARE_13 = {
		.boot_text = "\r\nStale Pixels ESP8266 AT emulation, AT 1.3.0.0\r\n",
		.gmr_text =
				"AT version:1.3.0.0(Jul 14 2016 18:54:01)\r\n"
				"SDK version:2.0.0(656edbf)\r\n"
				"compile time:Jul 19 2016 18:44:44\r\n",
		.absent = AT13_ABSENT,
		.tcp_wnd = 2 * ESP_TCP_MSS,
		.tx_ring = (2 * ESP_TCP_MSS) + 100,
		.block_free = 0,
		.recover_free = 0,
		.nv_offset = 0,
		.cwlap_cal = true,
		.status_tcp_local = true,
		.bufstatus_id = false,
		.translink_names = true,
		.drop_tcp_any_mode = false,
		.udp_restore = false,
		.keepalive_empty = true };

constexpr esp8266_firmware FIRMWARE_21 = {
		.boot_text = "\r\nStale Pixels ESP8266 AT emulation, AT 2.1.0.0-dev\r\n",
		.gmr_text =
				"AT version:2.1.0.0-dev(044790d - Dec 30 2019 10:14:44)\r\n"
				"SDK version:v3.3-rc1-4-g0a187af8\r\n"
				"compile time(d03b392):Jan  2 2020 22:44:10\r\n"
				"Bin version:2.0.0(WROOM-02)\r\n",
		.absent = AT21_ABSENT,
		.tcp_wnd = 4 * ESP_TCP_MSS,
		.tx_ring = RTOS_TX_RING,
		.block_free = 0,
		.recover_free = 0,
		.nv_offset = 3 * NV_BLOCK,
		.cwlap_cal = false,
		.status_tcp_local = true,
		.bufstatus_id = false,
		.translink_names = true,
		.drop_tcp_any_mode = false,
		.udp_restore = false,
		.passthrough_packet = ESP_TCP_MSS,
		.cwlap_five = true,
		.rtos = true,
		.server_timeout = 0,
		.nv_size = NV_BLOCK + NV_EXTRA,
		.scan_ms = 2'050 };

constexpr esp8266_firmware FIRMWARE_22 = {
		.boot_text = "\r\nStale Pixels ESP8266 AT emulation, AT 2.2.0.0\r\n",
		.gmr_text =
				"AT version:2.2.0.0(s-b097cdf - ESP8266 - Jun 17 2021 12:58:29)\r\n"
				"SDK version:v3.4\r\n"
				"compile time(6800286):Jul 14 2021 22:00:36\r\n"
				"Bin version:2.2.0(ESP8266_1MB)\r\n",
		.absent = AT21_ABSENT,
		.tcp_wnd = 4 * 1'440,
		.tx_ring = RTOS_TX_RING,
		.block_free = 0,
		.recover_free = 0,
		.nv_offset = (4 * NV_BLOCK) + NV_EXTRA,
		.cwlap_cal = false,
		.status_tcp_local = true,
		.bufstatus_id = false,
		.translink_names = true,
		.drop_tcp_any_mode = false,
		.udp_restore = false,
		.passthrough_packet = ESP_TCP_MSS,
		.rtos = true,
		.server_timeout = 0,
		.nv_size = NV_BLOCK + NV_RTOS2,
		.scan_ms = 2'050,
		.rx_event = 100,
		.cmd_list = true,
		.sysstore = true,
		.strict_params = true,
		.sysram_low = true,
		.sysmsg_bit2 = true,
		.sleep_modes = true,
		.uart_odd = true,
		.uart_flow = 1,
		.cwmode_autoconn = true,
		.cwjap_nine = true,
		.cwlap_eleven = true,
		.cipmux_clients = true,
		.port_nonzero = true,
		.sendex_escape = true,
		.cipserver_port = true,
		.sto_reset = true,
		.recvmode_wake = true,
		.udp_recvdata = true,
		.reconnintv_wide = true,
		.domain_quoted = true,
		.ping_wait = true,
		.translink_keepalive = true,
		.status_closed = true };


class esp8266_net
{
public:
	enum class event_type : u8
	{
		CONNECTED,
		CONNECT_FAILED,
		DNS_FAILED,
		DATA,
		SENT,
		SEND_FAILED,
		CLOSED,
		LISTENING,
		LISTEN_FAILED,
		ACCEPTED,
		PING_SENT,
		PING_FAILED,
		PING_REPLY,
		RESOLVED
	};

	static constexpr unsigned SERVER = LINK_COUNT;
	static constexpr unsigned PING = LINK_COUNT + 1;
	static constexpr unsigned LOOKUP = LINK_COUNT + 2;

	struct event
	{
		event_type type;
		unsigned link;
		u32 generation;
		std::vector<u8> data;
		std::string peer;
		u32 ticket = 0;
	};

	struct open_params
	{
		bool udp = false;
		std::string host;
		u16 port = 0;
		u16 local_port = 0;
		u8 udp_mode = 0;
		u16 keepalive = 0;
	};

	esp8266_net()
		: m_work(asio::make_work_guard(m_ioctx))
		, m_acceptor(m_ioctx)
		, m_server_generation(0)
		, m_next_ticket(0)
		, m_ping_resolver(m_ioctx)
		, m_icmp(m_ioctx)
		, m_ping_generation(0)
		, m_ping_seq(0)
		, m_dns_resolver(m_ioctx)
		, m_dns_generation(0)
		, m_pending(false)
	{
		for (auto &l : m_links)
			l = std::make_unique<net_link>(m_ioctx);
		m_thread = std::thread([this] () { m_ioctx.run(); });
	}

	~esp8266_net()
	{
		m_work.reset();
		m_ioctx.stop();
		m_thread.join();
	}


	void open(unsigned link, u32 generation, open_params const &params, unsigned delay_ms)
	{
		asio::post(m_ioctx, [this, link, generation, params, delay_ms] () { do_open(link, generation, params, delay_ms); });
	}

	void close(unsigned link, u32 generation)
	{
		asio::post(
				m_ioctx,
				[this, link, generation] ()
				{
					net_link &l = *m_links[link];
					close_sockets(l);
					l.generation = generation;
				});
	}

	void send(unsigned link, u32 generation, std::vector<u8> &&data, bool report)
	{
		auto buf = std::make_shared<std::vector<u8> >(std::move(data));
		asio::post(
				m_ioctx,
				[this, link, generation, buf = std::move(buf), report] ()
				{
					net_link &l = *m_links[link];
					if ((l.generation != generation) || !l.connected)
					{
						if (report)
							post_event(event_type::SEND_FAILED, link, generation);
						return;
					}
					l.send_queue.emplace_back(buf, report);
					if (!l.writing)
						write_next(link, generation);
				});
	}

	void udp_remote(unsigned link, u32 generation, std::string const &address, u16 port)
	{
		asio::post(
				m_ioctx,
				[this, link, generation, address, port] ()
				{
					net_link &l = *m_links[link];
					std::error_code err;
					asio::ip::address const ip = asio::ip::make_address(address, err);
					if ((l.generation != generation) || !l.is_udp || !l.connected || err)
						return;
					l.remote = asio::ip::udp::endpoint(ip, port);
					if (!l.udp_mode)
						l.udp.connect(l.remote, err);
				});
	}

	void resume(unsigned link, u32 generation)
	{
		asio::post(
				m_ioctx,
				[this, link, generation] ()
				{
					net_link const &l = *m_links[link];
					if ((l.generation == generation) && l.connected && !l.reading)
						start_read(link, generation);
				});
	}

	void shutdown(unsigned link, u32 generation)
	{
		asio::post(
				m_ioctx,
				[this, link, generation] ()
				{
					net_link &l = *m_links[link];
					if ((l.generation != generation) || !l.connected || l.is_udp)
						return;
					std::error_code err;
					l.tcp.shutdown(asio::ip::tcp::socket::shutdown_send, err);
					l.closing = true;
					if (!l.reading)
						start_read(link, generation);
				});
	}

	void sockopt(unsigned link, u32 generation, s32 linger, bool nodelay)
	{
		asio::post(
				m_ioctx,
				[this, link, generation, linger, nodelay] ()
				{
					net_link &l = *m_links[link];
					if ((l.generation != generation) || !l.connected || l.is_udp)
						return;
					std::error_code err;
					l.tcp.set_option(asio::socket_base::linger(linger >= 0, std::max<s32>(linger, 0)), err);
					l.tcp.set_option(asio::ip::tcp::no_delay(nodelay), err);
				});
	}

	void listen(u32 generation, bool any, unsigned port)
	{
		asio::post(m_ioctx, [this, generation, any, port] () { do_listen(generation, any, port); });
	}

	void stop_listen(u32 generation)
	{
		asio::post(
				m_ioctx,
				[this, generation] ()
				{
					close_acceptor();
					m_server_generation = generation;
				});
	}

	void adopt(unsigned link, u32 generation, u32 ticket)
	{
		asio::post(
				m_ioctx,
				[this, link, generation, ticket] ()
				{
					net_link &l = *m_links[link];
					close_sockets(l);
					l.generation = generation;
					l.is_udp = false;
					auto const found = m_accepted.find(ticket);
					if (found == m_accepted.end())
					{
						post_event(event_type::CLOSED, link, generation);
						return;
					}
					l.tcp = std::move(found->second);
					m_accepted.erase(found);
					std::error_code err;
					l.peer = l.tcp.remote_endpoint(err);
					l.connected = true;
					start_read(link, generation);
				});
	}

	void reject(u32 ticket)
	{
		asio::post(m_ioctx, [this, ticket] () { m_accepted.erase(ticket); });
	}

	void ping(u32 generation, std::string const &host)
	{
		asio::post(m_ioctx, [this, generation, host] () { do_ping(generation, host); });
	}

	void stop_ping(u32 generation)
	{
		asio::post(
				m_ioctx,
				[this, generation] ()
				{
					close_ping();
					m_ping_generation = generation;
				});
	}

	void resolve(u32 generation, std::string const &host)
	{
		asio::post(
				m_ioctx,
				[this, generation, host] ()
				{
					m_dns_resolver.cancel();
					m_dns_generation = generation;
					m_dns_resolver.async_resolve(
							asio::ip::udp::v4(),
							host,
							std::string(),
							[this, generation] (std::error_code const &err, asio::ip::udp::resolver::results_type results)
							{
								if (generation != m_dns_generation)
									return;
								std::string address;
								if (!err && !results.empty())
									address = results.begin()->endpoint().address().to_string();
								post_event(event_type::RESOLVED, LOOKUP, generation, std::vector<u8>(), std::move(address));
							});
				});
	}

	static std::string primary_ipv4()
	{
		asio::io_context ctx;
		asio::ip::udp::socket socket(ctx);
		std::error_code err;
		socket.open(asio::ip::udp::v4(), err);
		if (!err)
			socket.connect(asio::ip::udp::endpoint(asio::ip::make_address_v4("192.0.2.1"), 9), err);
		asio::ip::udp::endpoint const local = err ? asio::ip::udp::endpoint() : socket.local_endpoint(err);
		if (err || local.address().is_unspecified())
			return asio::ip::address_v4::loopback().to_string();
		return local.address().to_string();
	}

	bool fetch(std::deque<event> &out)
	{
		if (!m_pending.load(std::memory_order_acquire))
			return false;
		std::lock_guard<std::mutex> lock(m_mutex);
		for (auto &ev : m_events)
			out.emplace_back(std::move(ev));
		m_events.clear();
		m_pending.store(false, std::memory_order_release);
		return true;
	}

private:
	struct net_link
	{
		net_link(asio::io_context &ctx) : tcp(ctx), udp(ctx), resolver(ctx), timer(ctx) { }

		asio::ip::tcp::socket tcp;
		asio::ip::udp::socket udp;
		asio::ip::tcp::resolver resolver;
		asio::steady_timer timer;
		u32 generation = 0;
		bool is_udp = false;
		bool connected = false;
		bool writing = false;
		bool reading = false;
		bool closing = false;
		u8 udp_mode = 0;
		bool udp_peer_changed = false;
		asio::ip::tcp::endpoint peer;
		asio::ip::udp::endpoint remote;
		asio::ip::udp::endpoint from;
		std::deque<std::pair<std::shared_ptr<std::vector<u8> >, bool> > send_queue;
		std::array<u8, MAX_SEND> buffer;
	};

	bool current(unsigned link, u32 generation) const { return m_links[link]->generation == generation; }

	void post_event(event_type type, unsigned link, u32 generation, std::vector<u8> &&data = std::vector<u8>(), std::string &&peer = std::string(), u32 ticket = 0)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_events.emplace_back(event{ type, link, generation, std::move(data), std::move(peer), ticket });
		m_pending.store(true, std::memory_order_release);
	}

	static std::string peer_text(asio::ip::address const &address, u16 port)
	{
		std::string text;
		if (address.is_v6() && address.to_v6().is_v4_mapped())
			text = asio::ip::make_address_v4(asio::ip::v4_mapped, address.to_v6()).to_string();
		else
			text = address.to_string();
		return text + ',' + std::to_string(port);
	}

	void close_ping()
	{
		std::error_code err;
		m_ping_resolver.cancel();
		if (m_icmp.is_open())
			m_icmp.close(err);
	}

	void do_ping(u32 generation, std::string const &host)
	{
		close_ping();
		m_ping_generation = generation;
		m_ping_resolver.async_resolve(
				asio::ip::udp::v4(),
				host,
				std::string(),
				[this, generation] (std::error_code const &err, asio::ip::udp::resolver::results_type results)
				{
					if (generation != m_ping_generation)
						return;
					if (err || results.empty())
						post_event(event_type::PING_FAILED, PING, generation);
					else
						send_ping(generation, results.begin()->endpoint().address());
				});
	}

	void send_ping(u32 generation, asio::ip::address const &address)
	{
		std::error_code err;
		asio::ip::icmp const icmp = asio::ip::icmp::v4();
		m_icmp.open(asio::generic::datagram_protocol(icmp.family(), icmp.protocol()), err);

		m_ping_seq = (m_ping_seq % 0x7ffe) + 1;
		std::array<u8, 8 + 32> packet;
		packet[0] = 8;
		packet[1] = 0;
		put_u16be(&packet[2], 0);
		put_u16be(&packet[4], 0xafaf);
		put_u16be(&packet[6], m_ping_seq);
		for (unsigned i = 0; i < 32; i++)
			packet[8 + i] = u8(i);
		u32 sum = 0;
		for (unsigned i = 0; i < packet.size(); i += 2)
			sum += get_u16be(&packet[i]);
		while (sum >> 16)
			sum = (sum & 0xffff) + (sum >> 16);
		put_u16be(&packet[2], u16(~sum));

		if (!err)
			m_icmp.send_to(asio::buffer(packet), asio::generic::datagram_protocol::endpoint(asio::ip::icmp::endpoint(address, 0)), 0, err);
		m_ping_sent = std::chrono::steady_clock::now();
		post_event(event_type::PING_SENT, PING, generation, std::vector<u8>(), err ? err.message() : std::string());
		if (!err)
			start_ping_read(generation);
	}

	void start_ping_read(u32 generation)
	{
		m_icmp.async_receive(
				asio::buffer(m_ping_buffer),
				[this, generation] (std::error_code const &err, std::size_t length)
				{
					if ((generation != m_ping_generation) || err)
						return;

					u8 const *reply = m_ping_buffer.data();
					if ((length >= 20) && (BIT(reply[0], 4, 4) == 4) && (length >= (BIT(reply[0], 0, 4) * 4U)))
					{
						unsigned const header = BIT(reply[0], 0, 4) * 4;
						reply += header;
						length -= header;
					}

					if ((length >= 8) && (reply[0] == 0) && (get_u16be(&reply[6]) == m_ping_seq))
					{
						auto const elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - m_ping_sent);
						post_event(event_type::PING_REPLY, PING, generation, std::vector<u8>(), std::string(), u32(elapsed.count()));
						close_ping();
						return;
					}
					start_ping_read(generation);
				});
	}

	void close_acceptor()
	{
		std::error_code err;
		if (m_acceptor.is_open())
			m_acceptor.close(err);
		m_accepted.clear();
	}

	void do_listen(u32 generation, bool any, unsigned port)
	{
		close_acceptor();
		m_server_generation = generation;

		std::error_code err;
		if (port > 0xffff)
		{
			post_event(event_type::LISTEN_FAILED, SERVER, generation, std::vector<u8>(), "port out of range");
			return;
		}
		asio::ip::tcp::endpoint const endpoint(any ? asio::ip::address_v4::any() : asio::ip::address_v4::loopback(), asio::ip::port_type(port));
		m_acceptor.open(endpoint.protocol(), err);
		if (!err)
			m_acceptor.set_option(asio::ip::tcp::acceptor::reuse_address(true), err);
		if (!err)
			m_acceptor.bind(endpoint, err);
		if (!err)
			m_acceptor.listen(LINK_COUNT, err);
		if (err)
		{
			close_acceptor();
			post_event(event_type::LISTEN_FAILED, SERVER, generation, std::vector<u8>(), endpoint.address().to_string() + ':' + std::to_string(port) + ": " + err.message());
			return;
		}
		post_event(event_type::LISTENING, SERVER, generation, std::vector<u8>(), endpoint.address().to_string() + ':' + std::to_string(port));
		start_accept(generation);
	}

	void start_accept(u32 generation)
	{
		m_acceptor.async_accept(
				[this, generation] (std::error_code const &err, asio::ip::tcp::socket socket)
				{
					if ((generation != m_server_generation) || !m_acceptor.is_open())
						return;
					if (err && (err != asio::error::connection_aborted))
						return;
					if (!err)
					{
						std::error_code e;
						socket.set_option(asio::ip::tcp::no_delay(true), e);
						asio::ip::tcp::endpoint const remote = socket.remote_endpoint(e);
						u32 const ticket = ++m_next_ticket;
						m_accepted.emplace(ticket, std::move(socket));
						post_event(event_type::ACCEPTED, SERVER, generation, std::vector<u8>(), peer_text(remote.address(), remote.port()), ticket);
					}
					start_accept(generation);
				});
	}

	void close_sockets(net_link &l)
	{
		std::error_code err;
		l.resolver.cancel();
		l.timer.cancel();
		if (l.tcp.is_open())
			l.tcp.close(err);
		if (l.udp.is_open())
			l.udp.close(err);
		l.connected = false;
		l.writing = false;
		l.reading = false;
		l.closing = false;
		l.send_queue.clear();
	}

	void do_open(unsigned link, u32 generation, open_params const &params, unsigned delay_ms)
	{
		net_link &l = *m_links[link];
		close_sockets(l);
		l.generation = generation;
		l.is_udp = params.udp;
		l.udp_mode = params.udp_mode;
		l.udp_peer_changed = false;

		if (delay_ms)
		{
			l.timer.expires_after(std::chrono::milliseconds(delay_ms));
			l.timer.async_wait(
					[this, link, generation, params] (std::error_code const &err)
					{
						if (!err && current(link, generation))
							start_resolve(link, generation, params);
					});
		}
		else
		{
			start_resolve(link, generation, params);
		}
	}

	void start_resolve(unsigned link, u32 generation, open_params const &params)
	{
		m_links[link]->resolver.async_resolve(
				params.host,
				std::to_string(params.port),
				[this, link, generation, params] (std::error_code const &err, asio::ip::tcp::resolver::results_type results)
				{
					if (!current(link, generation))
						return;
					if (err || results.empty())
						post_event(event_type::DNS_FAILED, link, generation);
					else if (params.udp)
						start_udp(link, generation, params, results);
					else
						start_tcp(link, generation, params, results);
				});
	}

	void start_tcp(unsigned link, u32 generation, open_params const &params, asio::ip::tcp::resolver::results_type const &results)
	{
		net_link &l = *m_links[link];
		l.timer.expires_after(std::chrono::milliseconds(CONNECT_TIMEOUT_MS));
		l.timer.async_wait(
				[this, link, generation] (std::error_code const &err)
				{
					net_link &l = *m_links[link];
					if (!err && current(link, generation) && !l.connected)
					{
						std::error_code e;
						l.tcp.close(e);
					}
				});
		asio::async_connect(
				l.tcp,
				results,
				[this, link, generation, keepalive = params.keepalive] (std::error_code const &err, asio::ip::tcp::endpoint const &endpoint)
				{
					if (!current(link, generation))
						return;
					net_link &l = *m_links[link];
					l.timer.cancel();
					if (err)
					{
						close_sockets(l);
						post_event(event_type::CONNECT_FAILED, link, generation);
						return;
					}
					std::error_code e;
					l.peer = endpoint;
					l.tcp.set_option(asio::ip::tcp::no_delay(true), e);
					if (keepalive)
						l.tcp.set_option(asio::socket_base::keep_alive(true), e);
					l.connected = true;
					u16 const local = l.tcp.local_endpoint(e).port();
					post_event(event_type::CONNECTED, link, generation, std::vector<u8>(), peer_text(endpoint.address(), endpoint.port()), local);
					start_read(link, generation);
				});
	}

	void start_udp(unsigned link, u32 generation, open_params const &params, asio::ip::tcp::resolver::results_type const &results)
	{
		net_link &l = *m_links[link];

		asio::ip::address address = results.begin()->endpoint().address();
		for (auto const &entry : results)
		{
			if (entry.endpoint().address().is_v4())
			{
				address = entry.endpoint().address();
				break;
			}
		}
		l.remote = asio::ip::udp::endpoint(address, params.port);

		std::error_code err;
		l.udp.open(l.remote.protocol(), err);
		if (!err)
			l.udp.bind(asio::ip::udp::endpoint(l.remote.protocol(), params.local_port), err);
		if (!err && !params.udp_mode)
			l.udp.connect(l.remote, err);
		if (err)
		{
			close_sockets(l);
			post_event(event_type::CONNECT_FAILED, link, generation);
			return;
		}
		l.connected = true;
		u16 const local = l.udp.local_endpoint(err).port();
		post_event(event_type::CONNECTED, link, generation, std::vector<u8>(), peer_text(l.remote.address(), l.remote.port()), local);
		start_read(link, generation);
	}

	void start_read(unsigned link, u32 generation)
	{
		net_link &l = *m_links[link];
		l.reading = true;
		auto const handler =
				[this, link, generation] (std::error_code const &err, std::size_t length)
				{
					if (!current(link, generation))
						return;
					net_link &l = *m_links[link];
					l.reading = false;
					if (l.closing)
					{
						close_sockets(l);
						post_event(event_type::CLOSED, link, generation);
						return;
					}
					if (l.is_udp && (err == asio::error::connection_refused))
					{
						start_read(link, generation);
						return;
					}
					if (err || (!l.is_udp && !length))
					{
						close_sockets(l);
						post_event(event_type::CLOSED, link, generation);
						return;
					}
					if (l.is_udp && l.udp_mode && (l.from != l.remote) && ((l.udp_mode == 2) || !l.udp_peer_changed))
					{
						l.remote = l.from;
						l.udp_peer_changed = true;
					}
					std::string peer;
					if (!l.is_udp)
						peer = peer_text(l.peer.address(), l.peer.port());
					else if (l.udp_mode)
						peer = peer_text(l.from.address(), l.from.port());
					else
						peer = peer_text(l.remote.address(), l.remote.port());
					post_event(event_type::DATA, link, generation, std::vector<u8>(l.buffer.begin(), l.buffer.begin() + length), std::move(peer));
				};

		if (!l.is_udp)
			l.tcp.async_read_some(asio::buffer(l.buffer), handler);
		else if (!l.udp_mode)
			l.udp.async_receive(asio::buffer(l.buffer), handler);
		else
			l.udp.async_receive_from(asio::buffer(l.buffer), l.from, handler);
	}

	void write_next(unsigned link, u32 generation)
	{
		net_link &l = *m_links[link];
		if (l.send_queue.empty())
		{
			l.writing = false;
			return;
		}

		std::shared_ptr<std::vector<u8> > const buf = std::move(l.send_queue.front().first);
		bool const report = l.send_queue.front().second;
		l.send_queue.pop_front();
		l.writing = true;
		auto handler =
				[this, link, generation, buf, report] (std::error_code const &err, std::size_t length)
				{
					if (!current(link, generation))
						return;
					net_link &l = *m_links[link];
					l.writing = false;
					if (err)
					{
						if (report)
							post_event(event_type::SEND_FAILED, link, generation);
						if (!l.is_udp)
						{
							std::error_code e;
							l.tcp.close(e);
						}
						return;
					}
					if (report)
						post_event(event_type::SENT, link, generation);
					write_next(link, generation);
				};

		if (!l.is_udp)
			asio::async_write(l.tcp, asio::buffer(*buf), std::move(handler));
		else if (!l.udp_mode)
			l.udp.async_send(asio::buffer(*buf), std::move(handler));
		else
			l.udp.async_send_to(asio::buffer(*buf), l.remote, std::move(handler));
	}

	asio::io_context m_ioctx;
	asio::executor_work_guard<asio::io_context::executor_type> m_work;
	std::unique_ptr<net_link> m_links[LINK_COUNT];
	asio::ip::tcp::acceptor m_acceptor;
	u32 m_server_generation;
	u32 m_next_ticket;
	std::map<u32, asio::ip::tcp::socket> m_accepted;
	asio::ip::udp::resolver m_ping_resolver;
	asio::basic_datagram_socket<asio::generic::datagram_protocol> m_icmp;
	u32 m_ping_generation;
	u16 m_ping_seq;
	std::chrono::steady_clock::time_point m_ping_sent;
	std::array<u8, 256> m_ping_buffer;
	asio::ip::udp::resolver m_dns_resolver;
	u32 m_dns_generation;
	std::thread m_thread;
	std::mutex m_mutex;
	std::vector<event> m_events;
	std::atomic<bool> m_pending;
};


class esp8266_at_device : public buffered_rs232_device<16>, public device_nvram_interface, public device_esp8266_rst_interface
{
public:
	esp8266_at_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

	void update_serial(int state);

	virtual void rst_w(int state) override;

	virtual void input_rts(int state) override;

protected:
	virtual tiny_rom_entry const *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_post_load() override;

	virtual void tra_complete() override;

	virtual void nvram_default() override;
	virtual bool nvram_read(util::read_stream &file) override;
	virtual bool nvram_write(util::write_stream &file) override;
	virtual bool nvram_can_write() const override { return m_def_stored || m_wifi_stored || m_translink_stored || m_sysmsg_stored; }

private:
	enum : u8
	{
		MODE_COMMAND,
		MODE_CONNECTING,
		MODE_SEND_DATA,
		MODE_SEND_WAIT,
		MODE_PASSTHROUGH,
		MODE_RESTART,
		MODE_LISTEN,
		MODE_JOIN,
		MODE_PING,
		MODE_SCAN,
		MODE_DOMAIN,
		MODE_SEND_RESOLVE,
		MODE_CLOSING,
		MODE_USERRAM
	};

	enum : u8
	{
		CLOSE_NONE,
		CLOSE_WAIT,
		CLOSE_DONE,
		CLOSE_SERVER
	};

	enum : u8
	{
		SEND_PLAIN = 1,
		SEND_EX = 2,
		SEND_BUF = 3
	};

	enum : u8
	{
		BOOT_ROM,
		BOOT_READY
	};

	enum : u8
	{
		FIRMWARE_AT040 = 1,
		FIRMWARE_AT11 = 2,
		FIRMWARE_AT021 = 3,
		FIRMWARE_AT13 = 4,
		FIRMWARE_AT21 = 5,
		FIRMWARE_AT22 = 6
	};

	enum : u8
	{
		FORM_EXEC,
		FORM_QUERY,
		FORM_TEST,
		FORM_SET
	};

	enum : int
	{
		PARA_FAIL = -1,
		PARA_OK = 0,
		PARA_OMITTED = 1
	};

	enum : u8
	{
		TRANSLINK_TCP,
		TRANSLINK_UDP,
		TRANSLINK_SSL
	};

	enum : u8
	{
		LINK_IDLE,
		LINK_CONNECTING,
		LINK_OPEN
	};

	struct at_arg
	{
		std::string text;
		bool quoted;
	};

	struct tcp_model
	{
		std::deque<u8> remote;
		u32 rcv_nxt = 0;
		u32 rcv_wnd = 0;
		u32 rcv_ann_wnd = 0;
		u32 rcv_ann_right_edge = 0;
		u32 acked = 0;
		bool ack_delay = false;
		bool hold = false;
		u32 held = 0;
		unsigned blocks = 0;
		bool block = false;
		bool fin = false;
	};

	virtual void received_byte(u8 byte) override;

	TIMER_CALLBACK_MEMBER(poll_network);
	TIMER_CALLBACK_MEMBER(passthrough_flush);
	TIMER_CALLBACK_MEMBER(batch_end);
	TIMER_CALLBACK_MEMBER(restart_done);
	TIMER_CALLBACK_MEMBER(tcp_tick);
	TIMER_CALLBACK_MEMBER(join_tick);
	TIMER_CALLBACK_MEMBER(ping_done);
	TIMER_CALLBACK_MEMBER(domain_done);
	TIMER_CALLBACK_MEMBER(close_tick);
	TIMER_CALLBACK_MEMBER(uart_resume);
	TIMER_CALLBACK_MEMBER(rtos_batch);

	void command_byte(u8 byte);
	void send_byte(u8 byte);
	void send_release();
	void send_cancel();
	void force_restart();
	void execute(std::string const &line);
	void cmd_cipstart(std::vector<at_arg> const &args);
	void cmd_cipsend(std::string_view const *text, u8 kind, bool equals);
	void cmd_cipclose(std::vector<at_arg> const *args);
	void cmd_cipserver(std::vector<at_arg> const &args);
	void cmd_uart(std::vector<at_arg> const &args, bool def);
	void cmd_cwjap(std::string_view text, bool def);
	void cmd_cwlap(std::string_view const *text);
	void cmd_cwdhcp(std::vector<at_arg> const &args, bool def);
	void cmd_cipsta(std::string_view text, bool def);
	void cmd_cipstatus();
	u8 status_code() const;
	void cmd_ping(std::string_view text);
	void cmd_cipsslsize(std::string_view text);
	void cmd_cipalive();
	void cmd_cipdomain(std::string_view text);
	void cmd_savetranslink(std::string_view text);
	void cmd_cipbufreset(std::string_view const *text);
	void cmd_cipbufstatus(std::string_view const *text);
	void cmd_cipcheckseq(std::string_view text);
	void send_payload();
	void send_start();
	void send_prompt();
	void send_remote(std::string const &address);
	void sendbuf_ack(unsigned link);
	void sendbuf_fail(unsigned link);
	unsigned send_space(unsigned link) const;
	unsigned send_queue_space(unsigned link) const;
	bool link_number(std::string_view &p, unsigned &link);
	void restart();
	void boot_done();
	void store_cipmode();
	unsigned reconnect_delay();
	bool close_wait(unsigned link);
	bool close_chain(unsigned first);
	void stop_waits();
	void passthrough_byte(u8 byte);

	void reply(std::string_view text);
	void reply_ok() { reply("\r\nOK\r\n"); }
	void reply_error() { reply("\r\nERROR\r\n"); }
	void send_raw(u8 const *data, std::size_t length);
	void out_push(u8 const *data, std::size_t length);
	void clear_output();
	void pump();
	bool output_idle() const { return m_out.empty() && fifo_empty() && is_transmit_register_empty(); }
	bool tx_held() const { return m_cts_flow && m_host_rts; }
	void tx_resume();
	void apply_serial();
	void closed_notice(unsigned link);
	std::string ipd_text(unsigned link, unsigned length) const;

	void tcp_reset(unsigned link);
	void tcp_data(unsigned link, std::vector<u8> const &data);
	void tcp_deliver(unsigned link);
	void tcp_deliver_all();
	void tcp_segment(unsigned link, unsigned length);
	void tcp_ack(unsigned link);
	u32 tcp_update_ann_wnd(unsigned link);
	void tcp_recved(unsigned link, u32 length);
	void tcp_timer_needed();
	void ring_enqueue(unsigned link, std::vector<u8> &&text);
	bool ring_service();
	unsigned ring_used();
	void ring_block(unsigned link);
	bool ring_direct(unsigned link) const { return m_fw->ring_passthrough_only && ((m_mode != MODE_PASSTHROUGH) || link); }
	bool ring_fits(unsigned link, std::size_t length);
	bool ring_stalled() const { return m_fw->block_free && !m_ring_wait.empty(); }

	void process_events();
	void link_open(unsigned link, unsigned delay_ms);
	void link_close(unsigned link);
	void close_all_links();
	void server_close();
	void check_server_timeouts();
	void set_defaults();
	bool nv_parse(u8 const *buf, std::size_t length);
	std::size_t nv_build(u8 *buf) const;
	void station_info(u8 *info) const;
	void station_off();
	void link_gone();
	void set_remote(unsigned link, std::string const &peer);
	void stop_ping();
	std::string station_ip() const;

	struct rtos_command
	{
		char const *name;
		void (esp8266_at_device::*handler)(u8 form);
		u8 forms = 0;
	};
	static rtos_command const s_rtos_commands[];
	static rtos_command const s_rtos_list_commands[];

	void rtos_byte(u8 byte);
	void rtos_line_input(std::vector<u8> const &batch);
	void rtos_busy(std::vector<u8> const &batch);
	void rtos_send_input(std::vector<u8> const &batch);
	void rtos_passthrough_input(std::vector<u8> const &batch);
	void rtos_passthrough_flush();
	void rtos_passthrough_off();
	void rtos_listen();
	bool rtos_parse(std::string const &line);
	void rtos_dispatch();
	void rtos_err_code(u32 code);
	int rtos_digit(unsigned index, s32 &value);
	int rtos_str(unsigned index, std::string &value);
	void rtos_link_count();
	void rtos_close_clients();
	void rtos_closed(unsigned link);
	void rtos_client_gone(unsigned link);
	void rtos_send_start();
	void rtos_send_prompt();
	void rtos_events();
	void rtos_data(unsigned link, esp8266_net::event const &ev);
	bool rtos_service();
	bool rtos_block(unsigned link);
	void rtos_notice(unsigned link);
	unsigned rtos_held(unsigned link) const;
	void rtos_finish(unsigned link);
	void rtos_tick();
	static std::string rtos_peer(std::string const &peer);
	void rtos_cmd_e0(u8 form);
	void rtos_cmd_e1(u8 form);
	void rtos_cmd_rst(u8 form);
	void rtos_cmd_gmr(u8 form);
	void rtos_cmd_sysram(u8 form);
	void rtos_cmd_sysmsg(u8 form);
	void rtos_cmd_syslog(u8 form);
	void rtos_cmd_systimestamp(u8 form);
	void rtos_cmd_sleep(u8 form);
	void rtos_cmd_uart(u8 form);
	void rtos_cmd_cwmode(u8 form);
	void rtos_cmd_cwjap(u8 form);
	void rtos_cmd_cwlap(u8 form);
	void rtos_cmd_cwqap(u8 form);
	void rtos_cmd_cwdhcp(u8 form);
	void rtos_cmd_cipsta(u8 form);
	void rtos_cmd_cifsr(u8 form);
	void rtos_cmd_ping(u8 form);
	void rtos_cmd_cipdomain(u8 form);
	void rtos_cmd_cipstatus(u8 form);
	void rtos_cmd_cipstart(u8 form);
	void rtos_cmd_cipstartex(u8 form);
	void rtos_cmd_cipclose(u8 form);
	void rtos_cmd_cipsend(u8 form);
	void rtos_cmd_cipsendex(u8 form);
	void rtos_cmd_cipdinfo(u8 form);
	void rtos_cmd_cipmux(u8 form);
	void rtos_cmd_ciprecvmode(u8 form);
	void rtos_cmd_ciprecvdata(u8 form);
	void rtos_cmd_ciprecvlen(u8 form);
	void rtos_cmd_cipserver(u8 form);
	void rtos_cmd_cipservermaxconn(u8 form);
	void rtos_cmd_cipmode(u8 form);
	void rtos_cmd_cipsto(u8 form);
	void rtos_cmd_savetranslink(u8 form);
	void rtos_cmd_cipreconnintv(u8 form);
	void rtos_cmd_cmd(u8 form);
	void rtos_cmd_sysstore(u8 form);
	void rtos_cmd_userram(u8 form);
	void rtos_cmd_cwstate(u8 form);
	void rtos_cmd_cwreconncfg(u8 form);
	void rtos_cmd_ciptcpopt(u8 form);
	void rtos_userram_input(std::vector<u8> const &batch);
	void rtos_join();
	void rtos_translink_store();
	void rtos_tcpopt_apply(unsigned link);
	int rtos_heap();
	bool rtos_store() const { return !m_fw->sysstore || m_sysstore; }
	bool rtos_wifi_quiet() const { return m_fw->sysmsg_bit2 ? ((m_mode == MODE_PASSTHROUGH) && !BIT(m_sysmsg, 2)) : bool(m_cipmode); }
	void rtos_start(bool ex);
	void rtos_send(bool ex);

	static std::vector<at_arg> split_args(std::string_view text);
	static bool fw_int(std::string_view &p, s32 &value, int &err);
	static int fw_string(std::string_view &p, std::string &out, unsigned max);
	static u32 fw_ipaddr(std::string const &text);
	static std::optional<u32> parse_number(at_arg const &arg);
	static bool empty_arg(at_arg const &arg) { return !arg.quoted && arg.text.empty(); }
	static bool take_string(std::string_view &text, std::string &out, unsigned max);
	static bool parse_mac(std::string const &text, u8 *mac);
	static std::optional<u32> parse_ip(std::string const &text);
	static std::string ip_text(u8 const *ip);

	required_ioport m_baud_config;
	required_ioport m_server_config;

	std::unique_ptr<esp8266_net> m_net;
	emu_timer *m_poll_timer;
	emu_timer *m_passthrough_timer;
	emu_timer *m_batch_timer;
	emu_timer *m_restart_timer;
	emu_timer *m_tcp_timer;
	emu_timer *m_join_timer;
	emu_timer *m_ping_timer;
	emu_timer *m_domain_timer;
	emu_timer *m_close_timer;
	emu_timer *m_stall_timer;

	u8 m_echo;
	u8 m_mux;
	u8 m_cipmode;
	u32 m_cur_rate;
	u32 m_def_rate;
	u8 m_cur_frame[4];
	u8 m_def_frame[4];
	u8 m_link_state[LINK_COUNT];
	u8 m_dinfo;
	u16 m_server_timeout;
	u8 m_rst;
	u8 m_host_rts;
	u8 m_cwmode;
	u8 m_joined;
	u8 m_sleep;
	u8 m_dhcp;
	u8 m_sta_static;
	u8 m_sta_ip[12];
	char m_ssid[MAX_SSID + 1];
	u8 m_status;
	u16 m_ssl_size;
	u8 m_send_cur;
	u8 m_link_sendbuf[LINK_COUNT];
	u32 m_seg_sent[LINK_COUNT];
	u32 m_seg_done[LINK_COUNT];
	u32 m_seg_ok[LINK_COUNT];
	u32 m_seg_map[LINK_COUNT];

	u8 m_mode;
	bool m_cts_flow;
	bool m_serial_dirty;
	bool m_line_open;
	u8 m_head;
	bool m_ping_no_dns;
	std::string m_scan_text;
	std::string m_line;
	std::deque<u8> m_out;
	u64 m_out_pushed;
	u64 m_out_popped;
	std::deque<std::pair<u64, u64> > m_ring_chunks;
	std::deque<std::pair<unsigned, std::vector<u8> > > m_ring_wait;
	bool m_pumping;
	bool m_tcp_timer_on;
	std::deque<esp8266_net::event> m_events;
	unsigned m_connect_link;
	unsigned m_send_link;
	unsigned m_send_length;
	unsigned m_send_count;
	bool m_send_ex;
	bool m_send_escape;
	std::vector<u8> m_send_buf;
	std::vector<u8> m_passthrough_buf;
	attotime m_rx_last;
	u16 m_batch_count;
	u8 m_rst_match;
	bool m_busy_batch;
	bool m_send_queued;
	u8 m_send_plus;
	bool m_connect_named;
	u32 m_link_generation[LINK_COUNT];
	bool m_link_paused[LINK_COUNT];
	esp8266_net::open_params m_link_params[LINK_COUNT];
	bool m_link_server[LINK_COUNT];
	attotime m_link_active[LINK_COUNT];
	bool m_link_tcp[LINK_COUNT];
	std::string m_link_peer[LINK_COUNT];
	tcp_model m_tcp[LINK_COUNT];
	u16 m_server_port;
	u32 m_server_generation;
	bool m_def_stored;
	std::string m_link_remote[LINK_COUNT];
	u16 m_link_remote_port[LINK_COUNT];
	u16 m_link_local_port[LINK_COUNT];
	bool m_link_udp_moved[LINK_COUNT];
	u32 m_ping_generation;
	bool m_ping_replied;
	u8 m_send_kind;
	u8 m_boot_stage;
	u32 m_domain_generation;
	u16 m_send_port;
	std::deque<std::pair<unsigned, bool> > m_unacked[LINK_COUNT];

	u8 m_cwmode_def;
	u8 m_dhcp_def;
	u8 m_sta_static_def;
	u8 m_sta_ip_def[12];
	char m_ssid_def[MAX_SSID + 1];
	bool m_wifi_stored;

	u8 m_tl_mode;
	u8 m_tl_type;
	u16 m_tl_port;
	u16 m_tl_local_port;
	u16 m_tl_keepalive;
	char m_tl_host[MAX_TRANSLINK_HOST + 1];
	bool m_translink_stored;
	std::vector<u8> m_nv_other;
	std::string m_link_home[LINK_COUNT];
	u8 m_link_closing[LINK_COUNT];
	bool m_close_chain;
	attotime m_close_start;
	u8 m_reconnects;
	bool m_uart_stalled;
	std::vector<u8> m_uart_fifo;
	emu_timer *m_rx_timer;
	std::vector<u8> m_rx_batch;
	std::string m_cmd_name;
	u8 m_cmd_form;
	std::vector<std::string> m_para;
	u8 m_recv_mode;
	u8 m_syslog;
	s32 m_sysmsg;
	bool m_sysmsg_stored;
	s32 m_timestamp;
	attotime m_timestamp_base;
	u8 m_max_conn;
	u8 m_server_clients;
	bool m_server_up;
	bool m_listen_full;
	u32 m_reconnect_ms;
	u8 m_jap_pci;
	u8 m_jap_reconn;
	u8 m_jap_listen;
	u8 m_tl_flag;
	bool m_send_rtos_ex;
	bool m_ipd_notice[LINK_COUNT];
	bool m_link_ex[LINK_COUNT];
	u16 m_idle[LINK_COUNT];
	std::deque<std::pair<std::string, std::vector<u8> > > m_udp_hold[LINK_COUNT];
	attotime m_sto_next;
	u8 m_sysstore;
	s32 m_sysmsg_def;
	int m_heap_low;
	char m_password[MAX_PASSWORD + 1];
	char m_password_def[MAX_PASSWORD + 1];
	u8 m_jap_scan;
	u8 m_jap_pmf;
	u8 m_jap_def[4];
	u16 m_reconn_interval;
	u16 m_reconn_count;
	u16 m_reconn_def[2];
	u32 m_reconnect_def;
	bool m_ever_closed;
	std::vector<u8> m_userram;
	unsigned m_userram_at;
	unsigned m_userram_left;
	s32 m_tcpopt_linger[LINK_COUNT];
	u8 m_tcpopt_nodelay[LINK_COUNT];
	s32 m_tcpopt_sndtimeo[LINK_COUNT];
	bool m_tcpopt_set[LINK_COUNT];
	u32 m_ping_ms;
	esp8266_net::open_params m_tl_pending;
	esp8266_firmware const *m_fw;
};


ROM_START(esp8266_at)
	ROM_REGION(0x1000, "firmware", ROMREGION_ERASEFF)
	ROM_DEFAULT_BIOS("at11")
	ROM_SYSTEM_BIOS(0, "at040", "AT 0.40.0.0")
	ROM_SYSTEM_BIOS(1, "at11", "AT 1.1.0.0")
	ROM_SYSTEM_BIOS(2, "at021", "AT 0.21.0.0")
	ROM_SYSTEM_BIOS(3, "at13", "AT 1.3.0.0")
	ROM_SYSTEM_BIOS(4, "at21", "AT 2.1.0.0")
	ROM_SYSTEM_BIOS(5, "at22", "AT 2.2.0.0")
ROM_END


INPUT_PORTS_START(esp8266_at)
	PORT_RS232_BAUD("RS232_BAUD", RS232_BAUD_115200, "Baud (AT+UART_DEF)", esp8266_at_device, update_serial)

	PORT_START("SERVER")
	PORT_CONFNAME(0x01, 0x00, "AT+CIPSERVER interface")
	PORT_CONFSETTING(   0x00, "127.0.0.1")
	PORT_CONFSETTING(   0x01, "All interfaces")
	PORT_CONFNAME(0x70, 0x00, "AT+CIPSERVER port offset")
	PORT_CONFSETTING(   0x00, "None")
	PORT_CONFSETTING(   0x10, "+1000")
	PORT_CONFSETTING(   0x20, "+10000")
	PORT_CONFSETTING(   0x30, "+20000")
	PORT_CONFSETTING(   0x40, "+30000")
	PORT_CONFSETTING(   0x50, "+40000")
INPUT_PORTS_END


esp8266_at_device::esp8266_at_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: buffered_rs232_device<16>(mconfig, ESP8266_AT, tag, owner, clock)
	, device_nvram_interface(mconfig, *this)
	, device_esp8266_rst_interface(mconfig, *this)
	, m_baud_config(*this, "RS232_BAUD")
	, m_server_config(*this, "SERVER")
	, m_poll_timer(nullptr)
	, m_passthrough_timer(nullptr)
	, m_batch_timer(nullptr)
	, m_restart_timer(nullptr)
	, m_tcp_timer(nullptr)
	, m_join_timer(nullptr)
	, m_ping_timer(nullptr)
	, m_domain_timer(nullptr)
	, m_close_timer(nullptr)
	, m_stall_timer(nullptr)
	, m_echo(1)
	, m_mux(0)
	, m_cipmode(0)
	, m_cur_rate(115'200)
	, m_def_rate(115'200)
	, m_cur_frame{ 8, 1, 0, 0 }
	, m_def_frame{ 8, 1, 0, 0 }
	, m_link_state{ LINK_IDLE, LINK_IDLE, LINK_IDLE, LINK_IDLE, LINK_IDLE }
	, m_dinfo(0)
	, m_server_timeout(DEFAULT_SERVER_TIMEOUT)
	, m_rst(0)
	, m_host_rts(0)
	, m_cwmode(1)
	, m_joined(1)
	, m_sleep(2)
	, m_dhcp(3)
	, m_sta_static(0)
	, m_sta_ip{ 0 }
	, m_ssid{ 0 }
	, m_status(STATUS_GOT_IP)
	, m_ssl_size(SSL_SIZE_DEFAULT)
	, m_send_cur(0)
	, m_link_sendbuf{ 0, 0, 0, 0, 0 }
	, m_seg_sent{ 0, 0, 0, 0, 0 }
	, m_seg_done{ 0, 0, 0, 0, 0 }
	, m_seg_ok{ 0, 0, 0, 0, 0 }
	, m_seg_map{ 0, 0, 0, 0, 0 }
	, m_mode(MODE_COMMAND)
	, m_cts_flow(false)
	, m_serial_dirty(false)
	, m_line_open(false)
	, m_head(0)
	, m_ping_no_dns(false)
	, m_out_pushed(0)
	, m_out_popped(0)
	, m_pumping(false)
	, m_tcp_timer_on(false)
	, m_connect_link(0)
	, m_send_link(0)
	, m_send_length(0)
	, m_send_count(0)
	, m_send_ex(false)
	, m_send_escape(false)
	, m_batch_count(0)
	, m_rst_match(0)
	, m_busy_batch(false)
	, m_send_queued(false)
	, m_send_plus(0)
	, m_connect_named(false)
	, m_link_generation{ 0, 0, 0, 0, 0 }
	, m_link_paused{ false, false, false, false, false }
	, m_link_server{ false, false, false, false, false }
	, m_link_tcp{ false, false, false, false, false }
	, m_server_port(0)
	, m_server_generation(0)
	, m_def_stored(false)
	, m_link_remote_port{ 0, 0, 0, 0, 0 }
	, m_link_local_port{ 0, 0, 0, 0, 0 }
	, m_link_udp_moved{ false, false, false, false, false }
	, m_ping_generation(0)
	, m_ping_replied(false)
	, m_send_kind(SEND_PLAIN)
	, m_boot_stage(BOOT_ROM)
	, m_domain_generation(0)
	, m_send_port(0)
	, m_cwmode_def(1)
	, m_dhcp_def(3)
	, m_sta_static_def(0)
	, m_sta_ip_def{ 0 }
	, m_ssid_def{ 0 }
	, m_wifi_stored(false)
	, m_tl_mode(0)
	, m_tl_type(TRANSLINK_TCP)
	, m_tl_port(0)
	, m_tl_local_port(0)
	, m_tl_keepalive(0)
	, m_tl_host{ 0 }
	, m_translink_stored(false)
	, m_link_closing{ CLOSE_NONE, CLOSE_NONE, CLOSE_NONE, CLOSE_NONE, CLOSE_NONE }
	, m_close_chain(false)
	, m_reconnects(0)
	, m_uart_stalled(false)
	, m_rx_timer(nullptr)
	, m_cmd_form(FORM_EXEC)
	, m_recv_mode(0)
	, m_syslog(0)
	, m_sysmsg(0)
	, m_sysmsg_stored(false)
	, m_timestamp(0)
	, m_max_conn(LINK_COUNT)
	, m_server_clients(0)
	, m_server_up(false)
	, m_listen_full(false)
	, m_reconnect_ms(RTOS_RECONNECT_MS)
	, m_jap_pci(0)
	, m_jap_reconn(0)
	, m_jap_listen(0)
	, m_tl_flag(1)
	, m_send_rtos_ex(false)
	, m_ipd_notice{ false, false, false, false, false }
	, m_link_ex{ false, false, false, false, false }
	, m_idle{ 0, 0, 0, 0, 0 }
	, m_sysstore(1)
	, m_sysmsg_def(0)
	, m_heap_low(RTOS_HEAP_FREE)
	, m_password{ 0 }
	, m_password_def{ 0 }
	, m_jap_scan(0)
	, m_jap_pmf(0)
	, m_jap_def{ 0, 3, 0, 0 }
	, m_reconn_interval(1)
	, m_reconn_count(0)
	, m_reconn_def{ 1, 0 }
	, m_reconnect_def(RTOS_RECONNECT_MS)
	, m_ever_closed(false)
	, m_userram_at(0)
	, m_userram_left(0)
	, m_tcpopt_linger{ -1, -1, -1, -1, -1 }
	, m_tcpopt_nodelay{ 0, 0, 0, 0, 0 }
	, m_tcpopt_sndtimeo{ 0, 0, 0, 0, 0 }
	, m_tcpopt_set{ false, false, false, false, false }
	, m_ping_ms(0)
	, m_fw(&FIRMWARE_11)
{
}


tiny_rom_entry const *esp8266_at_device::device_rom_region() const
{
	return ROM_NAME(esp8266_at);
}


ioport_constructor esp8266_at_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(esp8266_at);
}


void esp8266_at_device::device_start()
{
	switch (system_bios())
	{
	case FIRMWARE_AT040:
		m_fw = &FIRMWARE_040;
		break;
	case FIRMWARE_AT021:
		m_fw = &FIRMWARE_021;
		break;
	case FIRMWARE_AT13:
		m_fw = &FIRMWARE_13;
		break;
	case FIRMWARE_AT21:
		m_fw = &FIRMWARE_21;
		break;
	case FIRMWARE_AT22:
		m_fw = &FIRMWARE_22;
		break;
	default:
		m_fw = &FIRMWARE_11;
		break;
	}

	m_poll_timer = timer_alloc(FUNC(esp8266_at_device::poll_network), this);
	m_passthrough_timer = timer_alloc(FUNC(esp8266_at_device::passthrough_flush), this);
	m_batch_timer = timer_alloc(FUNC(esp8266_at_device::batch_end), this);
	m_restart_timer = timer_alloc(FUNC(esp8266_at_device::restart_done), this);
	m_tcp_timer = timer_alloc(FUNC(esp8266_at_device::tcp_tick), this);
	m_join_timer = timer_alloc(FUNC(esp8266_at_device::join_tick), this);
	m_ping_timer = timer_alloc(FUNC(esp8266_at_device::ping_done), this);
	m_domain_timer = timer_alloc(FUNC(esp8266_at_device::domain_done), this);
	m_close_timer = timer_alloc(FUNC(esp8266_at_device::close_tick), this);
	m_stall_timer = timer_alloc(FUNC(esp8266_at_device::uart_resume), this);
	m_rx_timer = timer_alloc(FUNC(esp8266_at_device::rtos_batch), this);

	m_net = std::make_unique<esp8266_net>();

	save_item(NAME(m_echo));
	save_item(NAME(m_mux));
	save_item(NAME(m_cipmode));
	save_item(NAME(m_cur_rate));
	save_item(NAME(m_def_rate));
	save_item(NAME(m_cur_frame));
	save_item(NAME(m_def_frame));
	save_item(NAME(m_link_state));
	save_item(NAME(m_dinfo));
	save_item(NAME(m_server_timeout));
	save_item(NAME(m_rst));
	save_item(NAME(m_host_rts));
	save_item(NAME(m_cwmode));
	save_item(NAME(m_joined));
	save_item(NAME(m_sleep));
	save_item(NAME(m_dhcp));
	save_item(NAME(m_sta_static));
	save_item(NAME(m_sta_ip));
	save_item(NAME(m_ssid));
	save_item(NAME(m_status));
	save_item(NAME(m_ssl_size));
	save_item(NAME(m_send_cur));
	save_item(NAME(m_link_sendbuf));
	save_item(NAME(m_seg_sent));
	save_item(NAME(m_seg_done));
	save_item(NAME(m_seg_ok));
	save_item(NAME(m_seg_map));
	save_item(NAME(m_recv_mode));
	save_item(NAME(m_syslog));
	save_item(NAME(m_sysmsg));
	save_item(NAME(m_timestamp));
	save_item(NAME(m_timestamp_base));
	save_item(NAME(m_max_conn));
	save_item(NAME(m_reconnect_ms));
	save_item(NAME(m_jap_pci));
	save_item(NAME(m_jap_reconn));
	save_item(NAME(m_jap_listen));
	save_item(NAME(m_link_ex));
	save_item(NAME(m_sysstore));
	save_item(NAME(m_sysmsg_def));
	save_item(NAME(m_password));
	save_item(NAME(m_jap_scan));
	save_item(NAME(m_jap_pmf));
	save_item(NAME(m_reconn_interval));
	save_item(NAME(m_reconn_count));
	save_item(NAME(m_ever_closed));
	save_item(NAME(m_tcpopt_linger));
	save_item(NAME(m_tcpopt_nodelay));
	save_item(NAME(m_tcpopt_sndtimeo));
	save_item(NAME(m_tcpopt_set));
}


void esp8266_at_device::device_stop()
{
	m_net.reset();
}


void esp8266_at_device::device_reset()
{
	stop_waits();
	close_all_links();
	server_close();
	m_events.clear();
	clear_output();
	clear_fifo();

	set_defaults();
	m_rst = 0;

	m_passthrough_timer->adjust(attotime::never);
	m_tcp_timer->adjust(attotime::never);
	m_tcp_timer_on = false;
	m_join_timer->adjust(attotime::never);
	stop_ping();
	m_poll_timer->adjust(attotime::from_msec(1), 0, attotime::from_msec(1));

	output_rxd(1);
	output_dcd(0);
	output_dsr(0);
	output_cts(0);

	m_mode = MODE_RESTART;
	m_boot_stage = BOOT_ROM;
	m_restart_timer->adjust(attotime::zero);
}


void esp8266_at_device::device_post_load()
{
	stop_waits();
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		m_link_generation[i]++;
		m_net->close(i, m_link_generation[i]);
		m_link_paused[i] = false;
		m_link_server[i] = false;
		m_unacked[i].clear();
		tcp_reset(i);
	}
	server_close();
	m_events.clear();
	clear_output();
	m_line.clear();
	m_send_buf.clear();
	m_passthrough_buf.clear();
	m_line_open = false;
	m_head = 0;
	m_rx_last = attotime::zero;
	m_busy_batch = false;
	m_send_queued = false;
	m_send_plus = 0;
	m_mode = m_rst ? MODE_RESTART : MODE_COMMAND;
	m_passthrough_timer->adjust(attotime::never);
	m_batch_timer->adjust(attotime::never);
	m_restart_timer->adjust(attotime::never);
	m_tcp_timer->adjust(attotime::never);
	m_tcp_timer_on = false;
	m_join_timer->adjust(attotime::never);
	stop_ping();
	apply_serial();

	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		if (m_link_state[i] != LINK_IDLE)
		{
			LOGLINK("link %u lost on state load\n", i);
			m_link_state[i] = LINK_IDLE;
			closed_notice(i);
			sendbuf_fail(i);
			link_gone();
		}
	}
}


void esp8266_at_device::nvram_default()
{
	m_def_rate = convert_baud(m_baud_config->read());
	std::fill(std::begin(m_def_frame), std::end(m_def_frame), 0);
	m_def_frame[0] = 8;
	m_def_frame[1] = 1;
	m_def_stored = false;

	m_cwmode_def = 1;
	m_dhcp_def = 3;
	m_sta_static_def = 0;
	std::fill(std::begin(m_sta_ip_def), std::end(m_sta_ip_def), 0);
	std::fill(std::begin(m_ssid_def), std::end(m_ssid_def), 0);
	std::copy_n(DEFAULT_SSID, sizeof(DEFAULT_SSID), m_ssid_def);
	m_wifi_stored = false;

	m_tl_mode = 0;
	m_tl_type = TRANSLINK_TCP;
	m_tl_port = 0;
	m_tl_local_port = 0;
	m_tl_keepalive = 0;
	std::fill(std::begin(m_tl_host), std::end(m_tl_host), 0);
	m_translink_stored = false;
	m_jap_pci = 0;
	m_jap_reconn = 0;
	m_jap_listen = 0;
	m_tl_flag = 1;
	m_sysmsg = 0;
	m_sysmsg_stored = false;
	m_sysmsg_def = 0;
	std::fill(std::begin(m_password_def), std::end(m_password_def), 0);
	m_jap_def[0] = 0;
	m_jap_def[1] = 3;
	m_jap_def[2] = 0;
	m_jap_def[3] = 0;
	m_reconn_def[0] = 1;
	m_reconn_def[1] = 0;
	m_reconnect_def = RTOS_RECONNECT_MS;
	m_def_frame[3] = m_fw->uart_flow;
	m_nv_other.clear();
}


bool esp8266_at_device::nvram_read(util::read_stream &file)
{
	// 0-129: the AT 1.1.0.0 block; 130-259: the AT 0.40.0.0 block; 260-389: the AT 0.21.0.0
	// block; 390-527: the AT 2.1.0.0-dev block; 528-738: the AT 2.2.0.0 block. A later block is present once its version has
	// stored a setting, with any block before it padded. Each version keeps the others' blocks
	// as they are. A block:
	// 0-7: AT+UART_DEF rate as 32-bit little-endian, then the four AT+UART_DEF framing values
	// 8: 0 when nothing follows 9, 1 when the WiFi settings follow, 2 when the AT+SAVETRANSLINK
	// settings follow them
	// 9: bit 0 set when 0-7 are stored
	// 10: AT+CWMODE_DEF, 11: AT+CWDHCP_DEF, 12: 1 when 13-24 hold an AT+CIPSTA_DEF
	// address, gateway and netmask, 25-57: AT+CWJAP_DEF SSID, NUL-terminated
	// 58: AT+SAVETRANSLINK mode (0.21: AT+CIPMODE), 59: link type (0 TCP, 1 UDP, 2 SSL), 60-61: remote port,
	// 62-63: UDP local port, 64-65: TCP keep-alive, all 16-bit little-endian,
	// 66-129: remote host, NUL-terminated
	// The 2.1 block adds 130-133: AT+SYSMSG, 32-bit little-endian; 134-136: AT+CWJAP pci_en, reconn
	// and listen interval; 137: the last AT+SAVETRANSLINK value for TCP.
	// The 2.2 block adds 130-133: AT+SYSMSG; 134-137: AT+CWJAP pci_en, listen interval, scan mode
	// and PMF; 138-141: AT+CWRECONNCFG interval and count; 142-145: AT+CIPRECONNINTV in ms, 32-bit;
	// 146-210: AT+CWJAP password, NUL-terminated. Values over one byte are little-endian.
	// The 1.1 block alone may be cut after 8 bytes (AT+UART_DEF alone) or 58 (no AT+SAVETRANSLINK).
	u8 buf[(5 * NV_BLOCK) + NV_EXTRA + NV_RTOS2];
	auto const [err, actual] = util::read(file, buf, sizeof(buf));
	if (err || ((actual != 8) && (actual != 58) && (actual != NV_BLOCK) && (actual != (2 * NV_BLOCK)) && (actual != (3 * NV_BLOCK)) &&
			(actual != ((4 * NV_BLOCK) + NV_EXTRA)) && (actual != sizeof(buf))))
		return false;

	std::size_t const offset = m_fw->nv_offset;
	std::size_t const length = offset ? ((actual > offset) ? m_fw->nv_size : 0) : std::min<std::size_t>(actual, NV_BLOCK);
	if (!nv_parse(&buf[offset], length))
		return false;
	m_nv_other.assign(&buf[0], &buf[actual]);
	return true;
}


bool esp8266_at_device::nv_parse(u8 const *buf, std::size_t length)
{
	u8 const follows = (length >= 58) ? buf[8] : 0;
	if ((length == 58) ? (follows != 1) : (follows > 2))
		return false;
	bool const wifi = follows >= 1;
	bool const translink = follows == 2;
	bool const extra = translink && (length > NV_BLOCK);
	if (wifi && ((buf[10] < (m_fw->rtos ? 0 : 1)) || (buf[10] > 3) || (buf[11] > 3) || (buf[12] > 1) || buf[57]))
		return false;
	if (translink && ((buf[58] > 1) || (buf[59] > TRANSLINK_SSL) || buf[NV_BLOCK - 1]))
		return false;
	if (extra && !m_fw->sysstore && ((buf[134] > 1) || (buf[135] > 1) || (buf[136] > 99) || (buf[137] > 1)))
		return false;
	if (extra && m_fw->sysstore && ((buf[134] > 1) || (buf[135] > 100) || (buf[136] > 1) || (buf[137] > 3) ||
			(get_u16le(&buf[138]) > 7'200) || (get_u16le(&buf[140]) > 1'000) || (get_u32le(&buf[142]) < 100) ||
			(get_u32le(&buf[142]) > 3'600'000) || buf[146 + MAX_PASSWORD]))
		return false;

	bool const uart = length && ((length == 8) || BIT(buf[9], 0));
	u32 const rate = uart ? get_u32le(buf) : 0;
	if (uart && ((rate < 1) || (rate > 5'000'000) || (buf[4] < 5) || (buf[4] > 8) || (buf[5] < 1) || (buf[5] > 3) || (buf[6] > 2) || (buf[7] > 3)))
		return false;

	nvram_default();
	if (uart)
	{
		if (rate <= CARD_MAX_BAUD)
			m_def_rate = rate;
		std::copy(&buf[4], &buf[8], std::begin(m_def_frame));
		m_def_stored = true;
	}
	if (wifi)
	{
		m_cwmode_def = buf[10];
		m_dhcp_def = buf[11];
		m_sta_static_def = buf[12];
		std::copy(&buf[13], &buf[25], std::begin(m_sta_ip_def));
		std::copy(&buf[25], &buf[58], std::begin(m_ssid_def));
		m_wifi_stored = true;
	}
	if (translink)
	{
		m_tl_mode = buf[58];
		m_tl_type = buf[59];
		m_tl_port = get_u16le(&buf[60]);
		m_tl_local_port = get_u16le(&buf[62]);
		m_tl_keepalive = get_u16le(&buf[64]);
		std::copy(&buf[66], &buf[NV_BLOCK], std::begin(m_tl_host));
		m_translink_stored = true;
	}
	if (extra && m_fw->sysstore)
	{
		m_sysmsg_def = s32(get_u32le(&buf[130]));
		m_sysmsg_stored = true;
		std::copy(&buf[134], &buf[138], std::begin(m_jap_def));
		m_reconn_def[0] = get_u16le(&buf[138]);
		m_reconn_def[1] = get_u16le(&buf[140]);
		m_reconnect_def = get_u32le(&buf[142]);
		std::copy(&buf[146], &buf[147 + MAX_PASSWORD], std::begin(m_password_def));
	}
	else if (extra)
	{
		m_sysmsg = s32(get_u32le(&buf[130]));
		m_sysmsg_stored = true;
		m_jap_pci = buf[134];
		m_jap_reconn = buf[135];
		m_jap_listen = buf[136];
		m_tl_flag = buf[137];
	}
	return true;
}


bool esp8266_at_device::nvram_write(util::write_stream &file)
{
	u8 buf[(5 * NV_BLOCK) + NV_EXTRA + NV_RTOS2];
	std::fill(std::begin(buf), std::end(buf), 0);
	std::copy(m_nv_other.begin(), m_nv_other.end(), std::begin(buf));
	std::size_t const offset = m_fw->nv_offset;
	std::fill_n(&buf[offset], m_fw->nv_size, 0);
	std::size_t length = nv_build(&buf[offset]);
	if (offset || (m_nv_other.size() > NV_BLOCK))
	{
		if (offset && (m_nv_other.size() == 8))
			buf[9] = 1;
		length = std::max<std::size_t>(offset + m_fw->nv_size, m_nv_other.size());
	}
	auto const [err, actual] = util::write(file, buf, length);
	return !err;
}


std::size_t esp8266_at_device::nv_build(u8 *buf) const
{
	put_u32le(buf, m_def_rate);
	std::copy(std::begin(m_def_frame), std::end(m_def_frame), &buf[4]);
	buf[9] = m_def_stored ? 1 : 0;
	if (!m_wifi_stored && !m_translink_stored && !m_sysmsg_stored)
		return 8;
	buf[8] = 2;
	buf[10] = m_cwmode_def;
	buf[11] = m_dhcp_def;
	buf[12] = m_sta_static_def;
	std::copy(std::begin(m_sta_ip_def), std::end(m_sta_ip_def), &buf[13]);
	std::copy(std::begin(m_ssid_def), std::end(m_ssid_def), &buf[25]);
	buf[58] = m_tl_mode;
	buf[59] = m_tl_type;
	put_u16le(&buf[60], m_tl_port);
	put_u16le(&buf[62], m_tl_local_port);
	put_u16le(&buf[64], m_tl_keepalive);
	std::copy(std::begin(m_tl_host), std::end(m_tl_host), &buf[66]);
	if (m_fw->nv_size <= NV_BLOCK)
		return NV_BLOCK;
	if (m_fw->sysstore)
	{
		put_u32le(&buf[130], u32(m_sysmsg_def));
		std::copy(std::begin(m_jap_def), std::end(m_jap_def), &buf[134]);
		put_u16le(&buf[138], m_reconn_def[0]);
		put_u16le(&buf[140], m_reconn_def[1]);
		put_u32le(&buf[142], m_reconnect_def);
		std::copy(std::begin(m_password_def), std::end(m_password_def), &buf[146]);
		return m_fw->nv_size;
	}
	put_u32le(&buf[130], u32(m_sysmsg));
	buf[134] = m_jap_pci;
	buf[135] = m_jap_reconn;
	buf[136] = m_jap_listen;
	buf[137] = m_tl_flag;
	return m_fw->nv_size;
}


void esp8266_at_device::update_serial(int state)
{
	m_def_rate = m_cur_rate = convert_baud(m_baud_config->read());
	apply_serial();
}


void esp8266_at_device::set_defaults()
{
	m_echo = 1;
	m_mux = 0;
	m_cipmode = 0;
	m_dinfo = 0;
	m_server_timeout = m_fw->server_timeout;
	m_ssl_size = SSL_SIZE_DEFAULT;
	m_mode = MODE_COMMAND;
	m_line.clear();
	m_send_buf.clear();
	m_passthrough_buf.clear();
	m_line_open = false;
	m_head = 0;
	m_cur_rate = m_def_rate;
	std::copy(std::begin(m_def_frame), std::end(m_def_frame), std::begin(m_cur_frame));
	// a stored parity 1 is refused by the driver at start-up
	if (m_fw->rtos && !m_fw->uart_odd && (m_cur_frame[2] == 1))
		m_cur_frame[2] = 0;
	apply_serial();

	m_recv_mode = 0;
	m_syslog = 0;
	m_timestamp = 0;
	m_timestamp_base = machine().time();
	m_max_conn = LINK_COUNT;
	m_server_clients = 0;
	m_listen_full = false;
	m_reconnect_ms = RTOS_RECONNECT_MS;
	m_sto_next = attotime::never;
	m_send_cur = 0;
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		m_link_sendbuf[i] = 0;
		m_seg_sent[i] = m_seg_done[i] = m_seg_ok[i] = m_seg_map[i] = 0;
		m_unacked[i].clear();
		m_link_ex[i] = false;
		m_idle[i] = 0;
	}

	m_sysstore = 1;
	m_heap_low = RTOS_HEAP_FREE;
	m_ever_closed = false;
	m_userram.clear();
	m_ping_ms = 0;
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		m_tcpopt_linger[i] = -1;
		m_tcpopt_nodelay[i] = 0;
		m_tcpopt_sndtimeo[i] = 0;
		m_tcpopt_set[i] = false;
	}
	if (m_fw->sysstore)
	{
		m_sysmsg = m_sysmsg_def;
		std::copy(std::begin(m_password_def), std::end(m_password_def), std::begin(m_password));
		m_jap_pci = m_jap_def[0];
		m_jap_listen = m_jap_def[1];
		m_jap_scan = m_jap_def[2];
		m_jap_pmf = m_jap_def[3];
		m_reconn_interval = m_reconn_def[0];
		m_reconn_count = m_reconn_def[1];
		m_reconnect_ms = m_reconnect_def;
	}

	m_cwmode = m_cwmode_def;
	m_sleep = m_fw->sleep_modes ? 1 : 2;
	m_dhcp = m_dhcp_def;
	m_sta_static = !BIT(m_dhcp, 1) && m_sta_static_def;
	std::copy(std::begin(m_sta_ip_def), std::end(m_sta_ip_def), std::begin(m_sta_ip));
	std::copy(std::begin(m_ssid_def), std::end(m_ssid_def), std::begin(m_ssid));
	m_joined = BIT(m_cwmode, 0) && m_ssid[0];
	m_status = m_fw->status_commands ? STATUS_DISCONNECTED : m_joined ? STATUS_GOT_IP : STATUS_NO_WIFI;
}


void esp8266_at_device::station_info(u8 *info) const
{
	std::fill_n(info, 12, 0);
	if (!m_joined)
		return;
	if (m_sta_static)
	{
		std::copy_n(m_sta_ip, 12, info);
		return;
	}

	std::error_code err;
	asio::ip::address_v4 address = asio::ip::address_v4::loopback();
	if (BIT(m_server_config->read(), 0))
		address = asio::ip::make_address_v4(esp8266_net::primary_ipv4(), err);
	if (err)
		address = asio::ip::address_v4::loopback();
	auto const bytes = address.to_bytes();
	std::copy(bytes.begin(), bytes.end(), info);
	std::copy(bytes.begin(), bytes.begin() + 3, info + 4);
	info[7] = 1;
	info[8] = info[9] = info[10] = 255;
}


std::string esp8266_at_device::station_ip() const
{
	u8 info[12];
	station_info(info);
	return ip_text(info);
}


void esp8266_at_device::station_off()
{
	if (!m_joined)
		return;
	m_joined = 0;
	LOGLINK("station disconnected\n");
	if (m_fw->quiet_wifi || m_cipmode)
		return;
	reply("WIFI DISCONNECT\r\n");
	if ((m_cwmode != 1) && !m_fw->drop_tcp_any_mode)
		return;
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		if (m_link_tcp[i] && (m_link_state[i] != LINK_IDLE))
		{
			link_close(i);
			closed_notice(i);
			sendbuf_fail(i);
			link_gone();
		}
	}
}


void esp8266_at_device::link_gone()
{
	if (std::find_if(std::begin(m_link_state), std::end(m_link_state), [] (u8 s) { return s != LINK_IDLE; }) == std::end(m_link_state))
		m_status = STATUS_DISCONNECTED;
}


void esp8266_at_device::set_remote(unsigned link, std::string const &peer)
{
	auto const comma = peer.rfind(',');
	m_link_remote[link] = peer.substr(0, comma);
	if (m_link_remote[link] == "::1")
		m_link_remote[link] = "127.0.0.1";
	m_link_remote_port[link] = (comma == std::string::npos) ? 0 : u16(std::strtoul(peer.c_str() + comma + 1, nullptr, 10));
}


void esp8266_at_device::stop_ping()
{
	m_ping_timer->adjust(attotime::never);
	m_net->stop_ping(++m_ping_generation);
}


void esp8266_at_device::rst_w(int state)
{
	if (bool(state) == bool(m_rst))
		return;

	m_rst = state ? 1 : 0;
	if (m_rst)
	{
		LOGLINK("reset pin asserted\n");
		stop_waits();
		close_all_links();
		server_close();
		m_events.clear();
		clear_output();
		clear_fifo();
		m_mode = MODE_RESTART;
		m_passthrough_timer->adjust(attotime::never);
		m_restart_timer->adjust(attotime::never);
		m_join_timer->adjust(attotime::never);
		stop_ping();
	}
	else
	{
		LOGLINK("reset pin released\n");
		m_boot_stage = BOOT_ROM;
		m_restart_timer->adjust(attotime::zero);
	}
}


void esp8266_at_device::apply_serial()
{
	static parity_t const parities[] = { PARITY_NONE, PARITY_ODD, PARITY_EVEN };
	static stop_bits_t const stops[] = { STOP_BITS_1, STOP_BITS_1, STOP_BITS_1_5, STOP_BITS_2 };

	set_data_frame(1, m_cur_frame[0], parities[m_cur_frame[2]], stops[m_cur_frame[1]]);
	set_rate(m_cur_rate);
	m_cts_flow = BIT(m_cur_frame[3], 1);
	m_serial_dirty = false;
	LOGCMD("serial %u baud, %u data bits, stop bits %u, parity %u, flow control %u\n", m_cur_rate, m_cur_frame[0], m_cur_frame[1], m_cur_frame[2], m_cur_frame[3]);
	if (!tx_held())
		tx_resume();
}


void esp8266_at_device::input_rts(int state)
{
	m_host_rts = state ? 1 : 0;
	if (!tx_held())
		tx_resume();
}


void esp8266_at_device::tx_resume()
{
	if (is_transmit_register_empty())
		buffered_rs232_device<16>::tra_complete();
	pump();
}


void esp8266_at_device::reply(std::string_view text)
{
	if (VERBOSE & LOG_REPLY)
	{
		std::string shown;
		for (char const c : text)
		{
			if (c == '\r')
				shown += "\\r";
			else if (c == '\n')
				shown += "\\n";
			else
				shown += c;
		}
		LOGREPLY("reply: %s\n", shown);
	}
	out_push(reinterpret_cast<u8 const *>(text.data()), text.size());
	pump();
}


void esp8266_at_device::send_raw(u8 const *data, std::size_t length)
{
	out_push(data, length);
	pump();
}


void esp8266_at_device::out_push(u8 const *data, std::size_t length)
{
	m_out.insert(m_out.end(), data, data + length);
	m_out_pushed += length;
}


void esp8266_at_device::clear_output()
{
	m_out.clear();
	m_out_popped = m_out_pushed;
	m_ring_chunks.clear();
	m_ring_wait.clear();
	for (auto &tcp : m_tcp)
	{
		tcp.blocks = 0;
		tcp.block = false;
	}
}


void esp8266_at_device::pump()
{
	if (m_pumping)
		return;
	m_pumping = true;

	bool progress;
	do
	{
		progress = false;
		while (!m_out.empty() && !fifo_full() && !tx_held())
		{
			transmit_byte(m_out.front());
			m_out.pop_front();
			m_out_popped++;
			progress = true;
		}
		if (m_fw->rtos ? rtos_service() : ring_service())
			progress = true;
	}
	while (progress);

	if (!m_fw->rtos && (m_out.size() < OUT_HIGH_WATER))
	{
		for (unsigned i = 0; i < LINK_COUNT; i++)
		{
			if (m_link_paused[i] && !m_link_tcp[i])
			{
				m_link_paused[i] = false;
				m_net->resume(i, m_link_generation[i]);
			}
		}
	}

	m_pumping = false;
}


unsigned esp8266_at_device::ring_used()
{
	while (!m_ring_chunks.empty() && (m_ring_chunks.front().second <= m_out_popped))
		m_ring_chunks.pop_front();

	u64 used = 0;
	for (auto const &chunk : m_ring_chunks)
		used += chunk.second - std::max(chunk.first, m_out_popped);
	return unsigned(used);
}


void esp8266_at_device::ring_enqueue(unsigned link, std::vector<u8> &&text)
{
	if (m_ring_wait.empty() && ring_fits(link, text.size()))
	{
		if (!ring_direct(link))
			m_ring_chunks.emplace_back(m_out_pushed, m_out_pushed + text.size());
		out_push(text.data(), text.size());
		ring_block(link);
		return;
	}

	tcp_model &tcp = m_tcp[link];
	if (!tcp.hold)
	{
		tcp.hold = true;
		tcp.held = 0;
	}
	tcp.blocks++;
	m_ring_wait.emplace_back(link, std::move(text));
}


bool esp8266_at_device::ring_service()
{
	bool moved = false;
	while (!m_ring_wait.empty() && ring_fits(m_ring_wait.front().first, m_ring_wait.front().second.size()))
	{
		unsigned const link = m_ring_wait.front().first;
		std::vector<u8> const text = std::move(m_ring_wait.front().second);
		m_ring_wait.pop_front();
		if (!ring_direct(link))
			m_ring_chunks.emplace_back(m_out_pushed, m_out_pushed + text.size());
		out_push(text.data(), text.size());
		ring_block(link);
		moved = true;

		tcp_model &tcp = m_tcp[link];
		if (tcp.blocks && !--tcp.blocks && tcp.hold)
		{
			tcp.hold = false;
			u32 const held = tcp.held;
			tcp.held = 0;
			tcp_recved(link, held);
			tcp_deliver(link);
		}
	}

	if (moved && m_fw->block_free && m_ring_wait.empty())
	{
		for (unsigned i = 0; i < LINK_COUNT; i++)
		{
			if (m_link_tcp[i] && (m_link_state[i] == LINK_OPEN))
				tcp_deliver(i);
		}
	}
	if (m_tcp[0].block && ((m_fw->tx_ring - ring_used()) > m_fw->recover_free))
	{
		m_tcp[0].block = false;
		tcp_deliver(0);
		moved = true;
	}
	return moved;
}


bool esp8266_at_device::ring_fits(unsigned link, std::size_t length)
{
	if (ring_direct(link))
		return m_out.size() <= UART_FIFO;
	return (m_fw->tx_ring - ring_used()) >= length;
}


void esp8266_at_device::ring_block(unsigned link)
{
	if (m_fw->block_free && !link && m_link_tcp[0] && (m_mode == MODE_PASSTHROUGH) && ((m_fw->tx_ring - ring_used()) <= m_fw->block_free))
		m_tcp[0].block = true;
}


void esp8266_at_device::tcp_reset(unsigned link)
{
	m_tcp[link] = tcp_model();
	m_tcp[link].rcv_wnd = m_tcp[link].rcv_ann_wnd = m_tcp[link].rcv_ann_right_edge = m_fw->tcp_wnd;
	m_ipd_notice[link] = false;
	m_udp_hold[link].clear();
	m_idle[link] = 0;
	for (auto it = m_ring_wait.begin(); it != m_ring_wait.end(); )
	{
		if (it->first == link)
			it = m_ring_wait.erase(it);
		else
			++it;
	}
}


void esp8266_at_device::tcp_timer_needed()
{
	if (!m_tcp_timer_on)
	{
		m_tcp_timer_on = true;
		m_tcp_timer->adjust(attotime::from_msec(ESP_TCP_TMR_MS), 0, attotime::from_msec(ESP_TCP_TMR_MS));
	}
}


TIMER_CALLBACK_MEMBER(esp8266_at_device::tcp_tick)
{
	bool active = false;
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		if (!m_link_tcp[i] || (m_link_state[i] != LINK_OPEN))
			continue;
		active = true;
		if (m_tcp[i].ack_delay)
		{
			tcp_ack(i);
			tcp_deliver(i);
		}
	}
	if (!active)
	{
		m_tcp_timer_on = false;
		m_tcp_timer->adjust(attotime::never);
	}
	pump();
}


void esp8266_at_device::tcp_ack(unsigned link)
{
	tcp_model &tcp = m_tcp[link];
	tcp.ack_delay = false;
	tcp.acked = tcp.rcv_nxt;
	tcp.rcv_ann_right_edge = tcp.rcv_nxt + tcp.rcv_ann_wnd;
}


u32 esp8266_at_device::tcp_update_ann_wnd(unsigned link)
{
	tcp_model &tcp = m_tcp[link];
	u32 const new_right_edge = tcp.rcv_nxt + tcp.rcv_wnd;
	if (s32(new_right_edge - (tcp.rcv_ann_right_edge + std::min(m_fw->tcp_wnd / 2, ESP_TCP_MSS))) >= 0)
	{
		tcp.rcv_ann_wnd = tcp.rcv_wnd;
		return new_right_edge - tcp.rcv_ann_right_edge;
	}
	if (s32(tcp.rcv_nxt - tcp.rcv_ann_right_edge) > 0)
		tcp.rcv_ann_wnd = 0;
	else
		tcp.rcv_ann_wnd = tcp.rcv_ann_right_edge - tcp.rcv_nxt;
	return 0;
}


void esp8266_at_device::tcp_recved(unsigned link, u32 length)
{
	tcp_model &tcp = m_tcp[link];
	tcp.rcv_wnd = std::min<u32>(tcp.rcv_wnd + length, m_fw->tcp_wnd);
	if (tcp_update_ann_wnd(link) >= (m_fw->tcp_wnd / 4))
		tcp_ack(link);
}


void esp8266_at_device::tcp_data(unsigned link, std::vector<u8> const &data)
{
	tcp_model &tcp = m_tcp[link];
	tcp.remote.insert(tcp.remote.end(), data.begin(), data.end());
	tcp_deliver(link);
	if (tcp.remote.size() >= m_fw->tcp_wnd)
		m_link_paused[link] = true;
	else
		m_net->resume(link, m_link_generation[link]);
}


void esp8266_at_device::tcp_deliver(unsigned link)
{
	if (m_mode == MODE_SEND_DATA)
		return;

	tcp_model &tcp = m_tcp[link];
	while (!tcp.remote.empty() && !tcp.block && !ring_stalled())
	{
		s32 const window = s32(tcp.rcv_ann_right_edge - tcp.rcv_nxt);
		if (window <= 0)
			break;
		unsigned const length = std::min<unsigned>({ unsigned(tcp.remote.size()), ESP_TCP_MSS, unsigned(window) });
		if ((tcp.acked != tcp.rcv_nxt) && (length < ESP_TCP_MSS))
			break;
		tcp_segment(link, length);
	}

	if (m_link_paused[link] && (tcp.remote.size() < m_fw->tcp_wnd))
	{
		m_link_paused[link] = false;
		m_net->resume(link, m_link_generation[link]);
	}

	if (tcp.fin && tcp.remote.empty() && !tcp.blocks)
	{
		LOGLINK("link %u: closed by remote\n", link);
		tcp.fin = false;
		m_link_state[link] = LINK_IDLE;
		m_link_paused[link] = false;
		if ((m_mode == MODE_PASSTHROUGH) && !link)
		{
			link_open(link, reconnect_delay());
		}
		else
		{
			closed_notice(link);
			sendbuf_fail(link);
			link_gone();
		}
	}
}


void esp8266_at_device::tcp_deliver_all()
{
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		if (m_link_tcp[i] && (m_link_state[i] == LINK_OPEN))
			tcp_deliver(i);
	}
	pump();
}


void esp8266_at_device::tcp_segment(unsigned link, unsigned length)
{
	tcp_model &tcp = m_tcp[link];

	tcp.rcv_nxt += length;
	tcp.rcv_wnd -= length;
	tcp_update_ann_wnd(link);
	bool const ack_now = tcp.ack_delay;
	tcp.ack_delay = !tcp.ack_delay;

	if (!tcp.hold)
		tcp_recved(link, length);
	else
		tcp.held += length;

	LOGLINK("link %u: received %u bytes\n", link, length);
	m_link_active[link] = machine().time();
	std::vector<u8> text;
	if ((m_mode != MODE_PASSTHROUGH) || link)
	{
		std::string const header = ipd_text(link, length);
		text.assign(header.begin(), header.end());
	}
	text.insert(text.end(), tcp.remote.begin(), tcp.remote.begin() + length);
	tcp.remote.erase(tcp.remote.begin(), tcp.remote.begin() + length);
	ring_enqueue(link, std::move(text));

	if (ack_now)
		tcp_ack(link);
}


void esp8266_at_device::tra_complete()
{
	if (!tx_held())
		buffered_rs232_device<16>::tra_complete();
	pump();

	if (m_serial_dirty && output_idle())
		apply_serial();
}


void esp8266_at_device::received_byte(u8 byte)
{
	if (m_fw->rtos)
	{
		rtos_byte(byte);
		return;
	}

	attotime const now = machine().time();
	if ((now - m_rx_last) >= attotime::from_msec(RX_BATCH_GAP_MS))
	{
		if (m_send_plus || m_busy_batch || m_send_queued)
			batch_end(0);
		m_batch_count = 0;
		m_rst_match = 0;
	}
	m_rx_last = now;
	if (m_batch_count < 0xffff)
		m_batch_count++;

	switch (m_mode)
	{
	case MODE_RESTART:
		break;

	case MODE_SEND_DATA:
		if (!m_fw->send_busy_only && (m_send_plus || ((m_batch_count == 1) && (byte == '+'))))
		{
			if ((byte == '+') && (m_send_plus < 3))
			{
				m_send_plus++;
				m_batch_timer->adjust(attotime::from_msec(RX_BATCH_GAP_MS));
				break;
			}
			m_batch_timer->adjust(attotime::never);
			send_release();
			if (m_mode != MODE_SEND_DATA)
			{
				command_byte(byte);
				break;
			}
		}
		send_byte(byte);
		break;

	case MODE_PASSTHROUGH:
		passthrough_byte(byte);
		break;

	default:
		command_byte(byte);
		break;
	}
}


void esp8266_at_device::passthrough_byte(u8 byte)
{
	if (m_uart_stalled)
	{
		if (m_uart_fifo.size() < UART_FIFO)
			m_uart_fifo.push_back(byte);
		return;
	}
	m_passthrough_buf.push_back(byte);
	if (m_passthrough_buf.size() >= m_fw->passthrough_packet)
		passthrough_flush(0);
	else
		m_passthrough_timer->adjust(attotime::from_msec(RX_BATCH_GAP_MS));
}


TIMER_CALLBACK_MEMBER(esp8266_at_device::uart_resume)
{
	m_uart_stalled = false;
	std::vector<u8> const fifo = std::move(m_uart_fifo);
	m_uart_fifo.clear();
	for (u8 const byte : fifo)
	{
		if (m_mode == MODE_PASSTHROUGH)
			passthrough_byte(byte);
	}
}


void esp8266_at_device::send_byte(u8 byte)
{
	if (m_fw->send_echo && m_echo && (byte != '\n'))
		send_raw(&byte, 1);
	m_send_count++;
	if (!m_send_ex)
	{
		m_send_buf.push_back(byte);
	}
	else if (m_send_escape)
	{
		m_send_escape = false;
		if (byte == '0')
		{
			send_payload();
			return;
		}
		m_send_buf.push_back('\\');
		if (byte != '\\')
			m_send_buf.push_back(byte);
	}
	else if (byte == '\\')
	{
		m_send_escape = true;
	}
	else
	{
		m_send_buf.push_back(byte);
	}
	if (m_send_count >= m_send_length)
	{
		if (m_send_escape)
			m_send_buf.push_back('\\');
		send_payload();
	}
}


void esp8266_at_device::send_release()
{
	for (u8 count = std::exchange(m_send_plus, 0); count; count--)
	{
		if (m_mode == MODE_SEND_DATA)
			send_byte('+');
		else
			command_byte('+');
	}
}


void esp8266_at_device::send_cancel()
{
	unsigned const link = m_send_link;
	LOGLINK("link %u: send canceled\n", link);
	if (m_send_kind == SEND_BUF)
	{
		m_seg_sent[link]--;
		if (m_seg_sent[link] == m_seg_done[link])
			m_link_sendbuf[link] = 0;
	}
	m_send_buf.clear();
	m_mode = MODE_COMMAND;
	reply("\r\nSEND Canceled\r\n");
	tcp_deliver_all();
}


void esp8266_at_device::force_restart()
{
	reply("\r\nWill force to restart!!!\r\n");
	reply_ok();
	restart();
}


TIMER_CALLBACK_MEMBER(esp8266_at_device::batch_end)
{
	m_batch_timer->adjust(attotime::never);
	bool const busy = std::exchange(m_busy_batch, false) && !m_fw->send_busy_only;
	if (m_mode == MODE_RESTART)
	{
		m_send_plus = 0;
		m_send_queued = false;
	}
	else if (m_send_plus == 3)
	{
		m_send_plus = 0;
		send_cancel();
	}
	else if (m_send_plus)
	{
		send_release();
	}
	else if (busy && (m_rst_match == 8) && (m_batch_count == 8))
	{
		force_restart();
	}
	else
	{
		if (busy)
			reply("\r\nbusy s...\r\n");
		if (std::exchange(m_send_queued, false))
			send_start();
	}
}


void esp8266_at_device::command_byte(u8 byte)
{
	if (m_echo && (byte != '\n'))
		send_raw(&byte, 1);

	if (m_mode != MODE_COMMAND)
	{
		static char const force[] = "AT+RST\r\n";
		m_rst_match = ((m_rst_match < 8) && (byte == u8(force[m_rst_match]))) ? (m_rst_match + 1) : 0;
		if (m_mode == MODE_SEND_WAIT)
		{
			m_busy_batch = true;
			m_batch_timer->adjust(attotime::from_msec(RX_BATCH_GAP_MS));
			if (m_fw->send_busy_only && (byte == '\n'))
				reply("busy s...\r\n");
		}
		else if ((byte == '\n') && (m_rst_match == 8))
		{
			force_restart();
		}
		else if (byte == '\n')
		{
			reply("\r\nbusy p...\r\n");
		}
		return;
	}

	if (!m_line_open)
	{
		if (((m_head == 'A') && (byte == 'T')) || ((m_head == 'a') && (byte == 't')))
		{
			m_line_open = true;
			m_line.clear();
			m_head = 0;
		}
		else
		{
			if (byte == '\n')
				reply_error();
			m_head = byte;
		}
		return;
	}

	if (byte == '\n')
	{
		if (m_echo)
			reply("\r\n");
		m_line_open = false;
		auto const end = m_line.find('\r');
		execute((end == std::string::npos) ? std::string() : ("AT" + m_line.substr(0, end)));
	}
	else if (m_line.size() >= (MAX_LINE - 1))
	{
		m_line_open = false;
	}
	else
	{
		m_line.push_back(char(byte));
	}
}


std::vector<esp8266_at_device::at_arg> esp8266_at_device::split_args(std::string_view text)
{
	std::vector<at_arg> result;
	std::string current;
	bool quote = false;
	auto const finish =
			[&result, &current] ()
			{
				auto const first = current.find_first_not_of(' ');
				auto const last = current.find_last_not_of(' ');
				std::string trimmed = (first == std::string::npos) ? std::string() : current.substr(first, last - first + 1);
				bool const quoted = (trimmed.size() >= 2) && (trimmed.front() == '"') && (trimmed.back() == '"');
				if (quoted)
					trimmed = trimmed.substr(1, trimmed.size() - 2);
				result.emplace_back(at_arg{ std::move(trimmed), quoted });
				current.clear();
			};

	for (char const c : text)
	{
		if (c == '"')
			quote = !quote;
		if ((c == ',') && !quote)
			finish();
		else
			current.push_back(c);
	}
	finish();
	return result;
}


std::optional<u32> esp8266_at_device::parse_number(at_arg const &arg)
{
	if (arg.quoted || arg.text.empty() || (arg.text.size() > 9))
		return std::nullopt;
	u32 value = 0;
	for (char const c : arg.text)
	{
		if ((c < '0') || (c > '9'))
			return std::nullopt;
		value = (value * 10) + (c - '0');
	}
	return value;
}


void esp8266_at_device::execute(std::string const &line)
{
	LOGCMD("command: %s\n", line);

	if (line == "AT")
	{
		reply_ok();
		return;
	}
	if ((line.size() > 3) && !line.compare(0, 3, "ATE") && (line[3] >= '0') && (line[3] <= '9'))
	{
		std::string_view p = std::string_view(line).substr(3);
		s32 value;
		int err;
		if (fw_int(p, value, err) || ((value != 0) && (value != 1)))
		{
			reply_error();
			return;
		}
		m_echo = u8(value);
		reply_ok();
		return;
	}
	if (line.compare(0, 3, "AT+"))
	{
		reply_error();
		return;
	}

	std::string_view const rest = std::string_view(line).substr(3);
	auto const pos = rest.find_first_of("=?0123456789");
	std::string_view const name = rest.substr(0, pos);
	bool const exec = pos == std::string_view::npos;
	bool const query = !exec && (rest.substr(pos) == "?");
	bool const test = !exec && (rest.substr(pos) == "=?");
	bool const set = !exec && (rest[pos] != '?') && !test;
	std::string_view const text = set ? rest.substr(pos + 1) : std::string_view();
	std::vector<at_arg> const args = set ? split_args(text) : std::vector<at_arg>();
	std::string const shown = '+' + std::string(name);

	for (char const *const *absent = m_fw->absent; *absent; absent++)
	{
		if ((name == *absent) || (rest == *absent))
		{
			reply_error();
			return;
		}
	}

	if (exec && (name == "RST"))
	{
		reply_ok();
		restart();
	}
	else if (exec && (name == "GMR"))
	{
		// no blank line before the OK
		reply(m_fw->gmr_text);
		reply("OK\r\n");
	}
	else if (test && ((name == "CIFSR") || (name == "CIPSEND") || (name == "CIPSENDEX") || (name == "CIPCLOSE")))
	{
		reply_ok();
	}
	else if (test && (name == "CIPSTART"))
	{
		// "(id)" in single-link mode only
		if (m_mux)
			reply(
					"+CIPSTART:(\"type\"),(\"ip address\"),(port)\r\n"
					"+CIPSTART:(\"type\"),(\"domain name\"),(port)\r\n");
		else
			reply(
					"+CIPSTART:(id)(\"type\"),(\"ip address\"),(port)\r\n"
					"+CIPSTART:((id)\"type\"),(\"domain name\"),(port)\r\n");
		reply_ok();
	}
	else if (exec && (name == "CIFSR"))
	{
		if (m_fw->status_commands)
			m_status = STATUS_GOT_IP;
		if (m_cwmode != 2)
			reply(util::string_format("+CIFSR:STAIP,\"%s\"\r\n+CIFSR:STAMAC,\"%s\"\r\n", station_ip(), STATION_MAC));
		reply_ok();
	}
	else if (name == "CIPMUX")
	{
		if (query)
		{
			reply(util::string_format("+CIPMUX:%u\r\n", m_mux));
			reply_ok();
		}
		else if (set && (status_code() == STATUS_CONNECTED))
		{
			// checked before the argument
			reply("link is builded\r\n");
			reply_error();
		}
		else if (set && (args.size() == 1) && ((args[0].text == "0") || (args[0].text == "1")) && !args[0].quoted)
		{
			u8 const mux = args[0].text[0] - '0';
			bool const links = std::find_if(std::begin(m_link_state), std::end(m_link_state), [] (u8 s) { return s != LINK_IDLE; }) != std::end(m_link_state);
			if (mux && m_cipmode)
			{
				reply("IPMODE must be 0\r\n");
				reply_error();
			}
			else if (!mux && m_server_port)
			{
				reply("CIPSERVER must be 0\r\n");
				reply_error();
			}
			else if (!mux && m_mux && links)
			{
				reply("Connection exists\r\n");
				reply_error();
			}
			else
			{
				m_mux = mux;
				reply_ok();
			}
		}
		else
		{
			reply_error();
		}
	}
	else if (name == "CIPMODE")
	{
		if (query)
		{
			reply(util::string_format("+CIPMODE:%u\r\n", m_cipmode));
			reply_ok();
		}
		else if (set && m_mux)
		{
			// checked before the argument
			reply("CIPMUX and CIPSERVER must be 0\r\n");
			reply_error();
		}
		else if (set && (args.size() == 1) && ((args[0].text == "0") || (args[0].text == "1")) && !args[0].quoted)
		{
			m_cipmode = args[0].text[0] - '0';
			if (m_fw->cipmode_stored)
				store_cipmode();
			reply_ok();
		}
		else
		{
			reply_error();
		}
	}
	else if (set && (name == "CIPSTART"))
	{
		cmd_cipstart(args);
	}
	else if ((exec || set) && (name == "CIPSEND"))
	{
		cmd_cipsend(set ? &text : nullptr, SEND_PLAIN, !set || (rest[pos] == '='));
	}
	else if (set && (name == "CIPSENDEX"))
	{
		cmd_cipsend(&text, SEND_EX, rest[pos] == '=');
	}
	else if (set && (name == "CIPSENDBUF"))
	{
		cmd_cipsend(&text, SEND_BUF, rest[pos] == '=');
	}
	else if ((exec || set) && (name == "CIPBUFRESET"))
	{
		cmd_cipbufreset(set ? &text : nullptr);
	}
	else if ((exec || set) && (name == "CIPBUFSTATUS"))
	{
		cmd_cipbufstatus(set ? &text : nullptr);
	}
	else if (set && ((name == "CIPCHECKSEQ") || (name == "CIPCHECKQUEUE")))
	{
		cmd_cipcheckseq(text);
	}
	else if (name == "CIPSSLSIZE")
	{
		if (query)
		{
			reply(util::string_format("%s:%d\r\n", shown, m_ssl_size));
			reply_ok();
		}
		else if (set)
		{
			cmd_cipsslsize(text);
		}
		else
		{
			reply_error();
		}
	}
	else if (query && (name == "CIPALIVE"))
	{
		cmd_cipalive();
	}
	else if (set && (name == "CIPDOMAIN"))
	{
		cmd_cipdomain(text);
	}
	else if (set && (name == "SAVETRANSLINK"))
	{
		cmd_savetranslink(text);
	}
	else if (name == "CIPDINFO")
	{
		if (query)
		{
			// no blank line before the OK
			reply(util::string_format("+CIPDINFO:%s\r\nOK\r\n", m_dinfo ? "TRUE" : "FALSE"));
		}
		else if (set && (args.size() == 1) && ((args[0].text == "0") || (args[0].text == "1")) && !args[0].quoted)
		{
			m_dinfo = args[0].text[0] - '0';
			reply_ok();
		}
		else
		{
			reply_error();
		}
	}
	else if (set && (name == "CIPSERVER"))
	{
		cmd_cipserver(args);
	}
	else if (name == "CIPSTO")
	{
		auto const timeout = (set && (args.size() == 1)) ? parse_number(args[0]) : std::nullopt;
		if (query)
		{
			reply(util::string_format("+CIPSTO:%u\r\n", m_server_timeout));
			reply_ok();
		}
		else if (timeout && (*timeout <= 7200) && m_server_port)
		{
			m_server_timeout = u16(*timeout);
			reply_ok();
		}
		else
		{
			reply_error();
		}
	}
	else if ((exec || set) && (name == "CIPCLOSE"))
	{
		cmd_cipclose(set ? &args : nullptr);
	}
	else if ((name == "UART_CUR") || (name == "UART_DEF"))
	{
		if (set)
		{
			cmd_uart(args, name == "UART_DEF");
		}
		else
		{
			reply_error();
		}
	}
	else if (set && (name == "UART"))
	{
		cmd_uart(args, true);
	}
	else if ((name == "CWMODE") || (name == "CWMODE_CUR") || (name == "CWMODE_DEF"))
	{
		auto const mode = (set && (args.size() == 1)) ? parse_number(args[0]) : std::nullopt;
		if (test)
		{
			reply(shown + ":(1-3)\r\n");
			reply_ok();
		}
		else if (query)
		{
			reply(util::string_format("%s:%u\r\n", shown, (name == "CWMODE_DEF") ? m_cwmode_def : m_cwmode));
			reply_ok();
		}
		else if (mode && (*mode >= 1) && (*mode <= 3))
		{
			m_cwmode = u8(*mode);
			if (m_cwmode == 2)
				m_joined = 0;
			if (name != "CWMODE_CUR")
			{
				m_cwmode_def = m_cwmode;
				m_wifi_stored = true;
			}
			reply_ok();
		}
		else
		{
			reply_error();
		}
	}
	else if ((name == "CWJAP") || (name == "CWJAP_CUR") || (name == "CWJAP_DEF"))
	{
		if (query)
		{
			char const *const ssid = (name == "CWJAP_DEF") ? m_ssid_def : m_ssid;
			if (!m_joined)
			{
				reply("No AP\r\n");
				reply_ok();
			}
			else if (!ssid[0])
			{
				reply_error();
			}
			else if (m_fw->ap_ssid_only)
			{
				m_status = STATUS_GOT_IP;
				reply(util::string_format("%s:\"%s\"\r\n", shown, ssid));
				reply_ok();
			}
			else
			{
				reply(util::string_format("%s:\"%s\",\"%s\",%d,%d\r\n", shown, ssid, AP_BSSID, AP_CHANNEL, AP_RSSI));
				reply_ok();
			}
		}
		else if (set)
		{
			cmd_cwjap(text, name != "CWJAP_CUR");
		}
		else
		{
			reply_error();
		}
	}
	else if (name == "CWLAP")
	{
		if (exec || set)
			cmd_cwlap(set ? &text : nullptr);
		else
			reply_error();
	}
	else if (name == "CWQAP")
	{
		if (test)
		{
			reply_ok();
		}
		else if (exec)
		{
			reply_ok();
			station_off();
			if (m_fw->status_commands)
				m_status = STATUS_NO_WIFI;
		}
		else
		{
			reply_error();
		}
	}
	else if ((name == "CWDHCP") || (name == "CWDHCP_CUR") || (name == "CWDHCP_DEF"))
	{
		if (query)
		{
			// no line end before the OK
			reply(util::string_format("%s:%u", shown, (name == "CWDHCP_DEF") ? m_dhcp_def : m_dhcp));
			reply_ok();
		}
		else if (set)
		{
			cmd_cwdhcp(args, name != "CWDHCP_CUR");
		}
		else
		{
			reply_error();
		}
	}
	else if ((name == "CIPSTA") || (name == "CIPSTA_CUR") || (name == "CIPSTA_DEF"))
	{
		if (query)
		{
			u8 info[12];
			if (name == "CIPSTA_DEF")
			{
				std::fill(std::begin(info), std::end(info), 0);
				if (m_sta_static_def)
					std::copy(std::begin(m_sta_ip_def), std::end(m_sta_ip_def), std::begin(info));
			}
			else
			{
				station_info(info);
			}
			if (m_fw->cipsta_ip_only)
				reply(util::string_format("%s:\"%s\"\r\n", shown, ip_text(&info[0])));
			else
				reply(util::string_format(
						"%1$s:ip:\"%2$s\"\r\n%1$s:gateway:\"%3$s\"\r\n%1$s:netmask:\"%4$s\"\r\n",
						shown, ip_text(&info[0]), ip_text(&info[4]), ip_text(&info[8])));
			reply_ok();
		}
		else if (set)
		{
			cmd_cipsta(text, name != "CIPSTA_CUR");
		}
		else
		{
			reply_error();
		}
	}
	else if ((test || exec) && (name == "CIPSTATUS"))
	{
		if (test)
			reply_ok();
		else
			cmd_cipstatus();
	}
	else if (name == "SLEEP")
	{
		auto const type = (set && (args.size() == 1)) ? parse_number(args[0]) : std::nullopt;
		if (query)
		{
			reply(util::string_format("+SLEEP:%u\r\n", m_sleep));
			reply_ok();
		}
		else if (type && (*type <= 2))
		{
			m_sleep = u8(*type);
			reply_ok();
		}
		else
		{
			reply_error();
		}
	}
	else if (set && (name == "PING"))
	{
		cmd_ping(text);
	}
	else if (exec && (name == "CIPAPPUP") && m_fw->cipappup)
	{
	}
	else
	{
		reply_error();
	}
}


void esp8266_at_device::cmd_cipstart(std::vector<at_arg> const &args)
{
	if ((m_cwmode == 1) && !m_joined)
	{
		reply("no ip\r\n");
		reply_error();
		return;
	}

	unsigned first = 0;
	unsigned link = 0;
	if (m_mux)
	{
		auto const id = args.empty() ? std::nullopt : parse_number(args[0]);
		if (!id || (args.size() < 2))
		{
			reply_error();
			return;
		}
		if (*id >= LINK_COUNT)
		{
			reply("ID ERROR\r\n");
			reply_error();
			return;
		}
		link = *id;
		first = 1;
	}

	at_arg const *const type = (args.size() > first) ? &args[first] : nullptr;
	if (!type || !type->quoted || ((type->text != "TCP") && (type->text != "UDP")))
	{
		reply(m_fw->link_type_error);
		reply_error();
		return;
	}
	if (args.size() < (first + 2))
	{
		reply_error();
		return;
	}
	at_arg const &remote = args[first + 1];
	if (!remote.quoted || remote.text.empty() || (remote.text.size() > 64))
	{
		reply("IP ERROR\r\n");
		reply_error();
		return;
	}
	if (args.size() < (first + 3))
	{
		reply("ENTRY ERROR\r\n");
		reply_error();
		return;
	}

	esp8266_net::open_params params;
	params.udp = type->text == "UDP";

	auto const port = empty_arg(args[first + 2]) ? std::optional<u32>(0) : parse_number(args[first + 2]);
	unsigned const extra = args.size() - first - 3;
	if (!port || (*port > 0xffff))
	{
		reply_error();
		return;
	}
	if (!*port && params.udp && !extra)
	{
		reply("Miss param\r\n");
		reply_error();
		return;
	}
	params.host = args[first + 1].text;
	// a UDP remote port 0 becomes espconn_port(), 1024 to 49999
	params.port = (*port || !params.udp) ? u16(*port) : u16(1024 + (machine().rand() % 48'976));

	if (!params.udp)
	{
		if (extra > (m_fw->cipstart_checks ? 0 : 1))
		{
			reply_error();
			return;
		}
		if (extra)
		{
			at_arg const &arg = args[first + 3];
			auto const keepalive = (m_fw->keepalive_empty && (empty_arg(arg) || (!arg.quoted && (arg.text == "-")))) ? std::optional<u32>(0) : parse_number(arg);
			if (!keepalive || (*keepalive > 7200))
			{
				reply_error();
				return;
			}
			params.keepalive = u16(*keepalive);
		}
	}
	else
	{
		if (extra > 2)
		{
			reply_error();
			return;
		}
		if (extra)
		{
			at_arg const &arg = args[first + 3];
			auto const local = (empty_arg(arg) && !m_fw->cipstart_checks) ? std::optional<u32>(0) : parse_number(arg);
			if (!local || (*local > 0xffff) || (!*local && !empty_arg(arg)))
			{
				reply_error();
				return;
			}
			params.local_port = u16(*local);
		}
		if (extra > 1)
		{
			auto const mode = empty_arg(args[first + 4]) ? std::optional<u32>(0) : parse_number(args[first + 4]);
			if (!mode || (*mode > 2))
			{
				reply_error();
				return;
			}
			params.udp_mode = u8(*mode);
		}
	}

	if (m_link_state[link] != LINK_IDLE)
	{
		reply("ALREADY CONNECTED\r\n");
		reply_error();
		return;
	}

	m_link_params[link] = params;
	m_connect_link = link;
	m_connect_named = params.udp && (m_fw->udp_connect_id || ((fw_ipaddr(params.host) == IPADDR_NONE) && (params.host != "255.255.255.255")));
	m_mode = MODE_CONNECTING;
	link_open(link, 0);
}


void esp8266_at_device::cmd_cipsend(std::string_view const *text, u8 kind, bool equals)
{
	if (!text)
	{
		if (!m_cipmode || m_mux || (m_link_state[0] != LINK_OPEN))
		{
			reply_error();
			return;
		}
		reply("\r\nOK\r\n\r\n>");
		m_passthrough_buf.clear();
		m_mode = MODE_PASSTHROUGH;
		LOGLINK("link 0: passthrough on\n");
		return;
	}

	if (m_cipmode)
	{
		reply("IPMODE=1\r\n");
		reply_error();
		return;
	}
	if (!equals)
	{
		reply_error();
		return;
	}
	std::string_view p = *text;
	unsigned link = 0;
	if (m_mux && !link_number(p, link))
	{
		reply_error();
		return;
	}
	if (m_mux)
	{
		if (p.empty() || (p.front() != ','))
		{
			reply_error();
			return;
		}
		p.remove_prefix(1);
	}

	// the module checks the link before the length
	if (m_link_state[link] != LINK_OPEN)
	{
		reply("link is not valid\r\n");
		reply_error();
		return;
	}
	if (m_link_sendbuf[link] && (kind != SEND_BUF))
	{
		reply("send mode error\r\n");
		reply_error();
		return;
	}
	s32 length;
	int err;
	fw_int(p, length, err);
	if (!p.empty() && m_fw->send_checks)
	{
		reply("Too many inputs.\r\n");
		reply_error();
		return;
	}
	if (length > s32(MAX_SEND))
	{
		reply("too long\r\n");
		reply_error();
		return;
	}
	std::string remote;
	if (!p.empty() && !m_link_tcp[link] && (p.front() == ','))
	{
		p.remove_prefix(1);
		if (fw_string(p, remote, 64) < 1)
		{
			reply("ip error\r\n");
			reply_error();
			return;
		}
		if (p.empty() || (p.front() != ','))
		{
			reply("no port\r\n");
			reply_error();
			return;
		}
		p.remove_prefix(1);
		s32 port;
		fw_int(p, port, err);
		if (err)
		{
			reply("port error\r\n");
			reply_error();
			return;
		}
		if ((port < 1) || (port > 0xffff))
		{
			reply("port range error\r\n");
			reply_error();
			return;
		}
		m_send_port = u16(port);
	}
	if (!p.empty())
	{
		reply("no tail\r\n");
		reply_error();
		return;
	}
	if (length <= 0)
	{
		reply_error();
		return;
	}

	if (kind == SEND_BUF)
	{
		if (!m_link_tcp[link])
		{
			reply_error();
			return;
		}
		if (unsigned(length) > send_space(link))
		{
			reply("BUFFER FULL\r\n");
			reply_error();
			return;
		}
		if (!send_queue_space(link))
		{
			reply("QUEUE FULL\r\n");
			reply_error();
			return;
		}

		if (!++m_seg_sent[link])
			m_seg_sent[link]++;
		reply(util::string_format("%u,%u\r\n", m_seg_sent[link], m_seg_done[link]));
		m_link_sendbuf[link] = 1;
	}

	m_send_cur = link;
	m_send_kind = kind;
	m_send_link = link;
	m_send_length = length;
	m_send_count = 0;
	m_send_ex = kind == SEND_EX;
	m_send_escape = false;
	m_send_buf.clear();
	m_send_buf.reserve(m_send_length);
	m_link_active[link] = machine().time();
	if (remote.empty())
	{
		send_prompt();
		return;
	}

	u32 const ip = fw_ipaddr(remote);
	if ((ip != IPADDR_NONE) || (remote == "255.255.255.255"))
	{
		send_remote(util::string_format("%u.%u.%u.%u", BIT(ip, 24, 8), BIT(ip, 16, 8), BIT(ip, 8, 8), BIT(ip, 0, 8)));
		send_prompt();
		return;
	}
	m_mode = MODE_SEND_RESOLVE;
	m_net->resolve(++m_domain_generation, remote);
}


void esp8266_at_device::send_prompt()
{
	m_batch_count = 0;
	m_mode = MODE_SEND_DATA;
	reply("\r\nOK\r\n> ");
}


void esp8266_at_device::send_remote(std::string const &address)
{
	unsigned const link = m_send_link;
	m_link_remote[link] = address;
	m_link_remote_port[link] = m_send_port;
	m_net->udp_remote(link, m_link_generation[link], address, m_send_port);
}


void esp8266_at_device::send_payload()
{
	m_mode = MODE_SEND_WAIT;
	m_send_queued = true;
	m_batch_timer->adjust(attotime::from_hz(m_cur_rate) * RX_TOUT_BITS);
}


void esp8266_at_device::send_start()
{
	unsigned const link = m_send_link;
	unsigned const length = m_send_buf.size();
	LOGLINK("link %u: sending %u bytes\n", link, length);
	if (!m_fw->send_echo)
		reply(util::string_format("\r\nRecv %u bytes\r\n", length));

	if (m_send_kind == SEND_BUF)
	{
		m_mode = MODE_COMMAND;
		if (m_link_state[link] != LINK_OPEN)
		{
			m_seg_map[link] = (m_seg_map[link] << 1) & ~u32(1);
			if (m_mux)
				reply(util::string_format("\r\n%u,%u,SEND FAIL\r\n", link, m_seg_sent[link]));
			else
				reply(util::string_format("\r\n%u,SEND FAIL\r\n", m_seg_sent[link]));
			if (!++m_seg_done[link])
				m_seg_done[link]++;
			if (m_seg_sent[link] == m_seg_done[link])
				m_link_sendbuf[link] = 0;
			m_send_buf.clear();
			tcp_deliver_all();
			return;
		}
		m_seg_map[link] = (m_seg_map[link] << 1) & ~u32(1);
	}

	m_unacked[link].emplace_back(length, m_send_kind == SEND_BUF);
	m_net->send(link, m_link_generation[link], std::move(m_send_buf), true);
	m_send_buf.clear();

	if (m_link_tcp[link] && (m_link_state[link] == LINK_OPEN))
		tcp_ack(link);
	tcp_deliver_all();
}


void esp8266_at_device::sendbuf_ack(unsigned link)
{
	if (m_cipmode)
		return;
	if (!++m_seg_done[link])
		m_seg_done[link]++;
	unsigned const cur = m_send_cur;
	m_seg_map[cur] |= u32(1) << ((m_seg_sent[cur] - m_seg_done[cur]) & 31);
	if (m_mux)
		reply(util::string_format("\r\n%u,%u,SEND OK\r\n", link, m_seg_done[link]));
	else
		reply(util::string_format("\r\n%u,SEND OK\r\n", m_seg_done[link]));
	m_seg_ok[link] = m_seg_done[link];
	if (m_seg_sent[link] == m_seg_done[link])
		m_link_sendbuf[link] = 0;
}


void esp8266_at_device::sendbuf_fail(unsigned link)
{
	m_unacked[link].clear();
	if (!m_link_sendbuf[link])
		return;
	while (m_seg_done[link] != m_seg_sent[link])
	{
		if (!++m_seg_done[link])
			m_seg_done[link]++;
		if (m_mux)
			reply(util::string_format("%u,%u,SEND FAIL\r\n", link, m_seg_done[link]));
		else
			reply(util::string_format("%u,SEND FAIL\r\n", m_seg_done[link]));
	}
	m_link_sendbuf[link] = 0;
}


unsigned esp8266_at_device::send_space(unsigned link) const
{
	unsigned used = 0;
	for (auto const &write : m_unacked[link])
		used += write.first;
	return (used < TCP_SND_BUF) ? (TCP_SND_BUF - used) : 0;
}


unsigned esp8266_at_device::send_queue_space(unsigned link) const
{
	unsigned used = 0;
	for (auto const &write : m_unacked[link])
		used += (write.first + ESP_TCP_MSS - 1) / ESP_TCP_MSS;
	return (used < TCP_SND_QUEUELEN) ? (TCP_SND_QUEUELEN - used) : 0;
}


bool esp8266_at_device::link_number(std::string_view &p, unsigned &link)
{
	s32 id;
	int err;
	fw_int(p, id, err);
	if (err || (id < 0) || (id >= s32(LINK_COUNT)))
		return false;
	link = unsigned(id);
	return true;
}


void esp8266_at_device::cmd_cipbufreset(std::string_view const *text)
{
	if (m_cipmode)
	{
		reply("IPMODE=1\r\n");
		reply_error();
		return;
	}
	unsigned link = 0;
	if (text)
	{
		std::string_view p = *text;
		if (!m_mux || !link_number(p, link) || !p.empty())
		{
			reply_error();
			return;
		}
	}
	else if (m_mux)
	{
		reply_error();
		return;
	}
	if (m_link_state[link] != LINK_OPEN)
	{
		reply("link is not valid\r\n");
		reply_error();
		return;
	}

	if (!m_link_tcp[link] || (m_seg_sent[link] != m_seg_done[link]))
	{
		reply_error();
		return;
	}
	m_seg_sent[link] = m_seg_done[link] = m_seg_ok[link] = m_seg_map[link] = 0;
	reply_ok();
}


void esp8266_at_device::cmd_cipbufstatus(std::string_view const *text)
{
	if (m_cipmode)
	{
		reply("IPMODE=1\r\n");
		reply_error();
		return;
	}
	unsigned link = 0;
	if (text)
	{
		std::string_view p = *text;
		if (!m_mux || !link_number(p, link) || !p.empty())
		{
			reply_error();
			return;
		}
	}
	else if (m_mux)
	{
		reply_error();
		return;
	}

	unsigned space = 0, queue = 0;
	if (m_link_state[link] == LINK_OPEN)
	{
		if (!m_link_tcp[link])
		{
			reply_error();
			return;
		}
		space = send_space(link);
		queue = send_queue_space(link);
	}
	u32 const next = m_seg_sent[link] + 1;
	std::string const id = (text && m_fw->bufstatus_id) ? util::string_format("%u,", link) : std::string();
	reply(util::string_format("%s%u,%u,%u,%u,%u\r\n", id, next ? next : 1, m_seg_done[link], m_seg_ok[link], space, queue));
	reply_ok();
}


void esp8266_at_device::cmd_cipcheckseq(std::string_view text)
{
	if (m_cipmode)
	{
		reply("IPMODE=1\r\n");
		reply_error();
		return;
	}
	std::string_view p = text;
	unsigned link = 0;
	if (m_mux)
	{
		if (!link_number(p, link) || p.empty() || (p.front() != ','))
		{
			reply_error();
			return;
		}
		p.remove_prefix(1);
	}
	s32 value;
	int err;
	fw_int(p, value, err);
	if (err || !p.empty() || ((m_link_state[link] == LINK_OPEN) && !m_link_tcp[link]) || !value)
	{
		reply_error();
		return;
	}

	u32 const segment = u32(value);
	u32 const last = m_seg_sent[link];
	u32 const low = (last < 32) ? (last - 33) : (last - 32);
	s32 shift;
	if (low < last)
	{
		if ((last < segment) || (low >= segment))
		{
			reply_error();
			return;
		}
		shift = s32(last - segment);
	}
	else if (last >= segment)
	{
		shift = s32(last - segment);
	}
	else if (low < segment)
	{
		shift = s32(last - segment - 1);
	}
	else
	{
		reply_error();
		return;
	}
	if ((shift < 0) || (shift >= 32))
	{
		reply_error();
		return;
	}

	char const *const acked = BIT(m_seg_map[link], shift) ? "TRUE" : "FALSE";
	if (m_mux)
		reply(util::string_format("%u,%u,%s\r\n", link, segment, acked));
	else
		reply(util::string_format("%u,%s\r\n", segment, acked));
	reply_ok();
}


void esp8266_at_device::cmd_cipserver(std::vector<at_arg> const &args)
{
	auto const mode = args.empty() ? std::nullopt : parse_number(args[0]);
	auto const port = (args.size() == 2) ? parse_number(args[1]) : std::optional<u32>(DEFAULT_SERVER_PORT);
	if (!m_mux || !mode || (*mode > 1) || (args.size() > 2) || !port || !*port || (*port > 0xffff))
	{
		reply_error();
		return;
	}

	if (!*mode)
	{
		if (!m_server_port)
			reply("no change\r\n");
		server_close();
		reply_ok();
		for (unsigned i = 0; i < LINK_COUNT; i++)
		{
			if (m_link_server[i] && (m_link_state[i] == LINK_OPEN))
			{
				m_link_closing[i] = CLOSE_SERVER;
				m_net->shutdown(i, m_link_generation[i]);
			}
		}
	}
	else if (m_server_port)
	{
		reply("no change\r\n");
		reply_ok();
	}
	else
	{
		ioport_value const config = m_server_config->read();
		unsigned const host_port = *port + SERVER_PORT_OFFSETS[BIT(config, 4, 3)];
		LOGLINK("server: port %u, host port %u\n", *port, host_port);
		m_server_port = u16(*port);
		m_mode = MODE_LISTEN;
		m_net->listen(++m_server_generation, BIT(config, 0), host_port);
	}
}


void esp8266_at_device::cmd_cipclose(std::vector<at_arg> const *args)
{
	if (!args)
	{
		if (m_mux)
		{
			reply("MUX=1\r\n");
			reply_error();
			return;
		}
		if (m_link_state[0] == LINK_IDLE)
		{
			reply_error();
			return;
		}
		if (close_wait(0))
			return;
		link_close(0);
		link_gone();
		reply("CLOSED\r\n");
		reply_ok();
		sendbuf_fail(0);
		return;
	}

	auto const id = (args->size() == 1) ? parse_number((*args)[0]) : std::nullopt;
	if (id && !m_mux)
	{
		reply("MUX=0\r\n");
		reply_error();
		return;
	}
	if (!id || (*id > LINK_COUNT))
	{
		reply_error();
		return;
	}

	if ((*id == LINK_COUNT) && m_fw->close_waits)
	{
		if (!close_chain(0))
			reply_ok();
	}
	else if (*id == LINK_COUNT)
	{
		bool closed[LINK_COUNT] = { false, false, false, false, false };
		bool waiting = false;
		for (unsigned i = 0; i < LINK_COUNT; i++)
		{
			if ((m_link_state[i] != LINK_IDLE) && close_wait(i))
			{
				waiting = true;
			}
			else if (m_link_state[i] != LINK_IDLE)
			{
				link_close(i);
				closed_notice(i);
				link_gone();
				closed[i] = true;
			}
		}
		if (!waiting)
			reply_ok();

		for (unsigned i = 0; i < LINK_COUNT; i++)
		{
			if (closed[i])
				sendbuf_fail(i);
		}
	}
	else if (m_link_state[*id] == LINK_IDLE)
	{
		reply(m_fw->unlink_text);
		reply_error();
	}
	else if (!close_wait(*id))
	{
		link_close(*id);
		closed_notice(*id);
		link_gone();
		reply_ok();
		sendbuf_fail(*id);
	}
}


void esp8266_at_device::cmd_uart(std::vector<at_arg> const &args, bool def)
{
	if (args.size() != 5)
	{
		reply_error();
		return;
	}
	std::optional<u32> values[5];
	for (unsigned i = 0; i < 5; i++)
	{
		values[i] = parse_number(args[i]);
		if (!values[i])
		{
			reply_error();
			return;
		}
	}
	if ((*values[0] < 110) || (*values[0] > CARD_MAX_BAUD) ||
			(*values[1] < 5) || (*values[1] > 8) ||
			(*values[2] < 1) || (*values[2] > 3) ||
			(*values[3] > 2) ||
			(*values[4] > 3))
	{
		reply_error();
		return;
	}

	m_cur_rate = *values[0];
	for (unsigned i = 0; i < 4; i++)
		m_cur_frame[i] = u8(*values[i + 1]);
	if (def)
	{
		m_def_stored = true;
		m_def_rate = m_cur_rate;
		std::copy(std::begin(m_cur_frame), std::end(m_cur_frame), std::begin(m_def_frame));
	}

	reply_ok();
	m_serial_dirty = true;
}


void esp8266_at_device::cmd_cwjap(std::string_view text, bool def)
{
	std::string ssid, password, bssid;
	u8 mac[6];
	bool ok = (m_cwmode != 2) && take_string(text, ssid, MAX_SSID) && !text.empty() && (text.front() == ',');
	if (ok)
	{
		text.remove_prefix(1);
		ok = take_string(text, password, 64);
	}
	if (ok && !text.empty() && !m_fw->ap_ssid_only)
	{
		ok = (text.front() == ',');
		text.remove_prefix(1);
		ok = ok && take_string(text, bssid, 17) && parse_mac(bssid, mac);
	}
	if (!ok || !text.empty() || ssid.empty())
	{
		reply_error();
		return;
	}

	station_off();

	std::fill(std::begin(m_ssid), std::end(m_ssid), 0);
	std::copy(ssid.begin(), ssid.end(), std::begin(m_ssid));
	if (def)
	{
		std::copy(std::begin(m_ssid), std::end(m_ssid), std::begin(m_ssid_def));
		m_wifi_stored = true;
	}

	LOGLINK("station joined \"%s\"\n", ssid);
	m_joined = 1;
	if (m_fw->status_commands)
		m_status = STATUS_NO_WIFI;
	else if (m_status == STATUS_NO_WIFI)
		m_status = STATUS_GOT_IP;
	if (!m_fw->quiet_wifi && !m_cipmode)
		reply("WIFI CONNECTED\r\nWIFI GOT IP\r\n");

	m_mode = MODE_JOIN;
	m_join_timer->adjust(attotime::from_msec(CWJAP_TICK_MS));
}


TIMER_CALLBACK_MEMBER(esp8266_at_device::join_tick)
{
	if (m_mode == MODE_SCAN)
	{
		reply(m_scan_text);
		m_scan_text.clear();
	}
	else if (m_mode != MODE_JOIN)
	{
		return;
	}
	reply_ok();
	m_mode = MODE_COMMAND;
}


void esp8266_at_device::cmd_cwlap(std::string_view const *text)
{
	std::string ssid, mac_text;
	u8 mac[6];
	bool want_mac = false;
	std::optional<u32> channel = 0;
	bool ok = m_cwmode != 2;
	if (ok && text)
	{
		std::string_view rest = *text;
		ok = !rest.empty();
		if (ok && (rest.front() != ','))
			ok = take_string(rest, ssid, MAX_SSID);
		if (ok && !rest.empty())
		{
			ok = rest.front() == ',';
			rest.remove_prefix(1);
			ok = ok && take_string(rest, mac_text, 17) && (mac_text.empty() || parse_mac(mac_text, mac));
			want_mac = !mac_text.empty();
		}
		if (ok && !rest.empty())
		{
			ok = rest.front() == ',';
			rest.remove_prefix(1);
			channel = parse_number(at_arg{ std::string(rest), false });
			ok = ok && channel && (*channel <= 13);
			rest = std::string_view();
		}
		ok = ok && rest.empty();
	}
	if (!ok)
	{
		reply_error();
		return;
	}

	bool const match =
			(ssid.empty() || (ssid == m_ssid)) &&
			(!want_mac || (util::string_format("%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) == AP_BSSID)) &&
			(!*channel || (*channel == AP_CHANNEL));
	if (match && m_ssid[0])
	{
		if (m_fw->cwlap_five)
			m_scan_text = util::string_format("+CWLAP:(%d,\"%s\",%d,\"%s\",%d)\r\n", AP_ECN, m_ssid, AP_RSSI, AP_BSSID, AP_CHANNEL);
		else
			m_scan_text = util::string_format("+CWLAP:(%d,\"%s\",%d,\"%s\",%d,%d%s)\r\n", AP_ECN, m_ssid, AP_RSSI, AP_BSSID, AP_CHANNEL, 0, m_fw->cwlap_cal ? ",0" : "");
	}
	else
	{
		m_scan_text.clear();
	}

	m_mode = MODE_SCAN;
	m_join_timer->adjust(attotime::from_msec(CWLAP_SCAN_MS));
}


void esp8266_at_device::cmd_cwdhcp(std::vector<at_arg> const &args, bool def)
{
	auto const mode = (args.size() == 2) ? parse_number(args[0]) : std::nullopt;
	auto const enable = (args.size() == 2) ? parse_number(args[1]) : std::nullopt;
	if (!mode || (*mode > 2) || !enable || (*enable > 1))
	{
		reply_error();
		return;
	}

	u8 const mask = (*mode == 0) ? 1 : (*mode == 1) ? 2 : 3;
	if (*enable)
	{
		m_dhcp |= mask;
		if (BIT(mask, 1))
			m_sta_static = 0;
	}
	else
	{
		m_dhcp &= ~mask;
	}

	if (!BIT(m_dhcp, 1) && m_sta_static_def)
	{
		std::copy(std::begin(m_sta_ip_def), std::end(m_sta_ip_def), std::begin(m_sta_ip));
		m_sta_static = 1;
	}

	if (def)
	{
		m_dhcp_def = m_dhcp;
		m_wifi_stored = true;
	}
	reply_ok();
}


void esp8266_at_device::cmd_cipsta(std::string_view text, bool def)
{
	std::string item;
	std::optional<u32> values[3] = { std::nullopt, 0, 0 };
	bool ok = take_string(text, item, m_fw->cipsta_ip_only ? 32 : 31) && !item.empty();
	if (ok)
		ok = bool(values[0] = parse_ip(item));
	if (ok && m_fw->cipsta_ip_only)
	{
		u8 info[12];
		station_info(info);
		values[1] = get_u32be(&info[4]);
		values[2] = get_u32be(&info[8]);
		ok = text.empty();
	}
	for (unsigned i = 1; ok && (i < 3) && !text.empty(); i++)
	{
		ok = text.front() == ',';
		text.remove_prefix(1);
		ok = ok && take_string(text, item, 31);
		if (ok && !item.empty())
			ok = bool(values[i] = parse_ip(item));
	}
	if (!ok || !text.empty())
	{
		reply_error();
		return;
	}

	u32 const ip = *values[0];
	u32 gateway = *values[1];
	u32 netmask = *values[2];
	if (ip && !gateway)
		gateway = (ip & 0xffff'ff00) | 1;
	if (ip && !netmask)
		netmask = 0xffff'ff00;
	put_u32be(&m_sta_ip[0], ip);
	put_u32be(&m_sta_ip[4], gateway);
	put_u32be(&m_sta_ip[8], netmask);
	m_sta_static = 1;
	m_dhcp &= ~2;

	if (def)
	{
		std::copy(std::begin(m_sta_ip), std::end(m_sta_ip), std::begin(m_sta_ip_def));
		m_sta_static_def = 1;
		m_dhcp_def = m_dhcp;
		m_wifi_stored = true;
	}
	reply_ok();
}


u8 esp8266_at_device::status_code() const
{
	if (m_fw->status_commands)
		return m_status;
	if (!m_joined)
		return STATUS_NO_WIFI;
	return (m_status == STATUS_NO_WIFI) ? STATUS_GOT_IP : m_status;
}


void esp8266_at_device::cmd_cipstatus()
{
	m_status = status_code();
	reply(util::string_format("STATUS:%u\r\n", m_status));

	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		if (m_link_state[i] != LINK_OPEN)
			continue;
		std::string const local = (m_fw->status_tcp_local || !m_link_tcp[i]) ? util::string_format("%u,", m_link_local_port[i]) : std::string();
		reply(util::string_format(
				"+CIPSTATUS:%u,\"%s\",\"%s\",%u,%s%u\r\n",
				i,
				m_link_tcp[i] ? "TCP" : "UDP",
				m_link_remote[i],
				m_link_remote_port[i],
				local,
				m_link_server[i] ? 1 : 0));
	}
	reply_ok();
}


void esp8266_at_device::cmd_ping(std::string_view text)
{
	std::string host;
	if (!take_string(text, host, 127) || host.empty() || !text.empty())
	{
		reply_error();
		return;
	}

	LOGLINK("ping %s\n", host);
	m_ping_replied = false;
	m_ping_no_dns = false;
	m_ping_timer->adjust(attotime::never);
	m_mode = MODE_PING;
	if (!m_joined)
	{
		m_ping_no_dns = !parse_ip(host);
		m_ping_timer->adjust(attotime::from_msec(m_ping_no_dns ? PING_NO_DNS_MS : PING_COARSE_MS));
		return;
	}
	m_net->ping(++m_ping_generation, host);
}


TIMER_CALLBACK_MEMBER(esp8266_at_device::ping_done)
{
	if (m_mode != MODE_PING)
		return;
	if (m_fw->rtos)
	{
		if (m_ping_replied && m_ping_ms && (m_ping_ms < RTOS_PING_MS))
		{
			reply(util::string_format("+PING:%u\r\n", m_ping_ms));
			reply_ok();
		}
		else
		{
			if (!m_ping_replied || m_ping_ms)
				reply("+PING:TIMEOUT\r\n");
			reply_error();
		}
		m_mode = MODE_COMMAND;
		stop_ping();
		return;
	}
	if (!m_ping_replied && !m_ping_no_dns)
		reply("+timeout\r\n");
	if (m_ping_replied)
		reply_ok();
	else
		reply_error();
	m_mode = MODE_COMMAND;
	stop_ping();
}


void esp8266_at_device::cmd_cipsslsize(std::string_view text)
{
	s32 size;
	int err;
	fw_int(text, size, err);
	if (err || !text.empty() || (size < 2048) || (size > 4096))
	{
		reply_error();
		return;
	}
	m_ssl_size = u16(size);
	reply_ok();
}


void esp8266_at_device::cmd_cipalive()
{
	unsigned count = 0;
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		if ((m_link_state[i] == LINK_OPEN) && m_link_tcp[i] && !m_link_server[i] && m_link_params[i].keepalive)
		{
			reply(util::string_format("+CIPALIVE:id %u,%u\r\n", i, m_link_params[i].keepalive));
			count++;
		}
	}
	if (!count)
		reply("Not set TCP keep alive\r\n");
	reply_ok();
}


void esp8266_at_device::cmd_cipdomain(std::string_view text)
{
	std::string host;
	if (fw_string(text, host, 64) <= 0)
	{
		reply("IP ERROR\r\n");
		reply_error();
		return;
	}
	if (!text.empty())
	{
		reply_error();
		return;
	}

	u32 const ip = fw_ipaddr(host);
	if ((ip != IPADDR_NONE) || (host == "255.255.255.255"))
	{
		reply(util::string_format("+CIPDOMAIN:%u.%u.%u.%u\r\n", BIT(ip, 24, 8), BIT(ip, 16, 8), BIT(ip, 8, 8), BIT(ip, 0, 8)));
		reply_ok();
		return;
	}

	LOGLINK("AT+CIPDOMAIN %s\n", host);
	m_mode = MODE_DOMAIN;
	if (!m_joined)
		m_domain_timer->adjust(attotime::from_msec(DNS_NO_IP_MS));
	else
		m_net->resolve(++m_domain_generation, host);
}


void esp8266_at_device::cmd_savetranslink(std::string_view text)
{
	s32 mode;
	int err;
	fw_int(text, mode, err);
	if (err || (mode < 0) || (mode > 1))
	{
		reply_error();
		return;
	}

	std::string host;
	s32 port = 0, keepalive = 0, local_port = 0;
	u8 type = TRANSLINK_TCP;
	if (mode)
	{
		bool ok = !text.empty() && (text.front() == ',');
		if (ok)
		{
			text.remove_prefix(1);
			ok = fw_string(text, host, m_fw->translink_names ? MAX_TRANSLINK_HOST : 15) > 0;
		}
		if (ok)
		{
			u32 const ip = fw_ipaddr(host);
			ok = ip && ((ip != IPADDR_NONE) || (m_fw->translink_names && (host != "255.255.255.255")));
		}
		ok = ok && !text.empty() && (text.front() == ',');
		if (ok)
		{
			text.remove_prefix(1);
			fw_int(text, port, err);
			ok = !err && (port >= 1) && (port <= 0xffff);
		}
		if (ok && !text.empty() && (text.front() == ','))
		{
			text.remove_prefix(1);
			std::string name;
			int const length = fw_string(text, name, 4);
			ok = length >= 0;
			if (ok && (name == "UDP"))
				type = TRANSLINK_UDP;
			else if (ok && (name == "SSL") && m_fw->translink_names)
				type = TRANSLINK_SSL;
			else if (ok && (name != "TCP"))
				ok = !length;
			if (ok && !text.empty() && (text.front() == ','))
			{
				text.remove_prefix(1);
				if (type == TRANSLINK_UDP)
				{
					fw_int(text, local_port, err);
					ok = !err && (local_port >= 1) && (local_port <= 0xffff);
				}
				else
				{
					fw_int(text, keepalive, err);
					ok = !err && (keepalive >= 0) && (keepalive <= 7200);
				}
			}
		}
		if (!ok)
		{
			reply_error();
			return;
		}
	}
	if (!text.empty())
	{
		reply_error();
		return;
	}

	m_tl_mode = u8(mode);
	m_tl_type = type;
	m_tl_port = u16(port);
	m_tl_local_port = u16(local_port);
	m_tl_keepalive = u16(keepalive);
	std::fill(std::begin(m_tl_host), std::end(m_tl_host), 0);
	std::copy(host.begin(), host.end(), std::begin(m_tl_host));
	m_translink_stored = true;
	reply_ok();
}


bool esp8266_at_device::fw_int(std::string_view &p, s32 &value, int &err)
{
	value = 0;
	err = 0;
	bool negative = false;
	unsigned count = 0;
	while (count < 10)
	{
		char const c = p.empty() ? '\r' : p.front();
		if ((c >= '0') && (c <= '9'))
			value = s32((u32(value) * 10) + (c - '0'));
		else if (c == '-')
			negative = negative || !count;
		else
			break;
		p.remove_prefix(1);
		count++;
	}
	if (count == 9)
	{
		err = 2;
		return false;
	}
	if (!count)
	{
		err = 1;
		return true;
	}
	if (negative)
	{
		if (count == 1)
		{
			err = 3;
			return false;
		}
		value = -value;
	}
	return !p.empty();
}


int esp8266_at_device::fw_string(std::string_view &p, std::string &out, unsigned max)
{
	out.clear();
	if (p.empty() || (p.front() == ','))
		return 0;
	if (p.front() != '"')
		return -1;
	std::size_t i = 1;
	bool escape = false;
	while (out.size() < max)
	{
		if (i >= p.size())
			return -1;
		char const c = p[i++];
		if ((c == '\\') && !escape)
		{
			escape = true;
			continue;
		}
		if ((c == '"') && !escape)
		{
			p.remove_prefix(i);
			return int(out.size());
		}
		out.push_back(c);
		escape = false;
	}
	if ((i >= p.size()) || (p[i] != '"'))
		return -1;
	p.remove_prefix(i + 1);
	return int(out.size());
}


u32 esp8266_at_device::fw_ipaddr(std::string const &text)
{
	u32 parts[4];
	unsigned count = 0;
	std::size_t i = 0;
	auto const at = [&text] (std::size_t n) { return (n < text.size()) ? text[n] : '\0'; };
	u32 value;
	while (true)
	{
		char c = at(i);
		if ((c < '0') || (c > '9'))
			return IPADDR_NONE;
		value = 0;
		u32 base = 10;
		if (c == '0')
		{
			c = at(++i);
			if ((c == 'x') || (c == 'X'))
			{
				base = 16;
				c = at(++i);
			}
			else
			{
				base = 8;
			}
		}
		u32 const cutoff = 0xffff'ffff / base;
		u32 const cutlim = 0xffff'ffff % base;
		while (true)
		{
			u32 digit;
			if ((c >= '0') && (c <= '9'))
				digit = c - '0';
			else if ((base == 16) && (c >= 'a') && (c <= 'f'))
				digit = c - 'a' + 10;
			else if ((base == 16) && (c >= 'A') && (c <= 'F'))
				digit = c - 'A' + 10;
			else
				break;
			if ((value > cutoff) || ((value == cutoff) && (digit > cutlim)))
				return IPADDR_NONE;
			value = (base == 16) ? ((value << 4) | digit) : ((value * base) + digit);
			c = at(++i);
		}
		if (c != '.')
			break;
		if (count >= 3)
			return IPADDR_NONE;
		parts[count++] = value;
		i++;
	}
	char const tail = at(i);
	if (tail && (tail != ' ') && (tail != '\t') && (tail != '\n') && (tail != '\r') && (tail != '\v') && (tail != '\f'))
		return IPADDR_NONE;
	switch (count)
	{
	case 1:
		if ((value > 0xff'ffff) || (parts[0] > 0xff))
			return IPADDR_NONE;
		value |= parts[0] << 24;
		break;
	case 2:
		if ((value > 0xffff) || (parts[0] > 0xff) || (parts[1] > 0xff))
			return IPADDR_NONE;
		value |= (parts[0] << 24) | (parts[1] << 16);
		break;
	case 3:
		if ((value > 0xff) || (parts[0] > 0xff) || (parts[1] > 0xff) || (parts[2] > 0xff))
			return IPADDR_NONE;
		value |= (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8);
		break;
	}
	return value;
}


bool esp8266_at_device::take_string(std::string_view &text, std::string &out, unsigned max)
{
	out.clear();
	if (text.empty() || (text.front() != '"'))
		return false;
	std::size_t i = 1;
	for ( ; i < text.size(); i++)
	{
		char c = text[i];
		if (c == '"')
			break;
		if ((c == '\\') && ((i + 1) < text.size()))
			c = text[++i];
		out.push_back(c);
	}
	if ((i >= text.size()) || (out.size() > max))
		return false;
	text.remove_prefix(i + 1);
	return true;
}


bool esp8266_at_device::parse_mac(std::string const &text, u8 *mac)
{
	if (text.size() != 17)
		return false;
	for (unsigned i = 0; i < 6; i++)
	{
		if ((i < 5) && (text[(i * 3) + 2] != ':'))
			return false;
		unsigned value = 0;
		for (unsigned j = 0; j < 2; j++)
		{
			char const c = text[(i * 3) + j];
			if ((c >= '0') && (c <= '9'))
				value = (value << 4) | (c - '0');
			else if ((c >= 'a') && (c <= 'f'))
				value = (value << 4) | (c - 'a' + 10);
			else if ((c >= 'A') && (c <= 'F'))
				value = (value << 4) | (c - 'A' + 10);
			else
				return false;
		}
		mac[i] = u8(value);
	}
	return true;
}


std::optional<u32> esp8266_at_device::parse_ip(std::string const &text)
{
	std::error_code err;
	asio::ip::address_v4 const address = asio::ip::make_address_v4(text, err);
	if (err || (address.to_uint() == 0xffff'ffff))
		return std::nullopt;
	return u32(address.to_uint());
}


std::string esp8266_at_device::ip_text(u8 const *ip)
{
	return util::string_format("%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}


void esp8266_at_device::closed_notice(unsigned link)
{
	if (m_mux)
		reply(util::string_format("%u,CLOSED\r\n", link));
	else
		reply("CLOSED\r\n");
}


std::string esp8266_at_device::ipd_text(unsigned link, unsigned length) const
{
	std::string const peer = m_dinfo ? m_link_peer[link] : std::string();
	if (m_mux)
		return util::string_format("\r\n+IPD,%u,%u%s:", link, length, peer);
	else
		return util::string_format("\r\n+IPD,%u%s:", length, peer);
}


void esp8266_at_device::link_open(unsigned link, unsigned delay_ms)
{
	esp8266_net::open_params const &params = m_link_params[link];
	LOGLINK("link %u: %s %s:%u\n", link, params.udp ? "UDP" : "TCP", params.host, params.port);
	m_link_state[link] = LINK_CONNECTING;
	m_link_paused[link] = false;
	m_link_closing[link] = CLOSE_NONE;
	m_link_tcp[link] = !params.udp;
	m_link_sendbuf[link] = 0;
	m_unacked[link].clear();
	tcp_reset(link);
	m_net->open(link, ++m_link_generation[link], params, delay_ms);
}


void esp8266_at_device::link_close(unsigned link)
{
	LOGLINK("link %u: close\n", link);
	m_ever_closed = true;
	m_link_state[link] = LINK_IDLE;
	m_link_paused[link] = false;
	m_link_closing[link] = CLOSE_NONE;
	m_link_server[link] = false;
	tcp_reset(link);
	m_net->close(link, ++m_link_generation[link]);
}


void esp8266_at_device::close_all_links()
{
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		if (m_link_state[i] != LINK_IDLE)
			link_close(i);
	}
}


void esp8266_at_device::server_close()
{
	if (m_server_port)
		LOGLINK("server: close\n");
	m_server_port = 0;
	m_server_up = false;
	m_net->stop_listen(++m_server_generation);
}


void esp8266_at_device::check_server_timeouts()
{
	if (!m_server_timeout || (m_mode == MODE_SEND_DATA) || (m_mode == MODE_SEND_WAIT))
		return;

	attotime const limit = machine().time() - attotime::from_seconds(m_server_timeout);
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		if (m_link_server[i] && (m_link_state[i] == LINK_OPEN) && (m_link_active[i] < limit))
		{
			LOGLINK("link %u: server timeout\n", i);
			link_close(i);
			if (m_fw->cipsto_fault)
			{
				// AT 0.21 firmware bug, copied: the idle timeout prints CONNECT FAIL instead of
				// CLOSED, then the watchdog resets the module
				reply(util::string_format("%u,CONNECT FAIL\r\n", i));
				restart();
				return;
			}
			closed_notice(i);
			sendbuf_fail(i);
			link_gone();
		}
	}
}


TIMER_CALLBACK_MEMBER(esp8266_at_device::poll_network)
{
	m_net->fetch(m_events);
	if (m_fw->rtos)
	{
		rtos_events();
		rtos_tick();
	}
	else
	{
		process_events();
		check_server_timeouts();
	}

	if (m_serial_dirty && output_idle())
		apply_serial();
}


void esp8266_at_device::process_events()
{
	using event_type = esp8266_net::event_type;

	while (!m_events.empty())
	{
		esp8266_net::event &ev = m_events.front();
		u32 const generation =
				(ev.link == esp8266_net::SERVER) ? m_server_generation :
				(ev.link == esp8266_net::PING) ? m_ping_generation :
				(ev.link == esp8266_net::LOOKUP) ? m_domain_generation :
				m_link_generation[ev.link];
		if (ev.generation != generation)
		{
			m_events.pop_front();
			continue;
		}

		if ((m_mode == MODE_SEND_DATA) || m_busy_batch || m_send_queued)
			break;

		unsigned const link = ev.link;
		bool const passthrough = (m_mode == MODE_PASSTHROUGH) && !link;
		switch (ev.type)
		{
		case event_type::CONNECTED:
			LOGLINK("link %u: connected\n", link);
			m_link_state[link] = LINK_OPEN;
			if (m_link_tcp[link])
				m_status = STATUS_CONNECTED;
			if (!link)
				m_reconnects = 0;
			m_link_local_port[link] = u16(ev.ticket);
			m_link_udp_moved[link] = false;
			set_remote(link, ev.peer);
			m_link_home[link] = m_link_remote[link] + ',' + std::to_string(m_link_remote_port[link]);
			if (m_link_tcp[link])
				tcp_timer_needed();
			if ((m_mode == MODE_CONNECTING) && (link == m_connect_link))
			{
				if (m_mux || m_connect_named)
					reply(util::string_format("%u,CONNECT\r\n", link));
				else
					reply("CONNECT\r\n");
				reply_ok();
				m_mode = MODE_COMMAND;
			}
			break;

		case event_type::CONNECT_FAILED:
		case event_type::DNS_FAILED:
			LOGLINK("link %u: %s\n", link, (ev.type == event_type::DNS_FAILED) ? "host not found" : "connect failed");
			m_link_state[link] = LINK_IDLE;
			if (passthrough)
			{
				link_open(link, reconnect_delay());
			}
			else if ((m_mode == MODE_CONNECTING) && (link == m_connect_link))
			{
				if (ev.type == event_type::DNS_FAILED)
				{
					reply("DNS Fail\r\n");
					reply_error();
				}
				else if (m_connect_named)
				{
					reply(util::string_format(m_fw->udp_connect_id ? "%u,CONNECT Failed\r\n" : "%u,CONNECT FAIL\r\n", link));
					reply_error();
				}
				else
				{
					reply_error();
					closed_notice(link);
					link_gone();
				}
				m_mode = MODE_COMMAND;
			}
			break;

		case event_type::DATA:
			if (m_link_closing[link])
				break;
			m_link_peer[link] = ',' + ev.peer;
			if (!m_link_tcp[link] && m_link_params[link].udp_mode && ((m_link_params[link].udp_mode == 2) || !m_link_udp_moved[link]))
			{
				std::string const before = m_link_remote[link] + ',' + std::to_string(m_link_remote_port[link]);
				if (ev.peer != before)
				{
					set_remote(link, ev.peer);
					m_link_udp_moved[link] = true;
				}
			}
			else if (!m_link_tcp[link] && !m_link_params[link].udp_mode && m_fw->udp_restore &&
					((m_link_remote[link] + ',' + std::to_string(m_link_remote_port[link])) != m_link_home[link]))
			{
				set_remote(link, m_link_home[link]);
				m_net->udp_remote(link, m_link_generation[link], m_link_remote[link], m_link_remote_port[link]);
			}
			if (m_link_tcp[link])
			{
				tcp_data(link, ev.data);
				pump();
				break;
			}
			LOGLINK("link %u: received %u bytes\n", link, unsigned(ev.data.size()));
			m_link_active[link] = machine().time();
			if (passthrough && m_fw->block_free && !m_fw->ring_passthrough_only)
			{
				// 0.40 drops a passthrough datagram that does not fit the ring
				if ((m_fw->tx_ring - ring_used()) >= ev.data.size())
				{
					m_ring_chunks.emplace_back(m_out_pushed, m_out_pushed + ev.data.size());
					send_raw(ev.data.data(), ev.data.size());
				}
			}
			else
			{
				if (!passthrough)
					reply(ipd_text(link, ev.data.size()));
				send_raw(ev.data.data(), ev.data.size());
				if (!passthrough && m_fw->udp_ipd_ok)
					reply_ok();
			}
			if (m_out.size() < OUT_HIGH_WATER)
				m_net->resume(link, m_link_generation[link]);
			else
				m_link_paused[link] = true;
			break;

		case event_type::SENT:
		case event_type::SEND_FAILED:
			if (!m_unacked[link].empty())
			{
				bool const buffered = m_unacked[link].front().second;
				m_unacked[link].pop_front();
				if (buffered)
				{
					if (ev.type == event_type::SENT)
						sendbuf_ack(link);
					break;
				}
			}
			if ((m_mode == MODE_SEND_WAIT) && (link == m_send_link))
			{
				// no link id in multi-link mode, and no CR LF before SEND FAIL
				reply((ev.type == event_type::SENT) ? "\r\nSEND OK\r\n" : "SEND FAIL\r\n");
				m_mode = MODE_COMMAND;
			}
			break;

		case event_type::CLOSED:
			if (m_link_closing[link] == CLOSE_SERVER)
			{
				link_close(link);
				closed_notice(link);
				link_gone();
				break;
			}
			if (m_link_closing[link])
			{
				m_link_closing[link] = CLOSE_DONE;
				break;
			}
			if (m_link_tcp[link] && (m_link_state[link] == LINK_OPEN))
			{
				m_tcp[link].fin = true;
				m_link_paused[link] = false;
				tcp_deliver(link);
				pump();
				break;
			}
			LOGLINK("link %u: closed by remote\n", link);
			m_link_state[link] = LINK_IDLE;
			m_link_paused[link] = false;
			if (passthrough)
			{
				link_open(link, reconnect_delay());
			}
			else
			{
				closed_notice(link);
				sendbuf_fail(link);
				link_gone();
			}
			break;

		case event_type::LISTENING:
			osd_printf_verbose("[%s] AT+CIPSERVER port %u listening on %s\n", tag(), m_server_port, ev.peer);
			if (m_mode == MODE_LISTEN)
			{
				reply_ok();
				m_mode = MODE_COMMAND;
			}
			break;

		case event_type::LISTEN_FAILED:
			osd_printf_error("[%s] AT+CIPSERVER port %u: the host cannot listen on %s. Choose a different \"AT+CIPSERVER port offset\" in the Machine Configuration menu.\n", tag(), m_server_port, ev.peer);
			m_server_port = 0;
			if (m_mode == MODE_LISTEN)
			{
				reply_error();
				m_mode = MODE_COMMAND;
			}
			break;

		case event_type::ACCEPTED:
			{
				unsigned id = 0;
				while ((id < LINK_COUNT) && (m_link_state[id] != LINK_IDLE))
					id++;
				if (!m_server_port || (id == LINK_COUNT))
				{
					LOGLINK("server: no free link, connection refused\n");
					m_net->reject(ev.ticket);
					break;
				}
				LOGLINK("link %u: accepted\n", id);
				m_link_state[id] = LINK_OPEN;
				m_status = STATUS_CONNECTED;
				m_link_local_port[id] = m_server_port;
				set_remote(id, ev.peer);
				m_link_paused[id] = false;
				m_link_server[id] = true;
				m_link_tcp[id] = true;
				m_link_sendbuf[id] = 0;
				m_unacked[id].clear();
				tcp_reset(id);
				tcp_timer_needed();
				m_link_active[id] = machine().time();
				m_net->adopt(id, ++m_link_generation[id], ev.ticket);
				reply(util::string_format("%u,CONNECT\r\n", id));
			}
			break;

		case event_type::PING_SENT:
			if (!ev.peer.empty())
				logerror("AT+PING: the host cannot send ICMP echo requests: %s\n", ev.peer);
			if (m_mode == MODE_PING)
				m_ping_timer->adjust(attotime::from_msec(PING_COARSE_MS));
			break;

		case event_type::PING_FAILED:
			if (m_mode == MODE_PING)
			{
				reply_error();
				m_mode = MODE_COMMAND;
			}
			break;

		case event_type::RESOLVED:
			if (m_mode == MODE_SEND_RESOLVE)
			{
				if (ev.peer.empty() || (m_link_state[m_send_link] != LINK_OPEN))
				{
					reply_error();
					m_mode = MODE_COMMAND;
				}
				else
				{
					send_remote(ev.peer);
					send_prompt();
				}
			}
			else if (m_mode == MODE_DOMAIN)
			{
				if (ev.peer.empty())
				{
					reply("DNS Fail\r\n");
					reply_error();
				}
				else
				{
					reply(util::string_format("+CIPDOMAIN:%s\r\n", ev.peer));
					reply_ok();
				}
				m_mode = MODE_COMMAND;
			}
			break;

		case event_type::PING_REPLY:
			if ((m_mode == MODE_PING) && !m_ping_replied)
			{
				m_ping_replied = true;
				reply(util::string_format("+%u\r\n", ev.ticket));
			}
			break;
		}
		m_events.pop_front();
	}
}


TIMER_CALLBACK_MEMBER(esp8266_at_device::passthrough_flush)
{
	m_passthrough_timer->adjust(attotime::never);
	if ((m_mode != MODE_PASSTHROUGH) || m_passthrough_buf.empty())
		return;
	if (m_fw->rtos)
	{
		rtos_passthrough_flush();
		return;
	}

	static u8 const escape[] = { '+', '+', '+' };
	if ((m_passthrough_buf.size() == 3) && std::equal(m_passthrough_buf.begin(), m_passthrough_buf.end(), std::begin(escape)))
	{
		LOGLINK("link 0: passthrough off\n");
		m_passthrough_buf.clear();
		m_mode = MODE_COMMAND;
		if (m_link_state[0] == LINK_CONNECTING)
			link_close(0);
		return;
	}

	if (m_link_state[0] == LINK_OPEN)
	{
		m_net->send(0, m_link_generation[0], std::move(m_passthrough_buf), false);
		if (m_fw->uart_stall && m_link_tcp[0])
		{
			// AT 0.21 firmware bug, copied: UART reading stops until the remote ACKs the
			// packet, so the 128-byte FIFO overflows and later bytes are lost
			m_uart_stalled = true;
			m_stall_timer->adjust(attotime::from_msec(PASSTHROUGH_ACK_MS));
		}
		if (m_link_tcp[0])
		{
			tcp_ack(0);
			tcp_deliver(0);
			pump();
		}
	}
	m_passthrough_buf.clear();
}


void esp8266_at_device::restart()
{
	stop_waits();
	close_all_links();
	server_close();
	m_events.clear();
	m_passthrough_timer->adjust(attotime::never);
	m_join_timer->adjust(attotime::never);
	stop_ping();
	m_mode = MODE_RESTART;
	m_boot_stage = BOOT_ROM;
	m_restart_timer->adjust(attotime::zero);
}


TIMER_CALLBACK_MEMBER(esp8266_at_device::restart_done)
{
	if (!output_idle())
	{
		m_restart_timer->adjust(attotime::from_msec(10));
		return;
	}

	if (m_boot_stage == BOOT_ROM)
	{
		m_cur_rate = m_def_rate;
		std::copy(std::begin(m_def_frame), std::end(m_def_frame), std::begin(m_cur_frame));
		apply_serial();
		reply(m_fw->boot_text);
		m_boot_stage = BOOT_READY;
		m_restart_timer->adjust(attotime::from_msec(BOOT_DELAY_MS));
		return;
	}

	set_defaults();
	boot_done();
}


void esp8266_at_device::boot_done()
{
	bool const translink = (m_tl_mode == 1) && (!m_fw->cipmode_stored || m_tl_host[0]);
	if (m_fw->cipmode_stored)
		m_cipmode = m_tl_mode;
	if (translink)
	{
		m_cipmode = 1;
		reply("\r\n>");
	}
	reply("\r\nready\r\n");

	if (m_joined && (!m_cipmode || (m_fw->sysmsg_bit2 && BIT(m_sysmsg, 2))) && !m_fw->quiet_wifi)
		reply("WIFI CONNECTED\r\nWIFI GOT IP\r\n");

	if (translink)
	{
		esp8266_net::open_params params;
		params.udp = (m_tl_type == TRANSLINK_UDP) && !m_fw->cipmode_stored;
		params.port = m_tl_port;
		params.local_port = m_tl_local_port;
		params.keepalive = (m_tl_type == TRANSLINK_UDP) ? 0 : m_tl_keepalive;
		u32 const ip = fw_ipaddr(m_tl_host);
		if (ip != IPADDR_NONE)
			params.host = util::string_format("%u.%u.%u.%u", BIT(ip, 24, 8), BIT(ip, 16, 8), BIT(ip, 8, 8), BIT(ip, 0, 8));
		else
			params.host = m_tl_host;
		m_link_params[0] = params;
		m_link_server[0] = false;
		m_passthrough_buf.clear();
		m_mode = MODE_PASSTHROUGH;
		LOGLINK("link 0: transparent link at start-up\n");

		if ((m_tl_type != TRANSLINK_SSL) && m_joined)
			link_open(0, 0);
	}
}


void esp8266_at_device::store_cipmode()
{
	m_tl_mode = m_cipmode;
	if (m_link_state[0] == LINK_OPEN)
	{
		m_tl_type = m_link_tcp[0] ? TRANSLINK_TCP : TRANSLINK_UDP;
		m_tl_port = m_link_remote_port[0];
		m_tl_local_port = m_link_tcp[0] ? 0 : m_link_local_port[0];
		std::fill(std::begin(m_tl_host), std::end(m_tl_host), 0);
		m_link_remote[0].copy(m_tl_host, MAX_TRANSLINK_HOST);
	}
	m_translink_stored = true;
}


unsigned esp8266_at_device::reconnect_delay()
{
	if (m_fw->rtos)
		return m_reconnect_ms;
	if (!m_fw->reconnect_step_us)
		return RECONNECT_DELAY_MS;
	m_reconnects = std::min<u8>(m_reconnects + 1, 10);
	return u16(m_reconnects * m_fw->reconnect_step_us) / 1000;
}


bool esp8266_at_device::close_wait(unsigned link)
{
	if (!m_fw->close_waits || !m_link_tcp[link] || (m_link_state[link] != LINK_OPEN))
		return false;
	m_link_closing[link] = CLOSE_WAIT;
	m_net->shutdown(link, m_link_generation[link]);
	if (m_mode != MODE_CLOSING)
	{
		m_mode = MODE_CLOSING;
		m_close_start = machine().time();
		m_close_timer->adjust(attotime::from_msec(ESP_TCP_TMR_MS), 0, attotime::from_msec(ESP_TCP_TMR_MS));
	}
	return true;
}


bool esp8266_at_device::close_chain(unsigned first)
{
	m_close_chain = false;
	for (unsigned i = first; i < LINK_COUNT; i++)
	{
		if ((m_link_state[i] == LINK_IDLE) || m_link_server[i])
			continue;
		if (close_wait(i))
		{
			m_close_chain = true;
			m_close_start = machine().time();
			return true;
		}
		link_close(i);
		closed_notice(i);
		sendbuf_fail(i);
	}
	link_gone();
	return std::find_if(std::begin(m_link_state), std::end(m_link_state), [] (u8 s) { return s != LINK_IDLE; }) != std::end(m_link_state);
}


TIMER_CALLBACK_MEMBER(esp8266_at_device::close_tick)
{
	bool const expired = (machine().time() - m_close_start) >= attotime::from_msec(TCP_FIN_WAIT_MS);
	bool waiting = false;
	bool left_open = false;
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		if ((m_link_closing[i] == CLOSE_WAIT) && !expired)
		{
			waiting = true;
		}
		else if ((m_link_closing[i] != CLOSE_NONE) && (m_link_closing[i] != CLOSE_SERVER))
		{
			link_close(i);
			closed_notice(i);
			sendbuf_fail(i);
			if (m_close_chain)
			{
				left_open = close_chain(i + 1);
				if (m_close_chain)
					return;
			}
		}
	}
	if (waiting)
		return;
	m_close_timer->adjust(attotime::never);
	link_gone();
	if (!left_open)
		reply_ok();
	m_mode = MODE_COMMAND;
	tcp_deliver_all();
}


void esp8266_at_device::stop_waits()
{
	m_close_timer->adjust(attotime::never);
	std::fill(std::begin(m_link_closing), std::end(m_link_closing), CLOSE_NONE);
	m_close_chain = false;
	m_stall_timer->adjust(attotime::never);
	m_uart_stalled = false;
	m_uart_fifo.clear();
	m_rx_timer->adjust(attotime::never);
	m_rx_batch.clear();
}


TIMER_CALLBACK_MEMBER(esp8266_at_device::domain_done)
{
	if (m_mode != MODE_DOMAIN)
		return;
	reply("DNS Fail\r\n");
	reply_error();
	m_mode = MODE_COMMAND;
}


esp8266_at_device::rtos_command const esp8266_at_device::s_rtos_commands[] = {
		{ "E0", &esp8266_at_device::rtos_cmd_e0 },
		{ "E1", &esp8266_at_device::rtos_cmd_e1 },
		{ "+RST", &esp8266_at_device::rtos_cmd_rst },
		{ "+GMR", &esp8266_at_device::rtos_cmd_gmr },
		{ "+SYSRAM", &esp8266_at_device::rtos_cmd_sysram },
		{ "+SYSMSG", &esp8266_at_device::rtos_cmd_sysmsg },
		{ "+SYSLOG", &esp8266_at_device::rtos_cmd_syslog },
		{ "+SYSTIMESTAMP", &esp8266_at_device::rtos_cmd_systimestamp },
		{ "+SLEEP", &esp8266_at_device::rtos_cmd_sleep },
		{ "+UART", &esp8266_at_device::rtos_cmd_uart },
		{ "+UART_CUR", &esp8266_at_device::rtos_cmd_uart },
		{ "+UART_DEF", &esp8266_at_device::rtos_cmd_uart },
		{ "+CWMODE", &esp8266_at_device::rtos_cmd_cwmode },
		{ "+CWJAP", &esp8266_at_device::rtos_cmd_cwjap },
		{ "+CWLAP", &esp8266_at_device::rtos_cmd_cwlap },
		{ "+CWQAP", &esp8266_at_device::rtos_cmd_cwqap },
		{ "+CWDHCP", &esp8266_at_device::rtos_cmd_cwdhcp },
		{ "+CIPSTA", &esp8266_at_device::rtos_cmd_cipsta },
		{ "+CIFSR", &esp8266_at_device::rtos_cmd_cifsr },
		{ "+PING", &esp8266_at_device::rtos_cmd_ping },
		{ "+CIPDOMAIN", &esp8266_at_device::rtos_cmd_cipdomain },
		{ "+CIPSTATUS", &esp8266_at_device::rtos_cmd_cipstatus },
		{ "+CIPSTART", &esp8266_at_device::rtos_cmd_cipstart },
		{ "+CIPSTARTEX", &esp8266_at_device::rtos_cmd_cipstartex },
		{ "+CIPCLOSE", &esp8266_at_device::rtos_cmd_cipclose },
		{ "+CIPSEND", &esp8266_at_device::rtos_cmd_cipsend },
		{ "+CIPSENDEX", &esp8266_at_device::rtos_cmd_cipsendex },
		{ "+CIPDINFO", &esp8266_at_device::rtos_cmd_cipdinfo },
		{ "+CIPMUX", &esp8266_at_device::rtos_cmd_cipmux },
		{ "+CIPRECVMODE", &esp8266_at_device::rtos_cmd_ciprecvmode },
		{ "+CIPRECVDATA", &esp8266_at_device::rtos_cmd_ciprecvdata },
		{ "+CIPRECVLEN", &esp8266_at_device::rtos_cmd_ciprecvlen },
		{ "+CIPSERVER", &esp8266_at_device::rtos_cmd_cipserver },
		{ "+CIPSERVERMAXCONN", &esp8266_at_device::rtos_cmd_cipservermaxconn },
		{ "+CIPMODE", &esp8266_at_device::rtos_cmd_cipmode },
		{ "+CIPSTO", &esp8266_at_device::rtos_cmd_cipsto },
		{ "+SAVETRANSLINK", &esp8266_at_device::rtos_cmd_savetranslink },
		{ "+CIPRECONNINTV", &esp8266_at_device::rtos_cmd_cipreconnintv },
		{ nullptr, nullptr } };

// forms: bit 0 execute, bit 1 query, bit 2 test, bit 3 set
esp8266_at_device::rtos_command const esp8266_at_device::s_rtos_list_commands[] = {
		{ "E0", &esp8266_at_device::rtos_cmd_e0, 0x1 },
		{ "E1", &esp8266_at_device::rtos_cmd_e1, 0x1 },
		{ "+RST", &esp8266_at_device::rtos_cmd_rst, 0x1 },
		{ "+GMR", &esp8266_at_device::rtos_cmd_gmr, 0x1 },
		{ "+CMD", &esp8266_at_device::rtos_cmd_cmd, 0x2 },
		{ "+SYSTIMESTAMP", &esp8266_at_device::rtos_cmd_systimestamp, 0xa },
		{ "+SLEEP", &esp8266_at_device::rtos_cmd_sleep, 0xa },
		{ "+SYSRAM", &esp8266_at_device::rtos_cmd_sysram, 0x2 },
		{ "+SYSMSG", &esp8266_at_device::rtos_cmd_sysmsg, 0xa },
		{ "+SYSLOG", &esp8266_at_device::rtos_cmd_syslog, 0xa },
		{ "+SYSSTORE", &esp8266_at_device::rtos_cmd_sysstore, 0xa },
		{ "+USERRAM", &esp8266_at_device::rtos_cmd_userram, 0xa },
		{ "+CWMODE", &esp8266_at_device::rtos_cmd_cwmode, 0xa },
		{ "+CWSTATE", &esp8266_at_device::rtos_cmd_cwstate, 0x2 },
		{ "+CWJAP", &esp8266_at_device::rtos_cmd_cwjap, 0xb },
		{ "+CWRECONNCFG", &esp8266_at_device::rtos_cmd_cwreconncfg, 0xa },
		{ "+CWLAP", &esp8266_at_device::rtos_cmd_cwlap, 0x9 },
		{ "+CWQAP", &esp8266_at_device::rtos_cmd_cwqap, 0x1 },
		{ "+CWDHCP", &esp8266_at_device::rtos_cmd_cwdhcp, 0xa },
		{ "+CIFSR", &esp8266_at_device::rtos_cmd_cifsr, 0x1 },
		{ "+CIPSTA", &esp8266_at_device::rtos_cmd_cipsta, 0xa },
		{ "+CIPDOMAIN", &esp8266_at_device::rtos_cmd_cipdomain, 0x8 },
		{ "+CIPSTATUS", &esp8266_at_device::rtos_cmd_cipstatus, 0x1 },
		{ "+CIPSTART", &esp8266_at_device::rtos_cmd_cipstart, 0x8 },
		{ "+CIPSTARTEX", &esp8266_at_device::rtos_cmd_cipstartex, 0x8 },
		{ "+CIPTCPOPT", &esp8266_at_device::rtos_cmd_ciptcpopt, 0xa },
		{ "+CIPCLOSE", &esp8266_at_device::rtos_cmd_cipclose, 0x9 },
		{ "+CIPSEND", &esp8266_at_device::rtos_cmd_cipsend, 0x9 },
		{ "+CIPSENDEX", &esp8266_at_device::rtos_cmd_cipsendex, 0x8 },
		{ "+CIPDINFO", &esp8266_at_device::rtos_cmd_cipdinfo, 0xa },
		{ "+CIPMUX", &esp8266_at_device::rtos_cmd_cipmux, 0xa },
		{ "+CIPRECVMODE", &esp8266_at_device::rtos_cmd_ciprecvmode, 0xa },
		{ "+CIPRECVDATA", &esp8266_at_device::rtos_cmd_ciprecvdata, 0x8 },
		{ "+CIPRECVLEN", &esp8266_at_device::rtos_cmd_ciprecvlen, 0x2 },
		{ "+CIPSERVER", &esp8266_at_device::rtos_cmd_cipserver, 0xa },
		{ "+CIPSERVERMAXCONN", &esp8266_at_device::rtos_cmd_cipservermaxconn, 0xa },
		{ "+CIPMODE", &esp8266_at_device::rtos_cmd_cipmode, 0xa },
		{ "+CIPSTO", &esp8266_at_device::rtos_cmd_cipsto, 0xa },
		{ "+SAVETRANSLINK", &esp8266_at_device::rtos_cmd_savetranslink, 0x8 },
		{ "+CIPRECONNINTV", &esp8266_at_device::rtos_cmd_cipreconnintv, 0xa },
		{ "+PING", &esp8266_at_device::rtos_cmd_ping, 0x8 },
		{ "+UART", &esp8266_at_device::rtos_cmd_uart, 0xa },
		{ "+UART_CUR", &esp8266_at_device::rtos_cmd_uart, 0xa },
		{ "+UART_DEF", &esp8266_at_device::rtos_cmd_uart, 0xa },
		{ nullptr, nullptr } };


void esp8266_at_device::rtos_byte(u8 byte)
{
	if (m_mode == MODE_RESTART)
		return;
	m_rx_batch.push_back(byte);
	if (m_rx_batch.size() >= m_fw->rx_event)
		rtos_batch(0);
	else
		m_rx_timer->adjust(attotime::from_hz(m_cur_rate) * RTOS_RX_TOUT_BITS);
}


TIMER_CALLBACK_MEMBER(esp8266_at_device::rtos_batch)
{
	m_rx_timer->adjust(attotime::never);
	std::vector<u8> batch;
	batch.swap(m_rx_batch);
	if (batch.empty())
		return;

	switch (m_mode)
	{
	case MODE_RESTART:
	case MODE_SEND_WAIT:
		break;

	case MODE_COMMAND:
		rtos_line_input(batch);
		break;

	case MODE_SEND_DATA:
		rtos_send_input(batch);
		break;

	case MODE_PASSTHROUGH:
		rtos_passthrough_input(batch);
		break;

	case MODE_USERRAM:
		rtos_userram_input(batch);
		break;

	default:
		rtos_busy(batch);
		break;
	}
}


void esp8266_at_device::rtos_line_input(std::vector<u8> const &batch)
{
	auto it = batch.begin();
	while (it != batch.end())
	{
		u8 const byte = *it++;
		// a NUL is echoed only as the last byte of a read
		if (m_echo && (byte || (it == batch.end())))
			send_raw(&byte, 1);
		if (!byte)
		{
			m_line.clear();
			continue;
		}
		m_line.push_back(char(byte));
		if ((byte == '\n') && (m_line.size() >= 2) && (m_line[m_line.size() - 2] == '\r'))
		{
			std::string const line = m_line.substr(0, m_line.size() - 2);
			m_line.clear();
			if (!rtos_parse(line))
				continue;
			if (it != batch.end())
			{
				rtos_busy(std::vector<u8>(it, batch.end()));
				if (m_mode == MODE_RESTART)
					return;
			}
			rtos_dispatch();
			return;
		}
	}
	if (m_line.size() >= RTOS_LINE_MAX)
		m_line = (m_line.back() == '\r') ? "\r" : "";
}


void esp8266_at_device::rtos_busy(std::vector<u8> const &batch)
{
	static char const force[] = "AT+RST\r\n";
	if (std::search(batch.begin(), batch.end(), std::begin(force), std::end(force) - 1) != batch.end())
	{
		force_restart();
		return;
	}
	rtos_err_code(0x010b'0000);
	reply("\r\nbusy p...\r\n");
}


void esp8266_at_device::rtos_send_input(std::vector<u8> const &batch)
{
	static u8 const escape[] = { '+', '+', '+' };
	if ((batch.size() == 3) && std::equal(batch.begin(), batch.end(), std::begin(escape)))
	{
		LOGLINK("link %u: send canceled\n", m_send_link);
		m_send_buf.clear();
		m_mode = MODE_COMMAND;
		reply("\r\nSEND Canceled\r\n");
		return;
	}

	auto it = batch.begin();
	while (it != batch.end())
	{
		u8 const byte = *it++;
		if (m_send_rtos_ex && m_fw->sendex_escape && (m_send_escape || (byte == '\\')))
		{
			if (!std::exchange(m_send_escape, !m_send_escape))
				continue;
			if (byte != '0')
			{
				m_send_buf.push_back(byte);
				if (m_send_buf.size() < m_send_length)
					continue;
			}
		}
		else if (m_send_rtos_ex && !m_fw->sendex_escape && (byte == '0') && !m_send_buf.empty() && (m_send_buf.back() == '\\'))
		{
			m_send_buf.pop_back();
		}
		else
		{
			m_send_buf.push_back(byte);
			if (m_send_buf.size() < m_send_length)
				continue;
		}
		m_send_escape = false;
		m_mode = MODE_SEND_WAIT;
		if (it != batch.end())
			reply("\r\nbusy p...\r\n");
		rtos_send_start();
		return;
	}
}


void esp8266_at_device::rtos_userram_input(std::vector<u8> const &batch)
{
	unsigned const count = std::min<unsigned>(batch.size(), m_userram_left);
	std::copy_n(batch.begin(), count, m_userram.begin() + m_userram_at);
	m_userram_at += count;
	m_userram_left -= count;
	if (m_userram_left)
		return;
	m_mode = MODE_COMMAND;
	reply("\r\nWRITE OK\r\n");
	if (count < batch.size())
		rtos_line_input(std::vector<u8>(batch.begin() + count, batch.end()));
}


void esp8266_at_device::rtos_passthrough_input(std::vector<u8> const &batch)
{
	static u8 const escape[] = { '+', '+', '+' };
	if ((batch.size() == 3) && std::equal(batch.begin(), batch.end(), std::begin(escape)))
	{
		rtos_passthrough_off();
		return;
	}

	m_passthrough_buf.insert(m_passthrough_buf.end(), batch.begin(), batch.end());
	while (m_passthrough_buf.size() >= m_fw->passthrough_packet)
	{
		std::vector<u8> packet(m_passthrough_buf.begin(), m_passthrough_buf.begin() + m_fw->passthrough_packet);
		m_passthrough_buf.erase(m_passthrough_buf.begin(), m_passthrough_buf.begin() + m_fw->passthrough_packet);
		if (m_link_state[0] == LINK_OPEN)
			m_net->send(0, m_link_generation[0], std::move(packet), false);
	}
	m_passthrough_timer->adjust(m_passthrough_buf.empty() ? attotime::never : attotime::from_msec(RX_BATCH_GAP_MS));
}


void esp8266_at_device::rtos_passthrough_flush()
{
	static u8 const escape[] = { '+', '+', '+' };
	if ((m_passthrough_buf.size() == 3) && std::equal(m_passthrough_buf.begin(), m_passthrough_buf.end(), std::begin(escape)))
	{
		rtos_passthrough_off();
		return;
	}
	if (m_link_state[0] == LINK_OPEN)
		m_net->send(0, m_link_generation[0], std::move(m_passthrough_buf), false);
	m_passthrough_buf.clear();
}


void esp8266_at_device::rtos_passthrough_off()
{
	LOGLINK("link 0: passthrough off\n");
	m_passthrough_buf.clear();
	m_passthrough_timer->adjust(attotime::never);
	m_mode = MODE_COMMAND;
	if (m_link_state[0] == LINK_CONNECTING)
		link_close(0);
	if (BIT(m_sysmsg, 0))
		reply("\r\n+QUITT\r\n");
}


bool esp8266_at_device::rtos_parse(std::string const &line)
{
	LOGCMD("command: %s\n", line);

	if ((line.size() < 2) || ((line.size() + 2) > RTOS_LINE_MAX) || ((line[0] | 0x20) != 'a') || ((line[1] | 0x20) != 't'))
	{
		rtos_err_code(0x0103'0000);
		reply_error();
		return false;
	}

	static constexpr std::string_view name_chars = "!%-./:_";
	std::size_t p = 2;
	m_cmd_name.clear();
	if ((p < line.size()) && (line[p] == '+'))
		m_cmd_name.push_back(line[p++]);
	while (p < line.size())
	{
		char const c = line[p];
		if ((c >= 'a') && (c <= 'z'))
			m_cmd_name.push_back(c - 'a' + 'A');
		else if (((c >= 'A') && (c <= 'Z')) || ((c >= '0') && (c <= '9')) || (c && (name_chars.find(c) != std::string_view::npos)))
			m_cmd_name.push_back(c);
		else
			break;
		p++;
	}

	m_para.clear();
	if (p == line.size())
	{
		m_cmd_form = FORM_EXEC;
	}
	else if ((line[p] == '?') || !line.compare(p, 2, "=?"))
	{
		m_cmd_form = (line[p] == '?') ? FORM_QUERY : FORM_TEST;
		if ((p + ((line[p] == '?') ? 1 : 2)) != line.size())
		{
			rtos_err_code(0x0109'0000);
			reply_error();
			return false;
		}
	}
	else if (line[p] == '=')
	{
		m_cmd_form = FORM_SET;
		std::string para;
		for (std::size_t i = p + 1; i < line.size(); i++)
		{
			char const c = line[i];
			if ((c == '\\') && ((i + 1) < line.size()))
			{
				para.push_back(c);
				para.push_back(line[++i]);
			}
			else if (c == ',')
			{
				m_para.emplace_back(std::move(para));
				para.clear();
			}
			else
			{
				para.push_back(c);
			}
		}
		m_para.emplace_back(std::move(para));
	}
	else
	{
		// any other byte becomes part of the name, so the command is not found
		m_cmd_form = FORM_EXEC;
		m_cmd_name.append(line, p, std::string::npos);
	}
	return true;
}


void esp8266_at_device::rtos_dispatch()
{
	if (m_cmd_name.empty() && (m_cmd_form == FORM_EXEC))
	{
		reply_ok();
		return;
	}
	rtos_command const *cmd = m_fw->cmd_list ? s_rtos_list_commands : s_rtos_commands;
	while (cmd->name && (m_cmd_name != cmd->name))
		cmd++;
	if (!cmd->name)
	{
		rtos_err_code(0x0109'0000);
		reply_error();
		return;
	}
	if (m_fw->cmd_list && !BIT(cmd->forms, m_cmd_form))
	{
		reply_error();
		return;
	}
	(this->*cmd->handler)(m_cmd_form);
}


void esp8266_at_device::rtos_err_code(u32 code)
{
	if (m_syslog)
		reply(util::string_format("ERR CODE:0x%08x\r\n", code));
}


int esp8266_at_device::rtos_digit(unsigned index, s32 &value)
{
	if (index >= m_para.size())
	{
		rtos_err_code(0x0108'0000 | index);
		return PARA_FAIL;
	}
	std::string const &text = m_para[index];
	if (text.empty())
		return PARA_OMITTED;

	std::size_t p = 0;
	bool const negative = text[0] == '-';
	if (negative)
		p++;
	unsigned base = 10;
	if (((p + 1) < text.size()) && (text[p] == '0') && ((text[p + 1] | 0x20) == 'x'))
	{
		base = 16;
		p += 2;
	}
	if (p >= text.size())
	{
		rtos_err_code(0x0108'0000 | index);
		return PARA_FAIL;
	}
	u64 result = 0;
	for ( ; p < text.size(); p++)
	{
		char const c = text[p];
		unsigned digit;
		if ((c >= '0') && (c <= '9'))
		{
			digit = c - '0';
		}
		else if ((base == 16) && ((c | 0x20) >= 'a') && ((c | 0x20) <= 'f'))
		{
			digit = (c | 0x20) - 'a' + 10;
		}
		else
		{
			rtos_err_code(0x0108'0000 | index);
			return PARA_FAIL;
		}
		result = (result * base) + digit;
		if (result > 0xffff'ffff)
			return PARA_FAIL;
	}
	value = s32(negative ? (0 - u32(result)) : u32(result));
	return PARA_OK;
}


int esp8266_at_device::rtos_str(unsigned index, std::string &value)
{
	if (index >= m_para.size())
	{
		// firmware bug, copied: a missing string prints its code but reads as given, unchanged
		rtos_err_code(0x0108'0000 | index);
		return m_fw->strict_params ? PARA_FAIL : PARA_OK;
	}
	std::string const &text = m_para[index];
	if (text.empty())
		return PARA_OMITTED;

	bool ok = (text.size() >= 2) && (text.front() == '"') && (text.back() == '"');
	std::string result;
	for (std::size_t i = 1; ok && (i < (text.size() - 1)); i++)
	{
		char c = text[i];
		if (c == '\\')
		{
			ok = (i + 1) < (text.size() - 1);
			c = text[++i];
		}
		else if (c == '"')
		{
			ok = false;
		}
		result.push_back(c);
	}
	if (!ok)
	{
		rtos_err_code(0x0108'0000 | index);
		return PARA_FAIL;
	}
	value = std::move(result);
	return PARA_OK;
}


void esp8266_at_device::rtos_link_count()
{
	bool tcp = false;
	for (unsigned i = 0; i < LINK_COUNT; i++)
		tcp = tcp || ((m_link_state[i] == LINK_OPEN) && m_link_tcp[i]);
	m_status = tcp ? STATUS_CONNECTED : (!m_fw->status_closed || m_ever_closed) ? STATUS_DISCONNECTED : STATUS_GOT_IP;
}


void esp8266_at_device::rtos_close_clients()
{
	if ((m_cwmode != 1) || m_cipmode)
		return;
	bool closed = false;
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		if ((m_link_state[i] != LINK_IDLE) && !m_link_server[i])
		{
			link_close(i);
			closed_notice(i);
			closed = true;
		}
	}
	if (closed)
		rtos_link_count();
}


void esp8266_at_device::rtos_closed(unsigned link)
{
	LOGLINK("link %u: closed by remote\n", link);
	if ((m_mode == MODE_PASSTHROUGH) && !link)
	{
		m_link_state[link] = LINK_IDLE;
		if (m_fw->sysmsg_bit2 && BIT(m_sysmsg, 2))
			closed_notice(link);
		link_open(link, reconnect_delay());
		return;
	}
	bool const server = m_link_server[link];
	link_close(link);
	closed_notice(link);
	if (server)
		rtos_client_gone(link);
	rtos_link_count();
}


void esp8266_at_device::rtos_client_gone(unsigned link)
{
	if (m_server_clients)
		m_server_clients--;
	if (m_listen_full && m_server_up && (m_server_clients < m_max_conn))
	{
		m_listen_full = false;
		rtos_listen();
	}
}


void esp8266_at_device::rtos_listen()
{
	ioport_value const config = m_server_config->read();
	unsigned const host_port = m_server_port ? (m_server_port + SERVER_PORT_OFFSETS[BIT(config, 4, 3)]) : 0;
	LOGLINK("server: port %u, host port %u\n", m_server_port, host_port);
	m_net->listen(++m_server_generation, BIT(config, 0), host_port);
}


void esp8266_at_device::rtos_finish(unsigned link)
{
	if ((m_link_state[link] == LINK_OPEN) && m_tcp[link].fin && m_tcp[link].remote.empty())
		rtos_closed(link);
}


unsigned esp8266_at_device::rtos_held(unsigned link) const
{
	if (m_link_state[link] != LINK_OPEN)
		return 0;
	if (m_link_tcp[link])
		return std::min<unsigned>(m_tcp[link].remote.size(), m_fw->tcp_wnd);
	unsigned held = 0;
	for (auto const &datagram : m_udp_hold[link])
		held += datagram.second.size();
	return held;
}


void esp8266_at_device::rtos_notice(unsigned link)
{
	unsigned const held = rtos_held(link);
	if (m_ipd_notice[link] || !held)
		return;
	m_ipd_notice[link] = true;
	if (m_mux)
		reply(util::string_format("\r\n+IPD,%u,%u\r\n", link, held));
	else
		reply(util::string_format("\r\n+IPD,%u\r\n", held));
}


std::string esp8266_at_device::rtos_peer(std::string const &peer)
{
	auto const comma = peer.rfind(',');
	std::string address = peer.substr(0, comma);
	if (address == "::1")
		address = "127.0.0.1";
	return ",\"" + address + "\"," + ((comma == std::string::npos) ? std::string("0") : peer.substr(comma + 1));
}


void esp8266_at_device::rtos_data(unsigned link, esp8266_net::event const &ev)
{
	if (!m_link_tcp[link] && m_link_params[link].udp_mode && ((m_link_params[link].udp_mode == 2) || !m_link_udp_moved[link]))
	{
		std::string const before = m_link_remote[link] + ',' + std::to_string(m_link_remote_port[link]);
		if (ev.peer != before)
		{
			set_remote(link, ev.peer);
			m_link_udp_moved[link] = true;
		}
	}

	if (m_link_tcp[link])
	{
		tcp_model &tcp = m_tcp[link];
		tcp.remote.insert(tcp.remote.end(), ev.data.begin(), ev.data.end());
		if (tcp.remote.size() >= m_fw->tcp_wnd)
			m_link_paused[link] = true;
		else
			m_net->resume(link, m_link_generation[link]);
	}
	else
	{
		auto &hold = m_udp_hold[link];
		if (!m_recv_mode || (hold.size() < RTOS_UDP_MBOX))
			hold.emplace_back(ev.peer, ev.data);
		if (!m_recv_mode && (hold.size() >= RTOS_UDP_MBOX))
			m_link_paused[link] = true;
		else
			m_net->resume(link, m_link_generation[link]);
	}

	if (m_recv_mode && ((m_mode != MODE_PASSTHROUGH) || link))
		rtos_notice(link);
	pump();
}


bool esp8266_at_device::rtos_service()
{
	bool moved = false;
	bool more = true;
	while (more && (m_out.size() <= RTOS_TX_RING))
	{
		more = false;
		for (unsigned i = 0; (i < LINK_COUNT) && (m_out.size() <= RTOS_TX_RING); i++)
		{
			if (rtos_block(i))
				more = moved = true;
		}
	}
	return moved;
}


bool esp8266_at_device::rtos_block(unsigned link)
{
	bool const passthrough = (m_mode == MODE_PASSTHROUGH) && !link;
	if ((m_link_state[link] != LINK_OPEN) || (m_recv_mode && !passthrough))
		return false;

	std::vector<u8> data;
	std::string peer;
	if (m_link_tcp[link])
	{
		tcp_model &tcp = m_tcp[link];
		if (tcp.remote.empty())
			return false;
		unsigned const length = std::min<unsigned>(tcp.remote.size(), RTOS_IPD_MAX);
		data.assign(tcp.remote.begin(), tcp.remote.begin() + length);
		tcp.remote.erase(tcp.remote.begin(), tcp.remote.begin() + length);
		peer = m_link_remote[link] + ',' + std::to_string(m_link_remote_port[link]);
		if (m_link_paused[link] && (tcp.remote.size() < m_fw->tcp_wnd))
		{
			m_link_paused[link] = false;
			m_net->resume(link, m_link_generation[link]);
		}
	}
	else
	{
		auto &hold = m_udp_hold[link];
		if (hold.empty())
			return false;
		peer = std::move(hold.front().first);
		data = std::move(hold.front().second);
		hold.pop_front();
		if (m_link_paused[link] && (hold.size() < RTOS_UDP_MBOX))
		{
			m_link_paused[link] = false;
			m_net->resume(link, m_link_generation[link]);
		}
	}

	LOGLINK("link %u: received %u bytes\n", link, unsigned(data.size()));
	m_idle[link] = 0;
	m_link_active[link] = machine().time();
	if (!passthrough)
	{
		std::string const info = m_dinfo ? rtos_peer(peer) : std::string();
		std::string const header = m_mux
				? util::string_format("\r\n+IPD,%u,%u%s:", link, unsigned(data.size()), info)
				: util::string_format("\r\n+IPD,%u%s:", unsigned(data.size()), info);
		out_push(reinterpret_cast<u8 const *>(header.data()), header.size());
	}
	out_push(data.data(), data.size());
	if (!passthrough)
		out_push(reinterpret_cast<u8 const *>("\r\n"), 2);
	if (m_link_tcp[link])
		rtos_finish(link);
	return true;
}


void esp8266_at_device::rtos_tick()
{
	if (m_sto_next == attotime::never)
		return;
	attotime const now = machine().time();
	while (now >= m_sto_next)
	{
		m_sto_next += attotime::from_seconds(1);
		for (unsigned i = 0; i < LINK_COUNT; i++)
		{
			if (m_link_server[i] && (m_link_state[i] == LINK_OPEN) && (++m_idle[i] >= m_server_timeout))
			{
				LOGLINK("link %u: server timeout\n", i);
				link_close(i);
				closed_notice(i);
				rtos_client_gone(i);
				rtos_link_count();
			}
		}
	}
}


void esp8266_at_device::rtos_events()
{
	using event_type = esp8266_net::event_type;

	while (!m_events.empty())
	{
		esp8266_net::event const ev = std::move(m_events.front());
		m_events.pop_front();
		u32 const generation =
				(ev.link == esp8266_net::SERVER) ? m_server_generation :
				(ev.link == esp8266_net::PING) ? m_ping_generation :
				(ev.link == esp8266_net::LOOKUP) ? m_domain_generation :
				m_link_generation[ev.link];
		if (ev.generation != generation)
			continue;

		unsigned const link = ev.link;
		switch (ev.type)
		{
		case event_type::CONNECTED:
			LOGLINK("link %u: connected\n", link);
			m_link_state[link] = LINK_OPEN;
			if (!link)
				m_reconnects = 0;
			m_link_local_port[link] = u16(ev.ticket);
			m_link_udp_moved[link] = false;
			set_remote(link, ev.peer);
			if (m_link_tcp[link])
				rtos_tcpopt_apply(link);
			if ((m_mode == MODE_PASSTHROUGH) && !link && m_fw->sysmsg_bit2 && BIT(m_sysmsg, 2))
				reply("CONNECT\r\n");
			if ((m_mode == MODE_CONNECTING) && (link == m_connect_link))
			{
				if (BIT(m_sysmsg, 1))
					reply(util::string_format(
							"+LINK_CONN:0,%u,\"%s\",0,\"%s\",%u,%u\r\n",
							link, m_link_tcp[link] ? "TCP" : "UDP", m_link_remote[link], m_link_remote_port[link], m_link_local_port[link]));
				else if (m_mux)
					reply(util::string_format("%u,CONNECT\r\n", link));
				else
					reply("CONNECT\r\n");
				rtos_link_count();
				reply_ok();
				m_mode = MODE_COMMAND;
			}
			break;

		case event_type::CONNECT_FAILED:
		case event_type::DNS_FAILED:
			LOGLINK("link %u: %s\n", link, (ev.type == event_type::DNS_FAILED) ? "host not found" : "connect failed");
			m_link_state[link] = LINK_IDLE;
			if ((m_mode == MODE_PASSTHROUGH) && !link)
			{
				link_open(link, reconnect_delay());
			}
			else if ((m_mode == MODE_CONNECTING) && (link == m_connect_link))
			{
				reply_error();
				m_mode = MODE_COMMAND;
			}
			break;

		case event_type::DATA:
			if (m_link_state[link] == LINK_OPEN)
				rtos_data(link, ev);
			break;

		case event_type::SENT:
		case event_type::SEND_FAILED:
			if ((m_mode == MODE_SEND_WAIT) && (link == m_send_link))
			{
				reply((ev.type == event_type::SENT) ? "\r\nSEND OK\r\n" : "\r\nSEND FAIL\r\n");
				m_mode = MODE_COMMAND;
			}
			break;

		case event_type::CLOSED:
			if (m_link_state[link] == LINK_IDLE)
				break;
			if (m_link_tcp[link] && (m_link_state[link] == LINK_OPEN))
			{
				m_tcp[link].fin = true;
				m_link_paused[link] = false;
				rtos_finish(link);
			}
			else
			{
				rtos_closed(link);
			}
			break;

		case event_type::LISTENING:
			osd_printf_verbose("[%s] AT+CIPSERVER port %u listening on %s\n", tag(), m_server_port, ev.peer);
			if (m_mode == MODE_LISTEN)
			{
				reply_ok();
				m_mode = MODE_COMMAND;
			}
			break;

		case event_type::LISTEN_FAILED:
			osd_printf_error("[%s] AT+CIPSERVER port %u: the host cannot listen on %s. Choose a different \"AT+CIPSERVER port offset\" in the Machine Configuration menu.\n", tag(), m_server_port, ev.peer);
			if (m_mode == MODE_LISTEN)
			{
				m_server_port = 0;
				m_server_up = false;
				reply_error();
				m_mode = MODE_COMMAND;
			}
			break;

		case event_type::ACCEPTED:
			{
				unsigned id = 0;
				while ((id < m_max_conn) && (m_link_state[id] != LINK_IDLE))
					id++;
				if (!m_server_up || m_listen_full || (id >= m_max_conn))
				{
					LOGLINK("server: no free link, connection refused\n");
					m_net->reject(ev.ticket);
					break;
				}
				LOGLINK("link %u: accepted\n", id);
				m_link_state[id] = LINK_OPEN;
				m_link_local_port[id] = m_server_port;
				set_remote(id, ev.peer);
				m_link_paused[id] = false;
				m_link_server[id] = true;
				m_link_tcp[id] = true;
				m_link_ex[id] = false;
				tcp_reset(id);
				m_link_active[id] = machine().time();
				m_net->adopt(id, ++m_link_generation[id], ev.ticket);
				rtos_tcpopt_apply(id);
				m_server_clients++;
				if (m_fw->status_closed)
					rtos_link_count();
				if (BIT(m_sysmsg, 1))
					reply(util::string_format(
							"+LINK_CONN:0,%u,\"TCP\",1,\"%s\",%u,%u\r\n",
							id, m_link_remote[id], m_link_remote_port[id], m_link_local_port[id]));
				else
					reply(util::string_format("%u,CONNECT\r\n", id));

				bool full = true;
				for (unsigned i = 0; i < m_max_conn; i++)
					full = full && (m_link_state[i] != LINK_IDLE);
				if (full)
				{
					LOGLINK("server: every link in use, listening stops\n");
					m_listen_full = true;
					m_net->stop_listen(++m_server_generation);
				}
			}
			break;

		case event_type::PING_SENT:
			if (!ev.peer.empty())
				logerror("AT+PING: the host cannot send ICMP echo requests: %s\n", ev.peer);
			if (m_mode == MODE_PING)
				m_ping_timer->adjust(attotime::from_msec(RTOS_PING_MS));
			break;

		case event_type::PING_FAILED:
			if (m_mode == MODE_PING)
			{
				if (m_fw->ping_wait)
					reply("+PING:TIMEOUT\r\n");
				reply_error();
				m_mode = MODE_COMMAND;
				stop_ping();
			}
			break;

		case event_type::PING_REPLY:
			if ((m_mode == MODE_PING) && m_fw->ping_wait)
			{
				m_ping_replied = true;
				m_ping_ms = ev.ticket;
			}
			else if (m_mode == MODE_PING)
			{
				// a reply time of 0 ms reads as a failure in the firmware
				if (ev.ticket >= RTOS_PING_MS)
				{
					reply("+PING:TIMEOUT\r\n");
					reply_error();
				}
				else if (!ev.ticket)
				{
					reply_error();
				}
				else
				{
					reply(util::string_format("+PING:%u\r\n", ev.ticket));
					reply_ok();
				}
				m_mode = MODE_COMMAND;
				stop_ping();
			}
			break;

		case event_type::RESOLVED:
			if (m_mode == MODE_SEND_RESOLVE)
			{
				if (ev.peer.empty() || (m_link_state[m_send_link] != LINK_OPEN))
				{
					reply_error();
					m_mode = MODE_COMMAND;
				}
				else
				{
					send_remote(ev.peer);
					rtos_send_prompt();
				}
			}
			else if ((m_mode == MODE_DOMAIN) && (m_cmd_name == "+SAVETRANSLINK"))
			{
				if (ev.peer.empty())
				{
					reply_error();
				}
				else
				{
					rtos_translink_store();
					reply_ok();
				}
				m_mode = MODE_COMMAND;
			}
			else if (m_mode == MODE_DOMAIN)
			{
				if (ev.peer.empty())
				{
					reply_error();
				}
				else
				{
					reply(util::string_format(m_fw->domain_quoted ? "+CIPDOMAIN:\"%s\"\r\n" : "+CIPDOMAIN:%s\r\n", ev.peer));
					reply_ok();
				}
				m_mode = MODE_COMMAND;
			}
			break;
		}
	}
}


void esp8266_at_device::rtos_send_start()
{
	unsigned const link = m_send_link;
	unsigned const length = m_send_buf.size();
	LOGLINK("link %u: sending %u bytes\n", link, length);
	reply(util::string_format("\r\nRecv %u bytes\r\n", length));
	if (m_link_state[link] != LINK_OPEN)
	{
		m_send_buf.clear();
		reply("\r\nSEND FAIL\r\n");
		m_mode = MODE_COMMAND;
		return;
	}
	m_net->send(link, m_link_generation[link], std::move(m_send_buf), true);
	m_send_buf.clear();
}


void esp8266_at_device::rtos_send_prompt()
{
	m_send_buf.clear();
	m_send_escape = false;
	m_mode = MODE_SEND_DATA;
	reply("\r\nOK\r\n\r\n>");
}


void esp8266_at_device::rtos_send(bool ex)
{
	if (m_cipmode)
	{
		reply_error();
		return;
	}
	unsigned i = 0;
	unsigned link = 0;
	s32 value;
	if (m_mux)
	{
		if ((rtos_digit(0, value) != PARA_OK) || (u32(value) >= LINK_COUNT))
		{
			reply_error();
			return;
		}
		link = unsigned(value);
		i = 1;
	}
	if (m_link_state[link] != LINK_OPEN)
	{
		reply_error();
		return;
	}
	s32 length;
	if ((rtos_digit(i, length) != PARA_OK) || (length > s32(MAX_SEND)))
	{
		reply_error();
		return;
	}

	std::string host;
	s32 port = 0;
	if (m_link_tcp[link] ? (m_para.size() > (i + 1)) : (m_para.size() > (i + 3)))
	{
		reply_error();
		return;
	}
	if (m_para.size() > (i + 1))
	{
		int const result = (m_para.size() > (i + 2)) ? rtos_digit(i + 2, port) : PARA_OMITTED;
		if ((rtos_str(i + 1, host) == PARA_FAIL) || (result == PARA_FAIL) || ((result == PARA_OK) && ((port < 1) || (port > 0xffff))))
		{
			reply_error();
			return;
		}
		if (result == PARA_OMITTED)
			port = 0;
	}
	if (length <= 0)
	{
		reply_error();
		return;
	}

	m_send_link = link;
	m_send_length = unsigned(length);
	m_send_port = u16(port);
	if (ex)
		m_link_ex[link] = true;
	m_send_rtos_ex = m_link_ex[link];
	m_link_active[link] = machine().time();
	if (host.empty())
	{
		rtos_send_prompt();
		return;
	}

	u32 const ip = fw_ipaddr(host);
	if ((ip != IPADDR_NONE) || (host == "255.255.255.255"))
	{
		send_remote(util::string_format("%u.%u.%u.%u", BIT(ip, 24, 8), BIT(ip, 16, 8), BIT(ip, 8, 8), BIT(ip, 0, 8)));
		rtos_send_prompt();
		return;
	}
	m_mode = MODE_SEND_RESOLVE;
	m_net->resolve(++m_domain_generation, host);
}


void esp8266_at_device::rtos_start(bool ex)
{
	unsigned const count = m_para.size();
	if ((count < 3) && !m_fw->strict_params)
	{
		rtos_err_code(0x0106'0100 | count);
		reply_error();
		return;
	}

	unsigned i = 0;
	s32 id = 0;
	if (m_mux && !ex)
	{
		if (rtos_digit(0, id) != PARA_OK)
		{
			reply_error();
			return;
		}
		i = 1;
	}

	std::string type, host, local;
	s32 port;
	if ((rtos_str(i, type) != PARA_OK) || ((type != "TCP") && (type != "UDP")) ||
			(rtos_str(i + 1, host) != PARA_OK) || host.empty() ||
			(rtos_digit(i + 2, port) != PARA_OK) || (u32(port) > 0xffff) || (!port && m_fw->port_nonzero))
	{
		reply_error();
		return;
	}

	esp8266_net::open_params params;
	params.udp = type == "UDP";
	params.host = host;
	params.port = u16(port);
	unsigned next = i + 3;
	s32 value;
	if (params.udp)
	{
		int result = (count > next) ? rtos_digit(next, value) : PARA_OMITTED;
		if ((result == PARA_FAIL) || ((result == PARA_OK) && ((value < 1) || (value > 0xffff))))
		{
			reply_error();
			return;
		}
		if (result == PARA_OK)
			params.local_port = u16(value);
		next++;
		result = (count > next) ? rtos_digit(next, value) : PARA_OMITTED;
		if ((result == PARA_FAIL) || ((result == PARA_OK) && (u32(value) > 2)))
		{
			reply_error();
			return;
		}
		if (result == PARA_OK)
			params.udp_mode = u8(value);
		next++;
	}
	else
	{
		int const result = (count > next) ? rtos_digit(next, value) : PARA_OMITTED;
		if ((result == PARA_FAIL) || ((result == PARA_OK) && (u32(value) > 7200)))
		{
			reply_error();
			return;
		}
		if (result == PARA_OK)
			params.keepalive = u16(value);
		next++;
	}
	if ((count > next) && (rtos_str(next++, local) == PARA_FAIL))
	{
		reply_error();
		return;
	}
	if ((count > next) || !m_cwmode || ((m_cwmode == 1) && !m_joined))
	{
		reply_error();
		return;
	}

	unsigned link = m_mux ? unsigned(id) : 0;
	if (m_mux && ex)
	{
		link = 0;
		while ((link < LINK_COUNT) && (m_link_state[link] != LINK_IDLE))
			link++;
	}
	// the firmware leaves its socket mutex taken on an open link, and hangs; the card carries on
	if ((link >= LINK_COUNT) || (m_link_state[link] != LINK_IDLE) || (params.udp && m_cipmode && params.udp_mode))
	{
		reply_error();
		return;
	}

	if (m_cipmode && !link)
		m_link_ex[link] = false;
	m_link_params[link] = params;
	m_connect_link = link;
	m_mode = MODE_CONNECTING;
	link_open(link, 0);
}


void esp8266_at_device::rtos_cmd_e0(u8 form)
{
	if (form != FORM_EXEC)
	{
		reply_error();
		return;
	}
	m_echo = 0;
	reply_ok();
}


void esp8266_at_device::rtos_cmd_e1(u8 form)
{
	if (form != FORM_EXEC)
	{
		reply_error();
		return;
	}
	m_echo = 1;
	reply_ok();
}


void esp8266_at_device::rtos_cmd_rst(u8 form)
{
	if (form != FORM_EXEC)
	{
		reply_error();
		return;
	}
	reply_ok();
	restart();
}


void esp8266_at_device::rtos_cmd_gmr(u8 form)
{
	if (form != FORM_EXEC)
	{
		reply_error();
		return;
	}
	reply(m_fw->gmr_text);
	reply_ok();
}


void esp8266_at_device::rtos_cmd_sysram(u8 form)
{
	if (form != FORM_QUERY)
	{
		reply_error();
		return;
	}
	int const heap = rtos_heap();
	m_heap_low = std::min(m_heap_low, heap);
	if (m_fw->sysram_low)
		reply(util::string_format("%s:%d,%d", m_cmd_name, heap, m_heap_low));
	else
		reply(util::string_format("%s:%d", m_cmd_name, heap));
	reply_ok();
}


int esp8266_at_device::rtos_heap()
{
	int heap = RTOS_HEAP_FREE - int(m_userram.size());
	unsigned links = 0;
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		if (m_link_state[i] != LINK_IDLE)
			links++;
		if (m_recv_mode)
			heap -= int(rtos_held(i));
	}
	if (links)
		heap -= RTOS_HEAP_LINK + (RTOS_HEAP_MORE_LINK * (links - 1));
	if (m_server_up)
		heap -= RTOS_HEAP_SERVER;
	return std::max(heap, 0);
}


void esp8266_at_device::rtos_cmd_sysmsg(u8 form)
{
	s32 value;
	if (form == FORM_QUERY)
	{
		reply(util::string_format("%s:%d", m_cmd_name, m_sysmsg));
		reply_ok();
	}
	else if ((form == FORM_SET) && m_fw->sysmsg_bit2)
	{
		if ((rtos_digit(0, value) != PARA_OK) || (m_para.size() != 1) || (u32(value) > 7))
		{
			reply_error();
			return;
		}
		m_sysmsg = value;
		if (rtos_store())
		{
			m_sysmsg_def = value;
			m_sysmsg_stored = true;
		}
		reply_ok();
	}
	else if ((form == FORM_SET) && (m_para.size() != 1))
	{
		rtos_err_code(0x0106'0100 | m_para.size());
		reply_error();
	}
	else if ((form == FORM_SET) && (rtos_digit(0, value) == PARA_OK))
	{
		m_sysmsg = value;
		m_sysmsg_stored = true;
		reply_ok();
	}
	else
	{
		reply_error();
	}
}


void esp8266_at_device::rtos_cmd_syslog(u8 form)
{
	s32 value;
	if (form == FORM_QUERY)
	{
		reply(util::string_format("%s:%u\r\n", m_cmd_name, m_syslog));
		reply_ok();
	}
	else if ((form == FORM_SET) && (m_para.size() == 1) && (rtos_digit(0, value) == PARA_OK) && (u32(value) <= 1))
	{
		m_syslog = u8(value);
		reply_ok();
	}
	else
	{
		reply_error();
	}
}


void esp8266_at_device::rtos_cmd_systimestamp(u8 form)
{
	s32 value;
	if (form == FORM_QUERY)
	{
		u64 const elapsed = (machine().time() - m_timestamp_base).as_ticks(1'000'000);
		u64 const seconds = ((u64(s64(m_timestamp)) * 1'000'000) + elapsed) / 1'000'000;
		reply(util::string_format("%s:%d", m_cmd_name, s32(u32(seconds))));
		reply_ok();
	}
	else if ((form == FORM_SET) && m_fw->strict_params)
	{
		if ((rtos_digit(0, value) != PARA_OK) || (m_para.size() != 1))
		{
			reply_error();
			return;
		}
		m_timestamp = value;
		m_timestamp_base = machine().time();
		reply_ok();
	}
	else if ((form == FORM_SET) && (m_para.size() != 1))
	{
		rtos_err_code(0x0106'0100 | m_para.size());
		reply_error();
	}
	else if ((form == FORM_SET) && (rtos_digit(0, value) == PARA_OK))
	{
		m_timestamp = value;
		m_timestamp_base = machine().time();
		reply_ok();
	}
	else
	{
		reply_error();
	}
}


void esp8266_at_device::rtos_cmd_sleep(u8 form)
{
	s32 value;
	if (form == FORM_QUERY)
	{
		reply(util::string_format("%s:%u\r\n", m_cmd_name, m_sleep));
		reply_ok();
	}
	else if ((form == FORM_SET) && m_fw->sleep_modes)
	{
		if ((rtos_digit(0, value) != PARA_OK) || (m_para.size() != 1) || (u32(value) > 3) || (value && (m_cwmode > 1)))
		{
			reply_error();
			return;
		}
		m_sleep = (value == 0) ? 0 : (value == 1) ? 1 : 3;
		reply_ok();
	}
	else if ((form == FORM_SET) && (m_para.size() != 1))
	{
		rtos_err_code(0x0106'0100 | m_para.size());
		reply_error();
	}
	else if ((form == FORM_SET) && (rtos_digit(0, value) == PARA_OK) && (u32(value) <= 2))
	{
		// firmware bug, copied: 1 and 2 swap on the way to the SDK, and the query shows the SDK value
		m_sleep = (value == 1) ? 2 : (value == 2) ? 1 : 0;
		reply_ok();
	}
	else
	{
		reply_error();
	}
}


void esp8266_at_device::rtos_cmd_uart(u8 form)
{
	if (form == FORM_QUERY)
	{
		if (m_cmd_name == "+UART_DEF")
		{
			// a stored parity 1 reads back as -1
			int const parity = ((m_def_frame[2] == 1) && !m_fw->uart_odd) ? -1 : m_def_frame[2];
			reply(util::string_format("%s:%u,%u,%u,%d,%u\r\n", m_cmd_name, m_def_rate, m_def_frame[0], m_def_frame[1], parity, m_def_frame[3]));
		}
		else
		{
			u32 const rate = RTOS_UART_CLOCK / (RTOS_UART_CLOCK / m_cur_rate);
			reply(util::string_format("%s:%u,%u,%u,%u,%u\r\n", m_cmd_name, rate, m_cur_frame[0], m_cur_frame[1], m_cur_frame[2], m_cur_frame[3]));
		}
		reply_ok();
		return;
	}

	static constexpr s32 low[] = { 80, 5, 1, 0, 0 };
	static constexpr s32 high[] = { 5'000'000, 8, 3, 2, 3 };
	s32 values[5];
	bool ok = (form == FORM_SET) && (m_para.size() == 5);
	for (unsigned i = 0; ok && (i < 5); i++)
		ok = (rtos_digit(i, values[i]) == PARA_OK) && (values[i] >= low[i]) && (values[i] <= high[i]);
	if (!ok || (u32(values[0]) > CARD_MAX_BAUD))
	{
		reply_error();
		return;
	}

	if (m_cmd_name != "+UART_CUR")
	{
		m_def_stored = true;
		m_def_rate = u32(values[0]);
		for (unsigned i = 0; i < 4; i++)
			m_def_frame[i] = u8(values[i + 1]);
	}
	reply_ok();
	m_cur_rate = u32(values[0]);
	m_cur_frame[0] = u8(values[1]);
	m_cur_frame[1] = u8(values[2]);
	// the driver refuses odd parity as the firmware passes it, and keeps the old setting
	if ((values[3] != 1) || m_fw->uart_odd)
		m_cur_frame[2] = u8(values[3]);
	m_cur_frame[3] = u8(values[4]);
	m_serial_dirty = true;
}


void esp8266_at_device::rtos_cmd_cwmode(u8 form)
{
	s32 value;
	if (form == FORM_TEST)
	{
		reply(m_cmd_name + ":(1-3)\r\n");
		reply_ok();
	}
	else if (form == FORM_QUERY)
	{
		reply(util::string_format("%s:%u\r\n", m_cmd_name, m_cwmode));
		reply_ok();
	}
	else if (form != FORM_SET)
	{
		reply_error();
	}
	else if (m_fw->cwmode_autoconn)
	{
		s32 autoconn = 1;
		if (rtos_digit(0, value) != PARA_OK)
		{
			reply_error();
			return;
		}
		if (u32(value) > 3)
		{
			rtos_err_code(0x0107'0001);
			reply_error();
			return;
		}
		int const result = (m_para.size() > 1) ? rtos_digit(1, autoconn) : PARA_OMITTED;
		if ((m_para.size() > 2) || (result == PARA_FAIL) || (u32(autoconn) > 1))
		{
			reply_error();
			return;
		}
		if (result == PARA_OMITTED)
			autoconn = 1;
		if (u32(value) == m_cwmode)
		{
			reply_ok();
			return;
		}
		bool const join = BIT(value, 0) && !BIT(m_cwmode, 0) && autoconn && m_ssid[0];
		m_cwmode = u8(value);
		if (rtos_store())
		{
			m_cwmode_def = m_cwmode;
			m_wifi_stored = true;
		}
		if (!BIT(m_cwmode, 0))
			m_joined = 0;
		reply_ok();
		if (join)
		{
			m_joined = 1;
			if (!rtos_wifi_quiet())
				reply("WIFI CONNECTED\r\nWIFI GOT IP\r\n");
			rtos_link_count();
		}
	}
	else if (m_para.size() != 1)
	{
		rtos_err_code(0x0106'0100 | m_para.size());
		reply_error();
	}
	else if (rtos_digit(0, value) != PARA_OK)
	{
		reply_error();
	}
	else if (u32(value) > 3)
	{
		rtos_err_code(0x0107'0001);
		reply_error();
	}
	else
	{
		m_cwmode = m_cwmode_def = u8(value);
		m_wifi_stored = true;
		if (!BIT(m_cwmode, 0))
			m_joined = 0;
		reply_ok();
	}
}


void esp8266_at_device::rtos_cmd_cwjap(u8 form)
{
	if (m_fw->cwjap_nine && (form == FORM_QUERY))
	{
		if (m_joined && m_ssid[0])
			reply(util::string_format(
					"%s:\"%s\",\"%s\",%d,%d,%u,%u,%u,%u,%u\r\n",
					m_cmd_name, m_ssid, AP_BSSID, AP_CHANNEL, AP_RSSI, m_jap_pci, m_reconn_interval, m_jap_listen, m_jap_scan, m_jap_pmf));
		reply_ok();
		return;
	}
	if (m_fw->cwjap_nine && (form == FORM_EXEC))
	{
		if (m_joined && BIT(m_cwmode, 0))
			reply_ok();
		else if (!BIT(m_cwmode, 0) || !m_ssid[0])
			reply_error();
		else
			rtos_join();
		return;
	}
	if (m_fw->cwjap_nine)
	{
		if (!BIT(m_cwmode, 0))
		{
			reply_error();
			return;
		}
		std::string ssid, password, bssid;
		int const ssid_result = rtos_str(0, ssid);
		if ((ssid_result == PARA_FAIL) || ((ssid_result == PARA_OK) && (ssid.empty() || (ssid.size() > MAX_SSID))) ||
				((ssid_result == PARA_OMITTED) && !m_ssid[0]))
		{
			reply_error();
			return;
		}
		int const password_result = rtos_str(1, password);
		if ((password_result == PARA_FAIL) || ((ssid_result == PARA_OMITTED) && (password_result != PARA_OMITTED)) || (password.size() > MAX_PASSWORD))
		{
			reply_error();
			return;
		}
		if (ssid_result == PARA_OMITTED)
		{
			ssid = m_ssid;
			password = m_password;
		}
		u8 mac[6];
		unsigned next = 2;
		if ((m_para.size() > next) && ((rtos_str(next, bssid) == PARA_FAIL) || (!bssid.empty() && !parse_mac(bssid, mac))))
		{
			reply_error();
			return;
		}
		next++;
		// an empty listen interval field reads 0, not the default 3
		s32 values[6][3] = { { 0, 0, 1 }, { 1, 0, 7'200 }, { 3, 1, 100 }, { 0, 0, 1 }, { 15, 3, 600 }, { 0, 0, 3 } };
		for (unsigned i = 0; i < 6; i++, next++)
		{
			if (m_para.size() <= next)
				break;
			int const result = rtos_digit(next, values[i][0]);
			if (result == PARA_OMITTED)
			{
				if (i == 2)
					values[i][0] = 0;
			}
			else if ((result == PARA_FAIL) || (values[i][0] < values[i][1]) || (values[i][0] > values[i][2]))
			{
				reply_error();
				return;
			}
		}
		if (m_para.size() > 9)
		{
			reply_error();
			return;
		}

		if (m_joined)
			rtos_close_clients();
		std::fill(std::begin(m_ssid), std::end(m_ssid), 0);
		std::copy(ssid.begin(), ssid.end(), std::begin(m_ssid));
		std::fill(std::begin(m_password), std::end(m_password), 0);
		std::copy(password.begin(), password.end(), std::begin(m_password));
		m_jap_pci = u8(values[0][0]);
		m_reconn_interval = u16(values[1][0]);
		m_jap_listen = u8(values[2][0]);
		m_jap_scan = u8(values[3][0]);
		m_jap_pmf = u8(values[5][0]);
		if (rtos_store())
		{
			std::copy(std::begin(m_ssid), std::end(m_ssid), std::begin(m_ssid_def));
			std::copy(std::begin(m_password), std::end(m_password), std::begin(m_password_def));
			m_jap_def[0] = m_jap_pci;
			m_jap_def[1] = m_jap_listen;
			m_jap_def[2] = m_jap_scan;
			m_jap_def[3] = m_jap_pmf;
			m_reconn_def[0] = m_reconn_interval;
			m_wifi_stored = true;
		}
		LOGLINK("station joined \"%s\"\n", ssid);
		rtos_join();
		return;
	}
	if (form == FORM_QUERY)
	{
		if (m_joined && m_ssid[0])
			reply(util::string_format("%s:\"%s\",\"%s\",%d,%d,%u,%u,%u\r\n", m_cmd_name, m_ssid, AP_BSSID, AP_CHANNEL, AP_RSSI, m_jap_pci, m_jap_reconn, m_jap_listen));
		reply_ok();
		return;
	}
	if ((form != FORM_SET) || (m_para.size() > 6))
	{
		reply_error();
		return;
	}

	std::string ssid, password, bssid;
	s32 values[3] = { 0, 0, 0 };
	static constexpr s32 high[] = { 1, 1, 99 };
	u8 mac[6] = { 0, 0, 0, 0, 0, 0 };
	if (rtos_str(0, ssid) == PARA_FAIL)
	{
		reply_error();
		return;
	}
	if (ssid.empty() || (ssid.size() > MAX_SSID))
	{
		rtos_err_code(0x0104'0001);
		reply_error();
		return;
	}
	if (rtos_str(1, password) == PARA_FAIL)
	{
		reply_error();
		return;
	}
	if (password.size() > 64)
	{
		rtos_err_code(0x0104'0002);
		reply_error();
		return;
	}
	if ((m_para.size() > 2) && ((rtos_str(2, bssid) == PARA_FAIL) || (!bssid.empty() && !parse_mac(bssid, mac))))
	{
		if (!bssid.empty())
			rtos_err_code(0x0104'0003);
		reply_error();
		return;
	}
	for (unsigned i = 0; i < 3; i++)
	{
		int const result = (m_para.size() > (i + 3)) ? rtos_digit(i + 3, values[i]) : PARA_OMITTED;
		if (result == PARA_OMITTED)
			values[i] = 0;
		if ((result == PARA_FAIL) || (u32(values[i]) > u32(high[i])))
		{
			if (result != PARA_FAIL)
				rtos_err_code(0x0104'0000);
			reply_error();
			return;
		}
	}
	if (!BIT(m_cwmode, 0))
	{
		rtos_err_code(0x010a'0000);
		reply_error();
		return;
	}

	if (m_joined)
		rtos_close_clients();
	std::fill(std::begin(m_ssid), std::end(m_ssid), 0);
	std::copy(ssid.begin(), ssid.end(), std::begin(m_ssid));
	std::copy(std::begin(m_ssid), std::end(m_ssid), std::begin(m_ssid_def));
	m_jap_pci = u8(values[0]);
	m_jap_reconn = u8(values[1]);
	m_jap_listen = u8(values[2]);
	m_wifi_stored = true;

	LOGLINK("station joined \"%s\"\n", ssid);
	m_joined = 1;
	if (m_status == STATUS_NO_WIFI)
		m_status = STATUS_GOT_IP;
	if (!m_cipmode)
		reply("WIFI CONNECTED\r\nWIFI GOT IP\r\n");
	m_mode = MODE_JOIN;
	m_join_timer->adjust(attotime::from_msec(CWJAP_TICK_MS));
}


void esp8266_at_device::rtos_cmd_cwlap(u8 form)
{
	if ((form != FORM_EXEC) && (form != FORM_SET))
	{
		reply_error();
		return;
	}
	if (!BIT(m_cwmode, 0))
	{
		if (!m_fw->strict_params)
			rtos_err_code(0x010a'0000);
		reply_error();
		return;
	}

	std::string ssid, mac_text;
	s32 values[4] = { 0, 0, 0, 0 };
	static constexpr s32 high[] = { 13, 1, 1500, 1500 };
	u8 mac[6] = { 0, 0, 0, 0, 0, 0 };
	if (form == FORM_SET)
	{
		if ((m_para.size() > 6) || (rtos_str(0, ssid) == PARA_FAIL) ||
				((m_para.size() > 1) && (rtos_str(1, mac_text) == PARA_FAIL)))
		{
			reply_error();
			return;
		}
		if ((ssid.size() > MAX_SSID) || (!mac_text.empty() && !parse_mac(mac_text, mac)))
		{
			if (!m_fw->strict_params)
				rtos_err_code((ssid.size() > MAX_SSID) ? 0x0107'0001 : 0x0107'0002);
			reply_error();
			return;
		}
		for (unsigned i = 0; i < 4; i++)
		{
			int const result = (m_para.size() > (i + 2)) ? rtos_digit(i + 2, values[i]) : PARA_OMITTED;
			if (result == PARA_OMITTED)
				values[i] = 0;
			if ((result == PARA_FAIL) || (u32(values[i]) > u32(high[i])))
			{
				if ((result != PARA_FAIL) && !m_fw->strict_params)
					rtos_err_code(0x0107'0003 + i);
				reply_error();
				return;
			}
		}
	}

	bool const match =
			(ssid.empty() || (ssid == m_ssid)) &&
			(mac_text.empty() || (util::string_format("%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]) == AP_BSSID)) &&
			(!values[0] || (values[0] == AP_CHANNEL));
	if (match && m_ssid[0])
		m_scan_text = util::string_format(
				m_fw->cwlap_eleven ? "+CWLAP:(%d,\"%s\",%d,\"%s\",%d,-1,-1,4,4,7,0)\r\n" : "+CWLAP:(%d,\"%s\",%d,\"%s\",%d)\r\n",
				AP_ECN, m_ssid, AP_RSSI, AP_BSSID, AP_CHANNEL);
	else
		m_scan_text.clear();
	m_mode = MODE_SCAN;
	m_join_timer->adjust(attotime::from_msec(m_fw->scan_ms));
}


void esp8266_at_device::rtos_cmd_cwqap(u8 form)
{
	if (form == FORM_TEST)
	{
		reply_ok();
	}
	else if ((form == FORM_EXEC) && BIT(m_cwmode, 0))
	{
		reply_ok();
		if (m_joined)
		{
			LOGLINK("station disconnected\n");
			m_joined = 0;
			if (!rtos_wifi_quiet())
				reply("WIFI DISCONNECT\r\n");
			rtos_close_clients();
		}
	}
	else
	{
		reply_error();
	}
}


void esp8266_at_device::rtos_cmd_cwdhcp(u8 form)
{
	s32 enable, mask;
	if (form == FORM_QUERY)
	{
		// station DHCP is bit 0 here and bit 1 in the card's state
		reply(util::string_format("%s:%u", m_cmd_name, (BIT(m_dhcp, 1) ? 1 : 0) | (BIT(m_dhcp, 0) ? 2 : 0)));
		reply_ok();
	}
	else if (form != FORM_SET)
	{
		reply_error();
	}
	else if (m_para.size() != 2)
	{
		rtos_err_code(0x0106'0200 | m_para.size());
		reply_error();
	}
	else if ((rtos_digit(0, enable) != PARA_OK) || (rtos_digit(1, mask) != PARA_OK))
	{
		reply_error();
	}
	else if ((u32(enable) > 1) || (mask < 1) || (mask > 3))
	{
		rtos_err_code((u32(enable) > 1) ? 0x0107'0001 : 0x0107'0002);
		reply_error();
	}
	else
	{
		u8 const bits = (BIT(mask, 0) ? 2 : 0) | (BIT(mask, 1) ? 1 : 0);
		if (enable)
		{
			m_dhcp |= bits;
			if (BIT(bits, 1))
				m_sta_static = 0;
		}
		else
		{
			m_dhcp &= ~bits;
		}
		if (!BIT(m_dhcp, 1) && m_sta_static_def)
		{
			std::copy(std::begin(m_sta_ip_def), std::end(m_sta_ip_def), std::begin(m_sta_ip));
			m_sta_static = 1;
		}
		if (rtos_store())
		{
			m_dhcp_def = m_dhcp;
			m_wifi_stored = true;
		}
		reply_ok();
	}
}


void esp8266_at_device::rtos_cmd_cipsta(u8 form)
{
	if (form == FORM_QUERY)
	{
		u8 info[12];
		station_info(info);
		reply(util::string_format(
				"%1$s:ip:\"%2$s\"\r\n%1$s:gateway:\"%3$s\"\r\n%1$s:netmask:\"%4$s\"\r\n",
				m_cmd_name, ip_text(&info[0]), ip_text(&info[4]), ip_text(&info[8])));
		reply_ok();
		return;
	}

	std::optional<u32> values[3] = { std::nullopt, 0, 0 };
	bool ok = (form == FORM_SET) && (m_para.size() <= 3);
	for (unsigned i = 0; ok && (i < m_para.size()); i++)
	{
		std::string text;
		ok = (rtos_str(i, text) == PARA_OK) && (text.empty() ? (i != 0) : bool(values[i] = parse_ip(text)));
	}
	if (!ok)
	{
		reply_error();
		return;
	}

	u32 const ip = *values[0];
	u32 gateway = *values[1];
	u32 netmask = *values[2];
	if (ip && !gateway)
		gateway = (ip & 0xffff'ff00) | 1;
	if (ip && !netmask)
		netmask = 0xffff'ff00;
	put_u32be(&m_sta_ip[0], ip);
	put_u32be(&m_sta_ip[4], gateway);
	put_u32be(&m_sta_ip[8], netmask);
	m_sta_static = 1;
	m_dhcp &= ~2;
	if (rtos_store())
	{
		std::copy(std::begin(m_sta_ip), std::end(m_sta_ip), std::begin(m_sta_ip_def));
		m_sta_static_def = 1;
		m_dhcp_def = m_dhcp;
		m_wifi_stored = true;
	}
	reply_ok();
}


void esp8266_at_device::rtos_cmd_cifsr(u8 form)
{
	if (form != FORM_EXEC)
	{
		reply_error();
		return;
	}
	reply(util::string_format("+CIFSR:STAIP,\"%s\"\r\n+CIFSR:STAMAC,\"%s\"\r\n", station_ip(), STATION_MAC));
	reply_ok();
}


void esp8266_at_device::rtos_cmd_ping(u8 form)
{
	std::string host;
	if ((form != FORM_SET) || (m_para.size() != 1) || (rtos_str(0, host) != PARA_OK) || host.empty() || !m_joined)
	{
		reply_error();
		return;
	}
	LOGLINK("ping %s\n", host);
	m_ping_replied = false;
	m_ping_ms = 0;
	m_ping_timer->adjust(attotime::never);
	m_mode = MODE_PING;
	m_net->ping(++m_ping_generation, host);
}


void esp8266_at_device::rtos_cmd_cipdomain(u8 form)
{
	std::string host;
	s32 type = 1;
	if ((form != FORM_SET) || (rtos_str(0, host) != PARA_OK) || host.empty() || (m_para.size() > (m_fw->domain_quoted ? 2 : 1)) ||
			((m_para.size() > 1) && ((rtos_digit(1, type) != PARA_OK) || (type < 1) || (type > 3))))
	{
		reply_error();
		return;
	}
	// the build has no IPv6, so an IPv6-only lookup fails
	if (type == 2)
	{
		reply_error();
		return;
	}
	u32 const ip = fw_ipaddr(host);
	if ((ip != IPADDR_NONE) || (host == "255.255.255.255"))
	{
		reply(util::string_format(m_fw->domain_quoted ? "%s:\"%u.%u.%u.%u\"\r\n" : "%s:%u.%u.%u.%u\r\n", m_cmd_name, BIT(ip, 24, 8), BIT(ip, 16, 8), BIT(ip, 8, 8), BIT(ip, 0, 8)));
		reply_ok();
		return;
	}
	if (!m_joined)
	{
		reply_error();
		return;
	}
	LOGLINK("AT+CIPDOMAIN %s\n", host);
	m_mode = MODE_DOMAIN;
	m_net->resolve(++m_domain_generation, host);
}


void esp8266_at_device::rtos_cmd_cipstatus(u8 form)
{
	if (form != FORM_EXEC)
	{
		reply_error();
		return;
	}
	m_status = status_code();
	reply(util::string_format("STATUS:%u\r\n", m_status));
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		if (m_link_state[i] == LINK_OPEN)
			reply(util::string_format(
					"+CIPSTATUS:%u,\"%s\",\"%s\",%u,%u,%u\r\n",
					i, m_link_tcp[i] ? "TCP" : "UDP", m_link_remote[i], m_link_remote_port[i], m_link_local_port[i], m_link_server[i] ? 1 : 0));
	}
	reply_ok();
}


void esp8266_at_device::rtos_cmd_cipstart(u8 form)
{
	if (form == FORM_SET)
		rtos_start(false);
	else
		reply_error();
}


void esp8266_at_device::rtos_cmd_cipstartex(u8 form)
{
	if (form == FORM_SET)
		rtos_start(true);
	else
		reply_error();
}


void esp8266_at_device::rtos_cmd_cipclose(u8 form)
{
	s32 id;
	if (form == FORM_EXEC)
	{
		if (m_mux || (m_link_state[0] == LINK_IDLE))
		{
			reply_error();
			return;
		}
		bool const server = m_link_server[0];
		link_close(0);
		closed_notice(0);
		if (server)
			rtos_client_gone(0);
		rtos_link_count();
		reply_ok();
	}
	else if ((form != FORM_SET) || !m_mux || (m_para.size() != 1) || (rtos_digit(0, id) != PARA_OK) || (u32(id) > LINK_COUNT) ||
			((u32(id) < LINK_COUNT) && (m_link_state[id] == LINK_IDLE)))
	{
		reply_error();
	}
	else
	{
		for (unsigned i = 0; i < LINK_COUNT; i++)
		{
			if (((u32(id) == LINK_COUNT) || (u32(id) == i)) && (m_link_state[i] != LINK_IDLE))
			{
				bool const server = m_link_server[i];
				link_close(i);
				closed_notice(i);
				if (server)
					rtos_client_gone(i);
			}
		}
		rtos_link_count();
		reply_ok();
	}
}


void esp8266_at_device::rtos_cmd_cipsend(u8 form)
{
	if (form == FORM_TEST)
	{
		reply_ok();
	}
	else if (form == FORM_SET)
	{
		rtos_send(false);
	}
	else if ((form == FORM_EXEC) && !m_mux && m_cipmode && (m_link_state[0] == LINK_OPEN))
	{
		reply("\r\nOK\r\n\r\n>");
		m_passthrough_buf.clear();
		m_mode = MODE_PASSTHROUGH;
		LOGLINK("link 0: passthrough on\n");
	}
	else
	{
		reply_error();
	}
}


void esp8266_at_device::rtos_cmd_cipsendex(u8 form)
{
	if (form == FORM_TEST)
		reply_ok();
	else if (form == FORM_SET)
		rtos_send(true);
	else
		reply_error();
}


void esp8266_at_device::rtos_cmd_cipdinfo(u8 form)
{
	s32 value;
	if (form == FORM_QUERY)
	{
		// no line end before the OK
		reply(util::string_format("%s:%s", m_cmd_name, m_dinfo ? "true" : "false"));
		reply_ok();
	}
	else if ((form == FORM_SET) && (m_para.size() == 1) && (rtos_digit(0, value) == PARA_OK) && (u32(value) <= 1))
	{
		m_dinfo = u8(value);
		reply_ok();
	}
	else
	{
		reply_error();
	}
}


void esp8266_at_device::rtos_cmd_cipmux(u8 form)
{
	s32 value;
	if (form == FORM_QUERY)
	{
		reply(util::string_format("%s:%u\r\n", m_cmd_name, m_mux));
		reply_ok();
		return;
	}
	bool tcp = false;
	for (unsigned i = 0; i < LINK_COUNT; i++)
		tcp = tcp || ((m_link_state[i] == LINK_OPEN) && m_link_tcp[i]);
	if ((form != FORM_SET) || (m_fw->cipmux_clients ? tcp : (m_status == STATUS_CONNECTED)) || (m_para.size() != 1) || (rtos_digit(0, value) != PARA_OK))
	{
		reply_error();
		return;
	}
	bool const links = std::find_if(std::begin(m_link_state), std::end(m_link_state), [] (u8 s) { return s != LINK_IDLE; }) != std::end(m_link_state);
	if ((value == 1) ? m_cipmode : ((value != 0) || m_server_up || links))
	{
		reply_error();
		return;
	}
	m_mux = u8(value);
	reply_ok();
}


void esp8266_at_device::rtos_cmd_ciprecvmode(u8 form)
{
	s32 value;
	if (form == FORM_QUERY)
	{
		reply(util::string_format("%s:%u\r\n", m_cmd_name, m_recv_mode));
		reply_ok();
	}
	else if ((form == FORM_SET) && (m_para.size() == 1) && (rtos_digit(0, value) == PARA_OK) && (u32(value) <= 1))
	{
		m_recv_mode = u8(value);
		if (!m_recv_mode)
			std::fill(std::begin(m_ipd_notice), std::end(m_ipd_notice), false);
		reply_ok();
		for (unsigned i = 0; m_fw->recvmode_wake && m_recv_mode && (i < LINK_COUNT); i++)
			rtos_notice(i);
	}
	else
	{
		reply_error();
	}
}


void esp8266_at_device::rtos_cmd_ciprecvdata(u8 form)
{
	unsigned link = 0;
	unsigned i = 0;
	s32 value, length;
	if ((form != FORM_SET) || !m_recv_mode)
	{
		reply_error();
		return;
	}
	if (m_mux)
	{
		if ((rtos_digit(0, value) != PARA_OK) || (u32(value) >= LINK_COUNT))
		{
			reply_error();
			return;
		}
		link = unsigned(value);
		i = 1;
	}
	unsigned const held = rtos_held(link);
	if ((rtos_digit(i, length) != PARA_OK) || (length < 1) || (m_para.size() != (i + 1)) || !held || (!m_link_tcp[link] && !m_fw->udp_recvdata))
	{
		reply_error();
		return;
	}

	if (!m_link_tcp[link])
	{
		// each read takes a whole datagram; the part of one that does not fit is lost
		auto &hold = m_udp_hold[link];
		unsigned left = std::min<unsigned>(unsigned(length), held);
		std::string peer = hold.front().first;
		std::vector<u8> data;
		while (left && !hold.empty())
		{
			unsigned const take = std::min<unsigned>(left, hold.front().second.size());
			data.insert(data.end(), hold.front().second.begin(), hold.front().second.begin() + take);
			hold.pop_front();
			left -= take;
		}
		std::string head = util::string_format("%s:%u,", m_cmd_name, unsigned(data.size()));
		if (m_dinfo)
			head += rtos_peer(peer).substr(1) + ',';
		reply(head);
		send_raw(data.data(), data.size());
		reply_ok();
		if (m_link_paused[link] && (hold.size() < RTOS_UDP_MBOX))
		{
			m_link_paused[link] = false;
			m_net->resume(link, m_link_generation[link]);
		}
		m_ipd_notice[link] = false;
		rtos_notice(link);
		return;
	}

	tcp_model &tcp = m_tcp[link];
	unsigned const count = std::min<unsigned>(unsigned(length), held);
	std::vector<u8> const data(tcp.remote.begin(), tcp.remote.begin() + count);
	tcp.remote.erase(tcp.remote.begin(), tcp.remote.begin() + count);
	std::string head = util::string_format("%s:%u,", m_cmd_name, count);
	if (m_dinfo)
		head += util::string_format("\"%s\",%u,", m_link_remote[link], m_link_remote_port[link]);
	reply(head);
	send_raw(data.data(), data.size());
	reply_ok();
	if (m_link_paused[link] && (tcp.remote.size() < m_fw->tcp_wnd))
	{
		m_link_paused[link] = false;
		m_net->resume(link, m_link_generation[link]);
	}
	m_ipd_notice[link] = false;
	rtos_notice(link);
	rtos_finish(link);
}


void esp8266_at_device::rtos_cmd_ciprecvlen(u8 form)
{
	if (form != FORM_QUERY)
	{
		reply_error();
		return;
	}
	std::string text = m_cmd_name + ':';
	for (unsigned i = 0; i < (m_mux ? LINK_COUNT : 1); i++)
	{
		if (i)
			text += ',';
		text += (m_link_state[i] == LINK_OPEN) ? util::string_format("%u", rtos_held(i)) : std::string("-1");
	}
	reply(text + "\r\n");
	reply_ok();
}


void esp8266_at_device::rtos_cmd_cipserver(u8 form)
{
	if (m_fw->cipserver_port && (form == FORM_QUERY))
	{
		if (m_server_up)
			reply(util::string_format("%s:1,%u,\"TCP\"\r\n", m_cmd_name, m_server_port));
		else
			reply(m_cmd_name + ":0\r\n");
		reply_ok();
		return;
	}
	if (m_fw->cipserver_port)
	{
		s32 mode, value = DEFAULT_SERVER_PORT;
		std::string type;
		unsigned expect = 1;
		if (!m_mux || (rtos_digit(0, mode) != PARA_OK) || (u32(mode) > 1))
		{
			reply_error();
			return;
		}
		if ((mode == 1) && (m_para.size() > 1))
		{
			int const result = rtos_digit(1, value);
			if (result == PARA_OMITTED)
				value = DEFAULT_SERVER_PORT;
			if ((result == PARA_FAIL) || (value < 1) || (value > 0xffff))
			{
				reply_error();
				return;
			}
			expect = 2;
			if (m_para.size() > 2)
			{
				if ((rtos_str(2, type) == PARA_FAIL) || (!type.empty() && (type != "TCP")))
				{
					reply_error();
					return;
				}
				expect = 3;
			}
		}
		else if (!mode && (m_para.size() > 1))
		{
			if ((rtos_digit(1, value) != PARA_OK) || (u32(value) > 1))
			{
				reply_error();
				return;
			}
			expect = 2;
		}
		if (m_para.size() != expect)
		{
			reply_error();
			return;
		}

		if (mode)
		{
			m_server_port = u16(value);
			if (m_server_up)
			{
				reply_ok();
				return;
			}
			m_server_up = true;
			m_listen_full = false;
			m_server_clients = 0;
			m_mode = MODE_LISTEN;
			rtos_listen();
			return;
		}
		if (m_server_up)
			server_close();
		m_listen_full = false;
		// firmware bug, copied: =0,1 closes every link, client links too
		if ((expect == 2) && value)
		{
			for (unsigned i = 0; i < LINK_COUNT; i++)
			{
				if (m_link_state[i] != LINK_IDLE)
				{
					bool const server = m_link_server[i];
					link_close(i);
					closed_notice(i);
					if (server)
						rtos_client_gone(i);
				}
			}
			rtos_link_count();
		}
		reply_ok();
		return;
	}
	if (form == FORM_QUERY)
	{
		// with a server running the firmware prints an uninitialised buffer
		if (!m_server_up)
			reply(m_cmd_name + ":0\r\n");
		reply_ok();
		return;
	}

	s32 mode, port = DEFAULT_SERVER_PORT;
	if ((form != FORM_SET) || !m_mux || (rtos_digit(0, mode) != PARA_OK))
	{
		reply_error();
		return;
	}
	if (mode == 1)
	{
		int const result = (m_para.size() > 1) ? rtos_digit(1, port) : PARA_OMITTED;
		if (result == PARA_OMITTED)
			port = DEFAULT_SERVER_PORT;
		if ((m_para.size() > 2) || (result == PARA_FAIL) || (port < 1) || (port > 0xffff))
		{
			reply_error();
			return;
		}
	}
	else if (m_para.size() > 1)
	{
		reply_error();
		return;
	}

	if (!mode)
	{
		if (m_server_up)
			server_close();
		m_listen_full = false;
		reply_ok();
	}
	else if (m_server_up)
	{
		reply_ok();
	}
	else
	{
		m_server_port = (mode == 1) ? u16(port) : 0;
		m_server_up = true;
		m_listen_full = false;
		m_mode = MODE_LISTEN;
		rtos_listen();
	}
}


void esp8266_at_device::rtos_cmd_cipservermaxconn(u8 form)
{
	s32 value;
	if (form == FORM_QUERY)
	{
		reply(util::string_format("%s:%u\r\n", m_cmd_name, m_max_conn));
		reply_ok();
	}
	else if ((form != FORM_SET) || (m_para.size() != 1) || (rtos_digit(0, value) != PARA_OK) || m_server_up)
	{
		reply_error();
	}
	else if (value < s32(m_server_clients))
	{
		reply(util::string_format("Have %u Connections\r\n", m_server_clients));
		reply_error();
	}
	else if ((value < 1) || (value > s32(LINK_COUNT)))
	{
		reply_error();
	}
	else
	{
		m_max_conn = u8(value);
		reply_ok();
	}
}


void esp8266_at_device::rtos_cmd_cipmode(u8 form)
{
	s32 value;
	if (form == FORM_QUERY)
	{
		reply(util::string_format("%s:%u\r\n", m_cmd_name, m_cipmode));
		reply_ok();
	}
	else if ((form != FORM_SET) || m_mux || (m_para.size() != 1) || (rtos_digit(0, value) != PARA_OK) || (u32(value) > 1) ||
			(value && (m_link_state[0] == LINK_OPEN) && !m_link_tcp[0] && m_link_params[0].udp_mode))
	{
		reply_error();
	}
	else
	{
		m_cipmode = u8(value);
		if (m_cipmode && (m_link_state[0] == LINK_OPEN))
			m_link_ex[0] = false;
		reply_ok();
	}
}


void esp8266_at_device::rtos_cmd_cipsto(u8 form)
{
	s32 value;
	if (form == FORM_QUERY)
	{
		reply(util::string_format("%s:%u\r\n", m_cmd_name, m_server_timeout));
		reply_ok();
	}
	else if ((form != FORM_SET) || !m_server_up || (m_para.size() != 1) || (rtos_digit(0, value) != PARA_OK) || (u32(value) > 7200))
	{
		reply_error();
	}
	else
	{
		if (m_fw->sto_reset && (u16(value) != m_server_timeout))
			std::fill(std::begin(m_idle), std::end(m_idle), 0);
		m_server_timeout = u16(value);
		if (!m_server_timeout)
		{
			m_sto_next = attotime::never;
			std::fill(std::begin(m_idle), std::end(m_idle), 0);
		}
		else if (m_sto_next == attotime::never)
		{
			m_sto_next = machine().time() + attotime::from_seconds(1);
		}
		reply_ok();
	}
}


void esp8266_at_device::rtos_cmd_savetranslink(u8 form)
{
	s32 mode;
	if (m_fw->translink_keepalive)
	{
		std::string host, type;
		s32 port = 0, value = 0;
		if ((form != FORM_SET) || (rtos_digit(0, mode) != PARA_OK))
		{
			reply_error();
			return;
		}
		if (mode != 1)
		{
			if (m_para.size() != 1)
			{
				reply_error();
				return;
			}
			m_tl_mode = 0;
			m_translink_stored = true;
			reply_ok();
			return;
		}
		bool ok = (rtos_str(1, host) == PARA_OK) && !host.empty() && (host.size() <= MAX_TRANSLINK_HOST) &&
				(rtos_digit(2, port) == PARA_OK) && (port >= 1) && (port <= 0xffff) &&
				((m_para.size() <= 3) || (rtos_str(3, type) != PARA_FAIL)) &&
				(type.empty() || (type == "TCP") || (type == "UDP")) && (m_para.size() <= 5);
		bool const udp = type == "UDP";
		int const result = (ok && (m_para.size() > 4)) ? rtos_digit(4, value) : PARA_OMITTED;
		if (result == PARA_OMITTED)
			value = 0;
		ok = ok && (result != PARA_FAIL) && (udp ? ((result == PARA_OMITTED) || ((value >= 1) && (value <= 0xffff))) : (u32(value) <= 7200));
		if (!ok)
		{
			reply_error();
			return;
		}
		m_tl_pending.udp = udp;
		m_tl_pending.host = host;
		m_tl_pending.port = u16(port);
		m_tl_pending.keepalive = udp ? 0 : u16(value);
		m_tl_pending.local_port = udp ? u16(value) : 0;
		if ((fw_ipaddr(host) != IPADDR_NONE) || (host == "255.255.255.255"))
		{
			rtos_translink_store();
			reply_ok();
			return;
		}
		m_mode = MODE_DOMAIN;
		m_net->resolve(++m_domain_generation, host);
		return;
	}
	if ((form != FORM_SET) || (rtos_digit(0, mode) != PARA_OK) || ((mode != 1) && (m_para.size() != 1)))
	{
		reply_error();
		return;
	}

	std::string host, type;
	s32 port = 0;
	s32 values[3] = { 0, 0, 1 };
	u8 kind = TRANSLINK_TCP;
	if (mode == 1)
	{
		u32 const ip = (rtos_str(1, host) == PARA_OK) ? fw_ipaddr(host) : 0;
		bool ok = !host.empty() && (host.size() <= MAX_TRANSLINK_HOST) && ip && (host != "255.255.255.255") &&
				(rtos_digit(2, port) == PARA_OK) && (port >= 1) && (port <= 0xffff) &&
				((m_para.size() <= 3) || (rtos_str(3, type) != PARA_FAIL));
		if (ok && (type == "UDP"))
			kind = TRANSLINK_UDP;
		else if (ok && !type.empty() && (type != "TCP"))
			ok = false;
		static constexpr s32 low[] = { 0, 1, 0 };
		static constexpr s32 high[] = { 7200, 0xffff, 1 };
		unsigned const first = (kind == TRANSLINK_UDP) ? 1 : 0;
		unsigned const last = (kind == TRANSLINK_UDP) ? 2 : 3;
		ok = ok && (m_para.size() <= (4 + last - first));
		for (unsigned i = first; ok && (i < last); i++)
		{
			int const result = (m_para.size() > (4 + i - first)) ? rtos_digit(4 + i - first, values[i]) : PARA_OMITTED;
			if (result == PARA_OMITTED)
				values[i] = (i == 2) ? 1 : 0;
			ok = (result == PARA_OMITTED) || ((result == PARA_OK) && (values[i] >= low[i]) && (values[i] <= high[i]));
		}
		if (!ok)
		{
			reply_error();
			return;
		}
	}

	m_tl_mode = (mode == 1) ? 1 : 0;
	m_tl_type = kind;
	m_tl_port = u16(port);
	m_tl_keepalive = u16(values[0]);
	m_tl_local_port = u16(values[1]);
	m_tl_flag = u8(values[2]);
	std::fill(std::begin(m_tl_host), std::end(m_tl_host), 0);
	std::copy(host.begin(), host.end(), std::begin(m_tl_host));
	m_translink_stored = true;
	reply_ok();
}


void esp8266_at_device::rtos_cmd_cipreconnintv(u8 form)
{
	s32 value;
	if (m_fw->reconnintv_wide && (form == FORM_QUERY))
	{
		reply(util::string_format("%s:%u\r\n", m_cmd_name, m_reconnect_ms / 100));
		reply_ok();
		return;
	}
	if (m_fw->reconnintv_wide)
	{
		if ((m_para.size() != 1) || (rtos_digit(0, value) != PARA_OK) || (value < 1) || (value > 36'000))
		{
			reply_error();
			return;
		}
		m_reconnect_ms = u32(value) * 100;
		if (rtos_store())
		{
			m_reconnect_def = m_reconnect_ms;
			m_sysmsg_stored = true;
		}
		reply_ok();
		return;
	}
	if ((form != FORM_SET) || (m_para.size() != 1) || (rtos_digit(0, value) != PARA_OK) || (value < 2) || (value > 35'999))
	{
		reply_error();
		return;
	}
	m_reconnect_ms = u32(value) * 100;
	reply_ok();
}


void esp8266_at_device::rtos_join()
{
	m_joined = 1;
	rtos_link_count();
	if (!rtos_wifi_quiet())
		reply("WIFI CONNECTED\r\nWIFI GOT IP\r\n");
	m_mode = MODE_JOIN;
	m_join_timer->adjust(attotime::from_msec(CWJAP_TICK_MS));
}


void esp8266_at_device::rtos_translink_store()
{
	m_tl_mode = 1;
	m_tl_type = m_tl_pending.udp ? TRANSLINK_UDP : TRANSLINK_TCP;
	m_tl_port = m_tl_pending.port;
	m_tl_keepalive = m_tl_pending.keepalive;
	m_tl_local_port = m_tl_pending.local_port;
	std::fill(std::begin(m_tl_host), std::end(m_tl_host), 0);
	m_tl_pending.host.copy(m_tl_host, MAX_TRANSLINK_HOST);
	m_translink_stored = true;
}


void esp8266_at_device::rtos_tcpopt_apply(unsigned link)
{
	if (m_tcpopt_set[link] && (m_link_state[link] == LINK_OPEN) && m_link_tcp[link])
		m_net->sockopt(link, m_link_generation[link], m_tcpopt_linger[link], m_tcpopt_nodelay[link]);
}


void esp8266_at_device::rtos_cmd_cmd(u8 form)
{
	std::string text = "+CMD:0,\"AT\",0,0,0,1\r\n";
	unsigned index = 1;
	for (rtos_command const *cmd = s_rtos_list_commands; cmd->name; cmd++, index++)
		text += util::string_format(
				"+CMD:%u,\"AT%s\",%u,%u,%u,%u\r\n",
				index, cmd->name, BIT(cmd->forms, FORM_TEST), BIT(cmd->forms, FORM_QUERY), BIT(cmd->forms, FORM_SET), BIT(cmd->forms, FORM_EXEC));
	reply(text);
	reply_ok();
}


void esp8266_at_device::rtos_cmd_sysstore(u8 form)
{
	s32 value;
	if (form == FORM_QUERY)
	{
		reply(util::string_format("%s:%u\r\n", m_cmd_name, m_sysstore));
		reply_ok();
	}
	else if ((rtos_digit(0, value) != PARA_OK) || (u32(value) > 1) || (m_para.size() != 1))
	{
		reply_error();
	}
	else
	{
		m_sysstore = u8(value);
		reply_ok();
	}
}


void esp8266_at_device::rtos_cmd_userram(u8 form)
{
	if (form == FORM_QUERY)
	{
		reply(util::string_format("%s:%u\r\n", m_cmd_name, unsigned(m_userram.size())));
		reply_ok();
		return;
	}

	s32 op, length = 0, offset = 0;
	unsigned expect = 1;
	if ((rtos_digit(0, op) != PARA_OK) || (u32(op) > 4))
	{
		reply_error();
		return;
	}
	if ((op >= 1) && (op <= 3))
	{
		if ((rtos_digit(1, length) != PARA_OK) || (length < 1))
		{
			reply_error();
			return;
		}
		expect = 2;
	}
	if ((op >= 2) && (op <= 3) && (m_para.size() != expect))
	{
		int const result = rtos_digit(2, offset);
		if ((result == PARA_FAIL) || (offset < 0))
		{
			reply_error();
			return;
		}
		if (result == PARA_OMITTED)
			offset = 0;
		expect = 3;
	}
	if (m_para.size() != expect)
	{
		reply_error();
		return;
	}

	bool const fits = !m_userram.empty() && ((u64(u32(offset)) + u32(length)) <= m_userram.size());
	switch (op)
	{
	case 0:
	case 4:
		if (m_userram.empty())
		{
			reply_error();
			return;
		}
		if (op)
			std::fill(m_userram.begin(), m_userram.end(), 0);
		else
			m_userram.clear();
		reply_ok();
		return;

	case 1:
		if (!m_userram.empty() || (length > rtos_heap()))
		{
			reply_error();
			return;
		}
		m_userram.assign(length, 0);
		reply_ok();
		return;

	case 2:
		if (!fits)
		{
			reply_error();
			return;
		}
		reply("\r\nOK\r\n\r\n>");
		m_userram_at = unsigned(offset);
		m_userram_left = unsigned(length);
		m_mode = MODE_USERRAM;
		return;

	default:
		if (!fits)
		{
			reply_error();
			return;
		}
		for (unsigned done = 0; done < unsigned(length); )
		{
			unsigned const count = std::min<unsigned>(unsigned(length) - done, RTOS_USERRAM_CHUNK);
			reply(util::string_format("%s:%u,", m_cmd_name, count));
			send_raw(&m_userram[offset + done], count);
			done += count;
		}
		reply_ok();
		return;
	}
}


void esp8266_at_device::rtos_cmd_cwstate(u8 form)
{
	u8 const state = m_joined ? 2 : m_ssid[0] ? 4 : 0;
	std::string ssid = m_ssid;
	// a 32-byte SSID has no terminator, so the password is printed after it
	if (ssid.size() == MAX_SSID)
		ssid += m_password;
	reply(util::string_format("%s:%u,\"%s\"\r\n", m_cmd_name, state, ssid));
	reply_ok();
}


void esp8266_at_device::rtos_cmd_cwreconncfg(u8 form)
{
	s32 interval, count;
	if (form == FORM_QUERY)
	{
		reply(util::string_format("%s:%u,%u\r\n", m_cmd_name, m_reconn_interval, m_reconn_count));
		reply_ok();
	}
	else if ((rtos_digit(0, interval) != PARA_OK) || (u32(interval) > 7'200) || (rtos_digit(1, count) != PARA_OK) || (u32(count) > 1'000) ||
			(m_para.size() != 2))
	{
		reply_error();
	}
	else
	{
		m_reconn_interval = u16(interval);
		m_reconn_count = u16(count);
		if (rtos_store())
		{
			m_reconn_def[0] = m_reconn_interval;
			m_reconn_def[1] = m_reconn_count;
			m_sysmsg_stored = true;
		}
		reply_ok();
	}
}


void esp8266_at_device::rtos_cmd_ciptcpopt(u8 form)
{
	if (form == FORM_QUERY)
	{
		std::string text;
		for (unsigned i = 0; i < LINK_COUNT; i++)
			text += util::string_format("%s:%u,%d,%u,%d\r\n", m_cmd_name, i, m_tcpopt_linger[i], m_tcpopt_nodelay[i], m_tcpopt_sndtimeo[i]);
		reply(text);
		reply_ok();
		return;
	}

	unsigned const first = m_mux ? 1 : 0;
	s32 id = 0, linger = -1, nodelay = 0, sndtimeo = 0;
	int const results[] = {
			m_mux ? rtos_digit(0, id) : PARA_OK,
			(m_para.size() == (first + 3)) ? rtos_digit(first, linger) : PARA_FAIL,
			(m_para.size() == (first + 3)) ? rtos_digit(first + 1, nodelay) : PARA_FAIL,
			(m_para.size() == (first + 3)) ? rtos_digit(first + 2, sndtimeo) : PARA_FAIL };
	if (results[1] == PARA_OMITTED)
		linger = -1;
	if (results[2] == PARA_OMITTED)
		nodelay = 0;
	if (results[3] == PARA_OMITTED)
		sndtimeo = 0;
	if ((results[0] != PARA_OK) || (results[1] == PARA_FAIL) || (results[2] == PARA_FAIL) || (results[3] == PARA_FAIL) ||
			(u32(id) > LINK_COUNT) || (linger < -1) || (u32(nodelay) > 1))
	{
		reply_error();
		return;
	}
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		if ((u32(id) == LINK_COUNT) || (u32(id) == i))
		{
			m_tcpopt_linger[i] = linger;
			m_tcpopt_nodelay[i] = u8(nodelay);
			m_tcpopt_sndtimeo[i] = sndtimeo;
			m_tcpopt_set[i] = true;
			rtos_tcpopt_apply(i);
		}
	}
	reply_ok();
}

}


DEFINE_DEVICE_TYPE_PRIVATE(ESP8266_AT, device_rs232_port_interface, esp8266_at_device, "esp8266_at", "ESP8266 WiFi module (AT firmware)")
