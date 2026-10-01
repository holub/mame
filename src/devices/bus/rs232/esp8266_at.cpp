// license:BSD-3-Clause
// copyright-holders:D. Rimron-Soutter
/***************************************************************************

    ESP8266 module running Espressif's AT 1.1.0.0 firmware

    High-level emulation of the AT command interface.  Links are TCP and
    UDP sockets on the host; there is no WiFi radio.

***************************************************************************/

#include "emu.h"
#include "esp8266_at.h"

#include "multibyte.h"

#include "asio.h"

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
constexpr unsigned RECONNECT_DELAY_MS = 1'000;
constexpr u16 DEFAULT_SERVER_PORT = 333;
constexpr u16 DEFAULT_SERVER_TIMEOUT = 180;
constexpr unsigned BOOT_DELAY_MS = 300;

constexpr unsigned ESP_TCP_MSS = 1460;
constexpr unsigned ESP_TCP_WND = 2 * ESP_TCP_MSS;
constexpr unsigned ESP_WND_UPDATE_THRESHOLD = ESP_TCP_WND / 4;
constexpr unsigned ESP_TCP_TMR_MS = 125;
constexpr unsigned ESP_UART_TX_BUFFER = ESP_TCP_WND + 100;

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

constexpr char const BOOT_TEXT[] = "\r\nStale Pixels ESP8266 AT emulation, AT 1.1.0.0\r\n";

constexpr u8 STATUS_GOT_IP = 2;
constexpr u8 STATUS_CONNECTED = 3;
constexpr u8 STATUS_DISCONNECTED = 4;
constexpr u8 STATUS_NO_WIFI = 5;

constexpr char const GMR_TEXT[] =
		"AT version:1.1.0.0(May 11 2016 18:09:56)\r\n"
		"SDK version:1.5.4(baaeaebb)\r\n"
		"compile time:May 20 2016 15:08:19\r\n";


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
					if ((l.generation == generation) && l.connected)
						start_read(link, generation);
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
		auto const handler =
				[this, link, generation] (std::error_code const &err, std::size_t length)
				{
					if (!current(link, generation))
						return;
					net_link &l = *m_links[link];
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
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_post_load() override;

	virtual void tra_complete() override;

	virtual void nvram_default() override;
	virtual bool nvram_read(util::read_stream &file) override;
	virtual bool nvram_write(util::write_stream &file) override;
	virtual bool nvram_can_write() const override { return m_def_stored || m_wifi_stored || m_translink_stored; }

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
		MODE_SEND_RESOLVE
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
		u32 rcv_wnd = ESP_TCP_WND;
		u32 rcv_ann_wnd = ESP_TCP_WND;
		u32 rcv_ann_right_edge = ESP_TCP_WND;
		u32 acked = 0;
		bool ack_delay = false;
		bool hold = false;
		u32 held = 0;
		unsigned blocks = 0;
		bool fin = false;
	};

	virtual void received_byte(u8 byte) override;

	TIMER_CALLBACK_MEMBER(poll_network);
	TIMER_CALLBACK_MEMBER(passthrough_flush);
	TIMER_CALLBACK_MEMBER(restart_done);
	TIMER_CALLBACK_MEMBER(tcp_tick);
	TIMER_CALLBACK_MEMBER(join_tick);
	TIMER_CALLBACK_MEMBER(ping_done);
	TIMER_CALLBACK_MEMBER(domain_done);

	void command_byte(u8 byte);
	void execute(std::string const &line);
	void cmd_cipstart(std::vector<at_arg> const &args);
	void cmd_cipsend(std::string_view const *text, u8 kind);
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
	void send_prompt();
	void send_remote(std::string const &address);
	void sendbuf_ack(unsigned link);
	void sendbuf_fail(unsigned link);
	unsigned send_space(unsigned link) const;
	unsigned send_queue_space(unsigned link) const;
	bool link_number(std::string_view &p, unsigned &link);
	void restart();
	void boot_done();

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

	void process_events();
	void link_open(unsigned link, unsigned delay_ms);
	void link_close(unsigned link);
	void close_all_links();
	void server_close();
	void check_server_timeouts();
	void set_defaults();
	void station_info(u8 *info) const;
	void station_off();
	void link_gone();
	void set_remote(unsigned link, std::string const &peer);
	void stop_ping();
	std::string station_ip() const;

	static std::vector<at_arg> split_args(std::string_view text);
	static bool fw_int(std::string_view &p, s32 &value, int &err);
	static int fw_string(std::string_view &p, std::string &out, unsigned max);
	static u32 fw_ipaddr(std::string const &text);
	static std::optional<u32> parse_number(at_arg const &arg);
	static bool take_string(std::string_view &text, std::string &out, unsigned max);
	static bool parse_mac(std::string const &text, u8 *mac);
	static std::optional<u32> parse_ip(std::string const &text);
	static std::string ip_text(u8 const *ip);

	required_ioport m_baud_config;
	required_ioport m_server_config;

	std::unique_ptr<esp8266_net> m_net;
	emu_timer *m_poll_timer;
	emu_timer *m_passthrough_timer;
	emu_timer *m_restart_timer;
	emu_timer *m_tcp_timer;
	emu_timer *m_join_timer;
	emu_timer *m_ping_timer;
	emu_timer *m_domain_timer;

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
};


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
	, m_restart_timer(nullptr)
	, m_tcp_timer(nullptr)
	, m_join_timer(nullptr)
	, m_ping_timer(nullptr)
	, m_domain_timer(nullptr)
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
{
}


ioport_constructor esp8266_at_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(esp8266_at);
}


void esp8266_at_device::device_start()
{
	m_poll_timer = timer_alloc(FUNC(esp8266_at_device::poll_network), this);
	m_passthrough_timer = timer_alloc(FUNC(esp8266_at_device::passthrough_flush), this);
	m_restart_timer = timer_alloc(FUNC(esp8266_at_device::restart_done), this);
	m_tcp_timer = timer_alloc(FUNC(esp8266_at_device::tcp_tick), this);
	m_join_timer = timer_alloc(FUNC(esp8266_at_device::join_tick), this);
	m_ping_timer = timer_alloc(FUNC(esp8266_at_device::ping_done), this);
	m_domain_timer = timer_alloc(FUNC(esp8266_at_device::domain_done), this);

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
}


void esp8266_at_device::device_stop()
{
	m_net.reset();
}


void esp8266_at_device::device_reset()
{
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
	m_mode = m_rst ? MODE_RESTART : MODE_COMMAND;
	m_passthrough_timer->adjust(attotime::never);
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
}


bool esp8266_at_device::nvram_read(util::read_stream &file)
{
	// 0-7: AT+UART_DEF rate as 32-bit little-endian, then the four AT+UART_DEF framing values
	// 8: 1 when the WiFi settings follow, 2 when the AT+SAVETRANSLINK settings follow them
	// 9: bit 0 set when 0-7 are stored
	// 10: AT+CWMODE_DEF, 11: AT+CWDHCP_DEF, 12: 1 when 13-24 hold an AT+CIPSTA_DEF
	// address, gateway and netmask, 25-57: AT+CWJAP_DEF SSID, NUL-terminated
	// 58: AT+SAVETRANSLINK mode, 59: link type (0 TCP, 1 UDP, 2 SSL), 60-61: remote port,
	// 62-63: UDP local port, 64-65: TCP keep-alive, all 16-bit little-endian,
	// 66-129: remote host, NUL-terminated
	// A file of only the first 8 bytes holds AT+UART_DEF alone.
	u8 buf[130];
	auto const [err, actual] = util::read(file, buf, sizeof(buf));
	if (err || ((actual != 8) && (actual != 58) && (actual != sizeof(buf))))
		return false;

	bool const wifi = actual >= 58;
	bool const translink = actual == sizeof(buf);
	if (wifi && ((buf[8] != (translink ? 2 : 1)) || (buf[10] < 1) || (buf[10] > 3) || (buf[11] > 3) || (buf[12] > 1) || buf[57]))
		return false;
	if (translink && ((buf[58] > 1) || (buf[59] > TRANSLINK_SSL) || buf[sizeof(buf) - 1]))
		return false;

	bool const uart = !wifi || BIT(buf[9], 0);
	u32 const rate = get_u32le(buf);
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
		std::copy(&buf[66], &buf[130], std::begin(m_tl_host));
		m_translink_stored = true;
	}
	return true;
}


bool esp8266_at_device::nvram_write(util::write_stream &file)
{
	u8 buf[130];
	std::fill(std::begin(buf), std::end(buf), 0);
	put_u32le(buf, m_def_rate);
	std::copy(std::begin(m_def_frame), std::end(m_def_frame), &buf[4]);
	std::size_t length = 8;
	if (m_wifi_stored || m_translink_stored)
	{
		buf[8] = 2;
		buf[9] = m_def_stored ? 1 : 0;
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
		length = sizeof(buf);
	}
	auto const [err, actual] = util::write(file, buf, length);
	return !err;
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
	m_server_timeout = DEFAULT_SERVER_TIMEOUT;
	m_ssl_size = SSL_SIZE_DEFAULT;
	m_mode = MODE_COMMAND;
	m_line.clear();
	m_send_buf.clear();
	m_passthrough_buf.clear();
	m_line_open = false;
	m_head = 0;
	m_cur_rate = m_def_rate;
	std::copy(std::begin(m_def_frame), std::end(m_def_frame), std::begin(m_cur_frame));
	apply_serial();

	m_send_cur = 0;
	for (unsigned i = 0; i < LINK_COUNT; i++)
	{
		m_link_sendbuf[i] = 0;
		m_seg_sent[i] = m_seg_done[i] = m_seg_ok[i] = m_seg_map[i] = 0;
		m_unacked[i].clear();
	}

	m_cwmode = m_cwmode_def;
	m_sleep = 2;
	m_dhcp = m_dhcp_def;
	m_sta_static = !BIT(m_dhcp, 1) && m_sta_static_def;
	std::copy(std::begin(m_sta_ip_def), std::end(m_sta_ip_def), std::begin(m_sta_ip));
	std::copy(std::begin(m_ssid_def), std::end(m_ssid_def), std::begin(m_ssid));
	m_joined = (m_cwmode != 2) && m_ssid[0];
	m_status = m_joined ? STATUS_GOT_IP : STATUS_NO_WIFI;
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
	reply("WIFI DISCONNECT\r\n");
	if (m_cwmode != 1)
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
		tcp.blocks = 0;
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
		if (ring_service())
			progress = true;
	}
	while (progress);

	if (m_out.size() < OUT_HIGH_WATER)
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
	if (m_ring_wait.empty() && ((ESP_UART_TX_BUFFER - ring_used()) >= text.size()))
	{
		m_ring_chunks.emplace_back(m_out_pushed, m_out_pushed + text.size());
		out_push(text.data(), text.size());
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
	while (!m_ring_wait.empty() && ((ESP_UART_TX_BUFFER - ring_used()) >= m_ring_wait.front().second.size()))
	{
		unsigned const link = m_ring_wait.front().first;
		std::vector<u8> const text = std::move(m_ring_wait.front().second);
		m_ring_wait.pop_front();
		m_ring_chunks.emplace_back(m_out_pushed, m_out_pushed + text.size());
		out_push(text.data(), text.size());
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
	return moved;
}


void esp8266_at_device::tcp_reset(unsigned link)
{
	m_tcp[link] = tcp_model();
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
	if (s32(new_right_edge - (tcp.rcv_ann_right_edge + std::min(ESP_TCP_WND / 2, ESP_TCP_MSS))) >= 0)
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
	tcp.rcv_wnd = std::min<u32>(tcp.rcv_wnd + length, ESP_TCP_WND);
	if (tcp_update_ann_wnd(link) >= ESP_WND_UPDATE_THRESHOLD)
		tcp_ack(link);
}


void esp8266_at_device::tcp_data(unsigned link, std::vector<u8> const &data)
{
	tcp_model &tcp = m_tcp[link];
	tcp.remote.insert(tcp.remote.end(), data.begin(), data.end());
	tcp_deliver(link);
	if (tcp.remote.size() >= ESP_TCP_WND)
		m_link_paused[link] = true;
	else
		m_net->resume(link, m_link_generation[link]);
}


void esp8266_at_device::tcp_deliver(unsigned link)
{
	if (m_mode == MODE_SEND_DATA)
		return;

	tcp_model &tcp = m_tcp[link];
	while (!tcp.remote.empty())
	{
		s32 const window = s32(tcp.rcv_ann_right_edge - tcp.rcv_nxt);
		if (window <= 0)
			break;
		unsigned const length = std::min<unsigned>({ unsigned(tcp.remote.size()), ESP_TCP_MSS, unsigned(window) });
		if ((tcp.acked != tcp.rcv_nxt) && (length < ESP_TCP_MSS))
			break;
		tcp_segment(link, length);
	}

	if (m_link_paused[link] && (tcp.remote.size() < ESP_TCP_WND))
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
			link_open(link, RECONNECT_DELAY_MS);
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
	switch (m_mode)
	{
	case MODE_RESTART:
		break;

	case MODE_SEND_DATA:
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
				break;
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
		break;

	case MODE_PASSTHROUGH:
		m_passthrough_buf.push_back(byte);
		if (m_passthrough_buf.size() >= MAX_SEND)
			passthrough_flush(0);
		else
			m_passthrough_timer->adjust(attotime::from_msec(20));
		break;

	default:
		command_byte(byte);
		break;
	}
}


void esp8266_at_device::command_byte(u8 byte)
{
	if (m_echo && (byte != '\n'))
		send_raw(&byte, 1);

	if (m_mode != MODE_COMMAND)
	{
		if (byte == '\n')
			reply((m_mode == MODE_SEND_WAIT) ? "\r\nbusy s...\r\n" : "\r\nbusy p...\r\n");
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

	if (exec && (name == "RST"))
	{
		reply_ok();
		restart();
	}
	else if (exec && (name == "GMR"))
	{
		// no blank line before the OK
		reply(GMR_TEXT);
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
		cmd_cipsend(set ? &text : nullptr, SEND_PLAIN);
	}
	else if (set && (name == "CIPSENDEX"))
	{
		cmd_cipsend(&text, SEND_EX);
	}
	else if (set && (name == "CIPSENDBUF"))
	{
		cmd_cipsend(&text, SEND_BUF);
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

	if (args.size() < (first + 3))
	{
		reply_error();
		return;
	}

	esp8266_net::open_params params;
	std::string const &type = args[first].text;
	if (type == "UDP")
	{
		params.udp = true;
	}
	else if (type != "TCP")
	{
		reply("Link type ERROR\r\n");
		reply_error();
		return;
	}

	auto const port = parse_number(args[first + 2]);
	if (args[first + 1].text.empty() || !port || (!*port && params.udp) || (*port > 0xffff))
	{
		reply_error();
		return;
	}
	params.host = args[first + 1].text;
	params.port = u16(*port);

	unsigned const extra = args.size() - first - 3;
	if (!params.udp)
	{
		if (extra > 1)
		{
			reply_error();
			return;
		}
		if (extra)
		{
			auto const keepalive = parse_number(args[first + 3]);
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
			auto const local = parse_number(args[first + 3]);
			if (!local || (*local > 0xffff))
			{
				reply_error();
				return;
			}
			params.local_port = u16(*local);
		}
		if (extra > 1)
		{
			auto const mode = parse_number(args[first + 4]);
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
	m_mode = MODE_CONNECTING;
	link_open(link, 0);
}


void esp8266_at_device::cmd_cipsend(std::string_view const *text, u8 kind)
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
	unsigned const link = m_send_link;
	unsigned const length = m_send_buf.size();
	LOGLINK("link %u: sending %u bytes\n", link, length);
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
	else
	{
		m_mode = MODE_SEND_WAIT;
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
	reply(util::string_format("%u,%u,%u,%u,%u\r\n", next ? next : 1, m_seg_done[link], m_seg_ok[link], space, queue));
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
	}
	else if (m_server_port)
	{
		reply("no change\r\n");
		reply_ok();
	}
	else
	{
		static constexpr unsigned offsets[] = { 0, 1'000, 10'000, 20'000, 30'000, 40'000, 0, 0 };
		ioport_value const config = m_server_config->read();
		unsigned const host_port = *port + offsets[BIT(config, 4, 3)];
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

	if (*id == LINK_COUNT)
	{
		bool closed[LINK_COUNT] = { false, false, false, false, false };
		for (unsigned i = 0; i < LINK_COUNT; i++)
		{
			if (m_link_state[i] != LINK_IDLE)
			{
				link_close(i);
				closed_notice(i);
				link_gone();
				closed[i] = true;
			}
		}
		reply_ok();

		for (unsigned i = 0; i < LINK_COUNT; i++)
		{
			if (closed[i])
				sendbuf_fail(i);
		}
	}
	else if (m_link_state[*id] == LINK_IDLE)
	{
		reply("UNLINK\r\n");
		reply_error();
	}
	else
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
	if (ok && !text.empty())
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
	if (m_status == STATUS_NO_WIFI)
		m_status = STATUS_GOT_IP;
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
		m_scan_text = util::string_format("+CWLAP:(%d,\"%s\",%d,\"%s\",%d,%d,%d)\r\n", AP_ECN, m_ssid, AP_RSSI, AP_BSSID, AP_CHANNEL, 0, 0);
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
	bool ok = take_string(text, item, 31) && !item.empty();
	if (ok)
		ok = bool(values[0] = parse_ip(item));
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
		reply(util::string_format(
				"+CIPSTATUS:%u,\"%s\",\"%s\",%u,%u,%u\r\n",
				i,
				m_link_tcp[i] ? "TCP" : "UDP",
				m_link_remote[i],
				m_link_remote_port[i],
				m_link_local_port[i],
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
			ok = fw_string(text, host, MAX_TRANSLINK_HOST) > 0;
		}
		if (ok)
		{
			u32 const ip = fw_ipaddr(host);
			ok = ip && ((ip != IPADDR_NONE) || (host != "255.255.255.255"));
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
			else if (ok && (name == "SSL"))
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
	m_link_tcp[link] = !params.udp;
	m_link_sendbuf[link] = 0;
	m_unacked[link].clear();
	tcp_reset(link);
	m_net->open(link, ++m_link_generation[link], params, delay_ms);
}


void esp8266_at_device::link_close(unsigned link)
{
	LOGLINK("link %u: close\n", link);
	m_link_state[link] = LINK_IDLE;
	m_link_paused[link] = false;
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
			closed_notice(i);
			sendbuf_fail(i);
			link_gone();
		}
	}
}


TIMER_CALLBACK_MEMBER(esp8266_at_device::poll_network)
{
	m_net->fetch(m_events);
	process_events();
	check_server_timeouts();

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

		if (m_mode == MODE_SEND_DATA)
			break;

		unsigned const link = ev.link;
		bool const passthrough = (m_mode == MODE_PASSTHROUGH) && !link;
		switch (ev.type)
		{
		case event_type::CONNECTED:
			LOGLINK("link %u: connected\n", link);
			m_link_state[link] = LINK_OPEN;
			m_status = STATUS_CONNECTED;
			m_link_local_port[link] = u16(ev.ticket);
			m_link_udp_moved[link] = false;
			set_remote(link, ev.peer);
			if (m_link_tcp[link])
				tcp_timer_needed();
			if ((m_mode == MODE_CONNECTING) && (link == m_connect_link))
			{
				if (m_mux)
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
				link_open(link, RECONNECT_DELAY_MS);
			}
			else if ((m_mode == MODE_CONNECTING) && (link == m_connect_link))
			{
				if (ev.type == event_type::DNS_FAILED)
				{
					reply("DNS Fail\r\n");
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
			if (m_link_tcp[link])
			{
				tcp_data(link, ev.data);
				pump();
				break;
			}
			LOGLINK("link %u: received %u bytes\n", link, unsigned(ev.data.size()));
			m_link_active[link] = machine().time();
			if (!passthrough)
				reply(ipd_text(link, ev.data.size()));
			send_raw(ev.data.data(), ev.data.size());
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
				link_open(link, RECONNECT_DELAY_MS);
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
		reply(BOOT_TEXT);
		m_boot_stage = BOOT_READY;
		m_restart_timer->adjust(attotime::from_msec(BOOT_DELAY_MS));
		return;
	}

	set_defaults();
	boot_done();
}


void esp8266_at_device::boot_done()
{
	bool const translink = m_tl_mode == 1;
	if (translink)
	{
		m_cipmode = 1;
		reply("\r\n>");
	}
	reply("\r\nready\r\n");

	if (m_joined && !m_cipmode)
		reply("WIFI CONNECTED\r\nWIFI GOT IP\r\n");

	if (translink)
	{
		esp8266_net::open_params params;
		params.udp = m_tl_type == TRANSLINK_UDP;
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


TIMER_CALLBACK_MEMBER(esp8266_at_device::domain_done)
{
	if (m_mode != MODE_DOMAIN)
		return;
	reply("DNS Fail\r\n");
	reply_error();
	m_mode = MODE_COMMAND;
}

}


DEFINE_DEVICE_TYPE_PRIVATE(ESP8266_AT, device_rs232_port_interface, esp8266_at_device, "esp8266_at", "ESP8266 WiFi module (AT firmware)")
