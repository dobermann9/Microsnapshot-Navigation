#include "udp_client.h"
#include "boost/asio.hpp"

using boost::asio::ip::udp;

udp_client::udp_client(
	boost::asio::io_service& io_service,
	const std::string& host,
	const std::string& port
) : io_service_(io_service), socket_(io_service, udp::endpoint(udp::v4(), 0)) {
	udp::resolver resolver(io_service_);
	udp::resolver::query query(udp::v4(), host, port);
	udp::resolver::iterator iter = resolver.resolve(query);
	endpoint_ = *iter;
}

udp_client::~udp_client()
{
	socket_.close();
}

//sends message to "host" over "port", as specified in constructor
void udp_client::send(const std::string& msg) {
	socket_.send_to(boost::asio::buffer(msg, msg.size()), endpoint_);
}