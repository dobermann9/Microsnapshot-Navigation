#include "udp_server.h"
#include "boost/asio.hpp"
#include "boost/bind.hpp"

using boost::asio::ip::udp;

typedef unsigned char uchar;

udp_server::udp_server(boost::asio::io_service& io_service, int port)
	: socket_(io_service, udp::endpoint(udp::v4(), port))
{
	is_running= true;
	start_receive();
}

udp_server::~udp_server() {
	socket_.close();
}

std::vector<uchar> udp_server::get_received()
{
	return rec_encoded;
}

//reads udp buffer on "port" specified in constructor
void udp_server::start_receive()
{
	socket_.async_receive_from(
		boost::asio::buffer(rec_buf), remote_endpoint_,
		boost::bind(&udp_server::handle_receive, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
}

//temporarily stores udp buffer to be read out by get_received() and continues loop
void udp_server::handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred)
{
	if (!error || error == boost::asio::error::message_size)
	{
		rec_encoded.clear();
		rec_encoded.insert(rec_encoded.end(), rec_buf.begin(), rec_buf.begin() + bytes_transferred);
		if(is_running)start_receive();
	}
}