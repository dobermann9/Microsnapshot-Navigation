#pragma once
#include "boost/asio.hpp"
#include "boost/bind.hpp"

using boost::asio::ip::udp;

typedef unsigned char uchar;

//UDP server class modified from the boost tutorial at https://www.boost.org/doc/libs/1_35_0/doc/html/boost_asio/tutorial/tutdaytime6.html
//Asynchronous udp server to listen on port "port". 
//Requires a boost::asio::io_service object to run.
class udp_server {
public:
	udp_server(boost::asio::io_service& io_service, int port);
	~udp_server();
	std::vector<uchar> get_received();
	bool is_running;

private:
	void start_receive();
	void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);
	udp::socket socket_;
	udp::endpoint remote_endpoint_;
	std::vector<uchar> rec_buf = std::vector<uchar>(64000);
	std::vector<uchar> rec_encoded;
};

