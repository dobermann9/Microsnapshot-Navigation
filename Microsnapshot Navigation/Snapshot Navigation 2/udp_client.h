#pragma once
#include "boost/asio.hpp"

using boost::asio::ip::udp;

//UDP client class from the boost tutorial at https://www.boost.org/doc/libs/1_35_0/doc/html/boost_asio/tutorial/tutdaytime4.html
//Asynchronous udp client to send messages to "host" over port "port". 
//Requires a boost::asio::io_service object to run.
class udp_client
{
public:
	udp_client(boost::asio::io_service& io_service, const std::string& host, const std::string& port);
	~udp_client();
	void send(const std::string& msg);

private:
	boost::asio::io_service& io_service_;
	udp::socket socket_;
	udp::endpoint endpoint_;
};