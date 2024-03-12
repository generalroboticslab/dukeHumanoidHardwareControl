#include "asio.hpp"
#include "network.h"



void asioUdpServer::setLocalAddress(std::string ip_local, uint16_t port_local) {
	this->ip_local = ip_local;
	this->port_local = port_local;
}

void asioUdpServer::setRemoteAddress(std::string ip_remote, uint16_t port_remote) {
	this->ip_remote = ip_remote;
	this->port_remote = port_remote;
}

void asioUdpServer::setAddress(std::string ip_local, uint16_t port_local,
	std::string ip_remote, uint16_t port_remote) {
	this->ip_local = ip_local;
	this->port_local = port_local;
	this->ip_remote = ip_remote;
	this->port_remote = port_remote;
}

void asioUdpServer::run() {
	UDP_SHOULD_RUN = true;
    printf("reached 1\n");
	thread_recv = std::thread([=]() { this->_doReceive(); });
	thread_send = std::thread([=]() { this->_doSend(); });
    printf("reached 2\n");
}

void asioUdpServer::close() {
	UDP_SHOULD_RUN = false;
	_joinThread();
}

void asioUdpServer::send(std::string message) {
	//std::lock_guard<std::mutex> lck{ mutex_send };
	try {
		msg_send_queue.push_back(std::move(message));
	}
	catch (const std::exception& e) {
		printf("Caught exception %s,line %d: %s \n", __FILE__, __LINE__, e.what());
	}
}

void asioUdpServer::_doReceive()
{
	// receiver
	asio::io_context io_context;
	asio::ip::udp::socket socket_recv(io_context); // receiver socket

	asio::ip::address local_address = asio::ip::make_address(ip_local);
	asio::ip::udp::endpoint listen_endpoint(local_address.is_v4() ?
		asio::ip::udp::v4() : asio::ip::udp::v6(), port_local);
	socket_recv.open(listen_endpoint.protocol());
	socket_recv.set_option(asio::ip::udp::socket::reuse_address(true));
	if (local_address.is_multicast()) { // Join the multicast group.
		socket_recv.set_option(asio::ip::multicast::join_group(local_address));
	}
	// set timeout [ms] without using async function,  
	// ref: https://newbedev.com/how-to-set-a-timeout-on-blocking-sockets-in-boost-asio
	// asio async offical example: 
	// https://github.com/chriskohlhoff/asio/blob/master/asio/src/examples/cpp11/timeouts/blocking_udp_client.cpp
	// socket_recv.set_option(asio::detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO>{ 100 }); //TODO LOOKS LIKE THIS IS DEAD!!!!!!!!!!
	socket_recv.bind(listen_endpoint);

	// buffer
	const int buffer_size = 1024;
	std::array<char, buffer_size> data_recv = { 0 };
	auto buffer_recv = asio::mutable_buffer(data_recv.data(), data_recv.size());

	// reset counter and flags
	counter_rec = 0;

	// loop
	while (UDP_SHOULD_RUN) {
		try {
			size_t nbytes = socket_recv.receive(buffer_recv);
			if (nbytes > 0) {
				std::string data_recv_str(data_recv.data(), nbytes);
				msg_recv_queue.push_back(std::move(data_recv_str)); // push to msg queue
				counter_rec++;
				//std::cout.write(data_recv.data(), nbytes);
				//std::cout << std::endl;
			}
		}
		catch (asio::system_error& e) { // timed out
			//printf("Caught exception %s,line %d: %s \n", __FILE__, __LINE__, e.what());
			continue;
		}
		catch (const std::exception& e) {
			printf("Caught exception %s,line %d: %s \n", __FILE__, __LINE__, e.what());
		}
	}
}

void asioUdpServer::_doSend()
{
	asio::io_context io_context;
	asio::ip::udp::socket socket_send(io_context); // receiver socket
	// sender
	asio::ip::udp::endpoint remote_endpoint = asio::ip::udp::endpoint(asio::ip::make_address(ip_remote), port_remote);
	socket_send.open(remote_endpoint.protocol());
	socket_send.connect(remote_endpoint); // "connect" to remote endpoint

	while (UDP_SHOULD_RUN) {
		//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		while (msg_send_queue.size() > 0) {
			// printf("_dosend: msg_send_queue.size()=%ld\n", msg_send_queue.size());
			auto& message_ = msg_send_queue.front(); // oldest message
			std::size_t length = socket_send.send(asio::const_buffer(message_.data(), message_.size()));
			//std::lock_guard<std::mutex> lck{ mutex_send };
			msg_send_queue.pop_front();
		}
	}
}