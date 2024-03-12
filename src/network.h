#ifndef NETWORK_SERVER
#define NETWORK_SERVER

#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>

/// <summary>
/// ASIO UDP server to send and recive string messages
/// </summary>
class asioUdpServer
{
public:

	std::string ip_local; uint16_t port_local;
	std::string ip_remote; uint16_t port_remote;

	bool flag_new_received = false; // flag indicating a new message has received
	int counter_rec = 0; //  a counter for counting received message

	std::deque<std::string> msg_send_queue; // queue of data to be sent, new messages added to the back
	std::deque<std::string> msg_recv_queue; // queue of data received, new messages added to the back

	asioUdpServer() { }

	asioUdpServer(std::string ip_local, uint16_t port_local,
		std::string ip_remote, uint16_t port_remote) :
		ip_local(ip_local), port_local(port_local), ip_remote(ip_remote), port_remote(port_remote) {}
	
	~asioUdpServer() { _joinThread(); }

	/*run this to start receiving and sending udp*/
	void run();
	
	/*shutdown the udp server*/
	void close();

	void send(std::string message);

	/*set local address*/
	void setLocalAddress(std::string ip_local, uint16_t port_local);
	/*set remote address*/
	void setRemoteAddress(std::string ip_remote, uint16_t port_remote);
	/*set local and remote address*/
	void setAddress(std::string ip_local, uint16_t port_local, std::string ip_remote, uint16_t port_remote);

private:
	bool UDP_SHOULD_RUN = true;// flag indicating whether to stop sending/receiving

	std::thread thread_send; // thread for sending udp
	std::thread thread_recv; // thread for receiving udp

	//std::mutex mutex_send;//mutex for sending udp, ref: https://www.modernescpp.com/index.php/c-core-guidelines-sharing-data-between-threads

	void _doReceive();
	void _doSend();

	inline void _joinThread() {
		if (thread_send.joinable()) { thread_send.join(); }
		if (thread_recv.joinable()) { thread_recv.join(); }
	}
};


#endif //NETWORK_SERVER