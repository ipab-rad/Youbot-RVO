#pragma once

#include "InetAddress.h"

#define UDP_SOCKET_DEFAULT_BUFFER_SIZE 1500

/**
 * @class UdpSocket
 * 
 * @brief Class that defines an UDP socket.
 */
class UdpSocket
{
	protected:
		/**
		 * @brief address of the sender/receiver.
		 */
		InetAddress address;
		
		/**
		 * @brief socket descriptor.
		 */
		int socket;
		
		/**
		 * @brief size of the buffer.
		 */
		unsigned short bufferSize;
		
		/**
		 * @brief pointer to the buffer area.
		 */
		char* buffer;
		
	public:
		/**
		 * @brief Empty constructor.
		 * 
		 * It initializes the address with a default value and it tries to bind the socket.
		 */
		UdpSocket();
		
		/**
		 * @brief Constructor that takes a port as initialization value.
		 * 
		 * It initializes the address with the given port and it tries to bind the socket.
		 * 
		 * @param port sender/receiver port.
		 */
		UdpSocket(unsigned short port);
		
		/**
		 * @brief Constructor that takes an InetAddress as initialization value.
		 * 
		 * It initializes the sender/receiver address with the one given in input and it tries to bind the socket.
		 * 
		 * @param address reference to the sender/receiver address.
		 */
		UdpSocket(const InetAddress& address);
		
		/**
		 * @brief Destructor.
		 * 
		 * It deallocates the buffer memory and it unbinds the socket.
		 */
		~UdpSocket();
		
		/**
		 * @brief Function that binds the socket to a given port.
		 * 
		 * @param port sender/receiver port.
		 * 
		 * @return \b true if succeeded, \b false otherwise.
		 */
		bool bind(unsigned short port);
		
		/**
		 * @brief Function that binds the socket to the address given in input.
		 * 
		 * @param address reference to the sender/receiver address.
		 * 
		 * @return \b true if succeeded, \b false otherwise.
		 */
		bool bind(const InetAddress& address);
		
		/**
		 * @brief Function that returns the address of the sender/receiver.
		 * 
		 * @return the address of the sender/receiver.
		 */
		inline InetAddress getAddress() { return address; }
		
		/**
		 * @brief Function that returns the socket descriptor.
		 * 
		 * @return the socket descriptor.
		 */
		inline int getSocket() const { return socket; }
		
		/**
		 * @brief Function that checks if the socket is bound.
		 * 
		 * @return \b true if the socket is bound, \b false otherwise.
		 */
		bool isBound();
		
		/**
		 * @brief Function that binds (if needed close the socket before) the socket on the last address.
		 * 
		 * @return \b true if succeeded, \b false otherwise.
		 */
		bool rebind();
		
		/**
		 * @brief Function that receives a RAW packet and stores the sender address.
		 * 
		 * @param data received data.
		 * @param address reference to the sender address.
		 * @param timeoutSecs maximum number of seconds to wait before a message is received.
		 * 
		 * @return the number of bytes read.
		 */
		ssize_t recv(std::string& data, InetAddress& address, double timeoutSecs = 0.0);
		
		/**
		 * @brief Function that receives a RAW packet without storing the sender address.
		 * 
		 * @param data received data.
		 * 
		 * @return the number of bytes read.
		 */
		inline ssize_t recv(std::string& data) { InetAddress ia; return recv(data,ia); }
		
		/**
		 * @brief Function that sends a RAW packet to a specific address.
		 * 
		 * @param data data to be sent.
		 * @param address reference to the receiver address.
		 * 
		 * @return the number of bytes sent.
		 */
		ssize_t send(const std::string& data, const InetAddress& address);
		
		/**
		 * @brief Function that shutdowns the socket.
		 * 
		 * @param how shutting down modality.
		 * 
		 * @return \b true if succeeded, \b false otherwise.
		 */
		bool shutdown(int how);
		
		/**
		 * @brief Function that unbinds the socket.
		 */
		void unbind();
};
