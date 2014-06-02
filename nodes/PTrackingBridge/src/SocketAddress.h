#pragma once

#include <sys/socket.h>
#include <cstring>

/**
 * @class SocketAddress
 * 
 * @brief Class that redefines the \a sockaddr structure.
 */
class SocketAddress
{
	protected:
		/**
		 * @brief address of the sender/receiver.
		 */
		struct sockaddr address;
		
	public:
		/**
		 * @brief Empty constructor.
		 * 
		 * It initializes the address with the default value.
		 */
		SocketAddress() { bzero(&address,sizeof(address)); }
		
		/**
		 * @brief Constructor that takes an address as initialization value.
		 * 
		 * It initializes the address with the one given in input.
		 * 
		 * @param address new address to be set.
		 */
		inline SocketAddress(const struct sockaddr& address) : address(address) {;}
		
		/**
		 * @brief Function that returns the address of the sender/receiver.
		 * 
		 * @return the address of the sender/receiver.
		 */
		inline struct sockaddr getAddress() { return address; }
		
		/**
		 * @brief Function that returns the family of the sender/receiver address.
		 * 
		 * @return the family of the sender/receiver address.
		 */
		inline short getFamily() const { return address.sa_family; }
};
