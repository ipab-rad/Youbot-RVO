#pragma once

#include "SocketAddress.h"
#include <arpa/inet.h>     
#include <string>
#include <sstream>

/**
 * @class InetAddress
 * 
 * @brief Class that defines a new version of the \a sockaddr structure.
 */
class InetAddress : public SocketAddress
{
	private:
		/**
		 * @brief address of the object's creator.
		 */
		std::string creationAddress;
		
		/**
		 * @brief string representing the last error occurred.
		 */
		std::string lastError;
		
		/**
		 * @brief value representing if the object is valid or not.
		 */
		bool valid;
		
	public:
		/**
		 * @brief Empty constructor.
		 * 
		 * It initializes address, creationAddress, lastError and valid with the default value.
		 */
		InetAddress();
		
		/**
		 * @brief Constructor that takes a port as initialization value.
		 * 
		 * It initializes the sender/receiver address with the port given in input.
		 * 
		 * @param port sender/receiver port.
		 */
		InetAddress(unsigned short port);
		
		/**
		 * @brief Constructor that takes an address and a port as initialization values.
		 * 
		 * It initializes the sender/receiver address with the values given in input.
		 * 
		 * @param address sender/receiver address.
		 * @param port sender/receiver port.
		 */
		InetAddress(const std::string& address, unsigned short port);
		
		/**
		 * @brief Constructor that takes a \a sockaddr address as initialization value.
		 * 
		 * It initializes the sender/receiver address with the one given in input.
		 * 
		 * @param address sender/receiver address.
		 */
		inline InetAddress(const struct sockaddr& address) : SocketAddress(address) {;}
		
		/**
		 * @brief Constructor that takes a \a sockaddr_in address as initialization value.
		 * 
		 * It initializes the sender/receiver address with the one given in input.
		 * 
		 * @param address sender/receiver address.
		 */
		inline InetAddress(const struct sockaddr_in& address) : SocketAddress((struct sockaddr&) address) {;}
		
		/**
		 * @brief Constructor that takes a SocketAddress as initialization value.
		 * 
		 * It initializes the sender/receiver address with the one given in input.
		 * 
		 * @param address reference to the sender/receiver address.
		 */
		InetAddress(const SocketAddress& address);
		
		/**
		 * @brief Function that returns the address of the sender/receiver.
		 * 
		 * @return the address of the sender/receiver.
		 */
		inline struct sockaddr_in getAddress() const { return (struct sockaddr_in&) address; }
		
		/**
		 * @brief Function that returns the address of the object's creator.
		 * 
		 * @return the object's creator address.
		 */
		inline const std::string getCreationAddress() const { return creationAddress; }
		
		/**
		 * @brief Function that returns the family of the sender/receiver.
		 * 
		 * @return the family of the sender/receiver.
		 */
		inline short getFamily() const { return ((struct sockaddr_in&) address).sin_family; }
		
		/**
		 * @brief Function that returns the IP of the sender/receiver.
		 * 
		 * @return the IP of the sender/receiver.
		 */
		inline std::string getIPAddress() const { return inet_ntoa( ((struct sockaddr_in&) address).sin_addr ); }
		
		/**
		 * @brief Function that returns the last error occurred.
		 * 
		 * @return the last error occurred.
		 */
		inline std::string getLastError() { return lastError; }
		
		/**
		 * @brief Function that returns the port of the sender/receiver.
		 * 
		 * @return the port of the sender/receiver.
		 */
		inline unsigned short getPort() const { return ntohs( ((struct sockaddr_in&) address).sin_port); }
		
		/**
		 * @brief Function that returns the validity of the object.
		 * 
		 * @return \b true if the object is valid, \b false otherwise.
		 */
		inline bool isValid() { return valid; }
		
		/**
		 * @brief Function that updates the address with the one given in input.
		 * 
		 * @param address new sender/receiver address.
		 */
		inline void setAddress(const struct sockaddr& address) { this->address = address; creationAddress = ""; }
		
		/**
		 * @brief Function that updates the address with the one given in input.
		 * 
		 * @param address new sender/receiver address.
		 */
		inline void setAddress(const struct sockaddr_in& address) { this->address = (struct sockaddr&) address; creationAddress = ""; }
		
		/**
		 * @brief Function that updates the address with the one given in input.
		 * 
		 * @param address new sender/receiver address.
		 */
		void setAddress(const std::string& address);
		
		/**
		 * @brief Function that updates the port with the one given in input.
		 * 
		 * @param port new sender/receiver port.
		 */
		inline void setPort(unsigned short port) { ((struct sockaddr_in&) address).sin_port = htons(port); }
		
		/**
		 * @brief Function that converts the sender/receiver address into a string.
		 * 
		 * @return a string representing the sender/receiver address.
		 */
		inline std::string toString() const { std::ostringstream oss; oss << getIPAddress() << ":" << getPort(); return oss.str(); }
		
		/**
		 * @brief Operator that checks if the current address is less than the one given in input.
		 * 
		 * The comparison of the two addresses begins by first checking the IP values and, if equals, then by checking the port values.
		 * 
		 * @param addr address that we want to check.
		 * 
		 * @return \b true if the current address is less than addr, \b false otherwise.
		 */
		bool operator< (const InetAddress& addr) const;
};
