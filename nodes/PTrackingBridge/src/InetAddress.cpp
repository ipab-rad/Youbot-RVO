#include "InetAddress.h"
#include <netdb.h>
#include <cstring>
#include <sstream>

extern int h_errno;

InetAddress::InetAddress() : valid(false)
{
	((struct sockaddr_in&)address).sin_family = AF_INET;
	((struct sockaddr_in&)address).sin_port = 0;
	((struct sockaddr_in&)address).sin_addr.s_addr = INADDR_ANY;
	bzero(&(((struct sockaddr_in&)address).sin_zero), 8);
	valid = true;
}

InetAddress::InetAddress(unsigned short port) : valid(false)
{
	((struct sockaddr_in&)address).sin_family = AF_INET;
	((struct sockaddr_in&)address).sin_port = htons(port);
	((struct sockaddr_in&)address).sin_addr.s_addr = INADDR_ANY;
	bzero(&(((struct sockaddr_in&)address).sin_zero), 8);
	valid = true;
}

InetAddress::InetAddress(const std::string& address, unsigned short port) : valid(false)
{
	this->setAddress(address);
	
	if (this->valid)
	{
		((struct sockaddr_in&)this->address).sin_family = AF_INET;
		((struct sockaddr_in&)this->address).sin_port = htons(port);
		creationAddress = address;
		bzero(&(((struct sockaddr_in&)this->address).sin_zero), 8);
	}
}

InetAddress::InetAddress(const SocketAddress& address) : SocketAddress(address), valid(false)
{
	if (this->address.sa_family == AF_INET) valid = true;
	else lastError = "Invalid conversion from SocketAddress";
}

void InetAddress::setAddress(const std::string& address)
{
	if (inet_aton(address.c_str(), &((struct sockaddr_in&)this->address).sin_addr))
	{
		// the address was in dotted form
		valid = true;
	}
	else
	{
		// resolving name
		struct hostent *host = gethostbyname(address.c_str());
		
		if (host)
		{
			memcpy(&((struct sockaddr_in&)this->address).sin_addr.s_addr, host->h_addr_list[0], host->h_length);
			valid = true;
		}
		else
		{
			std::ostringstream oss;
			
			oss << "Unable to resolve address: " << address << " (";
			
			switch (h_errno)
			{
				case HOST_NOT_FOUND: oss << "Host not found"; break;
				case NO_ADDRESS: oss << "No address"; break;
				case NO_RECOVERY: oss << "No recovery"; break;
				case TRY_AGAIN: oss << "Try again later"; break;
				default: oss << "Unknown error type"; break;
			}
			
			oss << ").";
			
			lastError = oss.str();
			valid = false;
		}
	}
}

bool InetAddress::operator< (const InetAddress& addr) const
{
	if (((struct sockaddr_in&) address).sin_addr.s_addr < ((struct sockaddr_in&) addr.address).sin_addr.s_addr) return true;
	
	if (((struct sockaddr_in&) address).sin_port < ((struct sockaddr_in&) addr.address).sin_port) return true;
	
	return false;
}
