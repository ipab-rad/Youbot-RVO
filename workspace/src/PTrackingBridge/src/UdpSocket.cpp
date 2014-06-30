#include "UdpSocket.h"
#include <errno.h>
#include <sstream>
#include <iostream>
#include <unistd.h>

//#define _DEBUG
#ifdef _DEBUG
	#define DOUT(a) std::cout<<__func__<<": "<<a<<std::endl
#else
	#define DOUT(a)
#endif

//#define RECV_DEBUG
#ifdef RECV_DEBUG
	#define RECVDOUT(a) std::cout<<__func__<<": "<<a<<std::endl
#else
	#define RECVDOUT(a)
#endif

//#define IPERSTRANGEBUG_DEBUG
#ifdef IPERSTRANGEBUG_DEBUG
	#define IPERDOUT(a) std::cout<<__func__<<": "<<a<<std::endl
#else
	#define IPERDOUT(a)
#endif

using namespace std;

UdpSocket::UdpSocket() : address(), socket(-1), bufferSize(UDP_SOCKET_DEFAULT_BUFFER_SIZE)
{
	buffer = new char[UDP_SOCKET_DEFAULT_BUFFER_SIZE];
	rebind();
	
	DOUT("UdpSocket::UdpSocket( unsigned short bufferSize )");
	DOUT("Socket =	   " << socket);
	DOUT("bufferSize = " << UDP_SOCKET_DEFAULT_BUFFER_SIZE);
}

UdpSocket::UdpSocket(unsigned short port) : address(port), socket(-1), bufferSize(UDP_SOCKET_DEFAULT_BUFFER_SIZE)
{	
	buffer = new char[UDP_SOCKET_DEFAULT_BUFFER_SIZE];
	rebind();
	
	DOUT("UdpSocket::UdpSocket( unsigned short port, unsigned short bufferSize )");
	DOUT("Socket =		" << socket);
	DOUT("bufferSize =  " << UDP_SOCKET_DEFAULT_BUFFER_SIZE);
	DOUT("Port =		" << address.getPort());
}

UdpSocket::UdpSocket(const InetAddress& address) : address(address), socket(-1), bufferSize(UDP_SOCKET_DEFAULT_BUFFER_SIZE)
{
	buffer = new char[UDP_SOCKET_DEFAULT_BUFFER_SIZE];
	rebind();
	
	DOUT("UdpSocket::UdpSocket( const InetAddress& address, unsigned short bufferSize )");
	DOUT("Socket =		" << socket);
	DOUT("bufferSize =  " << UDP_SOCKET_DEFAULT_BUFFER_SIZE);
	DOUT("Port =		" << address.getPort());
}

UdpSocket::~UdpSocket()
{
	delete[] buffer;
	
	if (socket >= 0)
	{
		shutdown(SHUT_RDWR);
		close(socket);
	}
}

bool UdpSocket::bind(unsigned short port)
{
	unbind();
	address = InetAddress(port);
	
	return rebind();
}

bool UdpSocket::bind(const InetAddress& address)
{
	unbind();
	this->address = address;
	
	return rebind();
}

bool UdpSocket::isBound()
{
	return socket >= 0;
}

bool UdpSocket::rebind() 
{
	unbind();
	
	DOUT("Socket binding");
	DOUT("Socket =   " << socket);
	DOUT("Bind =	 " << address.getIPAddress());
	DOUT("Port =	 " << address.getPort());
	
	if (socket < 0)
	{
		if ((socket = ::socket(AF_INET, SOCK_DGRAM, 0)) < 0) return false;
	}
	
	InetAddress ia = address.getAddress();
	
	if (::bind(socket, (struct sockaddr *) &ia, sizeof(struct sockaddr)) < 0)
	{
		shutdown(SHUT_RDWR);
		
		return false;
	}
	
	// Set BROADCAST ACTIVE
	int on = 1;
	
	setsockopt(socket,SOL_SOCKET,SO_BROADCAST,&on,sizeof(on));
	
	DOUT("Socket bound");
	DOUT("Socket =  " << socket);
	DOUT("Bind =	" << address.getIPAddress());
	DOUT("Port =	" << address.getPort());
	
	return true;
}

ssize_t UdpSocket::recv(std::string& data, InetAddress& address, double timeoutSecs)
{
	RECVDOUT("socket     = " << socket);
	RECVDOUT("&address   = " << (std::hex) << &address << (std::dec));
	IPERDOUT("myaddr     = " << address.getIPAddress());
	
	bool canRead = true;
	
	if (timeoutSecs != 0.0)
	{
		fd_set rfds;
		struct timeval tv;
		
		FD_ZERO(&rfds);
		FD_SET(socket,&rfds);
		
		tv.tv_sec = 0;
		tv.tv_usec = (int) (timeoutSecs * 1000 * 1000);
		
		int retval = select(socket + 1, &rfds, 0, 0, &tv);
		canRead = (retval > 0);
	}
	
	int bytes_received = 0;
	struct sockaddr_in sa;
	socklen_t caller_address_lenght = sizeof(struct sockaddr_in);
	
	if (canRead) bytes_received = recvfrom(socket, buffer, bufferSize, 0, (struct sockaddr*)&sa, &caller_address_lenght);
	
	address = InetAddress(sa);
	
	RECVDOUT("Bytes received: " << bytes_received);
	
	if (bytes_received < 0)
	{
		std::ostringstream err;
		
		err << "Receive failed ( receiving from port " << this->address.getPort() << " ).";
		
		RECVDOUT("Failed IP:      " << address.getIPAddress());
		RECVDOUT("Failed Port:    " << address.getPort());
		
		return bytes_received;
	}
	else
	{
		RECVDOUT("IP:             " << address.getIPAddress());
		RECVDOUT("Port:           " << address.getPort());
		
		data.clear();
		data.append(buffer,bytes_received);
		
		return bytes_received;
	}
}

ssize_t UdpSocket::send(const std::string& data, const InetAddress& address)
{
	if (!data.size()) return 0;
	else if (data.size() > bufferSize) return -1;
	
	DOUT("socket    = " << socket);
	DOUT("data      = " << data);
	DOUT("data size = " << data.size()); 
	DOUT("IP        = " << address.getIPAddress());
	
	InetAddress ia = address.getAddress();
	int bytesent = sendto(socket, data.data(), data.size(), 0, (struct sockaddr *)&ia, sizeof(struct sockaddr));
	
	DOUT("Byte sent = " << bytesent);
	
	return bytesent;
}

bool UdpSocket::shutdown(int how)
{
	return ::shutdown(socket,how) != -1;
}

void UdpSocket::unbind()
{
	shutdown(SHUT_RDWR);
	
	if (socket >= 0)
	{
		close(socket);
		socket = -1;
	}
}
