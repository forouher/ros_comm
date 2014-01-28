/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/io.h"
#include "ros/transport/transport_kdbus.h"
#include "ros/poll_set.h"
#include "ros/header.h"
#include "ros/file_log.h"
#include <ros/assert.h>
#include <sstream>
#include <boost/bind.hpp>
#include <fcntl.h>
#include <errno.h>




#include <string>
#include <cstdlib>

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

extern "C"
{
#include "ros/transport/kdbus-util.h"
#include "ros/transport/kdbus-enum.h"
}

#include <boost/interprocess/managed_external_buffer.hpp>

#define KBUILD_MODNAME "kdbus"

#include "ros/transport/kdbus_transport.h"





namespace ros
{

bool TransportKDBus::s_use_keepalive_ = true;
bool TransportKDBus::s_use_ipv6_ = false;

TransportKDBus::TransportKDBus(PollSet* poll_set, int flags, int fd)
: sock_(ROS_INVALID_SOCKET)
, closed_(false)
, expecting_read_(false)
, expecting_write_(false)
, is_server_(false)
, server_port_(-1)
, poll_set_(poll_set)
, flags_(flags)
, fdc(fd)
{
}

TransportKDBus::~TransportKDBus()
{
  ROS_ASSERT_MSG(sock_ == -1, "TransportKDBus socket [%d] was never closed", sock_);
}

bool TransportKDBus::setSocket(int sock)
{
  sock_ = sock;
  return initializeSocket();
}

bool TransportKDBus::setNonBlocking()
{
  if (!(flags_ & SYNCHRONOUS))
  {
	  int result = set_non_blocking(sock_);
	  if ( result != 0 ) {
	      ROS_ERROR("setting socket [%d] as non_blocking failed with error [%d]", sock_, result);
      close();
      return false;
    }
  }

  return true;
}

bool TransportKDBus::initializeSocket()
{
  ROS_ASSERT(sock_ != ROS_INVALID_SOCKET);

  if (!setNonBlocking())
  {
    return false;
  }

  setKeepAlive(s_use_keepalive_, 60, 10, 9);

  // connect() will set cached_remote_host_ because it already has the host/port available
  if (cached_remote_host_.empty())
  {
    if (is_server_)
    {
      cached_remote_host_ = "KDBusServer Socket";
    }
    else
    {
      std::stringstream ss;
      ss << getClientURI() << " on socket " << sock_;
      cached_remote_host_ = ss.str();
    }
  }

#ifdef ROSCPP_USE_TCP_NODELAY
  setNoDelay(true);
#endif

  ROS_ASSERT(poll_set_ || (flags_ & SYNCHRONOUS));
  if (poll_set_)
  {
    ROS_DEBUG("Adding kdbus... socket [%d] to pollset", sock_);
    poll_set_->addSocket(sock_, boost::bind(&TransportKDBus::socketUpdate, this, _1), shared_from_this());
  }

  if (!(flags_ & SYNCHRONOUS))
  {
    //enableRead();
  }

  return true;
}

void TransportKDBus::parseHeader(const Header& header)
{
  std::string nodelay;
  if (header.getValue("tcp_nodelay", nodelay) && nodelay == "1")
  {
    ROSCPP_LOG_DEBUG("Setting nodelay on socket [%d]", sock_);
    setNoDelay(true);
  }
}

void TransportKDBus::setNoDelay(bool nodelay)
{
  int flag = nodelay ? 1 : 0;
  int result = setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));
  if (result < 0)
  {
    ROS_ERROR("setsockopt failed to set TCP_NODELAY on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
  }
}

void TransportKDBus::setKeepAlive(bool use, uint32_t idle, uint32_t interval, uint32_t count)
{
  if (use)
  {
    int val = 1;
    if (setsockopt(sock_, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&val), sizeof(val)) != 0)
    {
      ROS_DEBUG("setsockopt failed to set SO_KEEPALIVE on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
    }

/* cygwin SOL_TCP does not seem to support TCP_KEEPIDLE, TCP_KEEPINTVL, TCP_KEEPCNT */
#if defined(SOL_TCP) && !defined(__CYGWIN__)
    val = idle;
    if (setsockopt(sock_, SOL_TCP, TCP_KEEPIDLE, &val, sizeof(val)) != 0)
    {
      ROS_DEBUG("setsockopt failed to set TCP_KEEPIDLE on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
    }

    val = interval;
    if (setsockopt(sock_, SOL_TCP, TCP_KEEPINTVL, &val, sizeof(val)) != 0)
    {
      ROS_DEBUG("setsockopt failed to set TCP_KEEPINTVL on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
    }

    val = count;
    if (setsockopt(sock_, SOL_TCP, TCP_KEEPCNT, &val, sizeof(val)) != 0)
    {
      ROS_DEBUG("setsockopt failed to set TCP_KEEPCNT on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
    }
#endif
  }
  else
  {
    int val = 0;
    if (setsockopt(sock_, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char*>(&val), sizeof(val)) != 0)
    {
    	ROS_DEBUG("setsockopt failed to set SO_KEEPALIVE on socket [%d] [%s]", sock_, cached_remote_host_.c_str());
    }
  }
}

bool TransportKDBus::connect(const std::string& host, int port)
{
  sock_ = socket(s_use_ipv6_ ? AF_INET6 : AF_INET, SOCK_STREAM, 0);
  connected_host_ = host;
  connected_port_ = port;

  if (sock_ == ROS_INVALID_SOCKET)
  {
    ROS_ERROR("socket() failed with error [%s]",  last_socket_error_string());
    return false;
  }

  setNonBlocking();

  // create a kdbus connection. the name of the connection must be known to the server...

  sockaddr_storage sas;
  socklen_t sas_len;

  in_addr ina;
  in6_addr in6a;
  if (inet_pton(AF_INET, host.c_str(), &ina) == 1)
  {
    sockaddr_in *address = (sockaddr_in*) &sas;
    sas_len = sizeof(sockaddr_in);
    
    address->sin_family = AF_INET;
    address->sin_port = htons(port);
    address->sin_addr.s_addr = ina.s_addr;
  }
  else if (inet_pton(AF_INET6, host.c_str(), &in6a) == 1)
  {
    sockaddr_in6 *address = (sockaddr_in6*) &sas;
    sas_len = sizeof(sockaddr_in6);
    address->sin6_family = AF_INET6;
    address->sin6_port = htons(port);
    memcpy(address->sin6_addr.s6_addr, in6a.s6_addr, sizeof(in6a.s6_addr));
  }
  else
  {
    struct addrinfo* addr;
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;

    if (getaddrinfo(host.c_str(), NULL, &hints, &addr) != 0)
    {
      close();
      ROS_ERROR("couldn't resolve publisher host [%s]", host.c_str());
      return false;
    }

    bool found = false;
    struct addrinfo* it = addr;
    char namebuf[128];
    for (; it; it = it->ai_next)
    {
      if (!s_use_ipv6_ && it->ai_family == AF_INET)
      {
        sockaddr_in *address = (sockaddr_in*) &sas;
        sas_len = sizeof(*address);
        
        memcpy(address, it->ai_addr, it->ai_addrlen);
        address->sin_family = it->ai_family;
        address->sin_port = htons(port);
	
        strcpy(namebuf, inet_ntoa(address->sin_addr));
        found = true;
        break;
      }
      if (s_use_ipv6_ && it->ai_family == AF_INET6)
      {
        sockaddr_in6 *address = (sockaddr_in6*) &sas;
        sas_len = sizeof(*address);
      
        memcpy(address, it->ai_addr, it->ai_addrlen);
        address->sin6_family = it->ai_family;
        address->sin6_port = htons((u_short) port);
      
        // TODO IPV6: does inet_ntop need other includes for Windows?
        inet_ntop(AF_INET6, (void*)&(address->sin6_addr), namebuf, sizeof(namebuf));
        found = true;
        break;
      }
    }

    freeaddrinfo(addr);

    if (!found)
    {
      ROS_ERROR("Couldn't resolve an address for [%s]\n", host.c_str());
      return false;
    }

    ROSCPP_LOG_DEBUG("Resolved publisher host [%s] to [%s] for socket [%d]", host.c_str(), namebuf, sock_);
  }

  int ret = ::connect(sock_, (sockaddr*) &sas, sas_len);
  // windows might need some time to sleep (input from service robotics hack) add this if testing proves it is necessary.
  ROS_ASSERT((flags_ & SYNCHRONOUS) || ret != 0);
  if (((flags_ & SYNCHRONOUS) && ret != 0) || // synchronous, connect() should return 0
      (!(flags_ & SYNCHRONOUS) && last_socket_error() != ROS_SOCKETS_ASYNCHRONOUS_CONNECT_RETURN)) // asynchronous, connect() should return -1 and WSAGetLastError()=WSAEWOULDBLOCK/errno=EINPROGRESS
  {
    ROSCPP_LOG_DEBUG("Connect to kdbusros publisher [%s:%d] failed with error [%d, %s]", host.c_str(), port, ret, last_socket_error_string());
    close();

    return false;
  }

  // from daniel stonier:
#ifdef WIN32
  // This is hackish, but windows fails at recv() if its slow to connect (e.g. happens with wireless)
  // recv() needs to check if its connected or not when its asynchronous?
  Sleep(100);
#endif


  std::stringstream ss;
  ss << host << ":" << port << " on socket " << sock_;
  cached_remote_host_ = ss.str();

  if (!initializeSocket())
  {
    return false;
  }

  if (flags_ & SYNCHRONOUS)
  {
    ROSCPP_LOG_DEBUG("connect() succeeded to [%s:%d] on socket [%d]", host.c_str(), port, sock_);
  }
  else
  {
    ROSCPP_LOG_DEBUG("Async connect() in progress to [%s:%d] on socket [%d]", host.c_str(), port, sock_);
  }

  // everythings okay, now open kdbus connection
    int fd = -1;
    if(sas.ss_family == AF_INET) {
      sockaddr_storage sas;
      socklen_t sas_len = sizeof(sas);
      getsockname(sock_, (sockaddr *)&sas, &sas_len);
      sockaddr_in* saip4 = (sockaddr_in*)&sas;
      std::ostringstream stringStream;
      stringStream << "r_" << saip4->sin_port;
      fd = open_connection(stringStream.str());
    } else {
      ROS_ERROR("kdbus: dummy connection is not INET4");
      return TransportKDBusPtr();
    }

  ROS_DEBUG("connect: getClientURI()==%s", getClientURI().c_str());

  return true;
}

int TransportKDBus::create_bus() {

	struct {
		struct kdbus_cmd_make head;

		struct {
			uint64_t size;
			uint64_t type;
			uint64_t bloom_size;
		} bs;

		uint64_t n_size;
		uint64_t n_type;
		char name[64];
	} bus_make;

	int ret;

	ROS_DEBUG("-- opening /dev/kdbus/control");
	fdc = open("/dev/kdbus/control", O_RDWR|O_CLOEXEC);
	if (fdc < 0) {
		ROS_ERROR("--- error %d (%m)", fdc);
		return EXIT_FAILURE;
	}

	memset(&bus_make, 0, sizeof(bus_make));
	bus_make.bs.size = sizeof(bus_make.bs);
	bus_make.bs.type = KDBUS_ITEM_BLOOM_SIZE;
	bus_make.bs.bloom_size = 64;

	strncpy(bus_make.name, bus.c_str(),bus.length());
	bus_make.n_type = KDBUS_ITEM_MAKE_NAME;
	bus_make.n_size = KDBUS_ITEM_HEADER_SIZE + strlen(bus_make.name) + 1;

	bus_make.head.size = sizeof(struct kdbus_cmd_make) +
			     sizeof(bus_make.bs) +
			     bus_make.n_size;

	ROS_DEBUG("-- creating bus '%s'", bus_make.name);
	ret = ioctl(fdc, KDBUS_CMD_BUS_MAKE, &bus_make);
	if (ret) {
		ROS_ERROR("--- error %d (%m)", ret);
		return EXIT_FAILURE;
	}

	return 0;

}

bool TransportKDBus::listen(int port, int backlog, const AcceptCallback& accept_cb)
{
  is_server_ = true;
  accept_cb_ = accept_cb;

  bus = "1000-ros"; // TODO
  buspath = "/dev/kdbus/"+bus+"/bus";

  create_bus();

  if (s_use_ipv6_)
  {
    sock_ = socket(AF_INET6, SOCK_STREAM, 0);
    sockaddr_in6 *address = (sockaddr_in6 *)&server_address_;
    address->sin6_family = AF_INET6;
    address->sin6_addr = in6addr_any;
    address->sin6_port = htons(port);
    sa_len_ = sizeof(sockaddr_in6);
  }
  else
  {
    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in *address = (sockaddr_in *)&server_address_;
    address->sin_family = AF_INET;
    address->sin_addr.s_addr = INADDR_ANY;
    address->sin_port = htons(port);
    sa_len_ = sizeof(sockaddr_in);
  }

  if (sock_ <= 0)
  {
    ROS_ERROR("socket() failed with error [%s]", last_socket_error_string());
    return false;
  }


  if (bind(sock_, (sockaddr *)&server_address_, sa_len_) < 0)
  {
    ROS_ERROR("bind() failed with error [%s]", last_socket_error_string());
    return false;
  }

  ::listen(sock_, backlog);
  getsockname(sock_, (sockaddr *)&server_address_, &sa_len_);

  switch (server_address_.ss_family)
  {
    case AF_INET:
      server_port_ = ntohs(((sockaddr_in *)&server_address_)->sin_port);
      break;
    case AF_INET6:
      server_port_ = ntohs(((sockaddr_in6 *)&server_address_)->sin6_port);
      break;
  }

  if (!initializeSocket())
  {
    return false;
  }

  if (!(flags_ & SYNCHRONOUS))
  {
    enableRead();
  }

  return true;
}

void TransportKDBus::close()
{
  Callback disconnect_cb;

  if (!closed_)
  {
    {
      boost::recursive_mutex::scoped_lock lock(close_mutex_);


      if (!closed_)
      {
        closed_ = true;

        close_connection();

        ROS_ASSERT(sock_ != ROS_INVALID_SOCKET);

        if (poll_set_)
        {
          poll_set_->delSocket(sock_);
        }

        ::shutdown(sock_, ROS_SOCKETS_SHUT_RDWR);
        if ( close_socket(sock_) != 0 )
        {
          ROS_ERROR("Error closing socket [%d]: [%s]", sock_, last_socket_error_string());
        } else
        {
          ROSCPP_LOG_DEBUG("KDBus socket [%d] closed", sock_);
        }
        sock_ = ROS_INVALID_SOCKET;

        disconnect_cb = disconnect_cb_;

        disconnect_cb_ = Callback();
        read_cb_ = Callback();
        write_cb_ = Callback();
        accept_cb_ = AcceptCallback();
      }
    }
  }

  if (disconnect_cb)
  {
    disconnect_cb(shared_from_this());
  }
}

int32_t TransportKDBus::read(uint8_t* buffer, uint32_t size)
{
  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      ROSCPP_LOG_DEBUG("Tried to read on a closed socket [%d]", sock_);
      return -1;
    }
  }

  ROS_ASSERT(size > 0);

  // never read more than INT_MAX since this is the maximum we can report back with the current return type
  uint32_t read_size = std::min(size, static_cast<uint32_t>(INT_MAX));
  int num_bytes = ::recv(sock_, reinterpret_cast<char*>(buffer), read_size, 0);
  if (num_bytes < 0)
  {
	if ( !last_socket_error_is_would_block() ) // !WSAWOULDBLOCK / !EAGAIN && !EWOULDBLOCK
    {
      ROSCPP_LOG_DEBUG("recv() on socket [%d] failed with error [%s]", sock_, last_socket_error_string());
      close();
    }
    else
    {
      num_bytes = 0;
    }
  }
  else if (num_bytes == 0)
  {
    ROSCPP_LOG_DEBUG("Socket [%d] received 0/%u bytes, closing", sock_, size);
    close();
    return -1;
  }

  return num_bytes;
}

int32_t TransportKDBus::write(uint8_t* buffer, uint32_t size)
{
  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      ROSCPP_LOG_DEBUG("Tried to write on a closed socket [%d]", sock_);
      return -1;
    }
  }

  ROS_ASSERT(size > 0);

//  ROS_DEBUG("ich sollte jetzt eine msg an %s senden", get_remote_connection_name().c_str());

  // never write more than INT_MAX since this is the maximum we can report back with the current return type
  uint32_t writesize = std::min(size, static_cast<uint32_t>(INT_MAX));
  int num_bytes = ::send(sock_, reinterpret_cast<const char*>(buffer), writesize, 0);
  if (num_bytes < 0)
  {
    if ( !last_socket_error_is_would_block() )
    {
      ROSCPP_LOG_DEBUG("send() on socket [%d] failed with error [%s]", sock_, last_socket_error_string());
      close();
    }
    else
    {
      num_bytes = 0;
    }
  }

  return num_bytes;
}

void TransportKDBus::enableRead()
{
  ROS_ASSERT(!(flags_ & SYNCHRONOUS));

  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (!expecting_read_)
  {
    poll_set_->addEvents(sock_, POLLIN);
    expecting_read_ = true;
  }
}

void TransportKDBus::disableRead()
{
  ROS_ASSERT(!(flags_ & SYNCHRONOUS));

  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (expecting_read_)
  {
    poll_set_->delEvents(sock_, POLLIN);
    expecting_read_ = false;
  }
}

void TransportKDBus::enableWrite()
{
  ROS_ASSERT(!(flags_ & SYNCHRONOUS));

  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (!expecting_write_)
  {
    poll_set_->addEvents(sock_, POLLOUT);
    expecting_write_ = true;
  }
}

void TransportKDBus::disableWrite()
{
  ROS_ASSERT(!(flags_ & SYNCHRONOUS));

  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);

    if (closed_)
    {
      return;
    }
  }

  if (expecting_write_)
  {
    poll_set_->delEvents(sock_, POLLOUT);
    expecting_write_ = false;
  }
}

std::string TransportKDBus::get_remote_connection_name()
{
  ROS_ASSERT(!is_server_);

  sockaddr_storage sas;
  socklen_t sas_len = sizeof(sas);
  getpeername(sock_, (sockaddr *)&sas, &sas_len);
  if(sas.ss_family == AF_INET) {
      sockaddr_in* saip4 = (sockaddr_in*)&sas;
      std::ostringstream stringStream;
      stringStream << "r_" << saip4->sin_port;
      return stringStream.str();
  } else {
      ROS_ERROR("kdbus: dummy connection is not INET4");
      return "";
  }
}

TransportKDBusPtr TransportKDBus::accept()
{
  ROS_ASSERT(is_server_);

  sockaddr client_address;
  socklen_t len = sizeof(client_address);
  int new_sock = ::accept(sock_, (sockaddr *)&client_address, &len);
  if (new_sock >= 0)
  {
    ROSCPP_LOG_DEBUG("Accepted connection on socket [%d], new socket [%d]", sock_, new_sock);

    // everythings okay, now open kdbus connection
    int fd = -1;
    if(client_address.sa_family == AF_INET) {
      sockaddr_storage sas;
      socklen_t sas_len = sizeof(sas);
      getsockname(new_sock, (sockaddr *)&sas, &sas_len);
      sockaddr_in* saip4 = (sockaddr_in*)&sas;
      std::ostringstream stringStream;
      stringStream << "l_" << saip4->sin_port;
      fd = open_connection(stringStream.str());
    } else {
      ROS_ERROR("kdbus: dummy connection is not INET4");
      return TransportKDBusPtr();
    }

    TransportKDBusPtr transport(new TransportKDBus(poll_set_, flags_, fd));
    if (!transport->setSocket(new_sock))
    {
      ROS_ERROR("Failed to set socket on transport for socket %d", new_sock);
    }

    ROS_DEBUG("accept: getClientURI()==%s", transport->getClientURI().c_str());
    return transport;
  }
  else
  {
    ROS_ERROR("accept() on socket [%d] failed with error [%s]", sock_,  last_socket_error_string());
  }

  return TransportKDBusPtr();
}

void TransportKDBus::socketUpdate(int events)
{
  {
    boost::recursive_mutex::scoped_lock lock(close_mutex_);
    if (closed_)
    {
      return;
    }

    // Handle read events before err/hup/nval, since there may be data left on the wire
    if ((events & POLLIN) && expecting_read_)
    {
      if (is_server_)
      {
        // Should not block here, because poll() said that it's ready
        // for reading
        TransportKDBusPtr transport = accept();
        if (transport)
        {
          ROS_ASSERT(accept_cb_);
          accept_cb_(transport);
        }
      }
      else
      {
        if (read_cb_)
        {
          read_cb_(shared_from_this());
        }
      }
    }

    if ((events & POLLOUT) && expecting_write_)
    {
      if (write_cb_)
      {
        write_cb_(shared_from_this());
      }
    }
  }

  if((events & POLLERR) ||
     (events & POLLHUP) ||
     (events & POLLNVAL))
  {
    uint32_t error = -1;
    socklen_t len = sizeof(error);
    if (getsockopt(sock_, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&error), &len) < 0)
    {
      ROSCPP_LOG_DEBUG("getsockopt failed on socket [%d]", sock_);
    }
  #ifdef _MSC_VER
    char err[60];
    strerror_s(err,60,error);
    ROSCPP_LOG_DEBUG("Socket %d closed with (ERR|HUP|NVAL) events %d: %s", sock_, events, err);
  #else
    ROSCPP_LOG_DEBUG("Socket %d closed with (ERR|HUP|NVAL) events %d: %s", sock_, events, strerror(error));
  #endif

    close();
  }
}

std::string TransportKDBus::getTransportInfo()
{
  return "KDBusROS connection to [" + cached_remote_host_ + "]";
}

std::string TransportKDBus::getClientURI()
{
  ROS_ASSERT(!is_server_);

  sockaddr_storage sas;
  socklen_t sas_len = sizeof(sas);
  getpeername(sock_, (sockaddr *)&sas, &sas_len);
  
  sockaddr_in *sin = (sockaddr_in *)&sas;
  sockaddr_in6 *sin6 = (sockaddr_in6 *)&sas;

  char namebuf[128];
  int port;

  switch (sas.ss_family)
  {
    case AF_INET:
      port = ntohs(sin->sin_port);
      strcpy(namebuf, inet_ntoa(sin->sin_addr));
      break;
    case AF_INET6:
      port = ntohs(sin6->sin6_port);
      inet_ntop(AF_INET6, (void*)&(sin6->sin6_addr), namebuf, sizeof(namebuf));
      break;
    default:
      namebuf[0] = 0;
      port = 0;
      break;
  }

  std::string ip = namebuf;
  std::stringstream uri;
  uri << ip << ":" << port;

  return uri.str();
}

int KDBusTransport::destroy_bus() {
	close(fdc);
}

int TransportKDBus::open_connection(const std::string& name) {
	int fdc, ret;
	int r;

	ROS_DEBUG("Creating kdbus connection '%s'", name.c_str());

	conn = connect_to_bus(buspath.c_str(), 0);
	if (!conn)
		return EXIT_FAILURE;

	r = upload_policy(conn->fd, name.c_str());
	if (r < 0)
		return EXIT_FAILURE;
	r = name_acquire(conn, name.c_str(), 0);
	if (r < 0)
		return EXIT_FAILURE;

	add_match_empty(conn->fd);

	return conn->fd;

}

int TransportKDBus::close_connection() {
//	::close(conn->fd);
//	free(conn);

}

/*
KDBusMessage TransportKDBus::createMessage() {

        int ret;

        int memfd = -1;
        ret = ioctl(conn->fd, KDBUS_CMD_MEMFD_NEW, &memfd);
        if (ret < 0) {
                fprintf(stderr, "KDBUS_CMD_MEMFD_NEW failed: %m\n");
//                return EXIT_FAILURE;
        }

        const int memfd_size = 200000;

        char* address_fd = (char*)mmap(NULL, memfd_size,PROT_WRITE,MAP_SHARED,memfd,0);
        if (MAP_FAILED == address_fd) {
                fprintf(stderr, "mmap() to memfd failed: %m\n");
//                return EXIT_FAILURE;
        }

	KDBusMessage m(memfd, address_fd, memfd_size);

	return m;
}

int TransportKDBus::sendMessage(KDBusMessage& msg, const std::string& receiver) {

	const char* name = receiver.c_str();

        munmap(msg.address_fd,msg.size);

        int ret = ioctl(msg.memfd, KDBUS_CMD_MEMFD_SEAL_SET, true);
        if (ret < 0) {
                fprintf(stderr, "memfd sealing failed: %m\n");
                return EXIT_FAILURE;
        }

	uint64_t size = sizeof(struct kdbus_msg);
	size += KDBUS_ITEM_SIZE(sizeof(struct kdbus_memfd));

	if (name)
		size += KDBUS_ITEM_SIZE(strlen(name) + 1);

	struct kdbus_msg *kmsg = (kdbus_msg*)malloc(msg.size);
	if (!kmsg) {
		fprintf(stderr, "unable to malloc()!?\n");
		return EXIT_FAILURE;
	}

	// create new kdbus message
	memset(kmsg, 0, size);
	kmsg->size = size;
	kmsg->src_id = conn->id;
	kmsg->dst_id = 0; // name defines destination
	kmsg->cookie = 0;
	kmsg->payload_type = KDBUS_PAYLOAD_DBUS; // TODO??

	// start with first item
	struct kdbus_item *item = kmsg->items;

	// start with name
	if (name) {
		item->type = KDBUS_ITEM_DST_NAME;
		item->size = KDBUS_ITEM_HEADER_SIZE + strlen(name) + 1;
		strcpy(item->str, name);
		item = KDBUS_ITEM_NEXT(item);
	}

	// next is our memfd
	item->type = KDBUS_ITEM_PAYLOAD_MEMFD;
	item->size = KDBUS_ITEM_HEADER_SIZE + sizeof(struct kdbus_memfd);
	item->memfd.size = 16;
	item->memfd.fd = msg.memfd;
	item = KDBUS_ITEM_NEXT(item);

	ret = ioctl(conn->fd, KDBUS_CMD_MSG_SEND, kmsg);
	if (ret < 0) {
		fprintf(stderr, "error sending message: %d err %d (%m)\n", ret, errno);
		return EXIT_FAILURE;
	}

	close(msg.memfd);

	free(kmsg);

	return 0;

}

KDBusMessage TransportKDBus::receiveMessage() {


	uint64_t off;
	struct kdbus_msg *msg;
	int ret;

	KDBusMessage retm;

	ret = ioctl(conn->fd, KDBUS_CMD_MSG_RECV, &off);
	if (ret < 0) {
		fprintf(stderr, "error receiving message: %d (%m)\n", ret);
//		return EXIT_FAILURE;
	}

	msg = (struct kdbus_msg *)((char*)(conn->buf) + off);
	{
	const struct kdbus_item *item = msg->items;
	char buf_src[32];
	char buf_dst[32];
	uint64_t timeout = 0;
	uint64_t cookie_reply = 0;

	KDBUS_ITEM_FOREACH(item, msg, items) {
		if (item->size <= KDBUS_ITEM_HEADER_SIZE) {
			printf("  +%s (%llu bytes) invalid data record\n", enum_MSG(item->type), item->size);
			break;
		}

		switch (item->type) {

		case KDBUS_ITEM_PAYLOAD_MEMFD: {
			char *buf;
			uint64_t size;

			buf = (char*)mmap(NULL, 200000, PROT_READ, MAP_SHARED, item->memfd.fd, 0);
			if (buf == MAP_FAILED) {
				printf("mmap() fd=%i failed:%m", item->memfd.fd);
				break;
			}

			if (ioctl(item->memfd.fd, KDBUS_CMD_MEMFD_SIZE_GET, &size) < 0) {
				fprintf(stderr, "KDBUS_CMD_MEMFD_SIZE_GET failed: %m\n");
				break;
			}

			retm = KDBusMessage(0, buf, 100000);

			break;
		}
		case KDBUS_ITEM_NAME: {
			printf("  +%s (%llu bytes) '%s' (%zu) flags=0x%08llx\n",
			       enum_MSG(item->type), item->size, item->name.name, strlen(item->name.name),
			       item->name.flags);
			break;
		}

		default:
			printf("  +%s (%llu bytes)\n", enum_MSG(item->type), item->size);
			break;
		}
	}

	if ((char *)item - ((char *)msg + msg->size) >= 8)
		printf("invalid padding at end of message\n");

	printf("\n");
	}


	ret = ioctl(conn->fd, KDBUS_CMD_FREE, &off);
	if (ret < 0) {
		fprintf(stderr, "error free message: %d (%m)\n", ret);
//		return EXIT_FAILURE;
	}

	return retm;

}
*/
} // namespace ros
