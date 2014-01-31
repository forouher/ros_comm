/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/memfd_message.h>

extern "C"
{

#include "ros/transport/kdbus-util.h"
#include "ros/transport/kdbus-enum.h"
}

#include <boost/interprocess/managed_external_buffer.hpp>

namespace ros
{

int MemfdMessage::fd_control = -1;

MemfdMessage::MemfdMessage(int fd, void* buf, size_t size)
    : fd_(fd), size_(size), buf_(buf)
{

}

MemfdMessage::Ptr MemfdMessage::create(size_t size)
{
//  fprintf(stderr,"creating message size %i\n", size);
  if (fd_control<0)
  {
    fd_control = open("/dev/kdbus/control", O_RDWR|O_CLOEXEC);
    ROS_ASSERT(fd_control >= 0);
    // XXX: is never closed
  }

  int memfd = -1;
  int ret = ioctl(fd_control, KDBUS_CMD_MEMFD_NEW, &memfd);
  ROS_ASSERT_MSG(ret >= 0, "Could not create memfd, fd=%i; %d (%m)", fd_control, ret);
//  fprintf(stderr, "new memfd at fd=%i\n", memfd);
  ROS_ASSERT(memfd >= 0);

  void* buf = mmap(NULL, size,PROT_WRITE|PROT_READ,MAP_SHARED,memfd,0);
  ROS_ASSERT_MSG(buf != MAP_FAILED, "mmap failed on memfd=%i (%m)", memfd);
  memset(buf,0xAA,10000000); // workaround

  return MemfdMessage::Ptr(new MemfdMessage(memfd, (char*)buf, size));
}

MemfdMessage::~MemfdMessage()
{
    if (buf_)
    {
        munmap(buf_, size_);
        buf_ = NULL;
        size_ = 0;
    }
    if (fd_>=0)
    {
//	fprintf(stderr, "Closed memfd %i\n", fd_);
	close(fd_);
	fd_ = -1;
    }
}

}

