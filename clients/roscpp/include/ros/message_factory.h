
/*
 * Copyright (C) 2014 Dariush Forouher
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
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

#ifndef ROSCPP_MESSAGE_FACTORY_H
#define ROSCPP_MESSAGE_FACTORY_H

#include "ros/forwards.h"
#include "ros/time.h"
#include <ros/assert.h>
#include <ros/message_traits.h>
#include <ros/boost_container.h>
#include <ros/transport/memfd_message.h>
#include <ros/transport/memfd.h>

#include <boost/type_traits/is_void.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/type_traits/is_const.hpp>
#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#include <boost/interprocess/managed_external_buffer.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>
#include <boost/interprocess/interprocess_fwd.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/container/deque.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/smart_ptr/shared_ptr.hpp>
#include <boost/interprocess/managed_external_buffer.hpp>
#include <boost/thread/mutex.hpp>

#include <fcntl.h>

#include <map>

#define ELEMENT_NAME "foo"

namespace ros
{

// TODO: create msg storage. maybe inside factory?
class null_deleter
{
    public:
    static std::map<const void*, MemfdMessage::Ptr> map_;
    static boost::mutex mutex_;

    void operator()(const void* ptr)
    {
        boost::mutex::scoped_lock lock(null_deleter::mutex_);
        map_.erase(ptr);
    }
};


template<typename M>
static boost::shared_ptr<M> make_shared(size_t size = 1000000)
{

  int ret = syscall(__NR_memfd_create, "rosmsg" , MFD_ALLOW_SEALING);
  ROS_ASSERT_MSG(ret >= 0, "Could not create memfd, %d (%m)", ret);

  int memfd = ret;

  ret = ftruncate(memfd, size);
  ROS_ASSERT_MSG(!ret, "Could not truncate memfd, %d (%m)", ret);

  // map memfd file to mem
  void* buf = mmap(NULL, size,PROT_WRITE|PROT_READ,MAP_SHARED,memfd,0);
  ROS_ASSERT_MSG(buf != MAP_FAILED, "mmap failed on memfd=%i (%m)", memfd);

  // create object in mem
  ros_managed_external_buffer segment(boost::interprocess::create_only, buf, size);

  typename M::allocator alloc (segment.get_segment_manager());

  M* msg = segment.construct<M>(ELEMENT_NAME)(alloc);

  MemfdMessage::Ptr memfd_msg = boost::make_shared<MemfdMessage>(memfd, buf, size);
  null_deleter::mutex_.lock();
  null_deleter::map_[msg] = memfd_msg;
  null_deleter::mutex_.unlock();
  boost::shared_ptr<M> bp(msg, null_deleter());

  return bp;
};

class MessageFactory
{

public:


static int inline getKdbusMessageFd(const boost::shared_ptr<void const>& msg)
{
    boost::mutex::scoped_lock lock(null_deleter::mutex_);

    try {
        MemfdMessage::Ptr memfd_msg = null_deleter::map_.at(msg.get());
        return memfd_msg->fd_;
    } catch (const std::out_of_range& e) {
        return -1;
    }
}

static MemfdMessage::Ptr inline getMemfdMessage(const boost::shared_ptr<void const>& msg)
{
    boost::mutex::scoped_lock lock(null_deleter::mutex_);
    return null_deleter::map_.at(msg.get());
}

static bool inline isKdbusMessage(const boost::shared_ptr<void const>& msg)
{
    boost::mutex::scoped_lock lock(null_deleter::mutex_);
    const void* ptr = msg.get();
    int count = null_deleter::map_.count(msg.get());
    if (count == 0)
        return false;
    else
        return true;
}

template<typename M>
static const boost::shared_ptr<M> inline retrieveKdbusMessage(MemfdMessage::Ptr memfd_msg)
{

  ros_managed_external_buffer segment(boost::interprocess::open_only, memfd_msg->buf_, memfd_msg->size_);

  std::pair<M*, boost::interprocess::managed_shared_memory::size_type> result;
  result = segment.find<M>(ELEMENT_NAME);
  ROS_ASSERT_MSG(result.second == 1, "could not retrieve msg from segment, not found.");

  M* msg = result.first;
  ROS_ASSERT_MSG(msg != NULL, "could not retrieve msg from segment, strange stuff going on.");

  null_deleter::mutex_.lock();
  null_deleter::map_[msg] = memfd_msg;
  null_deleter::mutex_.unlock();

  boost::shared_ptr<M> bp(msg, null_deleter());

  return bp;
}

static inline void sealKdbusMessage(const boost::shared_ptr<const void>& msg)
{
  null_deleter::mutex_.lock();
  MemfdMessage::Ptr ptr = null_deleter::map_[msg.get()];
  null_deleter::mutex_.unlock();

  // remap to readonly. this is rather slow, unfortunately
  void* retval = mmap(ptr->buf_, ptr->size_, PROT_READ, MAP_PRIVATE|MAP_FIXED, ptr->fd_, 0);
  ROS_ASSERT_MSG(retval != MAP_FAILED, "mmap to readonly failed on memfd=%i (%m)", ptr->fd_);

  // seal memfd
  int ret = fcntl(ptr->fd_, F_ADD_SEALS, F_SEAL_SHRINK | F_SEAL_GROW | F_SEAL_WRITE);
  ROS_ASSERT_MSG(ret>=0, "failed to seal memfd=%i", ptr->fd_);
};

};

}

#endif // ROSCPP_MESSAGE_EVENT_H
