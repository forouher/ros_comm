
/*
 * Copyright (C) 2010, Willow Garage, Inc.
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

#ifndef ROSCPP_MEMFD_EVENT_H
#define ROSCPP_MEMFD_EVENT_H

#include "ros/forwards.h"
#include "ros/time.h"
#include <ros/assert.h>
#include <ros/message_traits.h>
#include <ros/memfd_message.h>

#include <boost/type_traits/is_void.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/type_traits/is_const.hpp>
#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#include <ros/message_factory.h>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/smart_ptr/deleter.hpp>
#include <boost/interprocess/smart_ptr/shared_ptr.hpp>


namespace ros
{

inline void dump_memory(void* d, size_t len)
{
    char* data = (char*)d;

    size_t i;
    printf("Data in [%p..%p): ",data,data+len);
    for (i=0;i<len;i++)
        printf("%02X ", ((unsigned char*)data)[i] );
    printf("\n");
}


template<typename T>
static void Deleter( T* ptr)
{
    if (ptr->mem_)
        ptr->mem_.reset();
}

template<typename T>
static void DeleterShmem( T* ptr)
{
}

// TODO: this is too late. we should never create a Shmem Message in the first place!
template<typename M>
boost::shared_ptr<M> makeSharedPtrFromMessage(M* msg, MemfdMessage::Ptr m,
                                              typename boost::enable_if<ros::message_traits::IsShmemReady<M> >::type*_=0)
{
    msg->mem_ = m;
    return boost::shared_ptr<M>(msg, &Deleter<M>);
}

template<typename M>
boost::shared_ptr<M> makeSharedPtrFromMessage(M* msg, MemfdMessage::Ptr m,
                                              typename boost::disable_if<ros::message_traits::IsShmemReady<M> >::type*_=0)
{
    ROS_ASSERT(false);
    return boost::shared_ptr<M>(msg);
}

template<typename M>
struct DefaultMemfdMessageCreator
{
//  typedef ParameterAdapter<M>::Message PureType; // sollte bereits richtiger typ sein

  boost::shared_ptr<M> operator()(MemfdMessage::Ptr m)
  {
    ROS_ASSERT(m);
    ROS_ASSERT(m->size_==MemfdMessage::MAX_SIZE);

//    boost::interprocess::managed_external_buffer segment(boost::interprocess::open_only, m->buf_, m->size_);
    M* msg = NULL; //segment.find<M>("DATA").first;
    ROS_ASSERT(msg != NULL);
    return  makeSharedPtrFromMessage<M>(msg, m);
  }
};

template<typename M>
struct DefaultShmemMessageCreator
{
  typedef boost::interprocess::managed_shared_memory::segment_manager segment_manager_type;
  typedef boost::interprocess::ros_allocator< void, segment_manager_type> void_allocator_type;

  typedef boost::interprocess::shared_ptr<M, void_allocator_type, boost::interprocess::deleter< M, segment_manager_type> > NonConstTypePtr;

  NonConstTypePtr operator()()
  {
    NonConstTypePtr msg = MessageFactory::createSharedMessage<M>();
    ROS_ASSERT(msg != NULL);
    return msg;
  }

};

template<typename M>
struct DefaultShmemMessageFinder
{
  typedef boost::interprocess::managed_shared_memory::segment_manager segment_manager_type;
  typedef boost::interprocess::ros_allocator< void, segment_manager_type> void_allocator_type;

  typedef boost::interprocess::shared_ptr<M, void_allocator_type, boost::interprocess::deleter< M, segment_manager_type> > NonConstTypePtr;

  NonConstTypePtr operator()(const boost::uuids::uuid uuid)
  {
    NonConstTypePtr* msg = MessageFactory::findMessage<M>(uuid);
    ROS_ASSERT(msg != NULL);
    return *msg;
  }

};



}

#endif // ROSCPP_MESSAGE_EVENT_H
