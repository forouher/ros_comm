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

#ifndef ROSCPP_MEMFD_SERIALIZE_H
#define ROSCPP_MEMFD_SERIALIZE_H

#include "forwards.h"
#include "common.h"
#include <sys/mman.h>

#include <boost/utility/enable_if.hpp>
#include <ros/memfd_message.h>
#include <ros/serialized_message.h>
#include <boost/interprocess/managed_external_buffer.hpp>
#include "ros/parameter_adapter.h"
#include <ros/boost_container.h>

namespace ros
{

inline void dump_memory2(void* d, size_t len)
{   
    char* data = (char*)d;
    
    size_t i;
    printf("Data in [%p..%p): ",data,data+len);
    for (i=0;i<len;i++)
        printf("%02X ", ((unsigned char*)data)[i] );
    printf("\n");
}

template<typename M>
inline SerializedMessage shmemSerializeMessage(const M& message)
{
    return shmemSerializeMessageI(message);
}

template<typename M>
inline SerializedMessage shmemSerializeMessageI(const M& message,
					       typename boost::enable_if<ros::message_traits::IsShmemReady<M> >::type*_=0)
{
  SerializedMessage m;
  typedef typename ParameterAdapter<M>::Message PureType;

//  if (message.mem_) {
    m.memfd_message = MemfdMessage::create(MemfdMessage::MAX_SIZE);
    ROS_ASSERT(m.memfd_message);
    ROS_ASSERT(m.memfd_message->size_==MemfdMessage::MAX_SIZE);
  
    boost::interprocess::managed_external_buffer segment(boost::interprocess::create_only, m.memfd_message->buf_, m.memfd_message->size_);
    typename PureType::allocator alloc (segment.get_segment_manager());
    PureType* p = segment.construct<PureType>("DATA")(message,alloc);
    p->__connection_header.reset();
    p->mem_.reset();
//  } else {
/*    fprintf(stderr, "using optimized shmemSerializeMessageI\n");

    const boost::shared_ptr<MemfdMessage> backup = message.mem_;
    message.mem_.reset();

    m.memfd_message = MemfdMessage::create(MemfdMessage::MAX_SIZE);
    ROS_ASSERT(m.memfd_message);
    ROS_ASSERT(m.memfd_message->size_==MemfdMessage::MAX_SIZE);
    memcpy(m.memfd_message->buf_, message.mem_->buf_, message.mem_->size_);

    boost::interprocess::managed_external_buffer segment(boost::interprocess::open_only, m.memfd_message->buf_, m.memfd_message->size_);
    PureType* p = segment.find<PureType>("DATA").first;
    p->__connection_header.reset();

    message.mem_ = backup;
*/
//  }
  return m;
}

template<typename M>
inline SerializedMessage shmemSerializeMessageI(const M& message,
					       typename boost::disable_if<ros::message_traits::IsShmemReady<M> >::type*_=0)
{
  SerializedMessage m;
  return m;
}

}

#endif // ROSCPP_MEMFD_SERIALIZE_H

