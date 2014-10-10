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

#include <sys/mman.h>

#include <boost/utility/enable_if.hpp>
#include <boost/interprocess/managed_external_buffer.hpp>

#include <ros/transport/memfd_message.h>
#include <ros/forwards.h>
#include <ros/common.h>
#include <ros/serialized_message.h>
#include <ros/boost_container.h>
#include <ros/message_factory.h>
#include <ros/parameter_adapter.h>
#include "ros/message_traits.h"

namespace ros
{

template<typename P>
inline typename boost::enable_if<ros::message_traits::IsShmemReady<typename ParameterAdapter<P>::Message>, SerializedMessage >::type kdbusCloneMessage(const P& new_msg)
{
    typedef typename ParameterAdapter<P>::Message NonConstType;

    // TODO: what about a message that is large than the default size?
    typename NonConstType::Ptr bare_msg = ros::make_shared<NonConstType>();
    *bare_msg = new_msg;

    SerializedMessage ret;
    ret.message = bare_msg;
    ret.memfd_message = ros::MessageFactory::getMemfdMessage(ret.message);
    return ret;
}

template<typename P>
inline typename boost::disable_if<ros::message_traits::IsShmemReady<typename ParameterAdapter<P>::Message>, SerializedMessage >::type kdbusCloneMessage(const P& new_msg)
{
    ROS_ASSERT_MSG(false, "Should never be called.");
    SerializedMessage ret;
    return ret;
}

}

#endif // ROSCPP_MEMFD_SERIALIZE_H

