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

#include <ros/platform.h>  // platform dependendant requirements

#include "ros/kdbus_transport_publisher_link.h"
#include "ros/subscription.h"
#include "ros/header.h"
#include "ros/connection.h"
#include "ros/transport/transport.h"
#include "ros/this_node.h"
#include "ros/connection_manager.h"
#include "ros/file_log.h"
#include "ros/poll_manager.h"
#include "ros/transport/transport_tcp.h"
#include "ros/transport/transport_kdbus.h"
#include "ros/timer_manager.h"
#include "ros/callback_queue.h"
#include "ros/internal_timer_manager.h"

#include <boost/bind.hpp>

#include <sstream>

namespace ros
{

KdbusTransportPublisherLink::KdbusTransportPublisherLink(const SubscriptionPtr& parent, const std::string& xmlrpc_uri, const TransportHints& transport_hints)
: PublisherLink(parent, xmlrpc_uri, transport_hints)
, dropping_(false)
{
}

KdbusTransportPublisherLink::~KdbusTransportPublisherLink()
{
}

bool KdbusTransportPublisherLink::initialize(std::string& endpoint_name)
{
  ROS_DEBUG("KdbusTransportPublisherLink::initialize(%s)", endpoint_name.c_str());

  // create kdbus connection, open bus

  // we have to aquire name endpoint_name, messages will be send there
  int sock_  = 0;
  ROS_DEBUG("Adding kdbus connection [%d] to pollset", sock_);
  PollManager::instance()->getPollSet().addSocket(sock_, boost::bind(&KdbusTransportPublisherLink::onMessage, this, _1));

  return true;
}

void KdbusTransportPublisherLink::onMessage(int events)
{
  
// SerializedMessage(boost::shared_ptr<MemfdMessage>)

//handleMessage(SerializedMessage(buffer, size), true, false);
}

void KdbusTransportPublisherLink::drop()
{
  dropping_ = true;

  if (SubscriptionPtr parent = parent_.lock())
  {
    parent->removePublisherLink(shared_from_this());
  }
}

void KdbusTransportPublisherLink::handleMessage(const SerializedMessage& m, bool ser, bool nocopy)
{
  stats_.bytes_received_ += m.num_bytes;
  stats_.messages_received_++;

  SubscriptionPtr parent = parent_.lock();

  if (parent)
  {
    //stats_.drops_ += parent->handleMessage(m, ser, nocopy, getConnection()->getHeader().getValues(), shared_from_this());
  }
}

std::string KdbusTransportPublisherLink::getTransportType()
{
  return "KDBusROS";
}

} // namespace ros

