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

#include "ros/shmem_publisher_link.h"
#include "ros/subscription.h"
#include "ros/header.h"
#include "ros/connection.h"
#include "ros/transport/transport.h"
#include "ros/this_node.h"
#include "ros/connection_manager.h"
#include "ros/file_log.h"

#include <boost/bind.hpp>
#include <ros/message_factory.h>

#include <sstream>

namespace ros
{

ShmemPublisherLink::ShmemPublisherLink(const SubscriptionPtr& parent, const std::string& xmlrpc_uri, const TransportHints& transport_hints)
: PublisherLink(parent, xmlrpc_uri, transport_hints)
, dropped_(false)
{
}

ShmemPublisherLink::~ShmemPublisherLink()
{
}

void ShmemPublisherLink::initialize(const std::string& deque_uuid)
{
  ROS_DEBUG("Trying to open shmem deque with uuid %s", deque_uuid.c_str());
  deque_ = MessageFactory::findDeque(deque_uuid);
  ROS_DEBUG("Opened shmem deque with uuid %s", deque_uuid.c_str());
  thread_ = boost::thread(&ShmemPublisherLink::threadRunner, this);
}

void ShmemPublisherLink::threadRunner()
{
  while (true) {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(deque_->mutex_);
    deque_->signal_.wait(lock);
    fprintf(stderr,"ShmemPublisherLink::threadRunner() has been awakened!\n");

    while (!deque_->isEmpty())
    {
      boost::uuids::uuid uuid = deque_->remove();
      fprintf(stderr, "removed uuid %s from deque\n", boost::uuids::to_string(uuid).c_str());
      SerializedMessage msg;
      msg.uuid = uuid;
      handleMessage(msg, false, true);
    }
  }
}

void ShmemPublisherLink::drop()
{
  {
    boost::recursive_mutex::scoped_lock lock(drop_mutex_);
    if (dropped_)
    {
      return;
    }

    dropped_ = true;
  }

  if (SubscriptionPtr parent = parent_.lock())
  {
    ROSCPP_LOG_DEBUG("Connection to local publisher on topic [%s] dropped", parent->getName().c_str());

    parent->removePublisherLink(shared_from_this());
  }
}

void ShmemPublisherLink::handleMessage(const SerializedMessage& m, bool ser, bool nocopy)
{
  boost::recursive_mutex::scoped_lock lock(drop_mutex_);
  if (dropped_)
  {
    return;
  }

  SubscriptionPtr parent = parent_.lock();

  if (parent)
  {
    parent->handleMessage(m, ser, nocopy, header_.getValues(), shared_from_this());
  }
}

std::string ShmemPublisherLink::getTransportType()
{
  return std::string("ShmemTransport");
}

void ShmemPublisherLink::getPublishTypes(bool& ser, bool& nocopy, const std::type_info& ti)
{
  boost::recursive_mutex::scoped_lock lock(drop_mutex_);
  if (dropped_)
  {
    ser = false;
    nocopy = false;
    return;
  }

  SubscriptionPtr parent = parent_.lock();
  if (parent)
  {
    parent->getPublishTypes(ser, nocopy, ti);
  }
  else
  {
    ser = true;
    nocopy = false;
  }
}

} // namespace ros

