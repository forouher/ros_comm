/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Dariush Forouher
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

#include "ros/kdbus_transport_subscriber_link.h"
#include "ros/publication.h"
#include "ros/header.h"
#include "ros/connection.h"
#include "ros/transport/transport.h"
#include "ros/this_node.h"
#include "ros/connection_manager.h"
#include "ros/topic_manager.h"
#include "ros/file_log.h"
#include "ros/transport/memfd_message.h"
#include <ros/message_factory.h>

#include <boost/bind.hpp>

namespace ros
{

KdbusTransportSubscriberLink::KdbusTransportSubscriberLink()
 : transport_("1000-ros")
{

}

KdbusTransportSubscriberLink::~KdbusTransportSubscriberLink()
{
  drop();
}

bool KdbusTransportSubscriberLink::initialize(const std::string& topic, const std::string& client_name)
{
  ROS_DEBUG("KdbusTransportSubscriberLink::initialize(%s,%s)", topic.c_str(), client_name.c_str());

  pt_ = TopicManager::instance()->lookupPublication(topic);
  if (!pt_)
  {
    std::string msg = std::string("received a connection for a nonexistent topic [") +
                    topic + std::string("] from [" + client_name + "].");

    ROSCPP_LOG_DEBUG("%s", msg.c_str());

    return false;
  }

  recv_name_ = client_name;
  topic_ = topic;
  running_ = false;

  transport_.create_bus();
  transport_.open_connection();

  pt_->addSubscriberLink(shared_from_this());

  return true;
}

void KdbusTransportSubscriberLink::enqueueMessage(const SerializedMessage& m, bool ser, bool nocopy)
{
  ROS_ASSERT_MSG(m.memfd_message, "bad message on topic '%s' to '%s'", topic_.c_str(), recv_name_.c_str());
  // TODO: set m.type_info correctly ???

  ros::MessageFactory::sealKdbusMessage(m.message);

  // check for error. if "no such process", then kill connection.
  if (!transport_.sendMessage(m, recv_name_))
  {
    // at the beginning, we may loose some messages due to race conditions.
    // only drop the connection after we have sent at least one message.
    // TODO: if the subscriber dies really early, this might leave us hanging...
    if (running_) {
      pt_->removeSubscriberLink(shared_from_this());
    }
  } else {
    running_ = true;
  }
}

std::string KdbusTransportSubscriberLink::getTransportType()
{
  return "KDBusROS";
}

std::string KdbusTransportSubscriberLink::getTransportInfo()
{
  return std::string("This is a KDBus subscriber link. So, now you know.");
}

void KdbusTransportSubscriberLink::drop()
{
  fprintf(stderr, "KdbusTransportSubscriberLink::drop");
  // TODO ???
}

} // namespace ros
