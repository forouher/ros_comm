
/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
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

#include "ros/shmem_subscriber_link.h"
#include "ros/publication.h"
#include "ros/header.h"
#include "ros/connection.h"
#include "ros/transport/transport.h"
#include "ros/this_node.h"
#include "ros/connection_manager.h"
#include "ros/topic_manager.h"
#include "ros/file_log.h"
#include <sensor_msgs/PointCloud3.h>

#include <boost/bind.hpp>

namespace ros
{

ShmemSubscriberLink::ShmemSubscriberLink()
: dropped_(false)
{
}

ShmemSubscriberLink::~ShmemSubscriberLink()
{
}

bool ShmemSubscriberLink::isLatching()
{
  return false;
}

void ShmemSubscriberLink::enqueueMessage(const SerializedMessage& m, bool ser, bool nocopy)
{
  boost::recursive_mutex::scoped_lock lock(drop_mutex_);

  boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock2(deque_->mutex_);

  sensor_msgs::PointCloud3::IPtr foo = ros::MessageFactory::createMessage<sensor_msgs::PointCloud3>();
  ShmemDequeVoid::VoidIPtr* foo_ptr = (ShmemDequeVoid::VoidIPtr*)(&foo);
  ShmemDequeVoid::VoidIPtr foo2 = ShmemDequeVoid::VoidIPtr(*foo_ptr);
  foo.reset(); foo_ptr = NULL;
  fprintf(stderr, "use count of foo2: %i\n", foo2.use_count());

  deque_->add(foo2);

  deque_->signal_.notify_one();

  if (dropped_)
  {
    return;
  }
}

std::string ShmemSubscriberLink::getTransportType()
{
  return std::string("ShmemTransport");
}

void ShmemSubscriberLink::initialize(const std::string& topic, const std::string& deque_uuid)
{
  PublicationPtr pt = TopicManager::instance()->lookupPublication(topic);
  if (!pt)
  {
    std::string msg = std::string("received a connection for a nonexistent topic [") +
                    topic + std::string("].");
  
    ROSCPP_LOG_DEBUG("%s", msg.c_str());
  
    return;
  }

  topic_ = topic;

  ROS_DEBUG("Creating shmem deque with UUID %s", deque_uuid.c_str());
  deque_ = MessageFactory::createDeque(deque_uuid);
  pt->addSubscriberLink(shared_from_this());

  return;

}

void ShmemSubscriberLink::drop()
{
  {
    boost::recursive_mutex::scoped_lock lock(drop_mutex_);
    if (dropped_)
    {
      return;
    }

    dropped_ = true;
  }
}

void ShmemSubscriberLink::getPublishTypes(bool& ser, bool& nocopy, bool& shmem, const std::type_info& ti)
{
  boost::recursive_mutex::scoped_lock lock(drop_mutex_);
  if (dropped_)
  {
    return;
  }
  shmem = true;
//  subscriber_->getPublishTypes(ser, nocopy, ti);
}

} // namespace ros
