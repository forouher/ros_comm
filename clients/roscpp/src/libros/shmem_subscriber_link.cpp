
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

#include <boost/bind.hpp>

namespace ros
{

ShmemSubscriberLink::ShmemSubscriberLink(const PublicationPtr& parent)
: dropped_(false)
{
  ROS_ASSERT(parent);
  parent_ = parent;
  topic_ = parent->getName();
}

ShmemSubscriberLink::~ShmemSubscriberLink()
{
}

bool ShmemSubscriberLink::isLatching()
{
  if (PublicationPtr parent = parent_.lock())
  {
    return parent->isLatching();
  }

  return false;
}

void ShmemSubscriberLink::enqueueMessage(const SerializedMessage& m, bool ser, bool nocopy)
{
  boost::recursive_mutex::scoped_lock lock(drop_mutex_);
  if (dropped_)
  {
    return;
  }
}

std::string ShmemSubscriberLink::getTransportType()
{
  return std::string("INTRAPROCESS");
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

  if (PublicationPtr parent = parent_.lock())
  {
    ROSCPP_LOG_DEBUG("Connection to local subscriber on topic [%s] dropped", topic_.c_str());

    parent->removeSubscriberLink(shared_from_this());
  }
}

void ShmemSubscriberLink::getPublishTypes(bool& ser, bool& nocopy, bool& shmem, const std::type_info& ti)
{
  boost::recursive_mutex::scoped_lock lock(drop_mutex_);
  if (dropped_)
  {
    return;
  }
  shmem = false;
//  subscriber_->getPublishTypes(ser, nocopy, ti);
}

} // namespace ros
