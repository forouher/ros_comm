/*
 * Copyright (C) 2013, Dariush Forouher
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

#include "ros/statistics.h"
#include "ros/node_handle.h"
#include <rosgraph_msgs/TopicStatistics.h>
#include "ros/this_node.h"

namespace ros
{

StatisticsLogger::StatisticsLogger()
{
  ros::NodeHandle n("~");
  pub_ = n.advertise<rosgraph_msgs::TopicStatistics>("/statistics",1);
  last_publish_ = ros::Time::now();
}

StatisticsLogger::~StatisticsLogger()
{
}

void StatisticsLogger::callback(const SerializedMessage& m)
{
  ros::Time now = ros::Time::now();

  if (last_publish_ + ros::Duration(1.0) < now) {
    ros::Time window_start = last_publish_;
    last_publish_ = now;

    rosgraph_msgs::TopicStatistics msg;
    msg.topic = "/foo";
    msg.node_pub = "unkonwn";
    msg.node_sub = ros::this_node::getName();
    msg.window_start = window_start;
    msg.window_stop = now;
    msg.dropped_msgs = 0;
    msg.traffic = 0;
    msg.period_mean = 0;
    msg.period_variance = 0;
    msg.period_max = 0;
    msg.stamp_delay_mean = 0;
    msg.stamp_delay_variance = 0;
    msg.stamp_delay_max = 0;
    pub_.publish(msg);
  }
}


} // namespace ros
