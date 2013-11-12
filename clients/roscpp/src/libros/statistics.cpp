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
#include "ros/message_traits.h"
#include "std_msgs/Header.h"

namespace ros
{

StatisticsLogger::StatisticsLogger()
{
  pub_frequency_ = 1.0;
}

StatisticsLogger::~StatisticsLogger()
{
}

void StatisticsLogger::init(const SubscriptionCallbackHelperPtr& helper) {
  hasHeader_ = helper->hasHeader();
}

void StatisticsLogger::callback(const boost::shared_ptr<M_string>& connection_header,
				const std::string topic, const std::string callerid, const SerializedMessage& m, const uint64_t bytes_sent,
				const ros::Time& received_time, const bool dropped)
{
  struct StatData stats;

  std::map<std::string,struct StatData>::iterator stats_it = map_.find(callerid);
  if (stats_it == map_.end()) {
    stats.stat_bytes_last = 0;
    stats.dropped_msgs = 0;
    stats.last_publish = ros::Time::now();
    map_[callerid] = stats;
  } else {
    stats = map_[callerid];
  }

  stats.arrival_time_list.push_back(received_time);

  if (dropped)
    stats.dropped_msgs++;

  if (hasHeader_) {
    // TODO: maybe wrap this with an exception handler,
    // in case serialization fails?
    std_msgs::Header header;
    ros::serialization::IStream stream(m.buf.get(), m.num_bytes);
    ros::serialization::deserialize(stream, header);
    stats.delay_list.push_back((received_time-header.stamp).toSec());
  }

  if (stats.last_publish + ros::Duration(pub_frequency_) < received_time) {
    ros::Time window_start = stats.last_publish;
    stats.last_publish = received_time;

    rosgraph_msgs::TopicStatistics msg;
    msg.topic = topic;
    msg.node_pub = callerid;
    msg.node_sub = ros::this_node::getName();
    msg.window_start = window_start;
    msg.window_stop = received_time;
    msg.dropped_msgs = stats.dropped_msgs;
    msg.traffic = (bytes_sent - stats.stat_bytes_last) / pub_frequency_;

    if (stats.delay_list.size()>0) {
      msg.stamp_delay_mean = 0;
      msg.stamp_delay_max = 0;
      for(std::list<double>::iterator it = stats.delay_list.begin(); it != stats.delay_list.end(); it++) {
	double delay = *it;
	msg.stamp_delay_mean += delay;
	if (delay > msg.stamp_delay_max)
	    msg.stamp_delay_max = delay;
      }
      msg.stamp_delay_mean /= stats.delay_list.size();

      msg.stamp_delay_variance = 0;
      for(std::list<double>::iterator it = stats.delay_list.begin(); it != stats.delay_list.end(); it++) {
	double t = msg.stamp_delay_mean - *it;
	msg.stamp_delay_variance += t*t;
      }
      msg.stamp_delay_variance /= stats.delay_list.size();

    } else {
        msg.stamp_delay_mean = nan("char-sequence");
        msg.stamp_delay_variance = nan("char-sequence");
        msg.stamp_delay_max = nan("char-sequence");
    }
    if (stats.arrival_time_list.size()>1) {
      ros::Time prev;
      msg.period_mean = 0;
      msg.period_max = 0;
      for(std::list<ros::Time>::iterator it = stats.arrival_time_list.begin(); it != stats.arrival_time_list.end(); it++) {
	if (it==stats.arrival_time_list.begin()) {
	  prev = *it;
	  continue;
	}
	double period = (*it-prev).toSec();
	msg.period_mean += period;
	if (period > msg.period_max)
	    msg.period_max = period;
	prev = *it;
      }
      msg.period_mean /= (stats.arrival_time_list.size()-1);

      msg.period_variance = 0;
      for(std::list<ros::Time>::iterator it = stats.arrival_time_list.begin(); it != stats.arrival_time_list.end(); it++) {
	if (it==stats.arrival_time_list.begin()) {
	  prev = *it;
	  continue;
	}
	double period = (*it-prev).toSec();
	double t = msg.period_mean - period;
	msg.period_variance += t*t;
	prev = *it;
	
      }
      msg.period_variance /= (stats.arrival_time_list.size()-1);
    } else {
        msg.period_mean = nan("char-sequence");
        msg.period_variance = nan("char-sequence");
        msg.period_max = nan("char-sequence");
    }
 
      if (!pub_.getTopic().length()) {
        ros::NodeHandle n("~");
	// creating the publisher in the constructor results in a deadlock. so do it here.
        pub_ = n.advertise<rosgraph_msgs::TopicStatistics>("/statistics",1);
      }
    pub_.publish(msg);

    stats.delay_list.clear();
    stats.arrival_time_list.clear();
    stats.dropped_msgs = 0;
    stats.stat_bytes_last = bytes_sent;

  }
  map_[callerid] = stats;
}


} // namespace ros
