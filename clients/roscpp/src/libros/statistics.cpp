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
  last_publish_ = ros::Time::now();
  stat_bytes_last_ = 0;
  dropped_msgs_ = 0;

  pub_frequency_ = 1.0;
}

StatisticsLogger::~StatisticsLogger()
{
}

void StatisticsLogger::callback(const std::string topic, const std::string callerid, const SerializedMessage& m, uint64_t bytes_sent)
{
  ros::Time now = ros::Time::now();

  arrival_time_list_.push_back(now);

  bool has_header = false;
  if (has_header) {
    ros::Time header_stamp = now;
    uint64_t seq_no = 0;

    delay_list_.push_back((now-header_stamp).toSec());

    if (++last_seq_ != seq_no) {
      last_seq_ = seq_no;
      dropped_msgs_++; // TODO: may have dropped more than one
      // TODO: maybe messages are dropped AFTER this statistics are done...?
    }
  }

  if (last_publish_ + ros::Duration(pub_frequency_) < now) {
    ros::Time window_start = last_publish_;
    last_publish_ = now;

    rosgraph_msgs::TopicStatistics msg;
    msg.topic = topic;
    msg.node_pub = callerid;
    msg.node_sub = ros::this_node::getName();
    msg.window_start = window_start;
    msg.window_stop = now;
    msg.dropped_msgs = dropped_msgs_;
    msg.traffic = (bytes_sent - stat_bytes_last_) / pub_frequency_;

    if (delay_list_.size()>0) {
      msg.stamp_delay_mean = 0;
      msg.stamp_delay_max = 0;
      for(std::list<double>::iterator it = delay_list_.begin(); it != delay_list_.end(); it++) {
	double delay = *it;
	msg.stamp_delay_mean += delay;
	if (delay > msg.stamp_delay_max)
	    msg.stamp_delay_max = delay;
      }
      msg.stamp_delay_mean /= delay_list_.size();

      msg.stamp_delay_variance = 0;
      for(std::list<double>::iterator it = delay_list_.begin(); it != delay_list_.end(); it++) {
	double t = msg.stamp_delay_mean - *it;
	msg.stamp_delay_variance += t*t;
      }
      msg.stamp_delay_variance /= delay_list_.size();

    } else {
        msg.stamp_delay_mean = nan("char-sequence");
        msg.stamp_delay_variance = nan("char-sequence");
        msg.stamp_delay_max = nan("char-sequence");
    }
    if (arrival_time_list_.size()>1) {
      ros::Time prev;
      msg.period_mean = 0;
      msg.period_max = 0;
      for(std::list<ros::Time>::iterator it = arrival_time_list_.begin(); it != arrival_time_list_.end(); it++) {
	if (it==arrival_time_list_.begin()) {
	  prev = *it;
	  continue;
	}
	double period = (*it-prev).toSec();
	msg.period_mean += period;
	if (period > msg.period_max)
	    msg.period_max = period;
	prev = *it;
      }
      msg.period_mean /= (arrival_time_list_.size()-1);

      msg.period_variance = 0;
      for(std::list<ros::Time>::iterator it = arrival_time_list_.begin(); it != arrival_time_list_.end(); it++) {
	if (it==arrival_time_list_.begin()) {
	  prev = *it;
	  continue;
	}
	double period = (*it-prev).toSec();
	double t = msg.period_mean - period;
	msg.period_variance += t*t;
	prev = *it;
	
      }
      msg.period_variance /= (arrival_time_list_.size()-1);
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

    delay_list_.clear();
    arrival_time_list_.clear();
    dropped_msgs_ = 0;
    stat_bytes_last_ = bytes_sent;

  }
}


} // namespace ros
