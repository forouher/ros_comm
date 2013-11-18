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

#ifndef STATISTICS_H
#define STATISTICS_H

#include "forwards.h"
#include "poll_set.h"
#include "common.h"
#include "publisher.h"
#include <ros/time.h>
#include "ros/subscription_callback_helper.h"
#include <map>

namespace ros
{

class ROSCPP_DECL StatisticsLogger
{
public:

  StatisticsLogger();
  ~StatisticsLogger();

  void init(const SubscriptionCallbackHelperPtr& helper);

  void callback(const boost::shared_ptr<M_string>& connection_header, const std::string topic, const std::string callerid, const SerializedMessage& m, const uint64_t bytes_sent, const ros::Time& received_time, const bool dropped);

private:

  // these are hard constrains
  static const int MAX_WINDOW = 64;
  static const int MIN_WINDOW = 4;

  // these are soft constrains 
  static const int MAX_ELEMENTS = 100;
  static const int MIN_ELEMENTS = 10;

  bool hasHeader_;
  double pub_frequency_;
  ros::Publisher pub_;
  struct StatData {
    ros::Time last_publish;
    std::list<ros::Time> arrival_time_list;
    std::list<double> delay_list;
    uint64_t dropped_msgs;
    uint64_t last_seq;
    uint64_t stat_bytes_last;
  };
  std::map<std::string, struct StatData> map_;
};

}

#endif
