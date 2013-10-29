# Software License Agreement (BSD License)
#
# Copyright (c) 2013 Dariush Forouher
# All rights reserved.
#
# Based on code adapted from diagnostics_updater by Blaise Gassend
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$



import struct
import select
try:
    from cStringIO import StringIO #Python 2.x
    import thread as _thread # Python 2
    python3 = 0
    def isstring(s):
        return isinstance(s, basestring) #Python 2.x
except ImportError:
    python3 = 1
    from io import StringIO, BytesIO #Python 3.x
    import _thread 
    def isstring(s):
        return isinstance(s, str) #Python 3.x
    		
import threading
import logging
import time

from itertools import chain
import traceback

import rosgraph.names

from rospy.core import *
from rospy.exceptions import ROSSerializationException, TransportTerminated
from rospy.msg import serialize_message, args_kwds_to_message
from rosgraph_msgs.msg import TopicStatistics

from rospy.impl.registration import get_topic_manager, set_topic_manager, Registration, get_registration_listeners
from rospy.impl.tcpros import get_tcpros_handler, DEFAULT_BUFF_SIZE

_logger = logging.getLogger('rospy.impl.statistics')

# wrap genpy implementation and map it to rospy namespace
import genpy
Message = genpy.Message



class SubscriberStatisticsLogger():
    """
    Class that monitors each subscriber.

    this class basically just keeps a collection of ConnectionStatisticsLogger.
    """

    def __init__(self, subscriber):
        self.subscriber = subscriber
	self.connections = dict()
        pass

    def callback(self,msg,publisher, stat_bytes):

	# create ConnectionStatisticsLogger for new connections
	logger = self.connections.get(publisher)
	if logger == None:
	    logger = ConnectionStatisticsLogger(self.subscriber.name, rospy.get_name(), publisher)
	    self.connections[publisher] = logger

	# delegate stuff to that instance
	logger.callback(msg, stat_bytes)

        pass

class ConnectionStatisticsLogger():
    """
    Class that monitors lots of stuff for each connection
    
    is created whenever a subscriber is created.
    is destroyed whenever its parent subscriber is destroyed.
    its lifecycle is therefore bound to its parent subscriber.
    
    XXX: the are threading/concurrent access issues meantioned at different
    places. what do i have to keep in mind?
    - may there be callbacks called in parallel? sounds unlikely
    - may additional methods (which i don't have any a.t.m., be called in
      parallel? probably.

    """

    def __init__(self, topic, subscriber, publisher):
        """
        Constructor: TODO
        
        - spawn a publisher thread
        - handle thread lifecycles (especally destruction)
        """
	self.topic = topic
        self.subscriber = subscriber
	self.publisher = publisher
	self.pub = rospy.Publisher("/statistics", TopicStatistics)
	self.last_pub_time = rospy.Time(0)
	self.pub_frequency = rospy.Duration(1.0)
	
	self.stat_bytes_last_ = 0
	self.stat_bytes_window_ = 0

        # timestamp delay
	self.delay_list_ = []

	# period calculations
	self.arrival_time_list_ = []

        self.last_seq_ = 0
        self.dropped_msgs_ = 0
	self.window_start = rospy.Time.now()

        pass

    def sendStatistics(self):
	"""
	stuff to publish
	- call hooks to do deep packet inspection? (later)
	"""
	curtime = rospy.Time.now()

	window_start = self.window_start
	self.window_start = curtime

	msg = TopicStatistics()
	msg.topic = self.topic
	msg.node_sub = self.subscriber
	msg.node_pub = self.publisher

	msg.traffic = self.stat_bytes_window_ / self.pub_frequency.to_sec()

	msg.window_start = window_start
	msg.window_stop  = curtime

        msg.dropped_msgs = self.dropped_msgs_

	if len(self.delay_list_)>0:
            msg.stamp_delay_mean = sum(self.delay_list_) / len(self.delay_list_)
	    msg.stamp_delay_variance = sum((msg.stamp_delay_mean - value) ** 2 for value in self.delay_list_) / len(self.delay_list_)
    	    msg.stamp_delay_max = max(self.delay_list_)
	else:
            msg.stamp_delay_mean = float('NaN')
	    msg.stamp_delay_variance = float('NaN')
	    msg.stamp_delay_max = float('NaN')

	if len(self.arrival_time_list_)>1:
	    periods = [j-i for i, j in zip(self.arrival_time_list_[:-1], self.arrival_time_list_[1:])]
            msg.period_mean = sum(periods)/len(periods)
	    msg.period_variance = sum((msg.period_mean - value) ** 2 for value in periods) / len(periods)
            msg.period_max = max(periods)
	else:
            msg.period_mean = float('NaN')
	    msg.period_variance = float('NaN')
            msg.period_max = float('NaN')

	self.delay_list_ = []
	self.arrival_time_list_ = []
        self.dropped_msgs_ = 0

        self.pub.publish(msg)

    def callback(self,msg, stat_bytes):
        """
        Callback: TODO
        
        needs to know:
        - packet content (data) DONE
        - all secret meta data there might be (time of arrival?)
        - topic message type (at least, if there is a header)
        - subscriber topic name
        - nodes of subscriber/publisher
        - traffic volume?
        - publisher topic name (on publisher side, ignoring remapping stuff)
          - these two are likely needed to create the graph.
          - buffer size (to measure message drops or similar)
        
        any computing-heavy stuff should likely be done somewhere else,
        as this callback will probably block the other callbacks?
        
        this callback will keep some statistics and publish the results
        periodically on a topic. the publishing should probably be done
        asynchronically in another thread.
        """

	self.arrival_time_list_.append(rospy.Time.now().to_sec())

	self.stat_bytes_window_ = stat_bytes - self.stat_bytes_last_
	self.stat_bytes_last_ = stat_bytes

	if msg._has_header:
	    self.delay_list_.append((rospy.Time.now() - msg.header.stamp).to_sec())

    	    if self.last_seq_ + 1 != msg.header.seq:
                self.dropped_msgs_ = self.dropped_msgs_ + 1
    	    self.last_seq_ = msg.header.seq

        if self.last_pub_time + self.pub_frequency < rospy.Time.now():
            self.last_pub_time = rospy.Time.now()
            self.sendStatistics()


