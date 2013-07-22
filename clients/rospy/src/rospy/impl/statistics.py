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
    Class that monitors lots of stuff on each subscriber.
    
    is created whenever a subscriber is created.
    is destroyed whenever its parent subscriber is destroyed.
    its lifecycle is therefore bound to its parent subscriber.
    
    XXX: the are threading/concurrent access issues meantioned at different
    places. what do i have to keep in mind?
    - may there be callbacks called in parallel? sounds unlikely
    - may additional methods (which i don't have any a.t.m., be called in
      parallel? probably.

    """

    def __init__(self, subscriber):
        """
        Constructor: TODO
        
        - spawn a publisher thread
        - add itself/the callback as a callback to the subscriber
        - handle thread lifecycles (especally destruction)
        """
        self.subscriber = subscriber
	self.pub = rospy.Publisher("/statistics", TopicStatistics)
	self.last_pub_time = rospy.Time.now()
	self.pub_frequency = rospy.Duration(1.0)
	
	self.window_size_ = 7

        # frequency
	self.count_ = 0
	self.seq_nums_ = [self.count_ for x in range(self.window_size_)]
	self.times_ = [self.last_pub_time for x in range(self.window_size_)]
	self.hist_indx_ = 0

        # timestamp delay
        self.delay_ = 0
        self.delay_count_ = 0

        self.last_seq_ = 0
        self.dropped_msgs_ = 0

        pass

    def sendStatistics(self):
	"""
	stuff to publish
	- message frequency
	- message variance
	- message min/max frequency variations (within window)
	- message drops/buffer overflow (counter)
	- delta time (diff header.stamp - now())
	- call hooks to do deep packet inspection? (later)
	
	create a new message type for this. while this is similar to
	diagnostics, it is
	a) too much string-based
	b) REP 107 restricts it to drivers (should double check that)
	c) might be hard to extend
	"""
	
	curtime = rospy.Time.now()
        curseq = self.count_
        events = curseq - self.seq_nums_[self.hist_indx_]
        window = (curtime - self.times_[self.hist_indx_]).to_sec()
        freq = events / window
        self.seq_nums_[self.hist_indx_] = curseq
        self.times_[self.hist_indx_] = curtime
        self.hist_indx_ = (self.hist_indx_ + 1) % self.window_size_

	
	msg = TopicStatistics()
	msg.header.stamp = rospy.Time.now()
	msg.topic = self.subscriber.name
	msg.node_sub = rospy.get_name()
        msg.stamp_delay_mean = self.delay_ / self.delay_count_
        msg.frequency_mean = freq
        msg.dropped_msgs = self.dropped_msgs_
        self.pub.publish(msg)

        self.delay_count_ = 0
        self.delay_ = 0
        self.dropped_msgs_ = 0

    def get_callback(self):
	return self.callback

    def get_callback_args(self):
	return "" # TODO ???

    def callback(self,msg,unknown):
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
        
        XXX: idea: can I call this callback *last*? this would allow me to
        also measure the callback runtime of the subscriber (unless it does
        its computation async)
        
        this callback will keep some statistics and publish the results
        periodically on a topic. the publishing should probably be done
        asynchronically in another thread.
        """

        # log for frequency
        self.count_ = self.count_ + 1

        # log for stamp_delay
        # TODO: maybe there's a better way than the exception
        try:
            self.delay_ = self.delay_ + (rospy.Time.now() - msg.header.stamp).to_sec()

            if self.last_seq_ + 1 != msg.header.seq:
                self.dropped_msgs_ = self.dropped_msgs_ + 1
            self.last_seq_ = msg.header.seq

        except:
            pass

        self.delay_count_ = self.delay_count_ + 1


        if self.last_pub_time + self.pub_frequency < rospy.Time.now():
            self.sendStatistics()
            self.last_pub_time = rospy.Time.now()
        pass

