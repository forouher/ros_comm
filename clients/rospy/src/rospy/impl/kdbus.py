# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
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
"""
KDBUS connection protocol.

Implements: U{http://ros.org/wiki/ROS/KDBUS}

The rospy kdbus implementation is split into three areas:
 - L{rospy.kdbus_base}: common KDBUS routines, including header and connection processing
 - L{rospy.kdbus_pubsub}: Topic-specific capabilities for publishing and subscribing
 - L{rospy.kdbus_service}: Service-specific capabilities 
"""

import rospy.impl.kdbus_service

from rospy.impl.kdbus_base import init_kdbus_server, DEFAULT_BUFF_SIZE
from rospy.impl.kdbus_pubsub import KDBUSHandler

_handler = KDBUSHandler()

def init_kdbus():
    server = init_kdbus_server()
    server.topic_connection_handler = _handler.topic_connection_handler
    server.service_connection_handler = rospy.impl.kdbus_service.service_connection_handler

def get_kdbus_handler():
    return _handler
