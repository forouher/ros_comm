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

"""Internal use: Topic-specific extensions for KDBUS support"""

import socket
import threading
import time

try:
    from xmlrpc.client import ServerProxy  # Python 3.x
except ImportError:
    from xmlrpclib import ServerProxy  # Python 2.x

from rospy.core import logwarn, logerr, logdebug, rospyerr
import rospy.exceptions
import rospy.names

import rospy.impl.registration
import rospy.impl.transport

from rospy.impl.kdbus_base import KDBUSTransport, KDBUSTransportProtocol, \
    get_kdbus_server_address, start_kdbus_server,\
    DEFAULT_BUFF_SIZE, KDBUS

class KDBUSSub(KDBUSTransportProtocol):
    """
    Subscription transport implementation for receiving topic data via
    peer-to-peer TCP/IP sockets
    """

    def __init__(self, resolved_name, recv_data_class, queue_size=None, \
                     buff_size=DEFAULT_BUFF_SIZE, tcp_nodelay=False):
        """
        ctor.

        @param resolved_name: resolved subscription name
        @type  resolved_name: str

        @param recv_data_class: class to instantiate to receive
        messages
        @type recv_data_class: L{rospy.Message}

        @param queue_size: maximum number of messages to
        deserialize from newly read data off socket
        @type queue_size: int

        @param buff_size: recv buffer size
        @type buff_size: int

        @param tcp_nodelay: If True, request TCP_NODELAY from publisher
        @type tcp_nodelay: bool
        """
        super(KDBUSSub, self).__init__(resolved_name, recv_data_class, queue_size, buff_size)
        self.direction = rospy.impl.transport.INBOUND
        self.tcp_nodelay = tcp_nodelay
        
    def get_header_fields(self):
        """
        @return: dictionary of subscriber fields
        @rtype: dict
        """
        return {'topic': self.resolved_name,
                'message_definition': self.recv_data_class._full_text,
                'tcp_nodelay': '1' if self.tcp_nodelay else '0',
                'md5sum': self.recv_data_class._md5sum,
                'type': self.recv_data_class._type,
                'callerid': rospy.names.get_caller_id()}        

# Separate method for easier testing
def _configure_pub_socket(sock, is_tcp_nodelay):
    """
    Configure socket options on a new publisher socket.
    @param sock: socket.socket
    @type sock: socket.socket
    @param is_tcp_nodelay: if True, TCP_NODELAY will be set on outgoing socket if available
    @param is_tcp_nodelay: bool
    """
    # #956: low latency, TCP_NODELAY support
    if is_tcp_nodelay:
        if hasattr(socket, 'TCP_NODELAY'):
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        else:
            logwarn("WARNING: cannot enable TCP_NODELAY as its not supported on this platform")

#TODO:POLLING: KDBUSPub currently doesn't actually do anything -- not until polling is implemented

class KDBUSPub(KDBUSTransportProtocol):
    """
    Publisher transport implementation for publishing topic data via
    peer-to-peer TCP/IP sockets. 
    """
    
    def __init__(self, resolved_name, pub_data_class, is_latch=False, headers=None):
        """
        ctor.
        @param resolved_name: resolved topic name
        @type  resolved_name: str
        @param pub_data_class: class to instance to receive messages
        @type  pub_data_class: L{rospy.Message} class
        @param is_latch: If True, Publisher is latching
        @type  is_latch: bool
        """
        # very small buffer size for publishers as the messages they receive are very small
        super(KDBUSPub, self).__init__(resolved_name, None, queue_size=None, buff_size=128)
        self.pub_data_class = pub_data_class
        self.direction = rospy.impl.transport.OUTBOUND
        self.is_latch = is_latch
        self.headers = headers if headers else {}
        
    def get_header_fields(self):
        base = {'topic': self.resolved_name,
                'type': self.pub_data_class._type,
                'latching': '1' if self.is_latch else '0',
                'message_definition': self.pub_data_class._full_text,
                'md5sum': self.pub_data_class._md5sum, 
                'callerid': rospy.names.get_caller_id() }
        
        # this implementation allows the user to override builtin
        # fields.  this could potentially enable some interesting
        # features... or it could be really bad.
        if self.headers:
            base.update(self.headers)
        return base

def robust_connect_subscriber(conn, dest_addr, dest_port, pub_uri, receive_cb, resolved_topic_name):
    """
    Keeps trying to create connection for subscriber.  Then passes off to receive_loop once connected.
    """
    # kwc: this logic is not very elegant.  I am waiting to rewrite
    # the I/O loop with async i/o to clean this up.
    
    # timeout is really generous. for now just choosing one that is large but not infinite
    interval = 0.5
    while conn.socket is None and not conn.done and not rospy.is_shutdown():
        try:
            conn.connect(dest_addr, dest_port, pub_uri, timeout=60.)
        except rospy.exceptions.TransportInitError as e:
            rospyerr("unable to create subscriber transport: %s.  Will try again in %ss", e, interval)
            interval = interval * 2
            time.sleep(interval)
            
            # check to see if publisher state has changed
            conn.done = not check_if_still_publisher(resolved_topic_name, pub_uri)
	
    if not conn.done:
        conn.receive_loop(receive_cb)	    

def check_if_still_publisher(resolved_topic_name, pub_uri):
    try:
        s = ServerProxy(pub_uri)
        code, msg, val = s.getPublications(rospy.names.get_name())
        if code == 1:
            return len([t for t in val if t[0] == resolved_topic_name]) > 0
        else:
            return False
    except:
        return False

class KDBUSHandler(rospy.impl.transport.ProtocolHandler):
    """
    ROS Protocol handler for KDBUS. Accepts both KDBUS topic
    connections as well as ROS service connections over TCP. TCP server
    socket is run once start_server() is called -- this is implicitly
    called during init_publisher().
    """

    def __init__(self):
        """ctor"""
        self.tcp_nodelay_map = {} # { topic : tcp_nodelay}
    
    def set_tcp_nodelay(self, resolved_name, tcp_nodelay):
        """
        @param resolved_name: resolved topic name
        @type  resolved_name: str

        @param tcp_nodelay: If True, sets TCP_NODELAY on publisher's
        socket (disables Nagle algorithm). This results in lower
        latency publishing at the cost of efficiency.
        @type  tcp_nodelay: bool
        """
        self.tcp_nodelay_map[resolved_name] = tcp_nodelay

    def shutdown(self):
        """
        stops the TCP/IP server responsible for receiving inbound connections        
        """
        pass

    def create_transport(self, resolved_name, pub_uri, protocol_params):
        """
        Connect to topic resolved_name on Publisher pub_uri using KDBUS.
        @param resolved_name str: resolved topic name
        @type  resolved_name: str
        @param pub_uri: XML-RPC URI of publisher 
        @type  pub_uri: str
        @param protocol_params: protocol parameters to use for connecting
        @type protocol_params: [XmlRpcLegal]
        @return: code, message, debug
        @rtype: (int, str, int)
        """
        
        #Validate protocol params = [KDBUS, address, port]
        if type(protocol_params) != list or len(protocol_params) != 3:
            return 0, "ERROR: invalid KDBUS parameters", 0
        if protocol_params[0] != KDBUS:
            return 0, "INTERNAL ERROR: protocol id is not KDBUS: %s"%id, 0
        id, dest_addr, dest_port = protocol_params

        sub = rospy.impl.registration.get_topic_manager().get_subscriber_impl(resolved_name)

        #Create connection 
        protocol = KDBUSSub(resolved_name, sub.data_class, \
                             queue_size=sub.queue_size, buff_size=sub.buff_size,
                             tcp_nodelay=sub.tcp_nodelay)
        conn = KDBUSTransport(protocol, resolved_name)
        t = threading.Thread(name=resolved_name, target=robust_connect_subscriber, args=(conn, dest_addr, dest_port, pub_uri, sub.receive_callback,resolved_name))
        # don't enable this just yet, need to work on this logic
        #rospy.core._add_shutdown_thread(t)
        t.start()

        # Attach connection to _SubscriberImpl
        if sub.add_connection(conn): #pass tcp connection to handler
            return 1, "Connected topic[%s]. Transport impl[%s]"%(resolved_name, conn.__class__.__name__), dest_port
        else:
            conn.close()
            return 0, "ERROR: Race condition failure: duplicate topic subscriber [%s] was created"%(resolved_name), 0

    def supports(self, protocol):
        """
        @param protocol: name of protocol
        @type protocol: str
        @return: True if protocol is supported
        @rtype: bool
        """
        return protocol == KDBUS
    
    def get_supported(self):
        """
        Get supported protocols
        """
        return [[KDBUS]]
        
    def init_publisher(self, resolved_name, protocol):
        """
        Initialize this node to receive an inbound TCP connection,
        i.e. startup a TCP server if one is not already running.
        
        @param resolved_name: topic name
        @type  resolved__name: str
        
        @param protocol: negotiated protocol
          parameters. protocol[0] must be the string 'KDBUS'
        @type  protocol: [str, value*]
        @return: (code, msg, [KDBUS, addr, port])
        @rtype: (int, str, list)
        """
        if protocol[0] != KDBUS:
            return 0, "Internal error: protocol does not match KDBUS: %s"%protocol, []
        start_kdbus_server()
        addr, port = get_kdbus_server_address()
        return 1, "ready on %s:%s"%(addr, port), [KDBUS, addr, port]

    def topic_connection_handler(self, sock, client_addr, header):
        """
        Process incoming topic connection. Reads in topic name from
        handshake and creates the appropriate L{KDBUSPub} handler for the
        connection.
        @param sock: socket connection
        @type sock: socket.socket
        @param client_addr: client address
        @type client_addr: (str, int)
        @param header: key/value pairs from handshake header
        @type header: dict
        @return: error string or None 
        @rtype: str
        """
        if rospy.core.is_shutdown_requested():
            return "Node is shutting down"
        for required in ['topic', 'md5sum', 'callerid']:
            if not required in header:
                return "Missing required '%s' field"%required
        else:
            resolved_topic_name = header['topic']
            md5sum = header['md5sum']
            tm = rospy.impl.registration.get_topic_manager()
            topic = tm.get_publisher_impl(resolved_topic_name)
            if not topic:
                return "[%s] is not a publisher of [%s]. Topics are %s"%(rospy.names.get_caller_id(), resolved_topic_name, tm.get_publications())
            elif not topic.data_class or topic.closed:
                return "Internal error processing topic [%s]"%(resolved_topic_name)
            elif md5sum != rospy.names.TOPIC_ANYTYPE and md5sum != topic.data_class._md5sum:
                data_class = topic.data_class
                actual_type = data_class._type

                # check to see if subscriber sent 'type' header. If they did, check that
                # types are same first as this provides a better debugging message
                if 'type' in header:
                    requested_type = header['type']
                    if requested_type != actual_type:
                        return "topic types do not match: [%s] vs. [%s]"%(requested_type, actual_type)
                else:
                    # defaults to actual type
                    requested_type = actual_type

                return "Client [%s] wants topic [%s] to have datatype/md5sum [%s/%s], but our version has [%s/%s] Dropping connection."%(header['callerid'], resolved_topic_name, requested_type, md5sum, actual_type, data_class._md5sum)

            else:
                #TODO:POLLING if polling header is present, have to spin up receive loop as well

                # #1334: tcp_nodelay support from subscriber option
                if 'tcp_nodelay' in header:
                    tcp_nodelay = True if header['tcp_nodelay'].strip() == '1' else False
                else:
                    tcp_nodelay = self.tcp_nodelay_map.get(resolved_topic_name, False)

                _configure_pub_socket(sock, tcp_nodelay)
                protocol = KDBUSPub(resolved_topic_name, topic.data_class, is_latch=topic.is_latch, headers=topic.headers)
                transport = KDBUSTransport(protocol, resolved_topic_name)
                transport.set_socket(sock, header['callerid'])
                transport.write_header()
                topic.add_connection(transport)
            

