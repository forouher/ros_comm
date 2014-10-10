/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Dariush Forouher
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <cstdlib>

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "ros/transport/kdbus-util.h"

#include <boost/interprocess/managed_external_buffer.hpp>

#define KBUILD_MODNAME "kdbus"

#include <ros/transport/transport_kdbus.h>

namespace ros {


#define POOL_SIZE (16 * 1024LU * 1024LU)

KDBusTransport::KDBusTransport(const std::string& name) {

    bus = name;
    buspath = "/dev/kdbus/"+name+"/bus";
    fdc = 0;

}

int KDBusTransport::create_bus() {

        // TODO: hacky. why defined not in some header?
	struct {
		struct kdbus_cmd_make head;

		struct {
			uint64_t size;
			uint64_t type;
			struct kdbus_bloom_parameter bloom;
		} bs;

		uint64_t n_size;
		uint64_t n_type;
		char name[64];
	} bus_make;

	int ret;

	ROS_DEBUG("opening /dev/kdbus/control");
	fdc = open("/dev/kdbus/control", O_RDWR|O_CLOEXEC);
	if (fdc < 0) {
		ROS_ERROR("Could not open kdbus control node: error %d (%m)", fdc);
		return EXIT_FAILURE;
	}

	memset(&bus_make, 0, sizeof(bus_make));
	bus_make.bs.size = sizeof(bus_make.bs);
	bus_make.bs.type = KDBUS_ITEM_BLOOM_PARAMETER;
        bus_make.bs.bloom.size = 64;
	bus_make.bs.bloom.n_hash = 1;

	strncpy(bus_make.name, bus.c_str(),bus.length());
	bus_make.n_type = KDBUS_ITEM_MAKE_NAME;
	bus_make.n_size = KDBUS_ITEM_HEADER_SIZE + strlen(bus_make.name) + 1;

	bus_make.head.size = sizeof(struct kdbus_cmd_make) +
			     sizeof(bus_make.bs) +
			     bus_make.n_size;

	ret = ioctl(fdc, KDBUS_CMD_BUS_MAKE, &bus_make);
	ROS_DEBUG("creating new kdbus '%s'...", bus_make.name);
	if (ret < -1) {
		ROS_ERROR("Could not create kdbus bus '%s': error %d (%m)", bus_make.name,  ret);
		return EXIT_FAILURE;
	}
	ROS_DEBUG("Created kdbus bus '%s'", bus_make.name);

	return 0;

}

int KDBusTransport::destroy_bus() {
	close(fdc);
}

int KDBusTransport::open_connection() {

        ROS_DEBUG("KDBusTransport::open_connection()");

	int fd = open(buspath.c_str(), O_RDWR|O_CLOEXEC);
        ROS_ASSERT_MSG(fd >= 0, "kdbus: could not open control file %s", buspath.c_str());

	struct kdbus_cmd_hello hello;
	memset(&hello, 0, sizeof(hello));

	hello.conn_flags = 0;
	hello.attach_flags = 0;
	hello.size = sizeof(struct kdbus_cmd_hello);
	hello.pool_size = POOL_SIZE;

	int ret = ioctl(fd, KDBUS_CMD_HELLO, &hello);
        ROS_ASSERT_MSG(!ret, "kdbus: error when saying hello: %d (%m)", ret);

	conn_.buf = mmap(NULL, POOL_SIZE, PROT_READ, MAP_SHARED, fd, 0);
        ROS_ASSERT_MSG(conn_.buf != MAP_FAILED, "kdbus: unable to mmap");

	conn_.fd = fd;
	conn_.id = hello.id;

	return conn_.fd;
}

bool KDBusTransport::aquire_name(const std::string& name) {

	ROS_DEBUG("Aquiring kdbus name '%s'", name.c_str());


	struct kdbus_cmd_name *cmd_name;
	int ret;
	uint64_t size = sizeof(*cmd_name) + strlen(name.c_str()) + 1;

	cmd_name = (struct kdbus_cmd_name*)alloca(size); // free?

	memset(cmd_name, 0, size);
	strcpy(cmd_name->name, name.c_str());
	cmd_name->size = size;
	cmd_name->flags = 0;

	ret = ioctl(conn_.fd, KDBUS_CMD_NAME_ACQUIRE, cmd_name);
	if (ret < 0) {
		ret = -errno;
		fprintf(stderr, "error aquiring name: %s\n", strerror(-ret));
		return false;
	}

	if (ret < 0) {
		ROS_ERROR("name_acquire %s failed: %d (%m)", name.c_str(), ret);
		return false;
	}

        my_name = name;

        return true;

}

int KDBusTransport::close_connection() {
        ROS_DEBUG("KDBusTransport::close_connection()");
	close(conn_.fd);
}

bool KDBusTransport::sendMessage(const SerializedMessage& msg, const std::string& receiver) {

        ROS_ASSERT(!receiver.empty());

        int ret = 0;

	uint64_t size = sizeof(struct kdbus_msg);
	size += KDBUS_ITEM_SIZE(sizeof(struct kdbus_memfd));
	size += KDBUS_ITEM_SIZE(receiver.length() + 1);

	struct kdbus_msg *kmsg = (kdbus_msg*)malloc(size);
        ROS_ASSERT_MSG(kmsg, "kdbus: unable to malloc");

	// create new kdbus message
	memset(kmsg, 0, size);
	kmsg->size = size;
	kmsg->src_id = conn_.id;
	kmsg->dst_id = 0; // name defines destination
	kmsg->cookie = 0;
	kmsg->payload_type = KDBUS_PAYLOAD_DBUS; // TODO??

	// start with first item
	struct kdbus_item *item = kmsg->items;

	item->type = KDBUS_ITEM_DST_NAME;
	item->size = KDBUS_ITEM_HEADER_SIZE + receiver.length() + 1;
	strcpy(item->str, receiver.c_str());
	item = KDBUS_ITEM_NEXT(item);

	// next is our memfd
	item->type = KDBUS_ITEM_PAYLOAD_MEMFD;
	item->size = KDBUS_ITEM_HEADER_SIZE + sizeof(struct kdbus_memfd);
	item->memfd.size = msg.memfd_message->size_;
	item->memfd.fd = msg.memfd_message->fd_;
	item = KDBUS_ITEM_NEXT(item);

	ret = ioctl(conn_.fd, KDBUS_CMD_MSG_SEND, kmsg);
	if (ret < 0) {
                fprintf(stderr, "Could not send message %d (%m)\n", ret);
	}

	free(kmsg);
	return true;
}

// TODO: maybe more than 1 msg waiting?
MemfdMessage::Ptr KDBusTransport::receiveMessage() {

        struct kdbus_cmd_recv recv = {};

	int ret = ioctl(conn_.fd, KDBUS_CMD_MSG_RECV, &recv);
	ROS_ASSERT_MSG(!ret, "kdbus: error receiving message: %d (%m)", ret);

        struct kdbus_msg *msg = (struct kdbus_msg *)(conn_.buf + recv.offset);

	MemfdMessage::Ptr m;
	const struct kdbus_item *item = msg->items;

        int fd = -1;
        int memfd_size = 0;
        std::string caller_id = "";

	KDBUS_ITEM_FOREACH(item, msg, items) {
		if (item->type == KDBUS_ITEM_DST_NAME) {
                        caller_id = "ros.uuid"+std::string(item->name.name);
		} else if (item->type == KDBUS_ITEM_PAYLOAD_MEMFD) {
                        fd = item->memfd.fd;
                        memfd_size = item->memfd.size;
		} else {
                        printf("unknown item->type: %llu\n", item->type);
                }
	}

        ROS_ASSERT_MSG(fd >= 0, "fd is %i", fd);
        ROS_ASSERT_MSG(caller_id==my_name, "caller_id=%s, my_name=%s", caller_id.c_str(), my_name.c_str());

	void* buf = mmap(NULL, memfd_size, PROT_READ, MAP_PRIVATE, fd, 0);
	ROS_ASSERT_MSG(buf != MAP_FAILED, "error: %i (%m)", errno);

	m = MemfdMessage::Ptr(new MemfdMessage(fd,buf,memfd_size));

	ROS_ASSERT_MSG((char *)item - ((char *)msg + msg->size) < 8, "kdbus: invalid padding at end of message");

	ret = ioctl(conn_.fd, KDBUS_CMD_FREE, &recv.offset);
	ROS_ASSERT_MSG(!ret, "kdbus: error freeing message: %d (%m)", ret);

	return m;

}

} // namespace ros

