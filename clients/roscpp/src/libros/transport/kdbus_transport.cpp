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

extern "C"
{
#include "ros/transport/kdbus-util.h"
#include "ros/transport/kdbus-enum.h"
}

#include <boost/interprocess/managed_external_buffer.hpp>

#define KBUILD_MODNAME "kdbus"

#include "ros/transport/kdbus_transport.h"

namespace ros {

KDBusTransport::KDBusTransport(const std::string& name) {

    bus = name;
    buspath = "/dev/kdbus/"+name+"/bus";
    fdc = 0;

}

int KDBusTransport::create_bus() {

	struct {
		struct kdbus_cmd_make head;

		struct {
			uint64_t size;
			uint64_t type;
			uint64_t bloom_size;
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
	bus_make.bs.type = KDBUS_ITEM_BLOOM_SIZE;
	bus_make.bs.bloom_size = 64;

	strncpy(bus_make.name, bus.c_str(),bus.length());
	bus_make.n_type = KDBUS_ITEM_MAKE_NAME;
	bus_make.n_size = KDBUS_ITEM_HEADER_SIZE + strlen(bus_make.name) + 1;

	bus_make.head.size = sizeof(struct kdbus_cmd_make) +
			     sizeof(bus_make.bs) +
			     bus_make.n_size;

	ret = ioctl(fdc, KDBUS_CMD_BUS_MAKE, &bus_make);
	ROS_DEBUG("creating new kdbus '%s'...", bus_make.name);
	if (ret) {
		ROS_ERROR("Could not create kdbus bus '%s': error %d (%m)", bus_make.name,  ret);
		return EXIT_FAILURE;
	}
	ROS_DEBUG("Created kdbus bus '%s'", bus_make.name);

	return 0;

}

int KDBusTransport::destroy_bus() {
	close(fdc);
}

int KDBusTransport::open_connection(const std::string& name) {
	int ret, r;
        ROS_DEBUG("KDBusTransport::open_connection(%s)", name.c_str());

	if (!fdc) {
	    printf("-- opening /dev/kdbus/control\n");
	    fdc = open("/dev/kdbus/control", O_RDWR|O_CLOEXEC);
	    if (fdc < 0) {
		ROS_ERROR("--- error %d (%m)", fdc);
		return -1;
	    }
	}

	conn = connect_to_bus(buspath.c_str(), 0);
	if (!conn) {
		ROS_ERROR("connect_to_bus failed");
		return -1;
	}
	r = upload_policy(conn->fd, name.c_str());
	if (r < 0) {
		ROS_ERROR("upload_policy failed");
		return -1;
	}
	if (name.length()>0) {
	  ROS_DEBUG("Aquiring kdbus name '%s'", name.c_str());
	  r = name_acquire(conn, name.c_str(), 0);
	  if (r < 0) {
		ROS_ERROR("name_acquire %s failed: %d (%m)", name.c_str(), r);
		return -1;
	  }
	}

	add_match_empty(conn->fd);

        ROS_DEBUG("Successfully opened connection with fd=%i", conn->fd);
	return conn->fd;

}

int KDBusTransport::close_connection() {
        ROS_DEBUG("KDBusTransport::close_connection()");
	close(conn->fd);
	free(conn);

}

bool KDBusTransport::sendMessage(MemfdMessage::Ptr msg, const std::string& receiver) {

	const char* name = receiver.c_str();

        munmap(msg->buf_,msg->size_);
	msg->buf_ = NULL;

        int ret = ioctl(msg->fd_, KDBUS_CMD_MEMFD_SEAL_SET, true);
        if (ret < 0) {
                fprintf(stderr, "memfd sealing failed: %m\n");
                return false;
        }

	uint64_t size = sizeof(struct kdbus_msg);
	size += KDBUS_ITEM_SIZE(sizeof(struct kdbus_memfd));

	if (name)
		size += KDBUS_ITEM_SIZE(strlen(name) + 1);

	struct kdbus_msg *kmsg = (kdbus_msg*)malloc(size);
	if (!kmsg) {
		fprintf(stderr, "unable to malloc()!?\n");
		return false;
	}

	// create new kdbus message
	memset(kmsg, 0, size);
	kmsg->size = size;
	kmsg->src_id = conn->id;
	kmsg->dst_id = 0; // name defines destination
	kmsg->cookie = 0;
	kmsg->payload_type = KDBUS_PAYLOAD_DBUS; // TODO??

	// start with first item
	struct kdbus_item *item = kmsg->items;

	// start with name
	if (name) {
		item->type = KDBUS_ITEM_DST_NAME;
		item->size = KDBUS_ITEM_HEADER_SIZE + strlen(name) + 1;
		strcpy(item->str, name);
		item = KDBUS_ITEM_NEXT(item);
	}

	// next is our memfd
	item->type = KDBUS_ITEM_PAYLOAD_MEMFD;
	item->size = KDBUS_ITEM_HEADER_SIZE + sizeof(struct kdbus_memfd);
	item->memfd.size = MemfdMessage::MAX_SIZE;
	item->memfd.fd = msg->fd_;
	item = KDBUS_ITEM_NEXT(item);

	ret = ioctl(conn->fd, KDBUS_CMD_MSG_SEND, kmsg);
	if (ret < 0) {
		fprintf(stderr, "error sending message: %d err %d (%m)\n", ret, errno);
//		msg->buf_ = mmap(NULL, msg->size_, PROT_READ, 0, msg->fd_, 0);
		return false;
	}

	free(kmsg);

	return true;

}

MemfdMessage::Ptr KDBusTransport::receiveMessage() {

	MemfdMessage::Ptr m;

	uint64_t off;
	struct kdbus_msg *msg;
	int ret;

	ret = ioctl(conn->fd, KDBUS_CMD_MSG_RECV, &off);
	if (ret < 0) {
		fprintf(stderr, "error receiving message: %d (%m)\n", ret);
//		return EXIT_FAILURE;
	}

	msg = (struct kdbus_msg *)((char*)(conn->buf) + off);
	{
	const struct kdbus_item *item = msg->items;
	char buf_src[32];
	char buf_dst[32];
	uint64_t timeout = 0;
	uint64_t cookie_reply = 0;

	KDBUS_ITEM_FOREACH(item, msg, items) {
		if (item->size <= KDBUS_ITEM_HEADER_SIZE) {
			printf("  +%s (%llu bytes) invalid data record\n", enum_MSG(item->type), item->size);
			break;
		}

		if (item->type == KDBUS_ITEM_PAYLOAD_MEMFD) {

			char *buf;
			uint64_t size;

		        int ret = ioctl(item->memfd.fd, KDBUS_CMD_MEMFD_SEAL_SET, false);
		        if (ret < 0) {
		                fprintf(stderr, "memfd unsealing failed: %m\n");
//		                break;
		        }

			buf = (char*)mmap(NULL, MemfdMessage::MAX_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, item->memfd.fd, 0);
			ROS_ASSERT_MSG(buf != MAP_FAILED, "error: %i (%m)", errno);

			ret = ioctl(item->memfd.fd, KDBUS_CMD_MEMFD_SIZE_GET, &size);
			ROS_ASSERT(ret>=0); // maybe we wanna handle this

			m = MemfdMessage::Ptr(new MemfdMessage(item->memfd.fd,buf,MemfdMessage::MAX_SIZE));

			break;
		}
	}

	if ((char *)item - ((char *)msg + msg->size) >= 8)
		printf("invalid padding at end of message\n");

	}


	ret = ioctl(conn->fd, KDBUS_CMD_FREE, &off);
	if (ret < 0) {
		fprintf(stderr, "error free message: %d (%m)\n", ret);
//		return EXIT_FAILURE;
	}

	return m;

}

} // namespace ros

