/*
 * Copyright (C) 2013 Daniel Mack
 * Copyright (C) 2013 Kay Sievers
 *
 * kdbus is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "ros/transport/kdbus-util.h"
#include "ros/transport/kdbus-enum.h"

#define POOL_SIZE (16 * 1024LU * 1024LU)
struct conn *connect_to_bus(const char *path, uint64_t hello_flags)
{
	int fd, ret;
	struct kdbus_cmd_hello hello;
	struct conn *conn;

	memset(&hello, 0, sizeof(hello));

	printf("-- opening bus connection %s\n", path);
	fd = open(path, O_RDWR|O_CLOEXEC);
	if (fd < 0) {
		fprintf(stderr, "--- error %d (%m)\n", fd);
		return NULL;
	}

	hello.conn_flags = hello_flags | KDBUS_HELLO_ACCEPT_FD;

	hello.attach_flags = KDBUS_ATTACH_TIMESTAMP |
			     KDBUS_ATTACH_CREDS |
			     KDBUS_ATTACH_NAMES |
			     KDBUS_ATTACH_COMM |
			     KDBUS_ATTACH_EXE |
			     KDBUS_ATTACH_CMDLINE |
			     KDBUS_ATTACH_CAPS |
			     KDBUS_ATTACH_CGROUP |
			     KDBUS_ATTACH_SECLABEL |
			     KDBUS_ATTACH_AUDIT;

	hello.size = sizeof(struct kdbus_cmd_hello);
	hello.pool_size = POOL_SIZE;

	ret = ioctl(fd, KDBUS_CMD_HELLO, &hello);
	if (ret < 0) {
		fprintf(stderr, "--- error when saying hello: %d (%m)\n", ret);
		return NULL;
	}
	printf("-- Our peer ID for %s: %llu -- bus uuid: '%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x'\n",
		path, (unsigned long long)hello.id,
		hello.id128[0],  hello.id128[1],  hello.id128[2],  hello.id128[3],
		hello.id128[4],  hello.id128[5],  hello.id128[6],  hello.id128[7],
		hello.id128[8],  hello.id128[9],  hello.id128[10], hello.id128[11],
		hello.id128[12], hello.id128[13], hello.id128[14], hello.id128[15]);

	conn = malloc(sizeof(*conn));
	if (!conn) {
		fprintf(stderr, "unable to malloc()!?\n");
		return NULL;
	}

	conn->buf = mmap(NULL, POOL_SIZE, PROT_READ, MAP_SHARED, fd, 0);
	if (conn->buf == MAP_FAILED) {
		free(conn);
		fprintf(stderr, "--- error mmap (%m)\n");
		return NULL;
	}

	conn->fd = fd;
	conn->id = hello.id;
	return conn;
}


char *msg_id(uint64_t id, char *buf)
{
	if (id == 0)
		return "KERNEL";
	if (id == ~0ULL)
		return "BROADCAST";
	sprintf(buf, "%llu", (unsigned long long)id);
	return buf;
}


int name_acquire(struct conn *conn, const char *name, uint64_t flags)
{
	struct kdbus_cmd_name *cmd_name;
	int ret;
	uint64_t size = sizeof(*cmd_name) + strlen(name) + 1;

	cmd_name = alloca(size);

	memset(cmd_name, 0, size);
	strcpy(cmd_name->name, name);
	cmd_name->size = size;
	cmd_name->flags = flags;

	ret = ioctl(conn->fd, KDBUS_CMD_NAME_ACQUIRE, cmd_name);
	if (ret < 0) {
		ret = -errno;
		fprintf(stderr, "error aquiring name: %s\n", strerror(-ret));
		return ret;
	}

	printf("%s(): flags after call: 0x%llx\n", __func__, cmd_name->conn_flags);

	return 0;
}

int name_release(struct conn *conn, const char *name)
{
	struct kdbus_cmd_name *cmd_name;
	int ret;
	uint64_t size = sizeof(*cmd_name) + strlen(name) + 1;

	cmd_name = alloca(size);

	memset(cmd_name, 0, size);
	strcpy(cmd_name->name, name);
	cmd_name->size = size;

	printf("conn %lld giving up name '%s'\n", (unsigned long long)conn->id, name);

	ret = ioctl(conn->fd, KDBUS_CMD_NAME_RELEASE, cmd_name);
	if (ret < 0) {
		ret = -errno;
		fprintf(stderr, "error releasing name: %s\n", strerror(-ret));
		return ret;
	}

	return 0;
}

int name_list(struct conn *conn, uint64_t flags)
{
	struct kdbus_cmd_name_list cmd_list;
	struct kdbus_name_list *list;
	struct kdbus_cmd_name *name;
	int ret;

	cmd_list.flags = flags;

	ret = ioctl(conn->fd, KDBUS_CMD_NAME_LIST, &cmd_list);
	if (ret < 0) {
		fprintf(stderr, "error listing names: %d (%m)\n", ret);
		return EXIT_FAILURE;
	}

	printf("REGISTRY:\n");
	list = (struct kdbus_name_list *)(conn->buf + cmd_list.offset);
	KDBUS_ITEM_FOREACH(name, list, names)
		printf("%8llu flags=0x%08llx conn=0x%08llx '%s'\n", name->owner_id,
		       name->flags, name->conn_flags,
		       name->size > sizeof(struct kdbus_cmd_name) ? name->name : "");
	printf("\n");

	ret = ioctl(conn->fd, KDBUS_CMD_FREE, &cmd_list.offset);
	if (ret < 0) {
		fprintf(stderr, "error free name list: %d (%m)\n", ret);
		return EXIT_FAILURE;
	}

	return 0;
}

void append_policy(struct kdbus_cmd_policy *cmd_policy, struct kdbus_item *policy, __u64 max_size)
{
	struct kdbus_item *dst = (struct kdbus_item *) ((char *) cmd_policy + cmd_policy->size);

	if (cmd_policy->size + policy->size > max_size)
		return;

	memcpy(dst, policy, policy->size);
	cmd_policy->size += KDBUS_ALIGN8(policy->size);
	free(policy);
}

struct kdbus_item *make_policy_name(const char *name)
{
	struct kdbus_item *p;
	__u64 size;

	size = offsetof(struct kdbus_item, policy.name) + strlen(name) + 1;
	p = malloc(size);
	if (!p)
		return NULL;
	memset(p, 0, size);
	p->size = size;
	p->type = KDBUS_ITEM_POLICY_NAME;
	strcpy(p->policy.name, name);

	return p;
}

struct kdbus_item *make_policy_access(__u64 type, __u64 bits, __u64 id)
{
	struct kdbus_item *p;
	__u64 size = sizeof(*p);

	p = malloc(size);
	if (!p)
		return NULL;

	memset(p, 0, size);
	p->size = size;
	p->type = KDBUS_ITEM_POLICY_ACCESS;
	p->policy.access.type = type;
	p->policy.access.bits = bits;
	p->policy.access.id = id;

	return p;
}

int upload_policy(int fd, const char *name)
{
	struct kdbus_cmd_policy *cmd_policy;
	struct kdbus_item *policy;
	int ret;
	int size = 0xffff;

	cmd_policy = (struct kdbus_cmd_policy *) alloca(size);
	memset(cmd_policy, 0, size);

	policy = (struct kdbus_item *) cmd_policy->policies;
	cmd_policy->size = offsetof(struct kdbus_cmd_policy, policies);

	policy = make_policy_name(name);
	append_policy(cmd_policy, policy, size);

	policy = make_policy_access(KDBUS_POLICY_ACCESS_USER, KDBUS_POLICY_OWN, getuid());
	append_policy(cmd_policy, policy, size);

	policy = make_policy_access(KDBUS_POLICY_ACCESS_WORLD, KDBUS_POLICY_RECV, 0);
	append_policy(cmd_policy, policy, size);

	policy = make_policy_access(KDBUS_POLICY_ACCESS_WORLD, KDBUS_POLICY_SEND, 0);
	append_policy(cmd_policy, policy, size);

	ret = ioctl(fd, KDBUS_CMD_EP_POLICY_SET, cmd_policy);
	if (ret < 0)
		fprintf(stderr, "--- error setting EP policy: %d (%m)\n", ret);

	return ret;
}

void add_match_empty(int fd)
{
	struct {
		struct kdbus_cmd_match cmd;
		struct kdbus_item item;
	} buf;
	int ret;

	memset(&buf, 0, sizeof(buf));

	buf.item.size = sizeof(uint64_t) * 3;
	buf.item.type = KDBUS_ITEM_ID;
	buf.item.id = KDBUS_MATCH_ID_ANY;

	buf.cmd.size = sizeof(buf.cmd) + buf.item.size;

	ret = ioctl(fd, KDBUS_CMD_MATCH_ADD, &buf);
	if (ret < 0)
		fprintf(stderr, "--- error adding conn match: %d (%m)\n", ret);
}
