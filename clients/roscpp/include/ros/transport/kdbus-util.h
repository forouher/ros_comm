/*
 * Copyright (C) 2013 Kay Sievers
 *
 * kdbus is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 */
#pragma once

extern "C"
{

#ifndef KDBUS_UTIL_H
#define KDBUS_UTIL_H

#define BIT(X) (1 << (X))

#include "kdbus.h"

#define _STRINGIFY(x) #x
#define STRINGIFY(x) _STRINGIFY(x)
#define ELEMENTSOF(x) (sizeof(x)/sizeof((x)[0]))

#define KDBUS_PTR(addr) ((void *)(uintptr_t)(addr))

#define KDBUS_ALIGN8(l) (((l) + 7) & ~7)
#define KDBUS_ITEM_HEADER_SIZE offsetof(struct kdbus_item, data)
#define KDBUS_ITEM_SIZE(s) KDBUS_ALIGN8((s) + KDBUS_ITEM_HEADER_SIZE)

#define KDBUS_ITEM_NEXT(item) \
	(typeof(item))(((uint8_t *)item) + KDBUS_ALIGN8((item)->size))
#define KDBUS_ITEM_FOREACH(item, head, first)				\
	for (item = (head)->first;					\
	     (uint8_t *)(item) < (uint8_t *)(head) + (head)->size;	\
	     item = KDBUS_ITEM_NEXT(item))

#ifndef __NR_memfd_create
#   define __NR_memfd_create 319
#endif

enum {
	CHECK_OK,
	CHECK_SKIP,
	CHECK_ERR,
};

struct conn {
	int fd;
	uint64_t id;
	void *buf;
	size_t size;
};

#endif

}
