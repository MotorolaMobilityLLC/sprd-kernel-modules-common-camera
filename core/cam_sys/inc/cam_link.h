/*
 * Copyright (C) 2021-2022 UNISOC Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _CAM_LINK_H_
#define _CAM_LINK_H_

#include <asm-generic/errno-base.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/types.h>

#define CAM_LINK_DEFAULT_NODE_ID     0xFFFF
#define CAM_LINK_DEFAULT_PORT_ID     0xFFFF

struct cam_linkage {
	uint32_t node_type;
	uint32_t node_id;
	uint32_t port_id;
};

int cam_link_init(struct cam_linkage *link);

#endif
