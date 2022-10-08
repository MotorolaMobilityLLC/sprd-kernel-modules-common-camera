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

#include "cam_link.h"
#include "cam_node.h"

int cam_link_init(struct cam_linkage *linkage)
{
	int ret = 0;

	if (!linkage) {
		pr_err("fail to get invalid param ptr\n");
		return -EFAULT;
	}

	linkage->node_type = CAM_NODE_TYPE_MAX;
	linkage->node_id = CAM_LINK_DEFAULT_NODE_ID;
	linkage->port_id = CAM_LINK_DEFAULT_PORT_ID;

	return ret;
}

