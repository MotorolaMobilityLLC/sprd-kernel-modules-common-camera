/*
 * xrp_firmware: firmware manipulation for the XRP
 *
 * Copyright (c) 2015 - 2017 Cadence Design Systems, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Alternatively you can use and distribute this file under the terms of
 * the GNU General Public License version 2 or later.
 */

#ifndef XRP_FACEID_H
#define XRP_FACEID_H

#define FACEID_FD_MEM_SIZE (1024*1024*3)
#define FACEID_FP_MEM_SIZE (1024*1024*2)
#define FACEID_FLV_MEM_SIZE (1024*1024*2)
#define FACEID_FV_MEM_SIZE (1024*1024*2)


struct xvp;
struct faceid_hw_sync_data
{
	__u32 fd_p_coffe_addr;
	__u32 fd_r_coffe_addr;
	__u32 fd_o_coffe_addr;
	__u32 fp_coffe_addr;
	__u32 flv_coffe_addr;
	__u32 fv_coffe_addr;
	__u32 mem_pool_addr;
	__u32 yuv_addr;
	__u32  frame_height;
	__u32  frame_width;
};

int sprd_faceid_request_weights(struct xvp *xvp);
void sprd_faceid_release_weights(struct xvp *xvp);

int sprd_iommu_map_faceid_weights(struct xvp *xvp);
int sprd_iommu_unmap_faceid_weights(struct xvp *xvp);


#endif
