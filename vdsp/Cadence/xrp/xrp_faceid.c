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

#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include "xrp_address_map.h"
#include "xrp_hw.h"
#include "xrp_internal.h"
#include "xrp_firmware.h"
#include "xrp_kernel_dsp_interface.h"
#include "xrp_faceid.h"



static int sprd_alloc_faceid_weights_buffer(struct xvp *xvp,struct ion_buf *ion_buf,size_t size)
{
	int ret;
	ret = xvp->vdsp_mem_desc->ops->mem_alloc(xvp->vdsp_mem_desc,
						ion_buf,
						ION_HEAP_ID_MASK_VDSP,/*todo vdsp head id*/
						size);
	if(0 != ret) {
		printk("yzl add %s failed\n" , __func__);
		return -ENOMEM;
	}
	ret = xvp->vdsp_mem_desc->ops->mem_kmap(xvp->vdsp_mem_desc, ion_buf);
	if(0 != ret) {
		xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, ion_buf);
		return -EFAULT;
	}
	//xvp->faceid_fw_viraddr = (void*)xvp->ion_faceid_fw.addr_k[0];
	ion_buf->dev = xvp->dev;
	printk("faceid alloc addr_p %lx  vaddr:%lx,size %ld\n" ,ion_buf->addr_p[0] , ion_buf->addr_k[0],ion_buf->size[0]);
	return 0;

}
static int sprd_free_faceid_weights_buffer(struct xvp *xvp,struct ion_buf *ion_buf)
{
	unsigned long dst_viraddr = ion_buf->addr_k[0];
	if(dst_viraddr) {
                xvp->vdsp_mem_desc->ops->mem_kunmap(xvp->vdsp_mem_desc, ion_buf);
                xvp->vdsp_mem_desc->ops->mem_free(xvp->vdsp_mem_desc, ion_buf);
	}
	return 0;
}
static int sprd_iommu_map_faceid_weights_buffer(struct xvp *xvp,struct ion_buf *ion_buf)
{
	int ret = -EFAULT;
	if(NULL == (void*)ion_buf->addr_k[0]) {
		printk("yzl add map faceid weights addr is NULL \n");
		return ret;
	}
	printk("ion_buf->addr_k[0] %lx\n",ion_buf->addr_k[0]);
	{
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_map(xvp->vdsp_mem_desc, ion_buf , IOMMU_ALL);
		if(ret) {
			printk("yzl add %s map faceid fialed\n" , __func__);
			return ret;
		}
		//xvp->dsp_firmware_addr = xvp->ion_faceid_fw.iova[0];
	}
	printk("yzl map faceid weights addr:%lx --> %lx\n",ion_buf->addr_k[0],ion_buf->iova[0]);

	return ret;
}
static int sprd_iommu_unmap_faceid_weights_buffer(struct xvp *xvp,struct ion_buf *ion_buf)
{
	int ret = -EFAULT;

	if(NULL == (void*)ion_buf->addr_k[0]) {
		printk("yzl add unmap faceid weights addr is NULL\n");
		return ret;
	}
	{
		ret = xvp->vdsp_mem_desc->ops->mem_iommu_unmap(xvp->vdsp_mem_desc, ion_buf, IOMMU_ALL);
		if(ret) {
			printk("yzl add %s unmap faceid weights buffer failed\n" , __func__);
			return ret;
		}
	}
	printk("yzl add %s :%lx\n" , __func__ ,ion_buf->addr_k[0]);
	return 0;
}

int sprd_faceid_request_algo_mem(struct xvp *xvp)
{
	int ret = sprd_alloc_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_mem_pool,FACEID_FD_MEM_SIZE);
	if (ret < 0){
		printk("yzl request fd mem fail\n");
		return ret;
	}
/*
	ret = sprd_alloc_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fp_mem_pool,FACEID_FP_MEM_SIZE);
	if (ret < 0){
		sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_mem_pool);
		printk("yzl request fp mem fail\n");
		return ret;
	}
	ret = sprd_alloc_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_flv_mem_pool,FACEID_FLV_MEM_SIZE);
	if (ret < 0){
		sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_mem_pool);
		sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fp_mem_pool);
		printk("yzl request flv mem fail\n");
		return ret;
	}
	ret = sprd_alloc_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fv_mem_pool,FACEID_FV_MEM_SIZE);
	if (ret < 0){
		sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_mem_pool);
		sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fp_mem_pool);
		sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_flv_mem_pool);
		printk("yzl request fv mem fail\n");
		return ret;
	}
*/
	return 0;
}
int sprd_faceid_release_algo_mem(struct xvp *xvp)
{
	sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_mem_pool);
	//sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fp_mem_pool);
	//sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_flv_mem_pool);
	//sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fv_mem_pool);
	return 0;
}

int sprd_faceid_request_weights_fd_p(struct xvp *xvp)
{
	unsigned long dst = 0;
	int ret = request_firmware(&xvp->faceid_fw, "network_coeff_fd_p.bin", xvp->dev);

	if (ret < 0){
		printk("yzl request fd p weights fail\n");
		return ret;
	}

	ret = sprd_alloc_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_weights_p,xvp->faceid_fw->size);
	if (ret < 0)
		return ret;

	dst = xvp->faceid_pool.ion_fd_weights_p.addr_k[0];
	memcpy((void*)dst , xvp->faceid_fw->data, xvp->faceid_fw->size);

	release_firmware(xvp->faceid_fw);
	printk("yzl request fd p weights done\n");
	return ret;
}
int sprd_faceid_request_weights_fd_r(struct xvp *xvp)
{
	unsigned long dst = 0;
	int ret = request_firmware(&xvp->faceid_fw, "network_coeff_fd_r.bin", xvp->dev);

	if (ret < 0){
		printk("yzl request fd r weights fail\n");
		return ret;
	}

	ret = sprd_alloc_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_weights_r,xvp->faceid_fw->size);
	if (ret < 0)
		return ret;

	dst = xvp->faceid_pool.ion_fd_weights_r.addr_k[0];
	memcpy((void*)dst, xvp->faceid_fw->data, xvp->faceid_fw->size);

	release_firmware(xvp->faceid_fw);
	printk("yzl request fd r weights done\n");

	return ret;
}
int sprd_faceid_request_weights_fd_o(struct xvp *xvp)
{
	unsigned long dst = 0;
	int ret = request_firmware(&xvp->faceid_fw, "network_coeff_fd_o.bin", xvp->dev);

	if (ret < 0){
		printk("yzl request fd o weights fail\n");
		return ret;
	}

	ret = sprd_alloc_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_weights_o,xvp->faceid_fw->size);
	if (ret < 0)
		return ret;

	dst = xvp->faceid_pool.ion_fd_weights_o.addr_k[0];
	memcpy((void*)dst, xvp->faceid_fw->data, xvp->faceid_fw->size);

	release_firmware(xvp->faceid_fw);
	printk("yzl request fd o weights done\n");

	return ret;
}
int sprd_faceid_request_weights_fp(struct xvp *xvp)
{
	unsigned long dst = 0;

	int ret = request_firmware(&xvp->faceid_fw, "network_coeff_fp.bin", xvp->dev);

	if (ret < 0){
		printk("yzl request fp weights fail\n");
		return ret;
	}

	ret = sprd_alloc_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fp_weights,xvp->faceid_fw->size);
	if (ret < 0)
		return ret;

	dst = xvp->faceid_pool.ion_fp_weights.addr_k[0];
	memcpy((void*)dst, xvp->faceid_fw->data, xvp->faceid_fw->size);

	release_firmware(xvp->faceid_fw);
	printk("yzl request fp weights done\n");

	return ret;
}
int sprd_faceid_request_weights_flv(struct xvp *xvp)
{
	unsigned long dst = 0;
	int ret = request_firmware(&xvp->faceid_fw, "network_coeff_flv.bin", xvp->dev);

	if (ret < 0){
		printk("yzl request flv weights fail\n");
		return ret;
	}

	ret = sprd_alloc_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_flv_weights,xvp->faceid_fw->size);
	if (ret < 0)
		return ret;

	dst = xvp->faceid_pool.ion_flv_weights.addr_k[0];
	memcpy((void*)dst, xvp->faceid_fw->data, xvp->faceid_fw->size);

	release_firmware(xvp->faceid_fw);
	printk("yzl request flv weights done\n");

	return ret;
}
int sprd_faceid_request_weights_fv(struct xvp *xvp)
{
	unsigned long dst = 0;
	int ret = request_firmware(&xvp->faceid_fw, "network_coeff_fv.bin", xvp->dev);

	if (ret < 0){
		printk("yzl request fv weights fail\n");
		return ret;
	}

	ret = sprd_alloc_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fv_weights,xvp->faceid_fw->size);
	if (ret < 0)
		return ret;

	dst = xvp->faceid_pool.ion_fv_weights.addr_k[0];
	memcpy((void*)dst, xvp->faceid_fw->data, xvp->faceid_fw->size);
	
	release_firmware(xvp->faceid_fw);
	printk("yzl request fv weights done\n");

	return ret;
}
int sprd_faceid_request_weights(struct xvp *xvp)
{
	sprd_faceid_request_weights_fd_p(xvp);
	sprd_faceid_request_weights_fd_r(xvp);
	sprd_faceid_request_weights_fd_o(xvp);
	sprd_faceid_request_weights_fp(xvp);
	sprd_faceid_request_weights_flv(xvp);
	sprd_faceid_request_weights_fv(xvp);
	sprd_faceid_request_algo_mem(xvp);
	return 0;
}
void sprd_faceid_release_weights(struct xvp *xvp)
{
	sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_weights_p);
	sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_weights_r);
	sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_weights_o);
	sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fp_weights);
	sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_flv_weights);
	sprd_free_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fv_weights);
	sprd_faceid_release_algo_mem(xvp);
}
int sprd_iommu_map_faceid_weights(struct xvp *xvp)
{
	sprd_iommu_map_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_weights_p);
	sprd_iommu_map_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_weights_r);
	sprd_iommu_map_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_weights_o);
	sprd_iommu_map_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fp_weights);
	sprd_iommu_map_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_flv_weights);
	sprd_iommu_map_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fv_weights);

	//sprd_iommu_map_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_mem_pool);
	//sprd_iommu_map_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fp_mem_pool);
	//sprd_iommu_map_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_flv_mem_pool);
	//sprd_iommu_map_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fv_mem_pool);
	return 0;
}
int sprd_iommu_unmap_faceid_weights(struct xvp *xvp)
{
	sprd_iommu_unmap_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_weights_p);
	sprd_iommu_unmap_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_weights_r);
	sprd_iommu_unmap_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_weights_o);
	sprd_iommu_unmap_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fp_weights);
	sprd_iommu_unmap_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_flv_weights);
	sprd_iommu_unmap_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fv_weights);

	//sprd_iommu_unmap_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fd_mem_pool);
	//sprd_iommu_unmap_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fp_mem_pool);
	//sprd_iommu_unmap_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_flv_mem_pool);
	//sprd_iommu_unmap_faceid_weights_buffer(xvp,&xvp->faceid_pool.ion_fv_mem_pool);
	return 0;
}

