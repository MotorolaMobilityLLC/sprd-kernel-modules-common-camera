#include "xrp_library_loader.h"
#include "xt_library_loader.h"
#include "xrp_kernel_dsp_interface.h"
#include "xrp_kernel_defs.h"
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include "xrp_internal.h"
#if 0
#define LIBRARY_LOAD_UNLOAD_NSID        "lib_load"
#define LIBRARY_LOAD_LOAD_FLAG      0
#define LIBRARY_LOAD_UNLOAD_FLAG    1
#define LIBRARY_CMD_PIL_INFO_OFFSET   40

struct xrp_alien_mapping {
	unsigned long vaddr;
	unsigned long size;
	phys_addr_t paddr;
	void *allocation;
	enum {
		ALIEN_GUP,
		ALIEN_PFN_MAP,
		ALIEN_COPY,
	} type;
};

struct xrp_mapping {
	enum {
		XRP_MAPPING_NONE,
		XRP_MAPPING_NATIVE,
		XRP_MAPPING_ALIEN,
		XRP_MAPPING_KERNEL = 0x4,
	} type;
	union {
		struct {
			struct xrp_allocation *xrp_allocation;
			unsigned long vaddr;
		} native;
		struct xrp_alien_mapping alien_mapping;
	};
};

struct xrp_request {
	struct xrp_ioctl_queue ioctl_queue;
	size_t n_buffers;
	struct xrp_mapping *buffer_mapping;
	struct xrp_dsp_buffer *dsp_buffer;
	phys_addr_t in_data_phys;
	phys_addr_t out_data_phys;
	phys_addr_t dsp_buffer_phys;
	union {
		struct xrp_mapping in_data_mapping;
		u8 in_data[XRP_DSP_CMD_INLINE_DATA_SIZE];
	};
	union {
		struct xrp_mapping out_data_mapping;
		u8 out_data[XRP_DSP_CMD_INLINE_DATA_SIZE];
	};
	union {
		struct xrp_mapping dsp_buffer_mapping;
		struct xrp_dsp_buffer buffer_data[XRP_DSP_CMD_INLINE_BUFFER_COUNT];
	};
	u8 nsid[XRP_DSP_CMD_NAMESPACE_ID_SIZE];
};

#endif



int32_t libinfo_list_init(struct libinfo_list *list)
{
    if(list == NULL)
        return LIST_ERROR;
    memset(list,0,sizeof(struct libinfo_list));
    return LIST_SUCCESS;
}
 
int32_t libinfo_list_size(const struct libinfo_list *list)
{
    if(list == NULL)
        return LIST_ERROR;
    return list->number;
}
void* libinfo_alloc_element()
{
	struct loadlib_info *pnew = NULL;
	pnew = vmalloc(sizeof(struct loadlib_info));
	if(pnew!=NULL)
	{
		memset(pnew , 0 , sizeof(struct loadlib_info));
	}
	printk("yzl add %s new element:%p\n" , __func__, pnew);
	return pnew;
} 
/*index start from 0*/
int32_t libinfo_list_add(struct libinfo_list *list,void *element,int32_t pos)
{
    int i = 0;
    struct libinfo_node *new_node = NULL;
    struct libinfo_node *node = NULL;
    if(list == NULL)
        return LIST_ERROR;
    
    new_node = (struct libinfo_node *)vmalloc(sizeof(struct libinfo_node));
    if(new_node == NULL)
        return LIST_NO_MEM;
    printk("yzl add %s , new new node:%p\n" , __func__ , new_node);
    memset(new_node,0,sizeof(struct libinfo_node));
    new_node->element = element;
 
    if(list->number == 0)
    {
        new_node->next = NULL;
        list->node = new_node;
        list->number++;
        return list->number;
    }
    if(pos < 0 || pos >= list->number)
        pos = list->number;
 
    node = list->node;
    
    if(pos == 0)
    {
        new_node->next = list->node;
        list->node = new_node;
        list->number++;
        return list->number;
    }
 
    while(i + 1 < pos)
    {
        i++;
        node = node->next;
    }
    if(pos == list->number)
    {
        new_node->next = NULL;
        node->next = new_node;
        list->number++;
        return list->number;
    }
    new_node->next = node->next;
    node->next->next = new_node;
    list->number++;
    
    return list->number;
}
 
void *libinfo_list_get(const struct libinfo_list *list,int32_t pos)
{
    int i = 0;
    struct libinfo_node *node = NULL;
    if(list == NULL)
        return NULL;
    if(pos < 0 || pos >= list->number)
        return NULL;
    node = list->node;
    while(i < pos)
    {
        i++;
        node = node->next;
    }
    
    return node->element;
}
 
int32_t libinfo_list_remove(struct libinfo_list *list,int32_t pos)
{
    int i = 0;
    struct libinfo_node *node = NULL;
    struct libinfo_node *del_node = NULL;
    if(list == NULL)
        return LIST_ERROR;
    if(pos < 0 || pos >= list->number)
        return LIST_ERROR;
    node = list->node;
    /*special case*/
    if(pos == 0)
    {
        list->node = node->next;
        list->number--;
	if(node->element) {
		printk("yzl add %s ,pos is 0 free element:%p\n" , __func__ , node->element);
		vfree(node->element);
	}
	printk("yzl add %s , pos is 0 free new node:%p\n" , __func__ , node);
	vfree(node);
	return LIST_SUCCESS;
    }
 
    while(pos > i + 1)
    {
        i++;
        node = node->next;
    }
    del_node = node->next;
    node->next = node->next->next;
    list->number--;
    if(del_node->element)
    {
	printk("yzl add %s , free element:%p\n" , __func__ , del_node->element);
	vfree(del_node->element); 
    }
    printk("yzl add %s , free new node:%p\n" , __func__ , del_node);
    vfree(del_node);
    return LIST_SUCCESS;
    
}
#if 0
static struct loadlib_info *xrp_library_getlibinfo(struct xvp *xvp , const char *libname)
{
	int i;
        struct loadlib_info *libinfo = NULL;
        for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
                libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
                if(0 == strcmp(libinfo->libname , libname)) {
                        break;
                }
        }
        /*find , and decrease*/
        if(i < libinfo_list_size(&(xvp->load_lib.lib_list))) {
                return libinfo;
        }
        else
                return NULL;
}
static int xrp_load_pi_to_buffer(const char *filename , void **out)
{
	loff_t pos = 0;
	void *libstart = NULL;
	struct kstat stat;
	__u32 libsize;
	mm_segment_t fs;
	int err;
	struct file *fp = NULL;
	set_fs(KERNEL_DS);
	err = vfs_stat(filename , &stat);
	if(err != 0)
		return -1;
	libsize = stat.size;
	fp = filp_open(filename ,O_RDONLY ,0);
	if(IS_ERR(fp)) {
		printk("yzl %s add open fp failed\n" , __func__);
		return -1;;
	}
	fs = get_fs();
	printk("yzl add %s , libsize:%d\n" , __func__  , libsize);
	libstart = vmalloc(libsize);
	if(NULL == libstart)
	{
		printk("yzl add %s malloc lib mem failed\n" , __func__);
                filp_close(fp, NULL);
		return -1;;
	}
	vfs_read(fp , libstart , libsize , &pos);
	filp_close(fp , NULL);
	set_fs(fs);
	/*yzl add load lib bin*/
	printk("yzl add %s after load lib to libstart:%p\n" ,__func__ , libstart);
	*out = libstart;
	return 0;
}
#if 0
static long xrp_library_unload_internal(struct xvp *xvp , const char* libname)
{
	
}
#endif
static int32_t xrp_library_load_internal(struct xvp *xvp , const char* libname)
{
	char *file_buffer = NULL;
	unsigned int size;
	int32_t ret = 0;
	struct loadlib_info *new_element;
	unsigned int result;
	char totallibname[64];// = "/vendor/firmware/";
	sprintf(totallibname , "/vendor/firmware/%s.bin" , libname);
	/*load library to ddr*/
	ret = xrp_load_pi_to_buffer(totallibname , (void**)(&file_buffer));
	if(ret != 0)
		return ret;
	size = xtlib_pi_library_size((xtlib_packaged_library *)file_buffer);
	/*alloc ion buffer*/

	/*mmap iommu*/
	
	/*alloc ion buffer for code*/

	/*onlyl for compile may be change later*/
	result = xtlib_host_load_pi_library((xtlib_packaged_library*)file_buffer , 0 , NULL , NULL, NULL, NULL);
	if(result == 0)
	{
		/*free ion buffer*/
		
		/**/
		vfree(file_buffer);		
		printk("yzl add %s xtlib_host_load_pi_library failed %d\n" , __func__ , result);
		return -1; 
	}
	vfree(file_buffer);
	new_element = (struct loadlib_info*)libinfo_alloc_element();
	if(new_element == NULL) {
		/*free ion buffer*/
		
		return -1;
	}
	else {
		
		sprintf(new_element->libname , "%s" , libname);
		/*may be change later*/
		new_element->address_start = 0;
		new_element->length = size;
		new_element->load_count = 0;
		new_element->ionhandle = NULL;
		new_element->pil_ionhandle = NULL;
		new_element->pil_info = 0;
		new_element->valid_flag = 1;
	}
	libinfo_list_add(&xvp->load_lib.lib_list , new_element , libinfo_list_size(&xvp->load_lib.lib_list));
	return 0;
	
}
enum load_unload_flag xrp_check_load_unload(struct xvp *xvp , struct xrp_request *rq)
{
	__u32 indata_size;
	enum load_unload_flag load_flag = 0;
	__u8 *input_ptr = NULL;
	struct xrp_mapping *mapping = &rq->in_data_mapping;
	indata_size = rq->ioctl_queue.in_data_size;
	if(0 == strcmp(rq->nsid , LIBRARY_LOAD_UNLOAD_NSID)) {
		if(indata_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
                        if(mapping->type == XRP_MAPPING_NATIVE) {
                                input_ptr = (__u8 *)mapping->native.vaddr;
                                printk("yzl add %s XRP_MAPPING_NATIVE input_vir:%p\n" , __func__ , input_ptr);
                        }
			else{
				input_ptr = (__u8 *)mapping->alien_mapping.vaddr;
				printk("yzl add %s other mapping input_vir:%p\n" , __func__ , input_ptr);
			}
                }
                else {
                        input_ptr = (__u8 *)rq->in_data;
                        printk("yzl add %s indata_size is small input_vir:%p\n" , __func__ , input_ptr);
                }
		load_flag = *input_ptr;
		return load_flag;
	}
	else
		return XRP_NOT_LOAD_UNLOAD;
}
int32_t xrp_library_decrease(struct xvp *xvp , const char *libname)
{
	int i;
	struct loadlib_info *libinfo = NULL;
	for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
		libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
		if(0 == strcmp(libinfo->libname , libname)) {
			break;
		}
	}
	/*find , and decrease*/
	if(i < libinfo_list_size(&(xvp->load_lib.lib_list))) {
		libinfo->load_count--;
	}
	else
		return -EINVAL;
	return libinfo->load_count;

}
int xrp_check_library_loaded(struct xvp *xvp , const char *libname)
{
	int i;
	int ret = 0;
	struct loadlib_info *libinfo = NULL;
	for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
		libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
		if(0 == strcmp(libinfo->libname , libname))
			break;
	}
	/**/
	if(i < libinfo_list_size(&(xvp->load_lib.lib_list))) {
		libinfo->load_count++;
		ret = 0; /*loaded*/
	}
	else
		ret = 1;
	return ret;
	
}
/* return value 0 is need load, 1 is loaded already*/
int32_t xrp_library_load(struct xvp *xvp , struct xrp_request *rq , char *outlibname)
{
	struct xrp_mapping *mapping;
	__u32 indata_size;
	__u8 load_flag = 0;
	int32_t ret = 0;
	int i;
	struct loadlib_info *libinfo = NULL;
	__u8 *input_ptr = NULL;
	char libname[64];
	indata_size = rq->ioctl_queue.in_data_size;
	mapping = &(rq->in_data_mapping);
	/*check whether loaded*/
	
	/*check whether load cmd*/
	if(0 == strcmp(rq->nsid , LIBRARY_LOAD_UNLOAD_NSID)) {
		/*check libname*/
		if(indata_size > XRP_DSP_CMD_INLINE_DATA_SIZE) {
			if(mapping->type == XRP_MAPPING_NATIVE) {
				input_ptr = (__u8*)mapping->native.vaddr;
				printk("yzl add %s XRP_MAPPING_NATIVE input_vir:%p\n" , __func__ , input_ptr);
			}
			else {
				input_ptr = (__u8*)mapping->alien_mapping.vaddr;
				printk("yzl add %s other mapping input_vir:%p\n" , __func__ , input_ptr);
			}
		}
		else {
			input_ptr = (__u8*)rq->in_data;
			printk("yzl add %s indata_size is small input_vir:%p\n" , __func__ , input_ptr);
		}
		/*input_vir first byte is load or unload*/
		load_flag = *input_ptr;
		if(LIBRARY_LOAD_LOAD_FLAG == load_flag) {
			/*load*/
			sprintf(libname , "%s" , input_ptr+1);
			/*check whether loaded*/
			for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
				libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
				if(0 == strcmp(libinfo->libname , libname))
					break;
			}
			/**/
			if(i < libinfo_list_size(&(xvp->load_lib.lib_list))) {
				libinfo->load_count++;
				ret = 1; /*loaded*/
			}
			else {
				/*not loaded alloc libinfo node ,load internal*/
				ret = xrp_library_load_internal(xvp , libname);
				if(ret != 0) {

					ret = -ENOMEM;
				}
				/*re edit rq for register libname , input data: input[0] load unload flag
				input[1] ~input[32] --- libname , input[LIBRARY_CMD_PIL_INFO_OFFSET]~input[43] ---- libinfo addr*/
				libinfo = xrp_library_getlibinfo(xvp , libname);
				if(NULL == libinfo) {
					printk("yzl add %s xrp_library_getlibinfo NULL\n" , __func__);
					xrp_library_release(xvp , libname);
				}
				*((uint32_t*)(input_ptr+LIBRARY_CMD_PIL_INFO_OFFSET)) = libinfo->pil_info;
				printk("yzl add %s load input param nsid:%s , loadflag:%d , libname:%s , pil_info:%x\n" , __func__ ,rq->nsid , 
										load_flag , libname , libinfo->pil_info);
				sprintf(outlibname , "%s" , libname);
			}
		}
		else {
			ret = -EINVAL;
		}
		return ret;
	}
	else
		return 0;
}
int32_t xrp_library_release_all(struct xvp *xvp)
{
	int i;
	int ret = 0;
	struct loadlib_info *libinfo = NULL;
	for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
		libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
		/*unmap iommu and release ion buffer*/
		
		/*remove lib info element from list*/
		libinfo_list_remove(&(xvp->load_lib.lib_list) , i);
	}
	return ret;
}
/*release ion buffer , unmap iommu , decount load_count*/
int32_t xrp_library_release(struct xvp *xvp , const char *libname)
{
	int i;
	int ret;
	struct loadlib_info *libinfo = NULL;
	/*unmap iommu*/
	/*free ion buffer*/
	
	/*decrease load_count*/
        for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
                libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
                if(0 == strcmp(libinfo->libname , libname))
                        break;
        }
        /**/
        if(i < libinfo_list_size(&(xvp->load_lib.lib_list))) {
                libinfo->load_count--;
		printk("yzl add %s load_count:%d\n" , __func__ , libinfo->load_count);
		/*remove this lib element*/
		libinfo_list_remove(&(xvp->load_lib.lib_list) , i);
                ret = 0; /*loaded*/
        }
        else {
		printk("yzl add %s not find lib:%s , may be some error\n" , __func__ , libname);
                ret = 1;
	}
        return ret;
}
int32_t xrp_register_libs(struct xvp *xvp , struct xrp_comm *comm)
{
	int i;
	int ret =0;
	int realret = 0;
	__u8 *input_ptr = NULL;
	struct loadlib_info *libinfo = NULL;
	struct xrp_request rq;
	rq.ioctl_queue.flags = XRP_DSP_CMD_FLAG_REQUEST_VALID;
	rq.ioctl_queue.in_data_size = 64;
	rq.ioctl_queue.out_data_size = 0;
	rq.ioctl_queue.buffer_size = 0;
	rq.ioctl_queue.in_data_addr = 0;
	rq.ioctl_queue.out_data_addr = 0;
	rq.ioctl_queue.buffer_addr = 0;
	rq.ioctl_queue.nsid_addr = 0;
	rq.n_buffers = 0;
	rq.buffer_mapping = NULL;
	rq.dsp_buffer = NULL;
	/*may change later*/
	rq.in_data_phys = 0;
	sprintf(rq.nsid , "%s" , LIBRARY_LOAD_UNLOAD_NSID);
	for(i = 0; i < libinfo_list_size(&(xvp->load_lib.lib_list)) ; i++) {
		libinfo = libinfo_list_get(&(xvp->load_lib.lib_list),i);
		/*re send register cmd*/
		input_ptr = (__u8*)rq.ioctl_queue.in_data_addr;
		input_ptr[0] = LIBRARY_LOAD_LOAD_FLAG;
		sprintf(input_ptr+1 , "%s" , libinfo->libname);
		*((unsigned int*)(input_ptr + LIBRARY_CMD_PIL_INFO_OFFSET)) = libinfo->pil_info;
		xrp_fill_hw_request(comm->comm , &rq , &xvp->address_map);
		xrp_send_device_irq(xvp);
		if(xvp->host_irq_mode) {
			ret = xvp_complete_cmd_irq(xvp , comm , xrp_cmd_complete);
		} else {
			ret = xvp_complete_cmd_poll(xvp, comm ,
				xrp_cmd_complete);
		}
		xrp_panic_check(xvp);
		if(0 == ret) {
			ret = xrp_complete_hw_request(comm->comm , &rq);
		}
		if(ret != 0) {
			/*set invalid*/
			printk("yzl add %s re load lib:%s failed\n" , __func__ , libinfo->libname);
			libinfo->valid_flag = 0;
			realret = -1;
		}
        }
	return realret;
}

#endif
