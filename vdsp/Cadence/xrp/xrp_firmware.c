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
#include "xrp_address_map.h"
#include "vdsp_hw.h"
#include "xrp_internal.h"
#include "xrp_firmware.h"
#include "xrp_kernel_dsp_interface.h"

#define DOWNLOAD_FIRMWARE

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "vdsp_firmware: %d %d %s : "\
        fmt, current->pid, __LINE__, __func__

static int xrp_load_segment_to_sysmem_ion(struct xvp *xvp , Elf32_Phdr *phdr)
{
#ifdef DOWNLOAD_FIRMWARE
	int32_t offset;
	uint8_t *virstart = NULL;
#endif
	pr_info("%s phdr->p_paddr:%lx , firmware viraddr:%lx\n" , __func__ , (unsigned long)phdr->p_paddr , (unsigned long)xvp->dsp_firmware_addr);
#ifdef DOWNLOAD_FIRMWARE
	if(phdr->p_paddr < xvp->dsp_firmware_addr)
		return -EFAULT;
	offset = phdr->p_paddr - xvp->dsp_firmware_addr;

	virstart = (uint8_t*)xvp->firmware_viraddr;

	pr_info("%s virstart:%p , offset:%x , poffset:%x ,pmemsz:%x , pfilesz:%x\n" , __func__ , virstart , offset , phdr->p_offset , phdr->p_memsz , phdr->p_filesz);
	memcpy((void*)(virstart + offset) , xvp->firmware->data + phdr->p_offset , phdr->p_filesz);
	if(phdr->p_memsz > phdr->p_filesz)
	{
		pr_info("%s memset vir:%p , size:%x\n" , __func__ , (void*)(virstart+ offset + phdr->p_filesz) , (phdr->p_memsz - phdr->p_filesz));
		memset((void*)(virstart + offset + phdr->p_filesz) , 0 , (phdr->p_memsz - phdr->p_filesz));
	}
	wmb();
#endif
	return 0;
}
static int xrp_load_segment_to_iomem(struct xvp *xvp, Elf32_Phdr *phdr)
{
	phys_addr_t pa = phdr->p_paddr;
	void __iomem *p = ioremap(pa, phdr->p_memsz);

	if (!p) {
		dev_err(xvp->dev,
			"couldn't ioremap %pap x 0x%08x\n",
			&pa, (u32)phdr->p_memsz);
		return -EINVAL;
	}
	pr_info("%s ioremap p:%p, p_paddr:%lx , memcpyhw:%p , memsethw:%p\n" , __func__ , p , (unsigned long)pa , xvp->hw_ops->memcpy_tohw , xvp->hw_ops->memset_hw);
	if (xvp->hw_ops->memcpy_tohw)
		xvp->hw_ops->memcpy_tohw(p, (void *)xvp->firmware->data +
					 phdr->p_offset, phdr->p_filesz);
	else
		memcpy_toio(p, (void *)xvp->firmware->data + phdr->p_offset,
			    ALIGN(phdr->p_filesz, 4));

	if (xvp->hw_ops->memset_hw)
		xvp->hw_ops->memset_hw(p + phdr->p_filesz, 0,
				       phdr->p_memsz - phdr->p_filesz);
	else
		memset_io(p + ALIGN(phdr->p_filesz, 4), 0,
			  ALIGN(phdr->p_memsz - ALIGN(phdr->p_filesz, 4), 4));

	iounmap(p);
	return 0;
}
static inline bool xrp_section_bad(struct xvp *xvp, const Elf32_Shdr *shdr)
{
	return shdr->sh_offset > xvp->firmware->size ||
		shdr->sh_size > xvp->firmware->size - shdr->sh_offset;
}

static int xrp_firmware_find_symbol(struct xvp *xvp, const char *name,
				    void **paddr, size_t *psize)
{
	const Elf32_Ehdr *ehdr = (Elf32_Ehdr *)xvp->firmware->data;
	const void *shdr_data = xvp->firmware->data + ehdr->e_shoff;
	const Elf32_Shdr *sh_symtab = NULL;
	const Elf32_Shdr *sh_strtab = NULL;
	const void *sym_data;
	const void *str_data;
	const Elf32_Sym *esym;
	void *addr = NULL;
	unsigned i;

	if (ehdr->e_shoff == 0) {
		pr_info("%s: no section header in the firmware image",
			__func__);
		return -ENOENT;
	}
	if (ehdr->e_shoff > xvp->firmware->size ||
	    ehdr->e_shnum * ehdr->e_shentsize > xvp->firmware->size - ehdr->e_shoff) {
		pr_info("%s: bad firmware SHDR information",
			__func__);
		return -EINVAL;
	}

	/* find symbols and string sections */

	for (i = 0; i < ehdr->e_shnum; ++i) {
		const Elf32_Shdr *shdr = shdr_data + i * ehdr->e_shentsize;

		switch (shdr->sh_type) {
		case SHT_SYMTAB:
			sh_symtab = shdr;
			break;
		case SHT_STRTAB:
			sh_strtab = shdr;
			break;
		default:
			break;
		}
	}

	if (!sh_symtab || !sh_strtab) {
		pr_info("%s: no symtab or strtab in the firmware image",
			__func__);
		return -ENOENT;
	}

	if (xrp_section_bad(xvp, sh_symtab)) {
		pr_info("%s: bad firmware SYMTAB section information",
			__func__);
		return -EINVAL;
	}

	if (xrp_section_bad(xvp, sh_strtab)) {
		pr_info("%s: bad firmware STRTAB section information",
			__func__);
		return -EINVAL;
	}

	/* iterate through all symbols, searching for the name */

	sym_data = xvp->firmware->data + sh_symtab->sh_offset;
	str_data = xvp->firmware->data + sh_strtab->sh_offset;

	for (i = 0; i < sh_symtab->sh_size; i += sh_symtab->sh_entsize) {
		esym = sym_data + i;

		if (!(ELF_ST_TYPE(esym->st_info) == STT_OBJECT &&
		      esym->st_name < sh_strtab->sh_size &&
		      strncmp(str_data + esym->st_name, name,
			      sh_strtab->sh_size - esym->st_name) == 0))
			continue;

		if (esym->st_shndx > 0 && esym->st_shndx < ehdr->e_shnum) {
			const Elf32_Shdr *shdr = shdr_data +
				esym->st_shndx * ehdr->e_shentsize;
			Elf32_Off in_section_off = esym->st_value - shdr->sh_addr;

			if (xrp_section_bad(xvp, shdr)) {
				pr_info("%s: bad firmware section #%d information",
					__func__, esym->st_shndx);
				return -EINVAL;
			}

			if (esym->st_value < shdr->sh_addr ||
			    in_section_off > shdr->sh_size ||
			    esym->st_size > shdr->sh_size - in_section_off) {
				pr_info("%s: bad symbol information",
					__func__);
				return -EINVAL;
			}
			addr = (void *)xvp->firmware->data + shdr->sh_offset +
				in_section_off;

			pr_info("%s: found symbol, st_shndx = %d, "
				"sh_offset = 0x%08x, sh_addr = 0x%08x, "
				"st_value = 0x%08x, address = %p",
				__func__, esym->st_shndx, shdr->sh_offset,
				shdr->sh_addr, esym->st_value, addr);
		} else {
			pr_info("%s: unsupported section index in found symbol: 0x%x",
				__func__, esym->st_shndx);
			return -EINVAL;
		}
		break;
	}

	if (!addr)
		return -ENOENT;

	*paddr = addr;
	*psize = esym->st_size;

	return 0;
}

static int xrp_firmware_fixup_symbol(struct xvp *xvp, const char *name,
				     phys_addr_t v)
{
	u32 v32 = XRP_DSP_COMM_BASE_MAGIC;
	void *addr;
	size_t sz;
	int rc;

	rc = xrp_firmware_find_symbol(xvp, name, &addr, &sz);
	if (rc < 0) {
		pr_info("%s: symbol \"%s\" is not found",
			__func__, name);
		return rc;
	}

	if (sz != sizeof(u32)) {
		pr_info("%s: symbol \"%s\" has wrong size: %zu",
			__func__, name, sz);
		return -EINVAL;
	}

	/* update data associated with symbol */

	if (memcmp(addr, &v32, sz) != 0) {
		pr_info("%s: value pointed to by symbol is incorrect: %*ph",
			__func__, (int)sz, addr);
	}

	v32 = v;
	memcpy(addr, &v32, sz);

	return 0;
}

static int xrp_load_firmware(struct xvp *xvp)
{
	Elf32_Ehdr *ehdr = (Elf32_Ehdr *)xvp->firmware->data;
	int i;

	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG)) {
		dev_err(xvp->dev, "bad firmware ELF magic\n");
		return -EINVAL;
	}

	if (ehdr->e_type != ET_EXEC) {
		dev_err(xvp->dev, "bad firmware ELF type\n");
		return -EINVAL;
	}

	if (ehdr->e_machine != 94 /*EM_XTENSA*/) {
		dev_err(xvp->dev, "bad firmware ELF machine\n");
		return -EINVAL;
	}

	if (ehdr->e_phoff >= xvp->firmware->size ||
	    ehdr->e_phoff +
	    ehdr->e_phentsize * ehdr->e_phnum > xvp->firmware->size) {
		dev_err(xvp->dev, "bad firmware ELF PHDR information\n");
		return -EINVAL;
	}
	xrp_firmware_fixup_symbol(xvp , "xrp_dsp_comm_base" , xvp->dsp_comm_addr);
	for (i = 0; i < ehdr->e_phnum; ++i) {
		Elf32_Phdr *phdr = (void *)xvp->firmware->data +
			ehdr->e_phoff + i * ehdr->e_phentsize;
		int rc;

		/* Only load non-empty loadable segments, R/W/X */
		if (!(phdr->p_type == PT_LOAD &&
		      (phdr->p_flags & (PF_X | PF_R | PF_W)) &&
		      phdr->p_memsz > 0))
			continue;

		if (phdr->p_offset >= xvp->firmware->size ||
		    phdr->p_offset + phdr->p_filesz > xvp->firmware->size) {
			dev_err(xvp->dev,
				"bad firmware ELF program header entry %d\n",
				i);
			return -EINVAL;
		}
		if(phdr->p_paddr >= xvp->dsp_firmware_addr) {
			rc = xrp_load_segment_to_sysmem_ion(xvp , phdr);
		} else {
			rc = xrp_load_segment_to_iomem(xvp , phdr);
		}

		if (rc < 0)
			return rc;
	}

	return 0;
}

int xrp_request_firmware(struct xvp *xvp)
{
	int ret = request_firmware(&xvp->firmware, xvp->firmware_name,
				   xvp->dev);

	if (ret < 0)
		return ret;

	ret = xrp_load_firmware(xvp);
	pr_info("%s ret:%d\n" , __func__ ,ret);
	if (ret < 0) {
		release_firmware(xvp->firmware);
	}
	return ret;
}
