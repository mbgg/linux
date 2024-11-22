// SPDX-License-Identifier: GPL-2.0-only
/*
 * kexec: kexec_file_load system call
 *
 * Copyright (C) 2014 Red Hat Inc.
 * Authors:
 *      Vivek Goyal <vgoyal@redhat.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/capability.h>
#include <linux/mm.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/kexec.h>
#include <linux/memblock.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/ima.h>
#include <crypto/hash.h>
#include <crypto/sha2.h>
#include <linux/elf.h>
#include <linux/elfcore.h>
#include <linux/kernel.h>
#include <linux/kernel_read_file.h>
#include <linux/syscalls.h>
#include <linux/vmalloc.h>
#include "kexec_internal.h"
#include "kexec_file.h"

extern char __ekdump_start[];
extern unsigned long __ekdump_size;

static void kimage_early_post_load_cleanup(struct kimage *image)
{
	struct purgatory_info *pi = &image->purgatory_info;

	kfree(image->kernel_buf);
	image->kernel_buf = NULL;

	kfree(pi->purgatory_buf);
	pi->purgatory_buf = NULL;

	kfree(pi->sechdrs);
	pi->sechdrs = NULL;

#ifdef CONFIG_IMA_KEXEC
	vfree(image->ima_buffer);
	image->ima_buffer = NULL;
#endif /* CONFIG_IMA_KEXEC */

	/* See if architecture has anything to cleanup post load */
	arch_kimage_file_post_load_cleanup(image);

	/*
	 * Above call should have called into bootloader to free up
	 * any data stored in kimage->image_loader_data. It should
	 * be ok now to free it up.
	 */
	kfree(image->image_loader_data);
	image->image_loader_data = NULL;

	kexec_file_dbg_print = false;
}

static int
kimage_early_prepare_segments(struct kimage *image)
	/*, int kernel_fd, int initrd_fd,
			     const char __user *cmdline_ptr,
			     unsigned long cmdline_len, unsigned flags)*/
{
	int ret;
	void *ldata;

// TODO do we really need that? __ekdump is already in kernel memory
 pr_err("%s %d\n", __func__, __LINE__);
	image->kernel_buf_len = (unsigned long) __ekdump_size;
	image->kernel_buf = kmemdup(__ekdump_start, image->kernel_buf_len,
								GFP_KERNEL);

 pr_err("%s %d\n", __func__, __LINE__);
	/* Call arch image probe handlers */
	ret = arch_kexec_kernel_image_probe(image, image->kernel_buf,
					    image->kernel_buf_len);
	if (ret)
		goto out;

 pr_err("%s %d\n", __func__, __LINE__);
#ifdef CONFIG_KEXEC_SIG
	ret = kimage_validate_signature(image);

	if (ret)
		goto out;
#endif
 pr_err("%s %d\n", __func__, __LINE__);

	/* IMA needs to pass the measurement list to the next kernel. */
	ima_add_kexec_buffer(image);

	/* Call arch image load handlers */
	ldata = kexec_image_load_default(image);

	if (IS_ERR(ldata)) {
		ret = PTR_ERR(ldata);
		goto out;
	}

 pr_err("%s %d\n", __func__, __LINE__);
	image->image_loader_data = ldata;
out:
 pr_err("%s %d\n", __func__, __LINE__);
	/* In case of error, free up all allocated memory in this function */
	if (ret)
		kimage_early_post_load_cleanup(image);
	return ret;
}

static int
kimage_early_alloc_init(struct kimage **rimage)
{
	int ret;
	struct kimage *image;

 pr_err("%s %d\n", __func__, __LINE__);
	image = do_kimage_alloc_init();
	if (!image)
		return -ENOMEM;

	image->file_mode = 1;
 pr_err("%s %d\n", __func__, __LINE__);

	/* Enable special crash kernel control page alloc policy. */
	image->control_page = crashk_res.start;
	image->type = KEXEC_TYPE_CRASH;

	ret = kimage_early_prepare_segments(image);
	if (ret)
		goto out_free_image;

 pr_err("%s %d\n", __func__, __LINE__);
	ret = sanity_check_segment_list(image);
	if (ret)
		goto out_free_post_load_bufs;

 pr_err("%s %d\n", __func__, __LINE__);
	ret = -ENOMEM;
	image->control_code_page = kimage_alloc_control_pages(image,
					   get_order(KEXEC_CONTROL_PAGE_SIZE));
	if (!image->control_code_page) {
		pr_err("Could not allocate control_code_buffer\n");
		goto out_free_post_load_bufs;
	}

 pr_err("%s %d\n", __func__, __LINE__);
	*rimage = image;
	return 0;
//out_free_control_pages:
//	kimage_free_page_list(&image->control_pages);
out_free_post_load_bufs:
	kimage_early_post_load_cleanup(image);
out_free_image:
 pr_err("%s %d\n", __func__, __LINE__);
	kfree(image);
	return ret;
}

void __init kexec_early_dump(void)
{
	int ret = 0, i;
	struct kimage **dest_image, *image;

 pr_err("%s %d\n", __func__, __LINE__);
	image = NULL;
	dest_image = &kexec_crash_image;

 pr_err("%s %d\n", __func__, __LINE__);
	ret = kimage_early_alloc_init(&image);
	if (ret)
		goto out;

 pr_err("%s %d\n", __func__, __LINE__);
	ret = kimage_crash_copy_vmcoreinfo(image);
	if (ret)
		goto out;

 pr_err("%s %d\n", __func__, __LINE__);
	ret = kexec_calculate_store_digests(image);
	if (ret)
		goto out;
	
 pr_err("%s %d\n", __func__, __LINE__);
	pr_err("nr_segments = %lu\n", image->nr_segments);
	for (i = 0; i < image->nr_segments; i++) {
		struct kexec_segment *ksegment;

		ksegment = &image->segment[i];
			pr_err("segment[%d]: buf=0x%p bufsz=0x%zx mem=0x%lx memsz=0x%zx\n",
			      i, ksegment->buf, ksegment->bufsz, ksegment->mem,
			      ksegment->memsz);

		ret = kimage_load_segment(image, &image->segment[i]);
		if (ret)
			goto out;
	}

	kimage_terminate(image);

 pr_err("%s %d\n", __func__, __LINE__);
	ret = machine_kexec_post_load(image);
	if (ret)
		goto out;

	pr_err("kexec_file_load: type:%u, start:0x%lx head:0x%lx\n",
		      image->type, image->start, image->head);
	/*
	 * Free up any temporary buffers allocated which are not needed
	 * after image has been loaded
	 */
	kimage_early_post_load_cleanup(image);
	image = xchg(dest_image, image);
 pr_err("%s %d\n", __func__, __LINE__);
out:
 pr_err("%s %d\n", __func__, __LINE__);
	arch_kexec_protect_crashkres();

 pr_err("%s %d\n", __func__, __LINE__);
	kimage_free(image);
	return;
}
