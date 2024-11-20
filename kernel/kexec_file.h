
/* SPDX-License-Identifier: GPL-2.0 */
/*
 * NOTE:
 *
 * This header has combined a lot of unrelated to each other stuff.
 * The process of splitting its content is in progress while keeping
 * backward compatibility. That's why it's highly recommended NOT to
 * include this header inside another header file, especially under
 * generic or architectural include/ directory.
 */
#ifndef _LINUX_KEXEC_FILE_H
#define _LINUX_KEXEC_FILE_H

void *kexec_image_load_default(struct kimage *image);
int kexec_calculate_store_digests(struct kimage *image);

int kexec_walk_resources(struct kexec_buf *kbuf,
					int (*func)(struct resource *, void *));
int locate_mem_hole_callback(struct resource *res, void *arg);

int kexec_walk_memblock(struct kexec_buf *kbuf,
					int (*func)(struct resource *, void *));
#endif