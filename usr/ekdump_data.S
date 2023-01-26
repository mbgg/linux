/* SPDX-License-Identifier: GPL-2.0 */
/*
  initramfs_data includes the compressed binary that is the
  filesystem used for early user space.
*/

.section .init.ekdump,"a"
__ekd_start:
.incbin "usr/ekdump_inc_data"
__ekd_end:
.section .init.ekdump.info,"a"
.globl __ekdump_size
__ekdump_size:
#ifdef CONFIG_64BIT
	.quad __ekd_end - __ekd_start
#else
	.long __ekd_end - __ekd_start
#endif
