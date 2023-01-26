/* SPDX-License-Identifier: GPL-2.0 */
/*
  initramfs_data includes the compressed binary that is the
  filesystem used for early user space.
*/

.section .init.ekdump,"a"
__ek_start:
.incbin "arch/early-kdump-input"
__ek_end:
.section .init.ekdump.info,"a"
.globl __initramfs_size
__ekdump_size:
#ifdef CONFIG_64BIT
	.quad __ek_end - __ek_start
#else
	.long __ek_end - __ek_start
#endif
