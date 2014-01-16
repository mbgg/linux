/* linux/include/asm-arm/arch-mt6575/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 2006 MediaTek Inc.
 *
 */

#if defined(CONFIG_DEBUG_MTK6589_UART0)
#define MEDIATEK_UART_PA_BASE	0x11006000
#define MEDIATEK_UART_VA_BASE	0xF1006000

#elif defined(CONFIG_DEBUG_MTK6589_UART1)
#define MEDIATEK_UART_PA_BASE	0x11007000
#define MEDIATEK_UART_VA_BASE	0xF1007000

#elif defined(CONFIG_DEBUG_MTK6589_UART2)
#define MEDIATEK_UART_PA_BASE	0x11008000
#define MEDIATEK_UART_VA_BASE	0xF1008000

#elif defined(CONFIG_DEBUG_MTK6589_UART3)
#define MEDIATEK_UART_PA_BASE	0x11009000
#define MEDIATEK_UART_VA_BASE	0xF1009000

#endif


        .macro  addruart, rp, rv, tmp
        ldr	\rp, =MEDIATEK_UART_PA_BASE       @ physical base address
        ldr	\rv, =MEDIATEK_UART_VA_BASE       @ virtual base
        .endm

        .macro  senduart,rd,rx
        strb    \rd, [\rx, #0x0]
        .endm

        .macro  waituart,rd,rx
1001:   ldr \rd, [\rx, #0x14]    @ line status reg
        tst \rd, #0x40            @ write allow
        beq 1001b
        .endm

        .macro  busyuart,rd,rx
1002:   ldr \rd, [\rx, #0x14]   @ line status reg
        tst \rd, #0x40            @ uart full .eq. busy
        beq 1002b
        .endm
