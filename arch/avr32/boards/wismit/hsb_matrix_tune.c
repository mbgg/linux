#include <linux/kernel.h>
//#include <mach/at32ap700x.h>
#include <linux/init.h>
#include <linux/clk.h>

#include <mach/hmatrix.h>

static int __init set_spi_hmatrix_prio(void)
{
   unsigned int scfg;
   unsigned int scfg2;

   clk_enable(&at32_hmatrix_clk);

   /* Set fixed default master for the EBI (S4) to HSB-HSB Bridge (M2) and increased burst to 64 cycles.  */
   scfg = 0x40// --> number of slot cycles per master
      | (  2 << 16)// --> fixed default master
      | ((0x2) << 18)// --> default master is HSB-HSB Bridge
      | (  1 << 24);// --> fixed priority turned on
//   hmatrix_writel2((0x0040 + 4 * (4)),scfg);// --> EBI slave config register
   hmatrix_write_reg(HMATRIX_SCFG(4), scfg);

   /* Set HSB-HSB to max priority when interfacing the EBI. */
   scfg2 = ((0x3) << 8)// --> HSB-HSB Brige
      | ((0x2) << 24)//  --> MAC0
      | ((0x2) << 28);// --> MAC1
   // ?? may be better icache and dcache
//   hmatrix_writel2((0x0080 + 8 * (4)),scfg2); // --> EBI priority register (which master has prio to access EBI)
   hmatrix_write_reg(HMATRIX_PRAS(4), scfg2);

   clk_disable(&at32_hmatrix_clk);

   printk(KERN_INFO"Wismit Board: HSB Matrix for spi changed - EBI slave config %x - EBI priority register %x\n", scfg, scfg2);

return 0;
}
postcore_initcall(set_spi_hmatrix_prio);

