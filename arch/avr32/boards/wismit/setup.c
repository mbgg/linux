/*
 * Board-specific setup code for WISMIT board
 *
 * Matthias Brugger  <matthias.bgg@gmail.com>
 * Steffen Mueller
 *
 */
#include <linux/clk.h>
#include <linux/etherdevice.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/init.h>
#include <linux/linkage.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/leds.h>
#include <linux/spi/spi.h>
#include <linux/w1-gpio.h>
#include <linux/atmel-mci.h>

#include <asm/io.h>
#include <asm/setup.h>

#include <mach/at32ap700x.h>
#include <mach/board.h>
#include <mach/init.h>
#include <mach/portmux.h>

#include <linux/gpio_keys.h>
#include <linux/input.h>

#define GCLK0_RATE	1000000		// for barometer (smd500) clock input
#define GCLK1_RATE	1311000		// for EMF (xen-1200) clock input


/* Oscillator frequencies. These are board-specific */
unsigned long at32_board_osc_rates[3] = {
	[0] = 32768,	/* 32.768 kHz on RTC osc */
	[1] = 20000000,	/* 20 MHz on osc0 */
	[2] = 12000000,	/* 12 MHz on osc1 */
};


/* Initialized by bootloader-specific startup code. */
struct tag *bootloader_tags __initdata;

struct eth_addr {
	u8 addr[6];
};
static struct eth_addr __initdata hw_addr[1];
static struct macb_platform_data __initdata eth_data[1];


static struct spi_board_info spi0_board_info[] __initdata = {
	{
		.modalias		= "spidev",	 	// hf (nanotron)
		.max_speed_hz	= 8000000,
		.chip_select	= 0,
		//.mode			= SPI_MODE_1,
		.mode			= SPI_MODE_3,
	},
	{
		//.modalias		= "spidev",		// lc-display
		.modalias		= "LC_Display",
		.max_speed_hz	= 500000, //8000000,
		.chip_select	= 1,
		.mode			= SPI_MODE_0,
	},
	{
		.modalias		= "spidev",		// rfid
		.max_speed_hz	= 8000000,
		.chip_select	= 2,
		.mode			= SPI_MODE_0,
	},
	{
		.modalias 		= "spidev",		// pbi
		.max_speed_hz	= 8000000,
		.chip_select	= 3,
		.mode			= SPI_MODE_0,
	},
};


struct adis16255_init_data {
        char direction;
	u8   negative;
	int irq;
};

struct xen1200_init_data {
    char axis;
    int irq;
};

struct adis16255_init_data pd_x = {
	.direction = 'x',
	.negative = 1,
	.irq = GPIO_PIN_PB(13),
};

struct adis16255_init_data pd_y = {
	.direction = 'y',
	.negative = 1,
	.irq = GPIO_PIN_PB(11),
};

struct adis16255_init_data pd_z = {
	.direction = 'z',
	.negative = 1,
        .irq = GPIO_PIN_PC(24),
};

static struct spi_board_info spi1_board_info[] __initdata = {
	{
		//.modalias		= "spidev",	 	// Accel
		.modalias		= "spi_lis3lv02",	 	// Accel
		.max_speed_hz	= 1*1000*1000, //1000000,
		.chip_select	= 0,
		.mode			= SPI_MODE_3,
	},
	{
		.modalias		= "spi_adis16255",		// Gyro 1
		.max_speed_hz	= 1000000,
		.chip_select	= 1,
		.mode			= SPI_MODE_3,
		.platform_data = &pd_z,
	},
	{
		.modalias		= "spi_adis16255",		// Gyro 2
		.max_speed_hz	= 1000000,
		.chip_select	= 2,
		.mode			= SPI_MODE_3,
		.platform_data = &pd_x,
	},
	{
		.modalias		= "spi_adis16255",		// Gyro 3
		.max_speed_hz	= 1000000,
		.chip_select	= 3,
		.mode			= SPI_MODE_3,
		.platform_data = &pd_y,
	},
	{
		.modalias       = "spi_xen1200",		// EMF 1
		.max_speed_hz	= 400000,
		.chip_select	= 4,
		.mode           = SPI_MODE_0,
                .platform_data  = &(struct xen1200_init_data) {
                    .axis   = 'z',
                    .irq    = GPIO_PIN_PC(25),
                },
	},
	{
		.modalias       = "spi_xen1200",		// EMF 2
		.max_speed_hz	= 400000,
		.chip_select	= 5,
		.mode           = SPI_MODE_0,
                .platform_data  = &(struct xen1200_init_data) {
                    .axis   = 'y',
                    .irq    = GPIO_PIN_PC(23),
                },
	},
	{
		.modalias       = "spi_xen1200",		// EMF 3
		.max_speed_hz	= 400000,
		.chip_select	= 6,
		.mode           = SPI_MODE_0,
                .platform_data  = &(struct xen1200_init_data) {
                    .axis   = 'x',
                    .irq    = GPIO_PIN_PB(15),
                },
	},
};


/*
 * 1-Wire Bus Master Platform Data
 */
static struct w1_gpio_platform_data w1_gpio_pdata = {
	.pin 			= GPIO_PIN_PC(29),
	.is_open_drain 	= 1
};

static struct platform_device w1_device = {
	.name		= "w1-gpio",
	.id			= -1,
	.dev.platform_data	= &w1_gpio_pdata,
};

/*
 * I2C/TWI Platform Data
 */
static struct i2c_gpio_platform_data i2c_gpio_data = {
    .sda_pin            = GPIO_PIN_PA(6),
    .scl_pin            = GPIO_PIN_PA(7),
    .sda_is_open_drain  = 1,
    .scl_is_open_drain  = 1,
    .udelay             = 2, /* close to 100 kHz */
};

static struct platform_device i2c_gpio_device = {
    .name               = "i2c-gpio",
    .id                 = 0,
    .dev                = {
            .platform_data = &i2c_gpio_data,
    },
};

static struct i2c_board_info __initdata i2c_info[] = {
   {
      I2C_BOARD_INFO("bmp085", 0x77),
   },
   {
      I2C_BOARD_INFO("tsl2550", 0x29),
   },
};

/*
 * Multimedia Card / SD-Card Interface
 */
static struct mci_platform_data __initdata mci0_data = {
	.slot[0] = {
		.detect_pin	= GPIO_PIN_PA(19),
		.wp_pin		= GPIO_PIN_NONE,
		.bus_width	= 4,
	}
};


/*
 * The next two functions should go away as the boot loader is
 * supposed to initialize the macb address registers with a valid
 * ethernet address. But we need to keep it around for a while until
 * we can be reasonably sure the boot loader does this.
 *
 * The phy_id is ignored as the driver will probe for it.
 */
static int __init parse_tag_ethernet(struct tag *tag)
{
	int i;

	i = tag->u.ethernet.mac_index;
	if (i < ARRAY_SIZE(hw_addr))
		memcpy(hw_addr[i].addr, tag->u.ethernet.hw_address,
		       sizeof(hw_addr[i].addr));

	return 0;
}
__tagtable(ATAG_ETHERNET, parse_tag_ethernet);

static void __init set_hw_addr(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	const u8 *addr;
	void __iomem *regs;
	struct clk *pclk;

	if (!res)
		return;
	if (pdev->id >= ARRAY_SIZE(hw_addr))
		return;

	addr = hw_addr[pdev->id].addr;
	if (!is_valid_ether_addr(addr))
		return;

	/*
	 * Since this is board-specific code, we'll cheat and use the
	 * physical address directly as we happen to know that it's
	 * the same as the virtual address.
	 */
	regs = (void __iomem __force *)res->start;
	pclk = clk_get(&pdev->dev, "pclk");
	if (!pclk)
		return;

	clk_enable(pclk);
	__raw_writel((addr[3] << 24) | (addr[2] << 16)
		     | (addr[1] << 8) | addr[0], regs + 0x98);
	__raw_writel((addr[5] << 8) | addr[4], regs + 0x9c);
	clk_disable(pclk);
	clk_put(pclk);
}

void __init setup_board(void)
{
	at32_map_usart(0, 0, 0);		/* map USART 0 to /dev/ttyS0 */
	at32_map_usart(1, 1, 0);		/* map USART 1 to /dev/ttyS1 */
	at32_setup_serial_console(0);
}

static const struct gpio_led wismit_leds[] = {
	{
		.name = "led_alive",
		.gpio = GPIO_PIN_PD(1),
		.active_low = 1,
		.default_trigger = "heartbeat",
	},
	{
		.name = "led_system",
		.gpio = GPIO_PIN_PD(0),
		.active_low = 1,
	},
	{
		.name = "led_error",
		.gpio = GPIO_PIN_PD(2),
		.active_low = 1,
	},
   {
      .name = "pwr_ins",
      .gpio = GPIO_PIN_PA(16),
      //.active_low = 1,
      .active_low = 0,
   },
   {
      .name = "pwr_hf",
      .gpio = GPIO_PIN_PA(24),
      .active_low = 1,
   },
   {
      .name = "reset_nano",
      .gpio = GPIO_PIN_PB(16),
      .active_low = 1,
   },
   {
      .name = "pwr_phy",
      .gpio = GPIO_PIN_PB(25),
      .active_low = 0,
   },
   {
      .name = "pwr_system",
      .gpio = GPIO_PIN_PC(30),
      .active_low = 1,
   },
   {
      .name = "pwr_speaker",
      .gpio = GPIO_PIN_PC(19),
      //.active_low = 1,
      .active_low = 0,
   },
   {
      .name = "pwr_rfid",
      .gpio = GPIO_PIN_PC(22),
      //.active_low = 1,
      .active_low = 0,
   },
/*
   {
	   .name = "pwr_lcd",
	   .gpio = GPIO_PIN_PA(21),
	   .active_low = 1,
   },
*/
   {
	   .name = "beep",
	   .gpio = GPIO_PIN_PD(11),
	   .active_low = 0,
   },
   {
	   .name = "set_ant0",
	   .gpio = GPIO_PIN_PD(16),
	   .active_low = 0,
   },
};

static const struct gpio_led_platform_data wismit_led_data = {
	.num_leds =	ARRAY_SIZE(wismit_leds),
	.leds =		(void *) wismit_leds,
};

static struct platform_device wismit_gpio_leds = {
	.name =		"leds-gpio",
	.id =		-1,
	.dev = {
		.platform_data = (void *) &wismit_led_data,
	}
};


/*
 * Create generic clock output for barometer
 */
static void __init gclk0_init(void)
{
	struct clk *gclk;
	unsigned long clkrate;
	int clken, clkset;

	gclk = clk_get(NULL, "gclk0");
	if (IS_ERR(gclk))
	{
		printk("Error: GCLK0 not found!\n");
		return;
	}

	clken = clk_enable(gclk);
	if (clken != 0)
		printk("Clock not enabled: %d\n", clken);

	//at32_select_periph(GPIO_PIN_PA(30), GPIO_PERIPH_A, 0);
	at32_select_periph(GPIO_PIOA_BASE, 1 << 30, GPIO_PERIPH_A, 0);
	clkset = clk_set_rate(gclk, GCLK0_RATE);

	if (clkset != 0)
		printk("Clock not set: %d\n", clkset);

	clkrate = clk_get_rate(gclk);
	printk("GCLK0 set to: %lu Hz\n", clkrate);
}


/*
 * Create generic clock output for earth magnetnic sensors
 */
static void __init gclk1_init(void)
{
	struct clk *gclk;
	unsigned long clkrate;
	int clken, clkset;

	gclk = clk_get(NULL, "gclk1");
	if (IS_ERR(gclk))
	{
		printk("Error: GCLK1 not found!\n");
		return;
	}

	clken = clk_enable(gclk);
	if (clken != 0)
		printk("Clock not enabled: %d\n", clken);

	//at32_select_periph(GPIO_PIN_PA(31), GPIO_PERIPH_A, 0);
	at32_select_periph(GPIO_PIOA_BASE, 1 << 31, GPIO_PERIPH_A, 0);
	clkset = clk_set_rate(gclk, GCLK1_RATE);

	if (clkset != 0)
		printk("Clock not set: %d\n", clkset);

	clkrate = clk_get_rate(gclk);
	printk("GCLK1 set to: %lu Hz\n", clkrate);
}

/* **************************************************** */

static struct gpio_keys_button wismit_button[] =
{
	{	.desc = "SW0", .gpio = GPIO_PIN_PD(6), .active_low = 1, .code = KEY_0, .type = EV_KEY,	},
	{	.desc = "SW1", .gpio = GPIO_PIN_PD(7), .active_low = 1, .code = KEY_1, .type = EV_KEY,	},
	{	.desc = "SW2", .gpio = GPIO_PIN_PD(8), .active_low = 1, .code = KEY_2, .type = EV_KEY,	},
	{	.desc = "SW3", .gpio = GPIO_PIN_PD(9), .active_low = 1, .code = KEY_3, .type = EV_KEY,	},
};

static struct gpio_keys_platform_data wismit_button_data =
{
	.buttons = wismit_button,
	.nbuttons = ARRAY_SIZE(wismit_button),
};

static struct platform_device gpio_keys_dev = {
	.name = "gpio-keys",
	.id = 0,
	.dev = { .platform_data = &wismit_button_data, }
};

static void __init setup_gpio_keys(void)
{
	int i;
	for(i=0; i< ARRAY_SIZE(wismit_button); i++)
	{
		at32_select_gpio(wismit_button[i].gpio, AT32_GPIOF_PULLUP|AT32_GPIOF_DEGLITCH);// 0);
	}

	platform_device_register(&gpio_keys_dev);
}

/* **************************************************** */

static int __init wismit_init(void)
{
	unsigned	i;

	//at32_add_system_devices();


	at32_add_device_usart(0);
	at32_add_device_usart(1);

	set_hw_addr(at32_add_device_eth(0, &eth_data[0]));

	at32_add_device_spi(0, spi0_board_info, ARRAY_SIZE(spi0_board_info));
	at32_add_device_spi(1, spi1_board_info, ARRAY_SIZE(spi1_board_info));
	at32_add_device_mci(0, &mci0_data);
	at32_add_device_usba(0, NULL);

        for (i = 0; i < ARRAY_SIZE(wismit_leds); i++) {
            // pwr_system must be high -- reset_nano must be high???
            if(wismit_leds[i].gpio == GPIO_PIN_PC(30) || wismit_leds[i].gpio == GPIO_PIN_PB(16))
                at32_select_gpio(wismit_leds[i].gpio, AT32_GPIOF_OUTPUT | AT32_GPIOF_HIGH);
            else
                at32_select_gpio(wismit_leds[i].gpio, AT32_GPIOF_OUTPUT );
        }
	platform_device_register(&wismit_gpio_leds);

	//at32_add_device_twi(0, NULL, 0); // i2c is done using the gpio-bitbang-driver

	at32_select_gpio(w1_gpio_pdata.pin, AT32_GPIOF_OUTPUT | AT32_GPIOF_MULTIDRV);
	platform_device_register(&w1_device);

	at32_add_device_pwm((1 << 2)); // register PWM - gpio for LCD
	at32_select_gpio(GPIO_PIN_PD(10), AT32_GPIOF_OUTPUT); //LCD_RS

	// set gpio keys sw0 to sw3 to event KEY_0 to KEY_3
	setup_gpio_keys();

   // clocks for EMF and Barometer
	gclk0_init();
	gclk1_init();

	at32_select_gpio(GPIO_PIN_PA(29), AT32_GPIOF_OUTPUT);	// Demux enable/disable
	at32_select_gpio(GPIO_PIN_PB(26), 0); // hf_irq as input
	at32_select_gpio(GPIO_PIN_PB(17), 0); // irq_acc as input

   /* all these i2c/smbus pins should have external pullups for
    * open-drain sharing among all I2C devices.  SDA and SCL do;
    *
    * It IS possible that you have to assemble the pullups on the
    * PCB first!
    */
   at32_select_gpio(i2c_gpio_data.sda_pin, AT32_GPIOF_MULTIDRV | AT32_GPIOF_OUTPUT | AT32_GPIOF_HIGH);
   at32_select_gpio(i2c_gpio_data.scl_pin, AT32_GPIOF_MULTIDRV | AT32_GPIOF_OUTPUT | AT32_GPIOF_HIGH);

   platform_device_register(&i2c_gpio_device);
   i2c_register_board_info(0, i2c_info, ARRAY_SIZE(i2c_info));

	return 0;
}
postcore_initcall(wismit_init);

