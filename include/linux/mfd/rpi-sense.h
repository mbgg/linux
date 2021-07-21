#ifndef __LINUX_MFD_RPI_SENSE_H
#define __LINUX_MFD_RPI_SENSE_H

/*
 * Register values.
 */
#define RPI_SENSE_FB			0x00
#define RPI_SENSE_WAI			0xF0
#define RPI_SENSE_VER			0xF1
#define RPI_SENSE_KEYS			0xF2
#define RPI_SENSE_EE_WP			0xF3

#define RPI_SENSE_ID			's'

struct rpi_sense_dev {
	struct device *dev;
	struct i2c_client *i2c_client;

	s32 (*read_reg)(struct rpi_sense_dev *rpisense, int reg);
	int (*write_block)(struct rpi_sense_dev *rpisense,
					const char *buf, int count);

};

#endif

