/*
 *
 * Copyright (c) 2021 SUSE
 *
 * Author: Matthias Brugger <mbrugger@suse.com>
 *
  */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mfd/core.h>
#include <linux/mfd/rpi-sense.h>

static const struct mfd_cell rpi_sense_devs[] = {
	{
		.name = "rpi-sense-fb",
	},
	{
		.name = "rpi-sense-js",
	},
};


static int rpi_sense_reg_read(struct rpi_sense_dev *rpisense, int reg)
{
	int ret = i2c_smbus_read_byte_data(rpisense->i2c_client, reg);

	if (ret < 0) {
		dev_err(rpisense->dev, "Read from reg %#x failed\n", reg);
		return reg;
	}

	/* Due to the BCM270x I2C clock stretching bug, some values
	 * may have MSB set. Clear it to avoid incorrect values.
	 * */
	return ret & 0x7F;
}

static int rpi_sense_block_write(struct rpi_sense_dev *rpisense, const char *buf, int count)
{
	int ret = i2c_master_send(rpisense->i2c_client, buf, count);

	if (ret < 0)
		dev_err(rpisense->dev, "Block write failed\n");
	return ret;
}

static int rpi_sense_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct rpi_sense_dev *rpisense;
	int ret;

printk(KERN_ERR"%s\n", __func__);
	rpisense = devm_kzalloc(&i2c->dev, sizeof(struct rpi_sense_dev),
				GFP_KERNEL);
	if (rpisense == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, rpisense);
	rpisense->dev = &i2c->dev;
	rpisense->i2c_client = i2c;
	rpisense->read_reg = rpi_sense_reg_read;
	rpisense->write_block = rpi_sense_block_write;

	ret = rpi_sense_reg_read(rpisense, RPI_SENSE_WAI);
	if (ret < 0)
		return ret;
	else if (ret != 's')
		return -ENODEV;

	ret = rpi_sense_reg_read(rpisense, RPI_SENSE_VER);
	if (ret < 0)
		return ret;

	dev_info(rpisense->dev,
		 "Raspberry Pi Sense HAT firmware version %i\n", ret);

	return devm_mfd_add_devices(rpisense->dev, -1, rpi_sense_devs,
				    ARRAY_SIZE(rpi_sense_devs), NULL, 0, NULL);
}

static const struct i2c_device_id rpi_sense_i2c_id[] = {
	{ "rpi_sense", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rpi_sense_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id rpi_sense_of_match[] = {
	{.compatible = "rpi,rpi-sense", },
	{},
};
MODULE_DEVICE_TABLE(of, rpi_sense_of_match);
#endif

static struct i2c_driver rpi_sense_driver = {
	.probe = rpi_sense_probe,
	.driver = {
		   .name = "rpi-sense",
		   .of_match_table = of_match_ptr(rpi_sense_of_match),
	},
	.id_table = rpi_sense_i2c_id,
};
module_i2c_driver(rpi_sense_driver);

MODULE_DESCRIPTION("RPi Sense multi-function driver");
MODULE_LICENSE("GPL");
