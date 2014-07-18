/*
 * I2C client/driver for the Maxim/Dallas DS2745 Low-Cost Battery Monitor
 *
 * Copyright (C) 2014 Matthias Brugger <matthias.bgg@gmail.com>
 *
 * DS2745 added by Matthias Brugger <matthias.bgg@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/swab.h>
#include <linux/i2c.h>
#include <linux/idr.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/ds2782_battery.h>

#define DS2745_REG_VOLT_MSB	0x0c
#define DS2745_REG_TEMP_MSB	0x0a
#define DS2745_REG_CURRENT_MSB	0x0e

#define DS2745_CURRENT_UNITS	15625

struct ds2745_info;

#define to_ds2745_info(x) container_of(x, struct ds2745_info, battery)

struct ds2745_info {
	struct i2c_client	*client;
	struct power_supply	battery;
	int			id;
	int                     rsns;
};

static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_lock);

static inline int ds2745_read_reg16(struct ds2745_info *info, int reg_msb,
				    s16 *val)
{
	int ret;

	ret = i2c_smbus_read_word_data(info->client, reg_msb);
	if (ret < 0) {
		dev_err(&info->client->dev, "register read failed\n");
		return ret;
	}

	*val = swab16(ret);
	return 0;
}

static int ds2745_get_temp(struct ds2745_info *info, int *temp)
{
	s16 raw;
	int err;

	/*
	 * Temperature is measured in units of 0.125 degrees celcius, the
	 * power_supply class measures temperature in tenths of degrees
	 * celsius. The temperature value is stored as a 10 bit number, plus
	 * sign in the upper bits of a 16 bit register.
	 */
	err = ds2745_read_reg16(info, DS2745_REG_TEMP_MSB, &raw);
	if (err)
		return err;
	*temp = ((raw / 32) * 125) / 100;
	return 0;
}

static int ds2745_get_current(struct ds2745_info *info, int *current_uA)
{
	int err;
	s16 raw;

	err = ds2745_read_reg16(info, DS2745_REG_CURRENT_MSB, &raw);
	if (err)
		return err;
	*current_uA = raw * ((DS2745_CURRENT_UNITS / 10000) / info->rsns);
	return 0;
}

static int ds2745_get_voltage(struct ds2745_info *info, int *voltage_uV)
{
	s16 raw;
	int err;

	/*
	 * Voltage is measured in units of 4.88mV. The voltage is stored as
	 * a 10-bit number plus sign, in the upper bits of a 16-bit register
	 */
	err = ds2745_read_reg16(info, DS2745_REG_VOLT_MSB, &raw);
	if (err)
		return err;
	*voltage_uV = (raw / 32) * 4880;
	return 0;
}

static int ds2745_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct ds2745_info *info = to_ds2745_info(psy);
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = ds2745_get_voltage(info, &val->intval);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = ds2745_get_current(info, &val->intval);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		ret = ds2745_get_temp(info, &val->intval);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static enum power_supply_property ds2745_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static void ds2745_power_supply_init(struct power_supply *battery)
{
	battery->type			= POWER_SUPPLY_TYPE_MAINS;
	battery->properties		= ds2745_battery_props;
	battery->num_properties		= ARRAY_SIZE(ds2745_battery_props);
	battery->get_property		= ds2745_battery_get_property;
}

static int ds2745_battery_remove(struct i2c_client *client)
{
	struct ds2745_info *info = i2c_get_clientdata(client);

	power_supply_unregister(&info->battery);
	kfree(info->battery.name);

	mutex_lock(&battery_lock);
	idr_remove(&battery_id, info->id);
	mutex_unlock(&battery_lock);

	kfree(info);
	return 0;
}

static int ds2745_battery_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct ds278x_platform_data *pdata = client->dev.platform_data;
	struct ds2745_info *info;
	int ret;
	int num;

dev_err(&client->dev, "ds2745 loaded and probed\n");
printk(KERN_INFO"ds2745 probing...\n");
	/*
	 * ds2745 should have the sense resistor value set
	 * in the platform data
	 */
	if (!pdata) {
		dev_err(&client->dev, "missing platform data for ds2745\n");
		return -EINVAL;
	}

	/* Get an ID for this battery */
	mutex_lock(&battery_lock);
	ret = idr_alloc(&battery_id, client, 0, 0, GFP_KERNEL);
	mutex_unlock(&battery_lock);
	if (ret < 0)
		goto fail_id;
	num = ret;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		goto fail_info;
	}

	info->battery.name = kasprintf(GFP_KERNEL, "%s-%d", client->name, num);
	if (!info->battery.name) {
		ret = -ENOMEM;
		goto fail_name;
	}


	i2c_set_clientdata(client, info);
	info->client = client;
	info->id = num;
	info->rsns = pdata->rsns;

	ds2745_power_supply_init(&info->battery);

	ret = power_supply_register(&client->dev, &info->battery);
	if (ret) {
		dev_err(&client->dev, "failed to register battery\n");
		goto fail_register;
	}

	return 0;

fail_register:
	kfree(info->battery.name);
fail_name:
	kfree(info);
fail_info:
	mutex_lock(&battery_lock);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_lock);
fail_id:
	return ret;
}

static const struct i2c_device_id ds2745_id[] = {
	{"ds2745", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ds2745_id);

static struct i2c_driver ds2745_battery_driver = {
	.driver 	= {
		.name	= "battery-monitor-ds2745",
		.owner = THIS_MODULE,
	},
	.probe		= ds2745_battery_probe,
	.remove		= ds2745_battery_remove,
	.id_table	= ds2745_id,
};
module_i2c_driver(ds2745_battery_driver);
/*
static int __init ds2745_init_module(void)
{
	printk(KERN_INFO"ds2745 loading...\n");

	return i2c_add_driver(&ds2745_battery_driver);
}

static void __exit ds2745_exit_module(void)
{
	i2c_del_driver(&ds2745_battery_driver);
}
module_init(ds2745_init_module);
module_exit(ds2745_exit_module);
*/
MODULE_AUTHOR("Matthias Brugger");
MODULE_DESCRIPTION("Maxim/Dallas DS2745 Low-Cost Battery Monitor");
MODULE_LICENSE("GPL");
