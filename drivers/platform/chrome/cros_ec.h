/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ChromeOS Embedded Controller core interface.
 *
 * Copyright (C) 2012 Google, Inc
 */

#ifndef __CROS_EC_H
#define __CROS_EC_H

/*
 * The EC is unresponsive for a time after a reboot command.  Add a
 * simple delay to make sure that the bus stays locked.
 */
#define EC_REBOOT_DELAY_MS	50

int cros_ec_register(struct cros_ec_device *ec_dev);
int cros_ec_unregister(struct cros_ec_device *ec_dev);

int cros_ec_suspend(struct cros_ec_device *ec_dev);
int cros_ec_resume(struct cros_ec_device *ec_dev);

bool cros_ec_handle_event(struct cros_ec_device *ec_dev);

#endif /* __CROS_EC_H */
