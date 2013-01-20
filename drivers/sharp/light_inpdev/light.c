/*
 * Copyright (C) 2010 SHARP CORPORATION All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/device.h>

#define LOG_ENABLED

#ifdef LOG_ENABLED
#define FUNC_LOG() printk(KERN_DEBUG "[LIGHT] %s is called\n", __func__)
#define DEBUG_LOG(format, ...)	printk(KERN_DEBUG "[LIGHT] " format "\n", ## __VA_ARGS__)
#else
#define FUNC_LOG()
#define DEBUG_LOG(format, ...)
#endif

static struct input_dev    *i_dev;

static int __devinit light_input_dev_init(void);
static void __devexit light_input_dev_exit(void);


static int __devinit light_input_dev_init(void)
{
    int ret;

    FUNC_LOG();

    // declare input device
    i_dev = input_allocate_device();

    if (!i_dev) {
        ret = -ENOMEM;
        DEBUG_LOG("failed to -Allocate- input device");
        return ret;
    }

    // setup input device
    set_bit(EV_ABS, i_dev->evbit);

    // set default value
    input_set_abs_params(i_dev, ABS_PRESSURE, 0, 255, 0, 0);
        // to handle lux value, use below
        //input_set_abs_params(i_dev, ABS_PRESSURE, 0, 120000, 0, 0);

    // set name
    i_dev->name = "light";

    // register input device
    ret = input_register_device(i_dev);

    if(ret) {
        DEBUG_LOG("failed to -Register- input device");
        input_free_device(i_dev);
        return ret;
    }

    return 0;
}


static void __devexit light_input_dev_exit(void)
{
    FUNC_LOG();

    input_unregister_device(i_dev);
}

module_init(light_input_dev_init);
module_exit(light_input_dev_exit);

MODULE_DESCRIPTION("light sensor input device");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
