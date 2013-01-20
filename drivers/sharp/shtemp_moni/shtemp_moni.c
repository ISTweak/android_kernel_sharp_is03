/* drivers/sharp/shtimer/shtemp_moni.c
 *
 * Copyright (C) 2010 Sharp Corporation
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
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <sharp/shterm_k.h>

#define EVENT_NAME_MAX 24

typedef struct {
    const char *name;
    struct kobject *kobj;
    struct kobj_type *ktype;
} shtemp_moni_data;

static struct kobject shtemp_moni_kobj;
static struct kset *shtemp_moni_kset;

static void shtemp_moni_release( struct kobject *kobj );
static ssize_t shtemp_moni_info_show( struct kobject *kobj, struct attribute *attr, char *buff );
static ssize_t shtemp_moni_info_store( struct kobject *kobj, struct attribute *attr, const char *buff, size_t len );

static struct sysfs_ops shtemp_moni_sysfs_ops = {
    .show  = shtemp_moni_info_show,
    .store = shtemp_moni_info_store,
};

static struct kobj_attribute shtemp_moni_attribute =
    __ATTR( high, 0600, NULL, NULL );

static struct attribute *shtemp_moni_attrs[] = {
    &shtemp_moni_attribute.attr,
    NULL,
};

static struct kobj_type shtemp_moni_ktype = {
    .release = shtemp_moni_release,
    .sysfs_ops = &shtemp_moni_sysfs_ops,
    .default_attrs = shtemp_moni_attrs,
};

static shtemp_moni_data data = {
    .name  = "info",
    .kobj  = &shtemp_moni_kobj,
    .ktype = &shtemp_moni_ktype,
};

static void shtemp_moni_release( struct kobject *kobj )
{
    kfree( kobj );
}

static ssize_t shtemp_moni_info_show( struct kobject *kobj, struct attribute *attr, char *buff )
{
    return 0;
}

static ssize_t shtemp_moni_info_store( struct kobject *kobj, struct attribute *attr, const char *buff, size_t len )
{
    char *envp[8];
    char e_num[EVENT_NAME_MAX];
    int idx = 0;
    int ret = -1;

    memset( e_num, 0x00, sizeof(e_num) );
    snprintf( e_num, EVENT_NAME_MAX - 1, "HIGH_TEMP_DETECT=1" );
    envp[idx++] = e_num;

    envp[idx++] = NULL;
    ret = kobject_uevent_env( data.kobj, KOBJ_CHANGE, envp );

    return ret;
}

static int __init shtemp_moni_kobject_init( void )
{
    int ret;

    /* Create a kset with the name of "shtemp_moni" */
    /* located under /sys/kernel/ */
    shtemp_moni_kset = kset_create_and_add( "temp_monitor", NULL, kernel_kobj );
    if( !shtemp_moni_kset ){
        printk( "%s : line %d error\n", __FUNCTION__, __LINE__ );
        return -ENOMEM;
    }

    data.kobj->kset = shtemp_moni_kset;
    ret = kobject_init_and_add( data.kobj, data.ktype, NULL, "%s", data.name );
    if( ret ){
        printk( "%s : line %d error\n", __FUNCTION__, __LINE__ );
        kobject_put( data.kobj );
    }
    return ret;
}

static void __exit shtemp_moni_kobject_exit( void )
{
    kset_unregister( shtemp_moni_kset );
}

module_init(shtemp_moni_kobject_init);
module_exit(shtemp_moni_kobject_exit);
