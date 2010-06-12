/*
 *		Stepper motor driver over gpio with pwm support
 *
 *		Copyright (R) 2010 Claudio Mignanti - <c.mignanti@gmail.com>
 *
 *		This program is free software; you can redistribute it and/or modify
 *		it under the terms of the GNU General Public License as published by
 *		the Free Software Foundation.
 *
 * $Id$
 * ---------------------------------------------------------------------------
*/

#include <linux/init.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/atmel_pwm.h>

#include <asm/ioctl.h>

#include "motor_pwm.h"
#include "motor.h"


/* module var*/
struct class *motor_class;
struct pwm_channel *pwmc;
int steps_max, steps;
static dev_t motor_devno = 0;

/* module parameters */
char *motor_name = NULL;
int g_enable = 0 , g_dir = 0, pwm_step = 0, g_step = 0, g_lpwr = 0;


module_param(motor_name, charp, 0);
MODULE_PARM_DESC(motor_name, "The device name of the motor.");
module_param(g_enable, int, 0);
MODULE_PARM_DESC(g_enable, "The gpio pin connected to stepper motor enable.");
module_param(g_dir, int, 0);
MODULE_PARM_DESC(g_dir, "The gpio pin connected to stepper motor direction pin.");
module_param(pwm_step, int, 0);
MODULE_PARM_DESC(pwm_step, "The pwm index of channel, if 0 g_step should be defined.");
module_param(g_step, int, 0);
MODULE_PARM_DESC(g_step, "The gpio pin connected to stepper motor step pin.");
module_param(g_lpwr, int, 0);
MODULE_PARM_DESC(g_lpwr, "The gpio pin connected to stepper motor lowpower pin.");

static void motor_pwm_set(struct pwm_channel *pwmc, unsigned long up, unsigned long period ) {

	//pwm_config(motor_data->pwmc, up, period);
	pwm_channel_enable(pwmc);
}

/* IRQ handler */
void irq_steps_handler (struct pwm_channel *ch) {

	steps++;
	if (steps >= steps_max)
		pwm_channel_disable(ch);
}

/* IOCTL interface */
static int motor_ioctl (struct inode *in, struct file *fl, unsigned int cmd, unsigned long arg) {

	int retval = 0;

	switch (cmd) {
		case MOTOR_ENABLE:
			if ((int)arg)
				gpio_set_value (g_enable, 1);
			else
				gpio_set_value (g_enable , 0);
			break;		

		case MOTOR_LOWPWR:
			if ((int)arg)
				gpio_set_value (g_lpwr, 1);
			else
				gpio_set_value (g_lpwr, 0);
			break;

		case MOTOR_DIR:
			if ((int)arg)
				gpio_set_value (g_dir, 1);
			else
				gpio_set_value (g_dir, 0);
			break;

		case MOTOR_PWM_OFF:
			pwm_channel_disable (pwmc);
			break;	

		case MOTOR_PWM_ON:
			pwm_channel_enable (pwmc);
			break;	

		case MOTOR_PWM_SET:
			//set the pwm period in ns
			motor_pwm_set (pwmc, (int)arg<<2, (int)arg );
			break;

		case MOTOR_STEPS_RESET:
			steps = 0;
			break;

		case MOTOR_STEPS_MAX:
			steps_max = (int) arg;
			break;

		case MOTOR_STEPS_ENABLE:
			if ((int)arg)
				retval = pwm_channel_handler (pwmc, &irq_steps_handler);
			else
				retval = pwm_channel_handler (pwmc, NULL);
			break;

		default:
			retval = -EINVAL;
	}

	return retval;
}

struct file_operations motor_fops = {
	.owner = THIS_MODULE,
	.ioctl = motor_ioctl,
};

static int __init motor_init(void)	
{
	int status;
	struct cdev *motor_cdev;

	/* sanity check */
	if ( !(motor_name[0] & g_enable & g_dir & (pwm_step | g_step))) {
		printk(KERN_INFO "stepper: missing parameters, exit driver.\n");
		goto err_para;
	}

	/* request and set pwm channel and gpio pins */
	pwm_channel_alloc(pwm_step, pwmc);
	if (IS_ERR(pwmc)) {
		goto err_pwm;
	}

	if ( !gpio_request(g_enable, "motor-enable")) {
		goto err_gpioenable;
	}
	if ( !gpio_request(g_dir, "motor-ccw")) {
		goto err_gpiodir;
	}
	if (!g_lpwr) {
		if ( !gpio_request(g_lpwr, "motor-lowpwr")) {
			goto err_gpiolwr;
		}
		gpio_set_value(g_lpwr, 0);
	}

	gpio_set_value(g_enable, 0);
	gpio_set_value(g_dir, 0);

	/* set the home */
	steps=0;

	/* alloc a new device number (major: dynamic, minor: 0) */
	status = alloc_chrdev_region(&motor_devno, 0, 1, "motor-pwm");

	/* create a new char device  */
	motor_cdev = cdev_alloc();
	if(motor_cdev == NULL) {
		status=-ENOMEM;
		goto err_dev;
	}

	motor_cdev->owner = THIS_MODULE;
	motor_cdev->ops = &motor_fops;
	status = cdev_add(motor_cdev, motor_devno, 1);
	if(status){
		goto err_dev;
	}

	/* register the class */
	motor_class = class_create(THIS_MODULE, "motor_class");
	if(IS_ERR(motor_class)){
		goto err_dev;
	}

	device_create(motor_class, NULL, motor_devno, NULL, motor_name);
	printk(KERN_INFO "motor: %s registerd on major: %u; minor: %u\n", \
		motor_name, MAJOR(motor_devno), MINOR(motor_devno));

	return 0;

err_dev:
err_gpiolwr:
err_gpiodir:
err_gpioenable:
err_pwm:
err_para:
	printk(KERN_INFO "stepper: Error management not implemented. \
		Please reboot your board\n");
	return -1;
}

static void __exit motor_exit(void)
{

}

module_init(motor_init);
module_exit(motor_exit);

MODULE_AUTHOR("Claudio Mignanti <c.mignanti@gmail.com>");
MODULE_DESCRIPTION("Stepper motor driver over gpio with pwm support");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:motor-pwm");

