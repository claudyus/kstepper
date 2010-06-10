/*
 *		Stepper motor driver over gpio with pwm support
 *
 *		Copyright (R) 2010 Claudio Mignanti - <c.mignanti@gmail.com>
 *
 *		This program is free software; you can redistribute it and/or modify
 *		it under the terms of the GNU General Public License v2 as published by
 *		the Free Software Foundation.
 *
 * $Id$
 * ---------------------------------------------------------------------------
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/atmel_pwm.h>

#include <asm/ioctl.h>

#include "motor_pwm.h"
#include "motor.h"

static dev_t motor_devno = 0;
struct cdev *motor_cdev;
struct motor_pwm *motor_data;

static void motor_pwm_set(struct pwm_channel *pwmc, unsigned long up, unsigned long period ) {

	motor_data->pwm_period = period;
	motor_data->pwm_up = up;

	//pwm_config(motor_data->pwmc, up, period);
	pwm_channel_enable(motor_data->pwmc);
}

/* IRQ handler */
void irq_steps_handler (struct pwm_channel *ch) {

	motor_data->steps = motor_data->steps + 1;
	if (motor_data->steps >= motor_data->steps_max)
		pwm_channel_disable(ch);
}

/* IOCTL interface */
static int motor_ioctl (struct inode *in, struct file *fl, unsigned int cmd, unsigned long arg) {

	int retval = 0;

	switch (cmd) {
		case MOTOR_ENABLE:
			if ((int)arg)
				gpio_set_value (motor_data->gpio_enable, 1);
			else
				gpio_set_value (motor_data->gpio_enable, 0);
			break;		

		case MOTOR_LOWPWR:
			if ((int)arg)
				gpio_set_value (motor_data->gpio_lowpwr, 1);
			else
				gpio_set_value (motor_data->gpio_lowpwr, 0);
			break;

		case MOTOR_DIR:
			if ((int)arg)
				gpio_set_value (motor_data->gpio_ccw, 1);
			else
				gpio_set_value (motor_data->gpio_ccw, 0);
			break;

		case MOTOR_PWM_OFF:
			pwm_channel_disable (motor_data->pwmc);
			break;	

		case MOTOR_PWM_ON:
			pwm_channel_enable (motor_data->pwmc);
			break;	

		case MOTOR_PWM_SET:
			//set the pwm period in ns
			motor_pwm_set (motor_data->pwmc, (int)arg<<2, (int)arg );
			break;

		case MOTOR_STEPS_RESET:
			motor_data->steps = 0;
			break;

		case MOTOR_STEPS_MAX:
			motor_data->steps_max = (int) arg;
			break;

		case MOTOR_STEPS_ENABLE:
			if ((int)arg)
				retval = pwm_channel_handler (motor_data->pwmc, &irq_steps_handler);
			else
				retval = pwm_channel_handler (motor_data->pwmc, NULL);
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

static int motor_pwm_probe(struct platform_device *pdev)
{
	struct motor_pwm_platform_data *pdata = pdev->dev.platform_data;
	struct motor_pwm *motor_data, *motor_dat;
	struct motor_pwm *cur_motor;

	int i, status, ret = 0;

	if (!pdata)
		return -EBUSY;

	motor_data = kzalloc(sizeof(struct motor_pwm) * pdata->num_motor,
				GFP_KERNEL);
	if (!motor_data)
		return -ENOMEM;
	memset ( motor_data, 0, sizeof(struct motor_pwm) * pdata->num_motor);

	for (i = 0; i < pdata->num_motor; i++) {
		cur_motor = &pdata->motor[i];

		pwm_channel_alloc(cur_motor->index, motor_data->pwmc);
		if (IS_ERR(motor_dat->pwmc)) {
			dev_err(&pdev->dev, "unable to request PWM %d\n",
					cur_motor->pwm_step);
			goto err_pwm;
		}

		motor_data->gpio_enable = gpio_request(cur_motor->gpio_enable, "motor-enable");
		motor_data->gpio_ccw = gpio_request(cur_motor->gpio_ccw, "motor-ccw");
		if (!cur_motor->gpio_lowpwr)
			motor_dat->gpio_lowpwr = gpio_request(cur_motor->gpio_lowpwr, "motor-lowpwr");

		motor_data->name = cur_motor->name;

		gpio_set_value(motor_dat->gpio_enable, 0);
		gpio_set_value(motor_dat->gpio_ccw, 0);
		gpio_set_value(motor_data->gpio_lowpwr, 0);
		motor_data->steps=0;

		/* alloc a new device number (major: dynamic, minor: 0) */
		status = alloc_chrdev_region(&motor_devno, 0, 1, "motor-pwm");

		/* create a new char device  */
		motor_cdev = cdev_alloc();
		if(motor_cdev == NULL) {
			status=-ENOMEM;
			goto dev_err;
		}

		motor_cdev->owner = THIS_MODULE;
		motor_cdev->ops = &motor_fops;
		status = cdev_add(motor_cdev, motor_devno, 1);
		if(status){
			goto err;
		}

		/* register the class */
		motor_class = class_create(THIS_MODULE, "motor_class");
		if(IS_ERR(motor_class)){
			status=-EFAULT;
			goto err;
		}

		device_create(motor_class, NULL, motor_devno, NULL, motor_data->name);
		printk(KERN_INFO "motor: %s registerd on major: %u; minor: %u\n", \
				motor_data[1]->name, MAJOR(motor_devno), MINOR(motor_devno));
	}

	platform_set_drvdata(pdev, motor_data);

	return 0;

err_pwm:
	if (i > 0) {
		for (i = i - 1; i >= 0; i--) {
			motor_classdev_unregister(&motor_data[i].cdev);
			pwm_channel_free(motor_data[i].pwm);
		}
	}

//TODO err_gpio:

	kfree(motor_data);

	return ret;
}

static int motor_pwm_remove(struct platform_device *pdev)
{
	int i;
	struct motor_pwm_platform_data *pdata = pdev->dev.platform_data;
	struct motor_pwm_data *motor_data;

	motor_data = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_motor; i++) {
		motor_classdev_unregister(&motor_data[i].cdev);
		//TODO pwm_channel_free(motor_data[i].pwm);
	}

	kfree(motor_data);

	return 0;
}

static struct platform_driver motor_pwm_driver = {
	.probe		= motor_pwm_probe,
	.remove		= motor_pwm_remove,
	.driver		= {
		.name	= "motor_pwm",
		.owner	= THIS_MODULE,
	},
};

static int __init motor_pwm_init(void)	
{
	return platform_driver_register(&motor_pwm_driver);
}

static void __exit motor_pwm_exit(void)
{
	platform_driver_unregister(&motor_pwm_driver);
}

module_init(motor_pwm_init);
module_exit(motor_pwm_exit);

MODULE_AUTHOR("Claudio Mignanti <c.mignanti@gmail.com>");
MODULE_DESCRIPTION("Stepper motor driver over gpio with pwm support");
MODULE_LICENSE("GPLv2");
MODULE_ALIAS("platform:motor-pwm");
