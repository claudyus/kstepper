/*
 *		Stepper motor driver over gpio with pwm support
 *
 *		Copyright (R) 2010 Claudio Mignanti - <c.mignanti@gmail.com>
 *
 *		Based on PWM based led control by Luotao Fu @ Pengutronix (l.fu@pengutronix.de)
 *		Based on ADC Atmel driver improvement by Antonio Galea
 *
 *		This program is free software; you can redistribute it and/or modify
 *		it under the terms of the GNU General Public License as published by
 *		the Free Software Foundation.
 *
 * $Id$
 * ---------------------------------------------------------------------------
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/pwm.h>

#include "motor_pwm.h"
#include "motor.h"

static dev_t motor_devno = 0;

/* ci va? mbho
struct platform_device motor_device = {
	.name					= "motor",
	.id						= -1,
	.dev.release	 = at91_adc_device_release,
}; */

static void motor_pwm_set(struct motor_classdev *motor_cdev,
	enum motor_brightness brightness)		//TODO redo
{
	struct motor_pwm_data *motor_dat =
		container_of(motor_cdev, struct motor_pwm_data, cdev);
	unsigned int max = motor_dat->cdev.max_brightness;
	unsigned int period =		motor_dat->period;

	if (brightness == 0) {
		pwm_config(motor_dat->pwm, 0, period);
		pwm_disable(motor_dat->pwm);
	} else {
		pwm_config(motor_dat->pwm, brightness * period / max, period);
		pwm_enable(motor_dat->pwm);
	}
}

/* IRQ handler */
void irq_steps_handler (struct pwm_channel *ch) {
	
	struct motor_pwm_data *motor_dat =
		container_of(ch, struct motor_pwm_data, mdata);

	motor_dat->mdata->steps = motor_dat->mdata->steps +1;
	if (motor_dat->mdata->steps > motor_dat->mdata->steps_max)
		pwm_disable(ch);
}

/* IOCTL interface */
static int motor_ioctl(
	struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg){

	int retval = 0;

	//da inode a motor_dat->pwm ???

	switch (cmd) {
		case MOTOR_ENABLE:
			if ((int)arg))
				retval = gpio_set_value(motor_data->gpio_enable, 1);
			else
				retval = gpio_set_value(motor_data->gpio_enable, 0);
			break;		
		case MOTOR_LOWPWR:
			if ((int)arg))
				retval = gpio_set_value(motor_data->gpio_lowpwr, 1);
			else
				retval = gpio_set_value(motor_data->gpio_lowpwr, 0);
			break;
		case MOTOR_DIR:
			if ((int)arg))
				retval = gpio_set_value(motor_data->gpio_ccw, 1);
			else
				retval = gpio_set_value(motor_data->gpio_ccw, 0);
			break;
		case MOTOR_PWM_OFF:
			pwm_disable(motor_dat->pwm);
			break;	
		case MOTOR_PWM_ON:
			pwm_enable(motor_dat->pwm);
			break;	
		case MOTOR_PWM_SET:
			//set the pwm period in ns
			retval = pwm_config(motor_dat->pwm, (int)arg/2, (int)arg );
			break;	
		case MOTOR_STEPS_RESET:
			motor_dat->steps = 0;
			break;
		case MOTOR_STEPS_MAX:
			motor_dat->steps_max = (int) arg;
			break;
		case MOTOR_STEPS_ENABLE:
			if ((int)arg)
				retval = pwm_channel_handler (motor_dat->pwm, &irq_steps_handler);
			else
				retval = pwm_channel_handler (motor_dat->pwm, NULL);
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
	//struct motor_pwm *cur_motor;
	struct motor_pwm_data *motor_data, *motor_dat;
	int i, ret = 0;

	if (!pdata)
		return -EBUSY;

	motor_data = kzalloc(sizeof(struct motor_pwm_data) * pdata->num_motor,
				GFP_KERNEL);
	if (!motor_data)
		return -ENOMEM;

	for (i = 0; i < pdata->num_motor; i++) {
		cur_motor = &pdata->motor[i];
		motor_dat = &motor_data[i];

		motor_dat->pwm = pwm_request(cur_motor->pwm_step,
				cur_motor->name);
		if (IS_ERR(motor_dat->pwm)) {
			dev_err(&pdev->dev, "unable to request PWM %d\n",
					cur_motor->pwm_step);
			goto err_pwm;
		}

		if (

		motor_dat->mdata->gpio_enable = gpio_request(cur_motor->gpio_enable, "motor-enable");
		motor_dat->mdata->gpio_ccw = gpio_request(cur_motor->gpio_ccw, "motor-ccw");
		if (!cur_motor->gpio_lowpwr)
			motor_dat->gpio_lowpwr = gpio_request(cur_motor->gpio_lowpwr, "motor-lowpwr");

		motor_dat->cdev.name = cur_motor->name;
		motor_dat->period = cur_motor->pwm_period_ns;
		motor_dat->cdev.brightness_set = motor_pwm_set;	//chiama la funzione di trigger

		retval = gpio_set_value(motor_dat->gpio_enable, 0);
		retval = gpio_set_value(motor_dat->gpio_ccw, 0);
		retval = gpio_set_value(motor_data->gpio_lowpwr, 0)
		motor_dat->steps=0;
/* tutta sta roba se può buttà
		//???motor_dat->cdev.brightness = motor_OFF;// ho camiato leds con motor in tutto il documento, tenere a mente
		/???motor_dat->cdev.max_brightness = cur_motor->max_brightness;
		//motor_dat->cdev.flags |= motor_CORE_SUSPENDRESUME;

/*		ret = motor_classdev_register(&pdev->dev, &motor_dat->cdev);
		if (ret < 0) {
			pwm_free(motor_dat->pwm);
			goto err;
		}
*/

		/* alloc a new device number (major: dynamic, minor: 0) */
		status = alloc_chrdev_region(&motor_devno, 0, 1, at91_adc_device.name);
		//if(status){ goto err; }

		/* create a new char device */
		motor_cdev = cdev_alloc();
		if(motor_cdev == NULL){
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

		device_create(motor_class, NULL, motor_devno, NULL, motor_dat->cdev.name);
		printk(KERN_INFO "motor: %s registerd on major: %u; minor: %u\n", \
				motor_dat->cdev.name, MAJOR(motor_devno), MINOR(motor_devno));
	}

	platform_set_drvdata(pdev, motor_data);

	return 0;

err_pwm:
	if (i > 0) {
		for (i = i - 1; i >= 0; i--) {
			motor_classdev_unregister(&motor_data[i].cdev);
			pwm_free(motor_data[i].pwm);
		}
	}

//TODO err_gpio:

	kfree(motor_data);

	return ret;
}

//TODO redo
static int __devexit motor_pwm_remove(struct platform_device *pdev)
{
	int i;
	struct motor_pwm_platform_data *pdata = pdev->dev.platform_data;
	struct motor_pwm_data *motor_data;

	motor_data = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_motor; i++) {
		motor_classdev_unregister(&motor_data[i].cdev);
		pwm_free(motor_data[i].pwm);
	}

	kfree(motor_data);

	return 0;
}

static struct platform_driver motor_pwm_driver = {	//OK
	.probe		= motor_pwm_probe,
	.remove		= __devexit_p(motor_pwm_remove),
	.driver		= {
		.name	= "motor_pwm",
		.owner	= THIS_MODULE,
	},
};

static int __init motor_pwm_init(void)		//OK
{
	return platform_driver_register(&motor_pwm_driver);
}

static void __exit motor_pwm_exit(void)		//OK
{
	platform_driver_unregister(&motor_pwm_driver);
}

module_init(motor_pwm_init);
module_exit(motor_pwm_exit);

MODULE_AUTHOR("Claudio Mignanti <c.mignanti@gmail.com>");
MODULE_DESCRIPTION("Stepper motor driver over gpio with pwm support");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:motor-pwm");
