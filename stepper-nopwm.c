/*
 *		Stepper motor driver with pwm emulating over gpio
 *
 *		Copyright (R) 2010-2011 Claudio Mignanti - <c.mignanti@gmail.com>
 *
 *		This program is free software; you can redistribute it and/or modify
 *		it under the terms of the GNU General Public License as published by
 *		the Free Software Foundation.
 *
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
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>

#include <asm/uaccess.h>

#include "motor.h"

#define DRV_NAME	"stepper-drv"
#define DRV_DESC	"Stepper motor driver using gpio and pwm pins"
#define DRV_VERSION	"0.3-emulating"

#define MAX_MOT_NUM 4

/* module var*/
struct class *motor_class;
static dev_t motor_devno = 0;

struct motor_device {
	unsigned long steps_max;
	unsigned long steps;

	int g_enable;
	int g_dir;
	int g_step;
	int g_lpwr;
	int polarity;

	struct hrtimer t;
	int status;
	ktime_t interval;

	struct cdev *motor_cdev;
};

static unsigned int mot0[6] __initdata;
static unsigned int mot1[6] __initdata;
static unsigned int mot2[6] __initdata;
static unsigned int mot3[6] __initdata;

static unsigned int mot_nump[MAX_MOT_NUM] __initdata;

static struct motor_device motor[MAX_MOT_NUM];

#define BUS_PARM_DESC \
	" config -> id,en,dir,step[,lowpwr,polarity]"

module_param_array(mot0, uint, &mot_nump[0], 0);
MODULE_PARM_DESC(mot0, "mot0" BUS_PARM_DESC);
module_param_array(mot1, uint, &mot_nump[1], 0);
MODULE_PARM_DESC(mot1, "mot1" BUS_PARM_DESC);
module_param_array(mot2, uint, &mot_nump[2], 0);
MODULE_PARM_DESC(mot2, "mot2" BUS_PARM_DESC);
module_param_array(mot3, uint, &mot_nump[3], 0);
MODULE_PARM_DESC(mot3, "mot3" BUS_PARM_DESC);

static int motor_pwm_set(unsigned long val, int id) {
	if (val == 0)
		val =1;

	motor[id].interval = ktime_set(0, val * 1000000UL);
	return 0;
}

static enum hrtimer_restart gpio_timeout(struct hrtimer *htime)
{
	struct motor_device *mot = container_of(htime, struct motor_device, t);

	printk(KERN_INFO "stepper: gpio_timeout %d\n",mot->status);

	if (mot->status) {
		gpio_set_value(mot->g_step ,0);
		mot->status = 0;
	} else {
		gpio_set_value(mot->g_step ,1);
		mot->status = 1;
		mot->steps++;
	}

	if (mot->steps >= mot->steps_max) {
		hrtimer_try_to_cancel(&(mot->t));
	} else {
		hrtimer_forward(&(mot->t), ktime_get(), mot->interval);
		return HRTIMER_RESTART;
	}

	return HRTIMER_NORESTART;
}

/* IOCTL interface */
static int motor_ioctl (/* struct inode *inode, */ struct file *file, unsigned int cmd, unsigned long arg){

	int retval = 0, id;
	unsigned long to_end;

	/* Ioctl was recently restructured http://lwn.net/Articles/119652/ */
	struct cdev *p = file->f_dentry->d_inode->i_cdev;
	struct motor_device *mot = container_of(p, struct motor_device, motor_cdev);

	switch (cmd) {
		case MOTOR_ENABLE:
			if ((int)arg)
				gpio_set_value (mot->g_enable, 0);
			else
				gpio_set_value (mot->g_enable, 1);
			break;

		case MOTOR_DIR:
			if ((int)arg)
				gpio_set_value (mot->g_dir, 1);
			else
				gpio_set_value (mot->g_dir, 0);
			break;

		case MOTOR_PWM_ON:
			hrtimer_start(&(mot->t), mot->interval, HRTIMER_MODE_REL);
			break;

		case MOTOR_PWM_OFF:
			hrtimer_cancel(&(mot->t));
			break;

		case MOTOR_PWM_SET:
			//set the pwm period in ms
			motor_pwm_set (arg, id);
			break;

		case MOTOR_RESET:
			mot->steps = 0; /* set the actual position as home */
			break;

		case MOTOR_STEPS:
			mot->steps_max = arg; /* set the steps limit */
			break;

		case MOTOR_START:
			hrtimer_start(&(mot->t), mot->interval, HRTIMER_MODE_REL);
			break;

		case MOTOR_LOWPWR:
			if ((int)arg)
				gpio_set_value (mot->g_lpwr, 1);
			else
				gpio_set_value (mot->g_lpwr, 0);
			break;

		/* return steps_max-step */
		case MOTOR_TO_END:
			to_end = mot->steps_max - mot->steps;
			copy_to_user(&arg, &to_end, sizeof(unsigned long));
			break;

		default:
			retval = -EINVAL;
	}

	return retval;
}

struct file_operations motor_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = motor_ioctl,
};

static int __init motor_add_one(unsigned int id, unsigned int *params)
{
	int status;
	struct cdev *motor_cdev;
//	struct platform_device *pdev;
	//struct gpio_pwm_platform_data pdata;

	if ( mot_nump[id] < 4 ) {
		printk(KERN_INFO "stepper: nothing to register for id: %d.\n", id);
		return 0;
	}

	motor[id].g_enable = params[1];
	motor[id].g_dir = params[2];
	motor[id].g_step = params[3];
	motor[id].g_lpwr = params[4];
	motor[id].polarity = params[5];

	/* sanity check */
	if ( !( motor[id].g_enable && motor[id].g_dir && motor[id].g_step)) {
		printk(KERN_INFO "stepper: missing parameters, exit driver.\n");
		goto err_para;
	}

//	INIT_WORK(&work[id], gpio_hr_work);

	if (gpio_request(motor[id].g_step, "motor-step"))
		return -EINVAL;

	hrtimer_init(&(motor[id].t), CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	motor[id].t.function = &gpio_timeout;
//	mot_map_hrtimer[id] = &t[id];
	motor[id].interval = ktime_set(0, 10000000UL);

	//motor_pwm_set(1, id);

	if ( gpio_request(motor[id].g_enable, "motor-enable") < 0 ) {
		goto err_gpioenable;
	}
	gpio_direction_output(motor[id].g_enable ,0);
	gpio_set_value (motor[id].g_enable, 0);

	if ( gpio_request(motor[id].g_dir, "motor-ccw") < 0) {
		goto err_gpiodir;
	}
	gpio_direction_output(motor[id].g_dir ,0);

	if (motor[id].g_lpwr != 0) {
		if ( gpio_request(motor[id].g_lpwr, "motor-lowpwr") < 0 ) {
			goto err_gpiolwr;
		}
		gpio_direction_output(motor[id].g_lpwr ,0);
	}

	/* set to home */
	motor[id].steps = 0;

	/* alloc a new device number (major: dynamic, minor: 0) */
	status = alloc_chrdev_region(&motor_devno, 0, 1, "motor");

	/* create a new char device  */
	motor[id].motor_cdev = cdev_alloc();
	if(motor[id].motor_cdev == NULL) {
		status=-ENOMEM;
		goto err_dev;
	}

	motor[id].motor_cdev->owner = THIS_MODULE;
	motor[id].motor_cdev->ops = &motor_fops;
	status = cdev_add(motor[id].motor_cdev, motor_devno, 1);
	if(status){
		goto err_dev;
	}

	device_create(motor_class, NULL, motor_devno, NULL, "motor%d", params[0]);
	printk(KERN_INFO "stepper: motor%d registred on major: %u; minor: %u\n", \
		params[0], MAJOR(motor_devno), MINOR(motor_devno));

	return 0;

//err:
	printk(KERN_INFO "stepper: err\n");
err_dev:
	printk(KERN_INFO "stepper: err_dev\n");
err_gpiolwr:
	printk(KERN_INFO "stepper: err_gpiolwr\n");
err_gpiodir:
	printk(KERN_INFO "stepper: err_gpiodir\n");
err_gpioenable:
	printk(KERN_INFO "stepper: err_gpioenable\n");
//err_gpiostep:
	printk(KERN_INFO "stepper: err_gpiostep ");
//err_pwm:
	printk(KERN_INFO "stepper: err_pwm\n");
err_para:
	printk(KERN_INFO "stepper: Error management not yet implemented. \
		Please reboot your board %d\n",motor[id].g_step);
	return -1;
}

static int __init motor_init(void)
{
	int err;
	struct timespec tp;

	printk(KERN_INFO DRV_DESC ", version " DRV_VERSION "\n");

	hrtimer_get_res(CLOCK_MONOTONIC, &tp);
	printk(KERN_INFO "Clock resolution is %ldns\n", tp.tv_nsec);

	/*register the class */
	motor_class = class_create(THIS_MODULE, "motor_class");
	if(IS_ERR(motor_class)){
		goto err;
	}

	err = motor_add_one(0, mot0);
	if (err) goto err;

	err = motor_add_one(1, mot1);
	if (err) goto err;

	err = motor_add_one(2, mot2);
	if (err) goto err;

	err = motor_add_one(3, mot3);
	if (err) goto err;

	return 0;

err:
	class_unregister(motor_class);
	return -1;
}

static void __exit motor_exit(void)
{

	class_unregister(motor_class);
}

module_init(motor_init);
module_exit(motor_exit);

MODULE_AUTHOR("Claudio Mignanti <c.mignanti@gmail.com>");
MODULE_DESCRIPTION("Stepper motor driver with pwm emulating over gpio");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:motor-pwm");
