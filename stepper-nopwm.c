/*
 *		Stepper motor driver over gpio with pwm support
 *
 *		Copyright (R) 2010 Claudio Mignanti - <c.mignanti@gmail.com>
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
#define DRV_VERSION	"0.1-no_pwm"

#define MAX_MOT_NUM 4

/* module var*/
struct class *motor_class;
static dev_t motor_devno = 0;
unsigned long steps_max[MAX_MOT_NUM] = {0}, steps[MAX_MOT_NUM] = {0};

/* module parameters */
int g_enable[MAX_MOT_NUM] = {0} , g_dir[MAX_MOT_NUM] = {0}, \
		g_step[MAX_MOT_NUM] = {0}, g_lpwr[MAX_MOT_NUM] = {0}, \
		polarity[MAX_MOT_NUM] = {0};

static unsigned int mot0[6] __initdata;
static unsigned int mot1[6] __initdata;
static unsigned int mot2[6] __initdata;
static unsigned int mot3[6] __initdata;

static unsigned int mot_nump[MAX_MOT_NUM] __initdata;

static int mot_map[MAX_MOT_NUM] = {0};
static int mot_map_pwm[MAX_MOT_NUM] = {0};
static struct hrtimer *mot_map_hrtimer[MAX_MOT_NUM] = {0};

// de-pwm
static struct hrtimer t[MAX_MOT_NUM];
static int status[MAX_MOT_NUM];
static struct work_struct work[MAX_MOT_NUM];
ktime_t interval[MAX_MOT_NUM];

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

	interval[id] = ktime_set(0, val * 1000000UL);
	return 0;
}

static int cdev_to_id (struct cdev* cdev) {
	int i;
	for (i=0; i < MAX_MOT_NUM; i++) {
		if (cdev == (struct cdev*) (mot_map[i]))
			return i;
	}
	return 0;
}

static int hrtimer_to_id (struct hrtimer *t) {
	int i;
	for (i=0; i < MAX_MOT_NUM; i++) {
		if (t == (struct hrtimer *) (mot_map_hrtimer[i]))
			return i;
	}
	return 0;
}

static void gpio_hr_work (struct work_struct *work) {
	printk(KERN_INFO "stepper: gpio_hr_work\n");
}

static enum hrtimer_restart gpio_timeout(struct hrtimer *t)
{
	int id = hrtimer_to_id(t), ret = 0;

	printk(KERN_INFO "stepper: gpio_timeout %d\n",status[id]);

	//TODO lock?
	if (status[id]) {
		gpio_set_value(g_step[id] ,0);
		status[id] = 0;
	} else {
		gpio_set_value(g_step[id] ,1);
		status[id] = 1;
		steps[id]++;
	}

	if (steps[id] >= steps_max[id]) {
		hrtimer_try_to_cancel(&t[id]);
	} else {
		hrtimer_forward(&t[id], ktime_get(), interval[id]);
		return HRTIMER_RESTART;
	}

	return HRTIMER_NORESTART;
}

/* IOCTL interface */
static int motor_ioctl (struct inode *in, struct file *fl, unsigned int cmd, \
						unsigned long arg) {

	int retval = 0, id;
	unsigned long to_end;
	struct cdev* p = in->i_cdev;

	id = cdev_to_id (p);

	//printk(KERN_INFO "stepper: arg: %l \n", arg);
	switch (cmd) {
		case MOTOR_ENABLE:
			if ((int)arg)
				gpio_set_value (g_enable[id], 0);
			else
				gpio_set_value (g_enable[id], 1);
			break;

		case MOTOR_DIR:
			if ((int)arg)
				gpio_set_value (g_dir[id], 1);
			else
				gpio_set_value (g_dir[id], 0);
			break;

		case MOTOR_PWM_ON:
			hrtimer_start(&t[id], interval[id], HRTIMER_MODE_REL);
			break;

		case MOTOR_PWM_OFF:
			hrtimer_cancel(&t[id]);
			break;

		case MOTOR_PWM_SET:
			//set the pwm period in ms
			motor_pwm_set (arg, id);
			break;

		case MOTOR_RESET:
			steps[id] = 0; /* set the actual position as home */
			break;

		case MOTOR_STEPS:
			steps_max[id] = arg; /* set the steps limit */
			break;

		case MOTOR_START:
			hrtimer_start(&t[id], interval[id], HRTIMER_MODE_REL);
			break;

		case MOTOR_LOWPWR:
			if ((int)arg)
				gpio_set_value (g_lpwr[id], 1);
			else
				gpio_set_value (g_lpwr[id], 0);
			break;

		/* return steps_max-step */
		case MOTOR_TO_END:
			to_end = steps_max[id] - steps[id];
			copy_to_user(&arg, &to_end, sizeof(unsigned long));
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

static int __init motor_add_one(unsigned int id, unsigned int *params)
{
	int status, err;
	struct cdev *motor_cdev;
	struct platform_device *pdev;
	//struct gpio_pwm_platform_data pdata;

	if ( mot_nump[id] < 4 ) {
		printk(KERN_INFO "stepper: nothing to register for id: %d.\n", id);
		return 0;
	}

	g_enable[id] = params[1];
	g_dir[id] = params[2];
	g_step[id] = params[3];
	g_lpwr[id] = params[4];
	polarity[id] = params[5];

	/* sanity check */
	if ( !( g_enable[id] && g_dir[id] && g_step[id])) {
		printk(KERN_INFO "stepper: missing parameters, exit driver.\n");
		goto err_para;
	}

	INIT_WORK(&work[id], gpio_hr_work);

	if (gpio_request(g_step[id], "motor-step"))
		return -EINVAL;

	hrtimer_init(&t[id], CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	t[id].function = &gpio_timeout;
	mot_map_hrtimer[id] = &t[id];
	interval[id] = ktime_set(0, 10000000UL);

	motor_pwm_set(1, id);

	if ( gpio_request(g_enable[id], "motor-enable") < 0 ) {
		goto err_gpioenable;
	}
	gpio_direction_output(g_enable[id] ,0);
	gpio_set_value (g_enable[id], 0);

	if ( gpio_request(g_dir[id], "motor-ccw") < 0) {
		goto err_gpiodir;
	}
	gpio_direction_output(g_dir[id] ,0);

	if (g_lpwr[id] != 0) {
		if ( gpio_request(g_lpwr[id], "motor-lowpwr") < 0 ) {
			goto err_gpiolwr;
		}
		gpio_direction_output(g_lpwr[id] ,0);
	}

	/* set to home */
	steps[id] = 0;

	/* alloc a new device number (major: dynamic, minor: 0) */
	status = alloc_chrdev_region(&motor_devno, 0, 1, "motor");

	/* create a new char device  */
	motor_cdev = cdev_alloc();
	if(motor_cdev == NULL) {
		status=-ENOMEM;
		goto err_dev;
	}

	/*save the cdev for id's */
	mot_map[id] = (int) motor_cdev;

	motor_cdev->owner = THIS_MODULE;
	motor_cdev->ops = &motor_fops;
	status = cdev_add(motor_cdev, motor_devno, 1);
	if(status){
		goto err_dev;
	}

	device_create(motor_class, NULL, motor_devno, NULL, "motor%d", params[0]);
	printk(KERN_INFO "stepper: motor%d registred on major: %u; minor: %u\n", \
		params[0], MAJOR(motor_devno), MINOR(motor_devno));

	return 0;

err:
	printk(KERN_INFO "stepper: err\n");
err_dev:
	printk(KERN_INFO "stepper: err_dev\n");
err_gpiolwr:
	printk(KERN_INFO "stepper: err_gpiolwr\n");
err_gpiodir:
	printk(KERN_INFO "stepper: err_gpiodir\n");
err_gpioenable:
	printk(KERN_INFO "stepper: err_gpioenable\n");
err_gpiostep:
	printk(KERN_INFO "stepper: err_gpiostep ");
err_pwm:
	printk(KERN_INFO "stepper: err_pwm\n");
err_para:
	printk(KERN_INFO "stepper: Error management not yet implemented. \
		Please reboot your board %d\n",g_step[id]);
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

	/*free the gpio_pins*/
//	for (i=0; i < 
}

module_init(motor_init);
module_exit(motor_exit);

MODULE_AUTHOR("Claudio Mignanti <c.mignanti@gmail.com>");
MODULE_DESCRIPTION("Stepper motor driver over gpio with pwm support");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:motor-pwm");
