/*
 * PWM Motor driver data
 */
#ifndef __LINUX_MOTOR_PWM_H
#define __LINUX_MOTOR_PWM_H

struct motor_pwm {
	const char	*name;
	unsigned	pwm_step;
	unsigned	pwm_period_ns;
	unsigned	gpio_enable;
	unsigned	gpio_ccw;
	unsigned	gpio_lowpwr
	u8 		active_low;	//UNSUPPORTED
	
	long steps;
	long steps_max;
};

struct motor_pwm_platform_data {
	int			num_motor;
	struct motor_pwm	*motor;
};

struct motor_pwm_data { //?
	struct motor_classdev	cdev;
	struct motor_pwm	*mdata;
	struct pwm_device	*pwm;
	unsigned int 		active_low;
	unsigned int		period;
};

#endif
