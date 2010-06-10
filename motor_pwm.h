/*
 * PWM Motor driver data
 */
#ifndef __MOTOR_PWM_H
#define __MOTOR_PWM_H

struct motor_pwm {
	const char   *name;
	unsigned int pwm_step;
	int index;
	unsigned int gpio_enable;
	unsigned int gpio_ccw;
	unsigned int gpio_lowpwr;
	u8 		active_low;	//UNSUPPORTED
	long steps;
	long steps_max;

	struct pwm_channel *pwmc;
	struct cdev *motor_cdev;
	unsigned long pwm_up;
	unsigned long pwm_period;
};

struct motor_pwm_platform_data {
	int			num_motor;
	struct motor_pwm	*motor;
};
/*
struct motor_pwm_data {
	//struct motor_classdev	cdev;
	struct motor_pwm	*mdata;
	struct pwm_device	*pwm;
	unsigned int 		active_low;
	unsigned int		period;
};
*/
#endif
