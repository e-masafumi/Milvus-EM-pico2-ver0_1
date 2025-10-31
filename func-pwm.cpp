#include "func-pwm.h"

//static int sysClk = 125000000; 
static int sysClk = 150000000; 
static int pwmPin[4] = {2, 3, 6, 7};
		
static int pwmFreq[2] = {5000, 5000};
static int pwmClkDiv[2] = {10, 10};
static pwm_config pwm_slice_config[4];

int pico_pwm::setup(void){
	int pwmWrap[2] = {sysClk/pwmFreq[0], sysClk/pwmFreq[1]};

	uint pwm_slice_num[2];

	gpio_set_function(pwmPin[0], GPIO_FUNC_PWM);
	gpio_set_function(pwmPin[1], GPIO_FUNC_PWM);
	gpio_set_function(pwmPin[2], GPIO_FUNC_PWM);
	gpio_set_function(pwmPin[3], GPIO_FUNC_PWM);

	pwm_slice_num[0] = pwm_gpio_to_slice_num(pwmPin[0]);
	pwm_slice_num[1] = pwm_gpio_to_slice_num(pwmPin[1]);
	pwm_slice_num[2] = pwm_gpio_to_slice_num(pwmPin[2]);
	pwm_slice_num[3] = pwm_gpio_to_slice_num(pwmPin[3]);

	pwm_slice_config[0] = pwm_get_default_config();
	pwm_slice_config[1] = pwm_get_default_config();
	pwm_slice_config[2] = pwm_get_default_config();
	pwm_slice_config[3] = pwm_get_default_config();

	pwm_config_set_wrap(&pwm_slice_config[0], pwmWrap[0]);
	pwm_config_set_wrap(&pwm_slice_config[1], pwmWrap[0]);
	pwm_config_set_wrap(&pwm_slice_config[2], pwmWrap[1]);
	pwm_config_set_wrap(&pwm_slice_config[3], pwmWrap[1]);
	
	pwm_config_set_clkdiv(&pwm_slice_config[0], 10);
	pwm_config_set_clkdiv(&pwm_slice_config[1], 10);
	pwm_config_set_clkdiv(&pwm_slice_config[2], 10);
	pwm_config_set_clkdiv(&pwm_slice_config[3], 10);

	pwm_init( pwm_slice_num[0], &pwm_slice_config[0], true );
	pwm_init( pwm_slice_num[1], &pwm_slice_config[1], true );
	pwm_init( pwm_slice_num[2], &pwm_slice_config[2], true );
	pwm_init( pwm_slice_num[3], &pwm_slice_config[3], true );

	pwm_set_gpio_level(pwmPin[0], (pwm_slice_config[0].top*0.75));
	pwm_set_gpio_level(pwmPin[1], (pwm_slice_config[1].top*0.75));
	pwm_set_gpio_level(pwmPin[2], (pwm_slice_config[2].top*0.75));
	pwm_set_gpio_level(pwmPin[3], (pwm_slice_config[3].top*0.75));

	return 0;
};


double pico_pwm::dutyFit(double input, double minDuty, double maxDuty){
	return (minDuty + (maxDuty-minDuty)*input);
}

double pico_pwm::dutyFitPct(double input, double minDuty, double maxDuty){
	double duty;
	duty = (input + 100.0) * 0.5 * 0.01;
	return dutyFit(duty, minDuty, maxDuty);
//	return (minDuty + (maxDuty-minDuty)*input);
}

int pico_pwm::duty(int pinNum, double duty){
	if(duty > 1 || duty < 0){
		return 1;
	}
	else{
		pwm_set_gpio_level(pwmPin[pinNum], (pwm_slice_config[pinNum].top*duty));
		return 0;
	}
};
