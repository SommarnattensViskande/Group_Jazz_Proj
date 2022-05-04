/*
* Copyright (c) 2022, Nikolaus Huber
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>

#include <drivers/pwm.h>
#include <devicetree.h>
#include <drivers/gpio.h>




#define PWM_LED0_NODE DT_ALIAS(pwm_led0)

#if DT_NODE_HAS_STATUS(PWM_LED0_NODE, okay)
#define PWM_CTLR DT_PWMS_CTLR(PWM_LED0_NODE)
#define PWM_CHANNEL DT_PWMS_CHANNEL(PWM_LED0_NODE)
#define PWM_FLAGS DT_PWMS_FLAGS(PWM_LED0_NODE)
#else
#error "Unsupported board: pwm-led0 devicetree alias is not defined"
#define PWM_CTLR DT_INVALID_NODE
#define PWM_CHANNEL 0
#define PWM_FLAGS 0
#endif

#define MIN_PERIOD_USEC (USEC_PER_SEC / 64U)
#define MAX_PERIOD_USEC USEC_PER_SEC

#define MY_STACK_SIZE 1000
#define BUTTON_DEBOUNCE_DELAY_MS 100

#define PERIOD 20000/1U
#define PERIOD0 20000/1U
#define PERIOD1 16000/1U
#define PERIOD2 12000/1U
#define PERIOD3 8000/1U
#define PERIOD4 4000/1U
#define PERIOD5 0

/* Semaphore used to signal from button ISR to task */
K_SEM_DEFINE(button_sem, 0, 1);

struct k_mutex my_mutex;
int k_mutex_init(struct k_mutex * my_mutex);

typedef enum _state 
{
bright_zero,
bright_one,
bright_two,
bright_three,
bright_four,
bright_five
} state_t;

static volatile state_t global_state = bright_zero;
static volatile state_t state1 = bright_zero; 

void blinky_task()
{
    const struct device *pwm;
    int ret;

    pwm = DEVICE_DT_GET(PWM_CTLR);


    while (1) 
    {
        k_msleep(100);

        k_mutex_lock(&my_mutex, K_FOREVER); //------------------------------
        state1 = global_state;
        k_mutex_unlock(&my_mutex); //------------------------------
        if(state1==bright_zero) 
        {   ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD, PERIOD0, PWM_FLAGS);
        }else if(state1==bright_one){
            ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD, PERIOD1, PWM_FLAGS);
        }else if(state1==bright_two){
            ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD, PERIOD2, PWM_FLAGS);
        }else if(state1==bright_three){
            ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD, PERIOD3, PWM_FLAGS);
        }else if(state1==bright_four){
            ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD, PERIOD4, PWM_FLAGS);
        }else if(state1==bright_five){
            ret = pwm_pin_set_usec(pwm, PWM_CHANNEL, PERIOD, PERIOD5, PWM_FLAGS);
        }
    }
}

#define SW0_NODE DT_ALIAS(sw0)
#define SW0 DT_GPIO_LABEL(SW0_NODE, gpios)
#define SW0_PIN DT_GPIO_PIN(SW0_NODE, gpios)
#define SW0_FLAGS (GPIO_INPUT | DT_GPIO_FLAGS(SW0_NODE, gpios))

static uint32_t time, last_time;
static struct gpio_callback button_cb_data;


/**** << START >> ********* VEML7700 TASK ***************/
void veml7700_task()
{
    const struct device *veml7700 = device_get_binding("VEML7700");

    if (veml7700 == NULL) {
    printf("No device \"%s\" found; did initialization fail?\n", "VEML7700");
    return -1;
}
/**** << END >> ********* VEML7700 TASK ***************/


struct sensor_value lux;

while (true) 
{
    sensor_sample_fetch(veml7700);
    sensor_channel_get(veml7700, SENSOR_CHAN_LIGHT, &lux);
    printf("Lux: %f\n", sensor_value_to_double(&lux));


    if( sensor_value_to_double(&lux) >= 1000 )
    {
        global_state = bright_zero;
        printf("--------------------------state_0--------------------------------");

    } 
    else if( (500 <= sensor_value_to_double(&lux)) && (sensor_value_to_double(&lux) < 1000 ))
    {

        global_state = bright_one;
        printf("--------------------------state_1--------------------------------");
    
    }
    else if( (200 <= sensor_value_to_double(&lux)) && (sensor_value_to_double(&lux) < 500 ))
    {

        global_state = bright_two;
        printf("---------------------------state_2--------------------------------");
        
    }
    else if( (100 <= sensor_value_to_double(&lux)) && (sensor_value_to_double(&lux) < 200 ))
    {

        global_state = bright_three;
        printf("---------------------------state_3--------------------------------");
        
    }
    else if( (20 <= sensor_value_to_double(&lux)) && (sensor_value_to_double(&lux) < 100 ))
    {

        global_state = bright_four;
        printf("---------------------------state_4--------------------------------");
        
    }
    else if( sensor_value_to_double(&lux) < 20)
    {
        global_state = bright_five;
        printf("---------------------------state_5--------------------------------");
    
    } 

    k_msleep(500);

    }

}


/**** Debounce the button ****/
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    time = k_uptime_get_32();

    if (time < last_time + BUTTON_DEBOUNCE_DELAY_MS)
    {
        last_time = time;
        return;
    }

    k_sem_give(&button_sem);
    time = last_time;
}


/**** << START >> ********* BUTTON_PRESSED TASK ***************/
void button_task()
{
    /* Set up button interrupt */
    const struct device *sw0;
    
    sw0 = device_get_binding(SW0);
    gpio_pin_configure(sw0, SW0_PIN, GPIO_INPUT | SW0_FLAGS);
    gpio_pin_interrupt_configure(sw0, SW0_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data, button_pressed, BIT(SW0_PIN));
    gpio_add_callback(sw0, &button_cb_data);


    while(1)
    {
        k_sem_take(&button_sem, K_FOREVER);

        k_mutex_lock(&my_mutex, K_FOREVER); 
        
        if (global_state == bright_zero)
        {
            global_state = bright_one;

        } else if (global_state == bright_one)
        {
            global_state = bright_two;

        }else if (global_state == bright_two)
        {
            global_state = bright_three;

        }else if (global_state == bright_three)
        {
            global_state = bright_four;

        }else if (global_state == bright_four)
        {
            global_state = bright_five;

        }else if (global_state == bright_five)
        {
            global_state = bright_zero;
        }
    k_mutex_unlock(&my_mutex); 

    }
}
/**** << END >> ********* BUTTON_PRESSED TASK ***************/


static int process_mpu6050(const struct device *dev)
{

	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
					accel);
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
					gyro);
	
		printf(" =========================================\n"
			  
		       "  accel %f %f %f m/s/s\n"
		       "  gyro  %f %f %f rad/s\n",
			  
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]),
		       sensor_value_to_double(&gyro[0]),
		       sensor_value_to_double(&gyro[1]),
		       sensor_value_to_double(&gyro[2]));
	} else {
		printf("sample fetch/get failed: %d\n", rc);
	}
	return rc;
}

/**** CONFIG_MPU6050_TRIGGER ****/
#ifdef CONFIG_MPU6050_TRIGGER
static struct sensor_trigger trigger;

static void handle_mpu6050_drdy(const struct device *dev,
				struct sensor_trigger *trig)
{
	int rc = process_mpu6050(dev);

	if (rc != 0) {
		printf("cancelling trigger due to failure: %d\n", rc);
		(void)sensor_trigger_set(dev, trig, NULL);
		return;
	}
}
#endif 


/**** << START >> ********* MPU6050 TASK ***************/
void mpu6050_task()
{
    const char *const label = DT_LABEL(DT_INST(0, invensense_mpu6050));
	const struct device *mpu6050 = device_get_binding(label);

	if (!mpu6050) {
		printf("Failed to find sensor %s\n", label);
		return;
	}

#ifdef CONFIG_MPU6050_TRIGGER
	trigger = (struct sensor_trigger) {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};
	if (sensor_trigger_set(mpu6050, &trigger,
			       handle_mpu6050_drdy) < 0) {
		printf("Cannot configure trigger\n");
		return;
	}
	printk("Configured for triggered sampling.\n");
#endif

	while (!IS_ENABLED(CONFIG_MPU6050_TRIGGER)) {
		int rc = process_mpu6050(mpu6050);

		if (rc != 0) {
			
			printf("°°°°°°°°°Pass out°°°°°°°°° \n");
			//break;
		}

		k_msleep(200);
	}

	/* triggered runs with its own thread after exit */

}
/**** << END >> ********* MPU6050 TASK ***************/




K_THREAD_DEFINE(blinky_id, MY_STACK_SIZE, blinky_task,
NULL, NULL, NULL, 5, 0, 0);

K_THREAD_DEFINE(button_id, MY_STACK_SIZE, button_task,
NULL, NULL, NULL, 4, 0, 0);

K_THREAD_DEFINE(veml7700_id, MY_STACK_SIZE, veml7700_task,
NULL, NULL, NULL, 3, 0, 0); 

K_THREAD_DEFINE(mpu6050_id, MY_STACK_SIZE, mpu6050_task,
NULL, NULL, NULL, 2, 0, 0); 