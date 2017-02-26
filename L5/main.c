/*
 * L5.c
 *
 * Created: 10/26/2016 7:42:16 PM
 * Author : wudao
 */ 

#include <avr/io.h>
#include "m_general.h"
#include "m_rf.h"
#include "m_bus.h"
#include "m_imu.h"


#define filter 0.95
#define Kp 30
#define Ki 0.1
#define Kd 70
#define DC_max 0.75
#define pi 3.1415926


int data[9];  //9 element buffer read from IMU
float DC; // duty cycle, control the motors speed
volatile int flag_IMU = 0; //flag that data is received

void init_timer1(); // timer1 initialization 
void init_timer3(); // timer3 initialization 
void init_imu(); // IMU initialization 
void PID(); // get duty cycle (torque) to retain balance
void direction(); // get motor rotation direction 
float get_acc(); // get current angle from accelerometer
float get_gyro(); // get current angle from gyrometer


int main(void)
{
    m_clockdivide(0); //16 mhz
	init_imu();
	init_timer1();
	init_timer3();
	m_disableJTAG(); // turn off JTAG port and allow access to F6,F7 as GPIO

	set(DDRF, 6); 
	set(DDRF, 7); 
	m_wait(2000);
    while(1)
	{
	   if(flag_IMU)
			{
				PID();
				flag_IMU = 0;
				OCR1C= OCR1A*DC;
			}
    }
    return 0;
}

void init_timer1() 
{
	OCR1A= 1000; //set frequency to 16mhz/8/1000 = 2khz

	clear(TCCR1B, CS12);
	set(TCCR1B, CS11);
	clear(TCCR1B, CS10); 
	//prescaler of system clock is /8
	set(TCCR1B,WGM13);
	set(TCCR1B,WGM12);
	set(TCCR1A,WGM11);
	set(TCCR1A,WGM10);
	//timer mode is mode 15 UP to OCR1A, PWM mode
	set(TCCR1A,COM1C1);
	clear(TCCR1A,COM1C0);
	//clear at OCR1C, set at rollover
	set(DDRB, 7);
	// enable B7 output
}

void init_timer3()
{
    OCR3A = 250; //set frequency to 62.5hz

    set(TCCR3B,CS32);    
    clear(TCCR3B,CS31);  
    set(TCCR3B,CS30);    
	//prescaler of system clock is /1024
    clear(TCCR3B,WGM33);
    set(TCCR3B,WGM32);
    clear(TCCR3A,WGM31);
	clear(TCCR3A,WGM30);
	//timer mode is mode4 UP to OCR3A
      
	set(TIMSK3, OCIE3A);
	 // enable timer3 match OCR3A interrupt
	sei();     
	
}

void init_imu()
{
     m_imu_init(1, 1); // +/-2g  +/-250deg/s 
}
     

float get_acc() // offset
{
	static float accel_filter = 0;
	float x_accel =  -(float) data[0]*2/32768;
	//float z_accel =  (float) data[2]*2/32768;
	//static float x_accel_filter = 0;
	//static float z_accel_filter = 0;

	//x_accel_filter = x_accel_filter * (filter) + x_accel * (1 - filter); 
	//z_accel_filter = z_accel_filter * (filter) + z_accel * (1 - filter); // low pass filter
	//accel_filter = atan2(z_accel_filter,x_accel_filter);
	accel_filter = accel_filter *filter + x_accel * (1-filter);
	return accel_filter;
}

float get_gyro() // offset
{
	float y_gyro = 0;
	static float gyro_angle_filter = 0;
	float TIME = 0.016;//Time depends on timer3
	static float prev_y_gyro_angle = 0;
	static float y_gyro_angle = 0;

	y_gyro = (-(float) data[4])*250*pi/(180*32768);
	prev_y_gyro_angle = y_gyro_angle;
	y_gyro_angle = y_gyro_angle + y_gyro * TIME; 
	gyro_angle_filter = (filter) * (gyro_angle_filter + y_gyro_angle - prev_y_gyro_angle); // high pass filter
	return gyro_angle_filter;
}

void PID()  //offset
{
	float prev_angle = 0;
	float angle = 0;
	float P = 0;
	float I = 0;
	float D = 0;
	
	prev_angle = angle;
	angle = get_acc() + get_gyro();
	P = angle * Kp; // Proportional
	I = I + angle * Ki; // Integral
	D = (angle - prev_angle) * Kd; // Derivative
	float pid = P + I + D;

	int sign = (angle > 0); // check the angle sign for direction
	direction(sign); // direction output
	
	DC = fabs(pid); // Duty Cycle output, not larger than 0.75
	if (DC > DC_max){DC = DC_max;}
	else {DC = DC;}
}

void direction(int direction) // when angle is 0
{
    set(PORTB, 7);
	if (direction)
	{
		set(PORTF, 6);
		clear(PORTF, 7); // positive direction 
		m_red(OFF);
		m_green(ON);
	}
	else
	{
		clear(PORTF, 6);
		set(PORTF, 7); // negative direction 
		m_red(ON);
		m_green(OFF);
	}
}


ISR(TIMER3_COMPA_vect)
{
		
	    if(flag_IMU) 
		{}                                                                                                                                                                                                                                                                                                                                                                         
		else 
		{
		m_imu_raw(data);
		flag_IMU = 1;
		}
}