#include "mbed.h"
#include "rtos.h"

DigitalOut led1(LED1);
//DigitalOut led1(PH_0);
DigitalOut led2(LED2);
DigitalOut IN1(D0);
DigitalOut IN2(D1);
PwmOut motorpwm(PA_5);

pwmout_t m_control;

void led2_thread(void const *args)
{
    while (true)
    {
        led2 = !led2;
        Thread::wait(2000);
    }
}

void motor_thread(void const *args)
{

    IN1 = 0;
    motorpwm.period_us(20);
    float duty_cycle = 0.001f;
    motorpwm = duty_cycle;
    //IN2 = 0;

    while (true)
    {
        Thread::wait(200);
        duty_cycle = duty_cycle + 0.01f;
			  if (duty_cycle >= 0.4f) {
            duty_cycle = 0.001;
        }
        motorpwm = duty_cycle;
    }
}

int main()
{
    Thread thread(led2_thread);
    Thread thread_motor(motor_thread);

    while (true)
    {
        led1 = !led1;
        Thread::wait(5);
    }
}
