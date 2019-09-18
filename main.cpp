#include "mbed.h"
#include "rtos.h"
#include "main.h"

DigitalOut led1(LED1);
//DigitalOut led1(PH_0);
DigitalOut led2(LED2);
PwmOut motorpwm(PA_5);

DigitalOut IN1(D6);
DigitalOut IN2(D7);
Serial debug_log(PB_3, PB_2);

pwmout_t m_control;

//Interrupt
InterruptIn echo();

uint8_t readRange(void) {
  // read range in mm
  uint8_t range = 0;

  return range;
}

void led2_thread(void const *args)
{
    while (true)
    {
        led2 = !led2;
        Thread::wait(2000);
        //debug_log.printf("led2_thread ...\r\n");
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
        if (duty_cycle >= 0.4f)
        {
            duty_cycle = 0.001;
        }
        motorpwm = duty_cycle;
    }
}

int main()
{
    debug_log.baud(115200);

    Thread thread_led(led2_thread);
    Thread thread_motor(motor_thread);

    debug_log.printf("hello world\r\n");

    while (true)
    {
        led1 = !led1;
        Thread::wait(5);
    }
}
