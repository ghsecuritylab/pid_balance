#include "mbed.h"
#include "rtos.h"

// define
#define VL6180x_I2C_R   0x53
#define VL6180x_I2C_W   0x52


DigitalOut led1(LED1);
//DigitalOut led1(PH_0);
DigitalOut led2(LED2);
DigitalOut IN1(D0);
DigitalOut IN2(D1);
PwmOut motorpwm(PA_5);
//I2CSlave vl6180(PG_1, PG_0);
I2C vl6180(PG_1, PG_0);//Ĭ��100KHz
Serial debug_log(PB_3, PB_2);

pwmout_t m_control;

boolean VL6180X_begin(void) {
  _i2caddr = VL6180X_DEFAULT_I2C_ADDR;
  if (! theWire) {
    _i2c = &Wire;
  } else {
    _i2c = theWire;
  }
  _i2c-> begin();


  if (read8(VL6180X_REG_IDENTIFICATION_MODEL_ID) != 0xB4) {
    return false;
  }

  //if (read8(VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET) == 0x01) {
    loadSettings();
  //}

  write8(VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00);

  return true;
}


void led2_thread(void const *args)
{
    while (true)
    {
        led2 = !led2;
        Thread::wait(2000);
			  debug_log.printf("led2_thread ...\r\n");
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
	  debug_log.baud(115200);
    Thread thread(led2_thread);
    Thread thread_motor(motor_thread);
	  debug_log.printf("hello world\r\n");

    while (true)
    {
        led1 = !led1;
        Thread::wait(5);
    }
}
