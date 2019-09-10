#include "mbed.h"
#include "rtos.h"
#include "main.h"

// define
#define VL6180x_I2C_R 0x53
#define VL6180x_I2C_W 0x52

DigitalOut led1(LED1);
//DigitalOut led1(PH_0);
DigitalOut led2(LED2);
DigitalOut IN1(D0);
DigitalOut IN2(D1);
PwmOut motorpwm(PA_5);
//I2CSlave vl6180(PG_1, PG_0);
I2C vl6180(PG_1, PG_0); //freq:100KHz
Serial debug_log(PB_3, PB_2);

pwmout_t m_control;

void write8(char addr_h, char addr_l, char data) 
{
	char buffer[3];
    buffer[0] = addr_h;
    buffer[1] = addr_l;
    buffer[2] = data;
	vl6180.write(VL6180X_WRITE_I2C_ADDR, buffer, 3);
}

void VL6180_loadSettings(void)
{
  // load settings!
  // private settings from page 24 of app note
  write8(0x02, 0x07, 0x01);
  write8(0x02, 0x08, 0x01);
  write8(0x00, 0x96, 0x00);
  write8(0x00, 0x97, 0xfd);
  write8(0x00, 0xe3, 0x00);
  write8(0x00, 0xe4, 0x04);
  write8(0x00, 0xe5, 0x02);
  write8(0x00, 0xe6, 0x01);
  write8(0x00, 0xe7, 0x03);
  write8(0x00, 0xf5, 0x02);
  write8(0x00, 0xd9, 0x05);
  write8(0x00, 0xdb, 0xce);
  write8(0x00, 0xdc, 0x03);
  write8(0x00, 0xdd, 0xf8);
  write8(0x00, 0x9f, 0x00);
  write8(0x00, 0xa3, 0x3c);
  write8(0x00, 0xb7, 0x00);
  write8(0x00, 0xbb, 0x3c);
  write8(0x00, 0xb2, 0x09);
  write8(0x00, 0xca, 0x09);
  write8(0x01, 0x98, 0x01);
  write8(0x01, 0xb0, 0x17);
  write8(0x01, 0xad, 0x00);
  write8(0x00, 0xff, 0x05);
  write8(0x01, 0x00, 0x05);
  write8(0x01, 0x99, 0x05);
  write8(0x01, 0xa6, 0x1b);
  write8(0x01, 0xac, 0x3e);
  write8(0x01, 0xa7, 0x1f);
  write8(0x00, 0x30, 0x00);

  // Recommended : Public registers - See data sheet for more detail
  write8(0x00, 0x11, 0x10); // Enables polling for 'New Sample ready'
                           // when measurement completes
  write8(0x01, 0x0a, 0x30); // Set the averaging sample period
                           // (compromise between lower noise and
                           // increased execution time)
  write8(0x00, 0x3f, 0x46); // Sets the light and dark gain (upper
                             // nibble). Dark gain should not be
                             // changed.
  write8(0x00, 0x31, 0xFF); // sets the # of range measurements after
                        // which auto calibration of system is
                        // performed
  write8(0x00, 0x40, 0x63); // Set ALS integration time to 100ms
  write8(0x00, 0x2e, 0x01); // perform a single temperature calibration
                        // of the ranging sensor

  // Optional: Public registers - See data sheet for more detail
  write8(0x0, 0x01b, 0x09); // Set default ranging inter-measurement
                        // period to 100ms
  write8(0x00, 0x3e, 0x31); // Set default ALS inter-measurement period
                        // to 500ms
  write8(0x00, 0x14, 0x24); // Configures interrupt on 'New Sample
                        // Ready threshold event'
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

void vl6180x_thread(void const *args)
{
	char tx[1];
	tx[0] = VL6180X_REG_IDENTIFICATION_MODEL_ID;
	char ret;
	vl6180.write(VL6180X_WRITE_I2C_ADDR, tx, 1);
	vl6180.read(VL6180X_READ_I2C_ADDR, &ret, 1);
	if (ret != 0xB4) {
		debug_log.printf("ERROR: read vl6180 ID failed!!, ret: %0x \r\n", ret);
		return ;
		}
	debug_log.printf("\r\n");
	VL6180_loadSettings();
	while(true) {
		Thread::wait(1000);
		debug_log.printf("this is thread vl6180x\r\n");
	}
}

int main()
{
    debug_log.baud(115200);
	
    Thread thread_led(led2_thread);
    Thread thread_motor(motor_thread);
	Thread thread_vl6180x(vl6180x_thread);
	
    debug_log.printf("hello world\r\n");

    while (true)
    {
        led1 = !led1;
        Thread::wait(5);
    }
}
