#include "mbed.h"
#include "rtos.h"
#include "main.h"

// define
#define VL6180x_I2C_R 0x53
#define VL6180x_I2C_W 0x52

DigitalOut led1(LED1);
//DigitalOut led1(PH_0);
DigitalOut led2(LED2);
PwmOut motorpwm(PA_5);
//I2CSlave vl6180(PG_1, PG_0);
I2C vl6180(PG_1, PG_0); //freq:100KHz

DigitalOut VL6180X_EN(D2);
DigitalOut IN1(D6);
DigitalOut IN2(D7);
Serial debug_log(PB_3, PB_2);

pwmout_t m_control;

void write8(int addr, char data) 
{
    int ret;
	char address[2];
    address[0] = (addr >> 8) & 0xFF;
	address[1] = addr & 0xFF;

    vl6180.start();
    ret = vl6180.write(VL6180X_WRITE_I2C_ADDR);
    //debug_log.printf("[1] write8 ack = %d \r\n", ret);
	ret = vl6180.write(address[0]);
    //debug_log.printf("[2] write8 ack = %d \r\n", ret);
	ret = vl6180.write(address[1]);
    //debug_log.printf("[3] write8 ack = %d \r\n", ret);
	ret = vl6180.write(data);
    debug_log.printf("[4] write8 ack = %d \r\n", ret);
	
	vl6180.stop();
}

//  int read(int address, char *data, int length, bool repeated = false);
char read8(int address)
{
    int ret;
    char data;
	char buf;
    //register addr
    vl6180.start();
	ret = vl6180.write(VL6180X_WRITE_I2C_ADDR);
	buf = (address >> 8) & 0xFF;
	ret = vl6180.write(buf);
	buf = address & 0xFF;
	ret = vl6180.write(buf);
	vl6180.stop();

    //read
    wait_ms(1);
    vl6180.start();
	vl6180.write(VL6180X_READ_I2C_ADDR);
	data = vl6180.read(0);
	vl6180.stop();
    //debug_log.printf("read8 data = 0x%02x \r\n", data);

    return data;
}

void VL6180_loadSettings(void)
{
    // load settings!

    // private settings from page 24 of app note
    write8(0x0207, 0x01);
    write8(0x0208, 0x01);
    write8(0x0096, 0x00);
    write8(0x0097, 0xfd);
    write8(0x00e3, 0x00);
    write8(0x00e4, 0x04);
    write8(0x00e5, 0x02);
    write8(0x00e6, 0x01);
    write8(0x00e7, 0x03);
    write8(0x00f5, 0x02);
    write8(0x00d9, 0x05);
    write8(0x00db, 0xce);
    write8(0x00dc, 0x03);
    write8(0x00dd, 0xf8);
    write8(0x009f, 0x00);
    write8(0x00a3, 0x3c);
    write8(0x00b7, 0x00);
    write8(0x00bb, 0x3c);
    write8(0x00b2, 0x09);
    write8(0x00ca, 0x09);
    write8(0x0198, 0x01);
    write8(0x01b0, 0x17);
    write8(0x01ad, 0x00);
    write8(0x00ff, 0x05);
    write8(0x0100, 0x05);
    write8(0x0199, 0x05);
    write8(0x01a6, 0x1b);
    write8(0x01ac, 0x3e);
    write8(0x01a7, 0x1f);
    write8(0x0030, 0x00);

    // Recommended : Public registers - See data sheet for more detail
    write8(0x0011, 0x10);       // Enables polling for 'New Sample ready'
                                // when measurement completes
    write8(0x010a, 0x30);       // Set the averaging sample period
                                // (compromise between lower noise and
                                // increased execution time)
    write8(0x003f, 0x46);       // Sets the light and dark gain (upper
                                // nibble). Dark gain should not be
                                // changed.
    write8(0x0031, 0xFF);       // sets the # of range measurements after
                                // which auto calibration of system is
                                // performed
    write8(0x0040, 0x63);       // Set ALS integration time to 100ms
    write8(0x002e, 0x01);       // perform a single temperature calibration
                                // of the ranging sensor

    // Optional: Public registers - See data sheet for more detail
    write8(0x001b, 0x09);       // Set default ranging inter-measurement
                                // period to 100ms
    write8(0x003e, 0x31);       // Set default ALS inter-measurement period
                                // to 500ms
    write8(0x0014, 0x24);       // Configures interrupt on 'New Sample
                                // Ready threshold event'
}
void VL6180_printLoadSettings(void)
{
    // load settings!

    // private settings from page 24 of app note
    debug_log.printf("VL6180X_REG_IDENTIFICATION_MODEL_ID: 0x%0x\r\n", read8(VL6180X_REG_IDENTIFICATION_MODEL_ID));
    debug_log.printf("0x0207: 0x%0x\r\n", read8(0x0207));
    debug_log.printf("0x0208: 0x%0x\r\n", read8(0x0208));
    debug_log.printf("0x0096: 0x%0x\r\n", read8(0x0096));
    debug_log.printf("0x0097: 0x%0x\r\n", read8(0x0097));
    debug_log.printf("0x00e3: 0x%0x\r\n", read8(0x00e3));
    debug_log.printf("0x00e4: 0x%0x\r\n", read8(0x00e4));
    //write8(0x00e5, 0x02);
    //write8(0x00e6, 0x01);
    //write8(0x00e7, 0x03);
    //write8(0x00f5, 0x02);
    //write8(0x00d9, 0x05);
    //write8(0x00db, 0xce);
    //write8(0x00dc, 0x03);
    //write8(0x00dd, 0xf8);
    //write8(0x009f, 0x00);
    //write8(0x00a3, 0x3c);
    //write8(0x00b7, 0x00);
    //write8(0x00bb, 0x3c);
    //write8(0x00b2, 0x09);
    //write8(0x00ca, 0x09);
    //write8(0x0198, 0x01);
    //write8(0x01b0, 0x17);
    //write8(0x01ad, 0x00);
    //write8(0x00ff, 0x05);
    //write8(0x0100, 0x05);
    //write8(0x0199, 0x05);
    debug_log.printf("0x01a6: 0x%0x\r\n", read8(0x01a6));
    debug_log.printf("0x01ac: 0x%0x\r\n", read8(0x01ac));
    debug_log.printf("0x01a7: 0x%0x\r\n", read8(0x01a7));
    debug_log.printf("0x0030: 0x%0x\r\n", read8(0x0030));

    //// Recommended : Public registers - See data sheet for more detail
    //write8(0x0011, 0x10);       // Enables polling for 'New Sample ready'
    //                            // when measurement completes
    //write8(0x010a, 0x30);       // Set the averaging sample period
    //                            // (compromise between lower noise and
    //                            // increased execution time)
    //write8(0x003f, 0x46);       // Sets the light and dark gain (upper
    //                            // nibble). Dark gain should not be
    //                            // changed.
    //write8(0x0031, 0xFF);       // sets the # of range measurements after
    //                            // which auto calibration of system is
    //                            // performed
    //write8(0x0040, 0x63);       // Set ALS integration time to 100ms
    //write8(0x002e, 0x01);       // perform a single temperature calibration
    //                            // of the ranging sensor

    //// Optional: Public registers - See data sheet for more detail
    //write8(0x001b, 0x09);       // Set default ranging inter-measurement
    //                            // period to 100ms
    //write8(0x003e, 0x31);       // Set default ALS inter-measurement period
    //                            // to 500ms
    //write8(0x0014, 0x24);       // Configures interrupt on 'New Sample
                                // Ready threshold event'
}

void vl6180_init()
{
    VL6180X_EN = 0;
    wait_ms(10);
    VL6180X_EN = 1;

    char ret = read8(VL6180X_REG_IDENTIFICATION_MODEL_ID);
    //debug_log.printf("vl6180_init ret: 0x%0x \r\n", ret);
	if (ret != 0xB4) {
		debug_log.printf("ERROR: read vl6180 ID failed!!, ret: %0x \r\n", ret);
		return ;
	}
	debug_log.printf("\r\n");
	//VL6180_loadSettings();
    write8(VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
    wait_ms(20);
    debug_log.printf("VL6180X_REG_RESULT_RANGE_STATUS: 0x%02x \r\n", read8(VL6180X_REG_RESULT_RANGE_STATUS));
}

/**************************************************************************/
/*! 
    @brief  Single shot ranging. Be sure to check the return of {@link readRangeStatus} to before using the return value!
    @return Distance in millimeters if valid
*/
/**************************************************************************/

uint8_t readRange(void) {
  // wait for device to be ready for range measurement
  while (! (read8(VL6180X_REG_RESULT_RANGE_STATUS) & 0x01));

  // Start a range measurement
  write8(VL6180X_REG_SYSRANGE_START, 0x01);

  // Poll until bit 2 is set
  while (! (read8(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04));

  // read range in mm
  uint8_t range = read8(VL6180X_REG_RESULT_RANGE_VAL);

  // clear interrupt
  write8(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

  return range;
}


/**************************************************************************/
/*! 
    @brief  Request ranging success/error message (retreive after ranging)
    @returns One of possible VL6180X_ERROR_* values
*/
/**************************************************************************/

uint8_t readRangeStatus(void) {
  return (read8(VL6180X_REG_RESULT_RANGE_STATUS) >> 4);
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
	vl6180_init();
	//VL6180_printLoadSettings();
	
	while(true) {
		Thread::wait(1000);
		debug_log.printf("this is thread vl6180x\r\n");
	    debug_log.printf("range: %ud\r\n", readRange());
	}
}

int main()
{
    debug_log.baud(115200);
	vl6180.frequency(10000);
	
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
