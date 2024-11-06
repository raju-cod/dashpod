#include "BNO055.h"  

extern const nrf_drv_twi_t m_twi_init;

int main(void) 
{
    twi_init();
    nrf_delay_ms(500);

    i2c_scanner_bno055(&m_twi_init, 0x00);
    nrf_delay_ms(100);

    setup_bno055(&m_twi_init, BNO055_ADDRESS);
    nrf_delay_ms(500);
    
    

   while(1)
    {
      displayCalStatus(&m_twi_init, BNO055_ADDRESS);
      nrf_delay_ms(500);

      read_euler(&m_twi_init, BNO055_ADDRESS);
      nrf_delay_ms(500);

      //read_quaternion(&m_twi_init, BNO055_ADDRESS);
      //nrf_delay_ms(500);

      //acceleration_values(&m_twi_init, BNO055_ADDRESS);
      //nrf_delay_ms(500);

    }


}