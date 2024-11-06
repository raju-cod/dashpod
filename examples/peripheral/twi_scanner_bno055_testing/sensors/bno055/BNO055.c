#include "BNO055.h"

 uint8_t sys, gyro, accel, mag;

/* @Brief This function is about setting up the bno055 sensor 
*
*  we are resetting BNO055 then configuring into NDOF mode for getting Fusion values like euler and quaternion
*
*/
void setup_bno055(const nrf_drv_twi_t* m_twi,uint8_t addr)
{
   
      if(!is_twi_initialized()) 
          {
              twi_init();
              printf("TWI initialized\n");
          }
 
      
      uint8_t data_reset[2] = {BNO055_SYS_TRIGGER_ADDR,0x20};
      uint8_t data_pwr_mode[2] = {BNO055_PWR_MODE_ADDR,0x00};
      uint8_t data_page[2]={0x07,0x00};
      uint8_t data_opr_mode[2] = {BNO055_OPR_MODE_ADDR,OPERATION_MODE_NDOF};

      ret_code_t err_code;

      //here resetting the sensor
      err_code = (nrf_drv_twi_tx(m_twi, addr, data_reset, sizeof(data_reset), false));   
      APP_ERROR_CHECK(err_code);
      nrf_delay_ms(1000);

      //here setting into power mode 
      err_code = (nrf_drv_twi_tx(m_twi, addr, data_pwr_mode, sizeof(data_pwr_mode), false));   
      APP_ERROR_CHECK(err_code);
      nrf_delay_ms(1000);

      //here  setting into page 0
      err_code = (nrf_drv_twi_tx(m_twi, addr, data_page, sizeof(data_page), false));   
      APP_ERROR_CHECK(err_code);
      nrf_delay_ms(10);
     

      //here setting into operation mode NDOF
      err_code = (nrf_drv_twi_tx(m_twi, addr, data_opr_mode, sizeof(data_opr_mode), false));   
      APP_ERROR_CHECK(err_code);
      nrf_delay_ms(20);

      printf("BNO055 CONFIGURED SUCCESFULLY\n");
   
}

/* @Brief This function is about finding slave address of BNO055
*
*
*/
void i2c_scanner_bno055(const nrf_drv_twi_t* m_twi,uint8_t addr)
  {

    if(!is_twi_initialized()) 
        {
            twi_init();
            printf("TWI initialized\n");
        }
 
    uint8_t data = 0xff;

    printf("Twi scanning started\n");
    for (; addr <= TWI_ADDRESSES; addr++)
    {
        ret_code_t err_code = nrf_drv_twi_tx(m_twi,addr,&data,1,false);
        nrf_delay_ms(50);
        if (err_code == NRF_SUCCESS)
        {

            printf("\nTWI device detected at addr 0x%x\n", addr);
        }
    }

  }

/* @Brief This is a function which displays sensors calibration values 
*
*/
void displayCalStatus(const nrf_drv_twi_t* m_twi,uint8_t addr) {

    uint8_t value = 0;
    uint8_t reg = BNO055_CALIB_STAT_ADDR;
   

    ret_code_t err_code;

    err_code = nrf_drv_twi_tx(m_twi, addr, &reg, 1, true);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_twi_rx(m_twi, addr, &value, 1);
    APP_ERROR_CHECK(err_code);

    sys = (value >> 6) & 0x03;
    gyro = (value >> 4) & 0x03;
    accel = (value >> 2) & 0x03;
    mag = value & 0x03;

    //printf("\nSys: %d\t",sys);
    printf(" Gyro: %d\t",gyro);
    printf(" Accel: %d\t",accel);
    printf(" Mag: %d\n",mag);

    uint8_t data[4];
    data[0] = sys;
    data[1] = gyro;
    data[2] = accel;
    data[3] = mag;
}


/* @Brief This function is about reading EULER ANGLES from desired registers
*
*
*/
 void read_euler(const nrf_drv_twi_t* m_twi, uint8_t addr)
 {

    if((accel==3) && (mag == 3) && (gyro == 3) ){
            uint8_t euler_buffer[6] = {0};
            uint8_t reg = 0x1A;

            ret_code_t err_code;
            int16_t x,y,z;

            int i;
            for (i=0;i<sizeof(euler_buffer);i++,reg++){

                  // this loop runs for 6 axis including positive and negative.
                  err_code = nrf_drv_twi_tx(m_twi,addr,&reg,1,true);
                  APP_ERROR_CHECK(err_code);

                  err_code = nrf_drv_twi_rx(m_twi, addr, &euler_buffer[i] , sizeof(euler_buffer[i]));
                  APP_ERROR_CHECK(err_code);
            }

            x = ((int16_t)euler_buffer[0]) | (((int16_t)euler_buffer[1]) << 8);
            y = ((int16_t)euler_buffer[2]) | (((int16_t)euler_buffer[3]) << 8);
            z = ((int16_t)euler_buffer[4]) | (((int16_t)euler_buffer[5]) << 8);

            printf("\n\nEULER ANGLES\n");
            printf("RAW_X: %d\tRAW_Y: %d\tRAW_Z: %d\n", x, y, z);
            printf("X: %-10.2fY: %-10.2fZ: %-10.2f\n", x / 16.0, y / 16.0, z / 16.0);
    
            float data[3];
            data[0] = x/16.0;
            data[1] = y/16.0;
            data[2] = z/16.0;
    
    }
 }

/*@Brief This function is about reading quaternion values
*
*
*/
 void read_quaternion(const nrf_drv_twi_t* m_twi, uint8_t addr)  {

    if((accel==3) && (mag == 3) && (gyro == 3) ){
          uint8_t quater_buffer[8] = {0};
          uint8_t reg = 0x20;

          ret_code_t err_code;
          int16_t qw,qx,qy,qz;

          for(int i =0; i < sizeof(quater_buffer); i++,reg++){

             err_code = nrf_drv_twi_tx(m_twi,addr,&reg,1,true);
             APP_ERROR_CHECK(err_code);

             err_code = nrf_drv_twi_rx(m_twi, addr, &quater_buffer[i] , sizeof(quater_buffer[i]));
             APP_ERROR_CHECK(err_code);

          }

          qw = ((int16_t)quater_buffer[0]) | (((int16_t)quater_buffer[1]) << 8);
          qx = ((int16_t)quater_buffer[2]) | (((int16_t)quater_buffer[3]) << 8);
          qy = ((int16_t)quater_buffer[4]) | (((int16_t)quater_buffer[5]) << 8);
          qz = ((int16_t)quater_buffer[6]) | (((int16_t)quater_buffer[7]) << 8);

          printf("\n\nRAW QUTERNION VALUES\n");
          printf("QWR: %d\tQXR: %d\tQYR: %d\tQZR: %d\n",qw, qx, qy, qz);

          float quater_data[4];

          quater_data[0] = qw / 16384.0;
          quater_data[1] = qx / 16384.0;
          quater_data[2] = qy / 16384.0;
          quater_data[3] = qz / 16384.0;

          printf(" Scaled values : QW: %-10.4f\tQX: %-10.4f\tQY: %-10.4f\tQZ: %-10.4f\n",quater_data[0], quater_data[1], quater_data[2], quater_data[3]);
          //quaternion_to_angles(quater_data[0], quater_data[1], quater_data[2], quater_data[3]);

    }

 }


 /* @Brief This function is about converting quaternion values into angles 
 *
 *
 */
 void quaternion_to_angles(float qw, float qx, float qy, float qz)  {
   
   float x,y,z;
    // Roll (x-axis rotation) in radians
    x = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    
    // Pitch (y-axis rotation) in radians
    y = asin(2.0 * (qw * qy - qz * qx));

    // Yaw (z-axis rotation) in radians
    z = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    printf("\nThe converted angles from quaternion\n");

    //converting radians to degree
    printf("X = %-10.2fY = %-10.2fZ = %-10.2f", x*180/M_PI, y*180/M_PI, z*180/M_PI);


 }


/*@Brief This function is about reading accelaration data from regesters to calculate step length and stride length
*
*
*/
 void read_accel_values(const nrf_drv_twi_t* m_twi, uint8_t addr) {
 
  ret_code_t err_code;

  int16_t x,y,z;
  float ax, ay, az;

  uint8_t accel_data[6] = {0};
  uint8_t reg = 0x08;

  for(int i = 0; i < sizeof(accel_data); i++,reg++){
  
      err_code = nrf_drv_twi_tx(m_twi,addr,&reg,1,true);
      APP_ERROR_CHECK(err_code);

      err_code = nrf_drv_twi_rx(m_twi,addr,&accel_data[i],sizeof(accel_data[i]));
      APP_ERROR_CHECK(err_code);

  }

    x = ((int16_t)accel_data[0]) | (((int16_t)accel_data[1]) << 8);
    y = ((int16_t)accel_data[2]) | (((int16_t)accel_data[3]) << 8);
    z = ((int16_t)accel_data[4]) | (((int16_t)accel_data[5]) << 8);


    ax = x/100.0;
    ay = y/100.0;
    az = z/100.0;

    printf("\n\nACCELERATION DATA\n");
    printf("X: %d\tY: %d\tZ: %d\n", x, y, z);
    printf("X: %-10.2fY: %-10.2fZ: %-10.2f\n", ax, ay, az);

 }

/**@Brief This function is about reading gyroscope values
*
*
*/
void read_gyro_values(const nrf_drv_twi_t* m_twi, uint8_t addr) {
 
  ret_code_t err_code;

  int16_t x,y,z;

  uint8_t gyro_data[6] = {0};
  uint8_t reg = 0x14;

  for(int i = 0; i < sizeof(gyro_data); i++,reg++){
  
      err_code = nrf_drv_twi_tx(m_twi,addr,&reg,1,true);
      APP_ERROR_CHECK(err_code);

      err_code = nrf_drv_twi_rx(m_twi,addr,&gyro_data[i],sizeof(gyro_data[i]));
      APP_ERROR_CHECK(err_code);

  }

    x = ((int16_t)gyro_data[0]) | (((int16_t)gyro_data[1]) << 8);
    y = ((int16_t)gyro_data[2]) | (((int16_t)gyro_data[3]) << 8);
    z = ((int16_t)gyro_data[4]) | (((int16_t)gyro_data[5]) << 8);

    printf("\n\nGYRO DATA\n");
    printf("X: %d\tY: %d\tZ: %d\n", x, y, z);
    //printf("X: %-10.2fY: %-10.2fZ: %-10.2f\n", x, y, z);
   

 }

/**@Brief This function is about reading magneto meter values
*
*
*/
void read_mag_values(const nrf_drv_twi_t* m_twi, uint8_t addr) {
 
  ret_code_t err_code;

  int16_t x,y,z;
  float ax, ay, az;

  uint8_t mag_data[6] = {0};
  uint8_t reg = 0x0E;

  for(int i = 0; i < sizeof(mag_data); i++,reg++){ 
  
      err_code = nrf_drv_twi_tx(m_twi,addr,&reg,1,true);
      APP_ERROR_CHECK(err_code);

      err_code = nrf_drv_twi_rx(m_twi,addr,&mag_data[i],sizeof(mag_data[i]));
      APP_ERROR_CHECK(err_code);

  }

    x = ((int16_t)mag_data[0]) | (((int16_t)mag_data[1]) << 8);
    y = ((int16_t)mag_data[2]) | (((int16_t)mag_data[3]) << 8);
    z = ((int16_t)mag_data[4]) | (((int16_t)mag_data[5]) << 8);


    printf("\n\nMAGNETO DATA\n");
    printf("X: %d\tY: %d\tZ: %d\n", x, y, z);
    //printf("X: %-10.2fY: %-10.2fZ: %-10.2f\n", x, y, z);

 }