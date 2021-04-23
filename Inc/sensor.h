
#include "stm32f2xx_hal.h"

/*!
 *
 *@brief: acc raw data
 *
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    //uint32_t time_stamp;
}acc_raw_t;


/*!
 *
 *@brief: gyro raw data
 *
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    //uint32_t time_stamp;
}gyro_raw_t;


typedef struct{
    acc_raw_t imu_accdata;
	  gyro_raw_t imu_gyrodata;
}imu_size_t;
		