//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

//----------------------------------------------------------------------------------------------------
// Variable declaration

typedef struct
{
    float beta;

    float q0;
    float q1;
    float q2;
    float q3;

    float pitch;
    float roll;
    float yaw;
} AlgoAttData_TypeDef;

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSInit(AlgoAttData_TypeDef *att);
void MadgwickAHRSupdate(AlgoAttData_TypeDef *att,float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(AlgoAttData_TypeDef *att,float gx, float gy, float gz, float ax, float ay, float az);

#ifdef __cplusplus
}
#endif

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
