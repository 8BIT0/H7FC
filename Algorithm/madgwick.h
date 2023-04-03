//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>
#include "../common/util.h"

typedef enum
{
    q0 = 0,
    q1,
    q2,
    q3,
    q_sum,
} MadgwickQuat_List;

typedef enum
{
    Att_Pitch = 0,
    Att_Roll,
    Att_Yaw,
    Att_Sum,
} Magwick_Attitude_List;

#pragma pack(1)
typedef union
{
    struct
    {
        float pitch;
        float roll;
        float yaw;

        float q0;
        float q1;
        float q2;
        float q3;
    } Att;

    uint8_t Att_Int[28];
} Madgwick_AttitudeData_TypeDef;
#pragma pack()

typedef struct
{
    float beta;
    float q[q_sum];
    float imu_period;
    uint32_t imu_sample_rate;

    float att_offset[Att_Sum];

    float att[Att_Sum];
} Madgwick_Monitor_TypeDef;

//--------------------------------------------------------------------------------------------

typedef struct
{
    void (*init)(uint32_t sample_freq);
    void (*imu_angle_update)(float *gyr, float *acc, float *att, float *q_buff);
    void (*imu_mag_angle_update)(float *gyr, float *acc, float *mag, float *att, float *q_buf);
} Madgwick_TypeDef;

// Variable declaration
class Madgwick
{
private:
    static float invSqrt(float x);
    float beta; // algorithm gain
    float q0;
    float q1;
    float q2;
    float q3; // quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
    void computeAngles();

    //-------------------------------------------------------------------------------------------
    // Function declarations
public:
    Madgwick(void);
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    // float getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);};
    // float getRoll(){return -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);};
    // float getYaw(){return atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
    float getRoll()
    {
        if (!anglesComputed)
            computeAngles();
        return roll * 57.29578f;
    }
    float getPitch()
    {
        if (!anglesComputed)
            computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw()
    {
        if (!anglesComputed)
            computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    float getRollRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return roll;
    }
    float getPitchRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return pitch;
    }
    float getYawRadians()
    {
        if (!anglesComputed)
            computeAngles();
        return yaw;
    }
};
#endif