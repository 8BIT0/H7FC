#include "Alt_est.h"
#include <math.h>
#include <Eigen>

using namespace std;
using namespace Eigen;

typedef union
{
    float buf[3];

    struct
    {
        float alt;          /* altitude             unit: m */
        float ver_speed;    /* vertical speed       unit: m/s */
        float ver_accel;    /* vertical accelerate  unit: m/s^2 */
    } sec;
} BaroStatus_TypeDef;

typedef struct
{
    uint32_t iter_num;              /* number of iterations */
    Matrix<float, 3, 1> InitStatus; /* Init baro altitude state */
    Matrix<float, 3, 1> CurStatus;  /* current baro altitude state */
    Matrix<float, 3, 1> LstStatus;  /* last baro altitude state */
    Matrix<float, 3, 3> StateCnvM;  /* state convert matrix */
} BaroAltEstimateObj_TypeDef;

static BaroAltEstimateObj_TypeDef BaroAltObj;

void BaroAltEstimate_Init(float baro_alt, float acc_z, float delta_T)
{
    memset(&BaroAltObj, 0, sizeof(BaroAltEstimateObj_TypeDef));

    /* init state convert matrix */
    BaroAltObj.StateCnvM(0, 0) = 1;
    BaroAltObj.StateCnvM(0, 1) = delta_T;
    BaroAltObj.StateCnvM(0, 2) = 0.5 * (delta_T * delta_T);

    BaroAltObj.StateCnvM(1, 0) = 0;
    BaroAltObj.StateCnvM(1, 1) = 1;
    BaroAltObj.StateCnvM(1, 2) = delta_T;
    
    BaroAltObj.StateCnvM(2, 0) = 0;
    BaroAltObj.StateCnvM(2, 1) = 0;
    BaroAltObj.StateCnvM(2, 2) = 1;
}

float BaroAltEstimate_Update(float baro, float acc_z)
{
    float tmp = 0.0f;

    if (BaroAltObj.iter_num != 0)
    {
        /* step 1: get current state predict */
        BaroAltObj.CurStatus = BaroAltObj.StateCnvM * BaroAltObj.LstStatus;
    
        /* step 2: */
    }
    else
    {
        /* update last state only */
        BaroAltObj.LstStatus = BaroAltObj.CurStatus;
    }

    BaroAltObj.iter_num ++;
    return tmp;
}

