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
    Matrix<float, 3, 1> InitStatus; /* Init baro altitude state */
    Matrix<float, 3, 1> P_X;        /* predict baro altitude state */
    Matrix<float, 3, 1> C_X;        /* current baro altitude state */
    Matrix<float, 3, 1> L_X;        /* last baro altitude state */

    Matrix<float, 3, 3> P_C;        /* predict covariance */
    Matrix<float, 3, 3> C_C;        /* current covariance */
    Matrix<float, 3, 3> L_C;        /* last covariance */

    Matrix<float, 3, 3> Proc_Q;     /* process bias matrix */
    Matrix<float, 3, 3> Noise;      /* noise matrix */
    Matrix<float, 2, 3> OutMatrix;  /* output matrix */
    Matrix<float, 3, 3> Gain;       /* gain matrix */

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

    /* step 1: get state predict */
    BaroAltObj.P_X = BaroAltObj.StateCnvM * BaroAltObj.L_X;

    /* step 2: get covariance predict*/
    BaroAltObj.P_C = BaroAltObj.StateCnvM * BaroAltObj.L_C * BaroAltObj.StateCnvM.transpose() + BaroAltObj.Proc_Q;

    /* step 3: get gain matrix */
    // BaroAltObj.Gain = ;

    return tmp;
}

