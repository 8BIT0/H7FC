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
        float alt;                  /* altitude             unit: m */
        float ver_speed;            /* vertical speed       unit: m/s */
        float ver_accel;            /* vertical accelerate  unit: m/s^2 */
    } sec;
} BaroStatus_TypeDef;

typedef struct
{
    Matrix<float, 3, 1> P_X;        /* predict baro altitude state */
    Matrix<float, 3, 1> C_X;        /* current baro altitude state */
    Matrix<float, 3, 1> L_X;        /* last baro altitude state */

    Matrix<float, 3, 3> P_C;        /* predict covariance */
    Matrix<float, 3, 3> C_C;        /* current covariance */
    Matrix<float, 3, 3> L_C;        /* last covariance */

    Matrix<float, 3, 3> Proc_Q;     /* process bias matrix */
    Matrix<float, 2, 2> Noise;      /* noise matrix */
    Matrix<float, 2, 3> OutMatrix;  /* output matrix */
    Matrix<float, 3, 2> Gain;       /* gain matrix */

    Matrix<float, 3, 3> StateCnvM;  /* state convert matrix */
    Matrix<float, 2, 1> InitState;  /* Baro Acc init state */
} BaroAltEstimateObj_TypeDef;

static BaroAltEstimateObj_TypeDef BaroAltObj;

void BaroAltEstimate_Init(float baro, float delta_T, float baro_bias, float acc_bias)
{
    BaroAltObj.P_X.Zero();
    BaroAltObj.C_X.Zero();
    BaroAltObj.L_X.Zero();
    BaroAltObj.P_C.Zero();
    BaroAltObj.C_C.Zero();
    BaroAltObj.L_C.Zero();
    BaroAltObj.Proc_Q.Zero();
    BaroAltObj.Noise.Zero();
    BaroAltObj.OutMatrix.Zero();
    BaroAltObj.Gain.Zero();
    BaroAltObj.StateCnvM.Zero();
    BaroAltObj.InitState.Zero();

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

    /* init output matrix */
    BaroAltObj.OutMatrix(0, 0) = (-1.0 / 0.09f);
    BaroAltObj.OutMatrix(0, 1) = 0.0f;
    BaroAltObj.OutMatrix(0, 2) = 0.0f;

    BaroAltObj.OutMatrix(1, 0) = 0.0f;
    BaroAltObj.OutMatrix(1, 1) = 0.0f;
    BaroAltObj.OutMatrix(1, 2) = 1.0f;

    /* set noise matrix */
    BaroAltObj.Noise(0, 0) = baro_bias;
    BaroAltObj.Noise(1, 1) = acc_bias;

    /* set init state */
    BaroAltObj.InitState(0, 0) = baro;
    BaroAltObj.InitState(1, 0) = 0.0f;

    BaroAltObj.Proc_Q(0, 0) = 0.0001f;
    BaroAltObj.Proc_Q(1, 1) = 0.0001f;
    BaroAltObj.Proc_Q(2, 2) = 0.00001f;
}

RelMov_TypeDef BaroAltEstimate_Update(float baro, float acc_z)
{
    RelMov_TypeDef out;
    Matrix<float, 3, 2> M32_tmp_1;
    Matrix<float, 2, 2> M32_tmp_2;
    Matrix<float, 2, 1> M_Mea;
    Matrix<float, 3, 3> M_Unit;

    memset(&out, 0, sizeof(RelMov_TypeDef));

    M32_tmp_1.Zero();
    M32_tmp_2.Zero();
    M_Unit.Identity();

    M_Mea(0, 0) = baro;
    M_Mea(1, 0) = (float)((int16_t)(acc_z * 100.0) / 100.0f);

    /* step 1: get state predict */
    BaroAltObj.P_X = BaroAltObj.StateCnvM * BaroAltObj.L_X;

    /* step 2: get covariance predict*/
    BaroAltObj.P_C = BaroAltObj.StateCnvM * BaroAltObj.L_C * BaroAltObj.StateCnvM.transpose() + BaroAltObj.Proc_Q;

    /* step 3: get gain matrix */
    M32_tmp_1 = BaroAltObj.P_C * BaroAltObj.OutMatrix.transpose();
    M32_tmp_2 = BaroAltObj.OutMatrix * M32_tmp_1 + BaroAltObj.Noise;
    BaroAltObj.Gain = M32_tmp_1 * M32_tmp_2.inverse();

    /* step 4: get current state estimate */
    BaroAltObj.C_X = BaroAltObj.P_X + BaroAltObj.Gain * (M_Mea - BaroAltObj.OutMatrix * BaroAltObj.P_X - BaroAltObj.InitState);

    /* step 5: update convariance */
    BaroAltObj.C_C = (M_Unit - BaroAltObj.Gain * BaroAltObj.OutMatrix) * BaroAltObj.P_C;

    BaroAltObj.L_C = BaroAltObj.C_C;
    BaroAltObj.L_X = BaroAltObj.C_X;

    out.pos = BaroAltObj.C_X(0, 0);
    out.vel = -BaroAltObj.C_X(1, 0);
    out.acc = BaroAltObj.C_X(2, 0);

    return out;
}

