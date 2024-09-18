#include "Att_Casecade_PID.h"


bool Att_CheckParam_Validation(AttCaseCadePID_Param_TypeDef para);
bool Att_Casecade_PID(AttCaseCadePID_Param_TypeDef *para, bool angular_only, AttControl_ExpIn_TypeDef exp_att, AngControl_ExpIn_TypeDef exp_ang, AngControl_Out_TypeDef ctl_out);
