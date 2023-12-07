#include "Srv_CtlDataNegociate.h"

/* internal vriable */
static Srv_CtlArbitrateMonitor_TypeDef SrvCtlArbitrateMonitor;

/* external function */
static bool Srv_CtlDataArbitrate_Init(void);
static void Srv_CtlDataArbitrate_Update(void);

/* external vriable */
Srv_CtlDataArbitrate_TypeDef Srv_CtlDataArbitrate = {
    .init = Srv_CtlDataArbitrate_Init,
};

static bool Srv_CtlDataArbitrate_Init(void)
{
    memset(&SrvCtlArbitrateMonitor, 0, sizeof(SrvCtlArbitrateMonitor));

    return true;
}

static void Srv_CtlDataArbitrate_Update(void)
{
    /* convert gimbal value to physical expection */
}
