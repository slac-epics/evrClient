#ifdef __rtems__
#include <rtems.h>            /* required for timex.h      */
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/timex.h>        /* for ntp_adjtime           */
#include "dbAccess.h"
#include "epicsTypes.h"
#include "registryFunction.h" /* for epicsExport           */
#include "epicsExport.h"      /* for epicsRegisterFunction */
#include "longSubRecord.h"    /* for struct longSubRecord */
#include "aSubRecord.h"       /* for struct aSubRecord  */
#include "alarm.h"           

static long lsubTrigSelInit(longSubRecord *prec)
{
    /* printf("lsubTrigSelInit for %s\n", prec->name); */

    return 0;
}

static long lsubTrigSel(longSubRecord *prec)
{
   epicsUInt32    i   = 0;
   epicsUInt32    *p  = &prec->a;
   epicsUInt32    *pp = &prec->z;

   /* printf("lsubTrigSel for %s\n", prec->name);  */

   for(i=0; (p+i) <= pp; i++) {
       if(*(p+i)) {
           prec->val = i;
           return 0;
       }
   }

   return -1;
}

static long lsubEvSelInit(longSubRecord *prec)
{
    /* printf("lsubEvSelInit for %s\n", prec->name); */

    return 0;
}

static long lsubEvSel(longSubRecord *prec)
{
    epicsUInt32  i  = prec->v;
    epicsUInt32  *p = &prec->a;

    /* printf("lsubEvSel for %s\n", prec->name); */

    prec->val = *(p+i);

    return 0;
}

static long aSubEvOffsetInit(aSubRecord *prec)
{

    return 0;
}

/*
 *   -----------------
 *   Input/Output list 
 *   -----------------
 *
 * INPA: Input for event number: event number selector (long type)
 * INPB: Activate/Deactivate event invariant delay (long type)
 * INPC: Input for EVG delay - lookup PV (long type waveform)
 * INPD: Input for previous delay (just in case, if EVNT:SYSx:1:DELAY array is not available, invalid severity)

 * OUTA: Output for delay
 *
 */
static long aSubEvOffset(aSubRecord *prec)
{
    long eventNumber   = *(long*)(prec->a);
    long activeFlag    = *(long*)(prec->b);
    long *pdelayArray  =  (long*)(prec->c);
    long defaultDelay  = *(long*)(prec->d);
    long *poutputDelay =  (long*)(prec->vala);
    epicsEnum16 sevr;

    if(dbGetSevr(&prec->inpc, &sevr)) {
        printf("%s: CA connection serverity check error\n", prec->name);
        return 0;
    }


    if(sevr                          ||     /* record is not initialized */
       !activeFlag                   ||     /* deactivate */
       !pdelayArray                  ||     /* no lookup table */
       (eventNumber<0 || eventNumber>255)   /* out of range for event number */) {
        *poutputDelay = defaultDelay;                   /* if something is wrong, just use default delay */
    }
    else *poutputDelay = *(pdelayArray + eventNumber);  /* Everything is OK, let's use look up table */


    return 0;
}



epicsRegisterFunction(lsubTrigSelInit);
epicsRegisterFunction(lsubTrigSel);
epicsRegisterFunction(lsubEvSelInit);
epicsRegisterFunction(lsubEvSel);
epicsRegisterFunction(aSubEvOffsetInit);
epicsRegisterFunction(aSubEvOffset);
