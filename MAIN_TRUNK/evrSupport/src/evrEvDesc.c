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
#include "aSubRecord.h"       /* for struct aSubRecord  */
#include "alarm.h"

static long aSubEvDescInit(aSubRecord *prec)
{
    return 0; /* Nothing todo during initialization */
}

/*
 *    -----------------
 *    INPUT/OUTPUT List
 *    -----------------
 *
 *    INPA: input for event number (long type)
 *    INPB: input for event name previous setting, 
 *          if severity is not good then just choose this event name for current one (string type)
 *    INPC: input for event name list (string waveform)
 *
 *    OUTA: event name output (string type)
 *
 */

static long aSubEvDesc(aSubRecord *prec)
{
    char *prevString = (char*)prec->b;       /* get previous Event name from the INPB */
    char *inString   = (char*)prec->c;       /* get Event name list from the string waveform from INPC */
    char *outString  = (char*)prec->vala;    /* Event Name Output to OUTA */
    long eventNumber = *((long*)(prec->a));  /* get Event number from INPA */
    epicsEnum16        sevr;                 /* alarm severity for event name array */
    long status;

    if(dbGetSevr(&prec->inpc, &sevr)) {
        printf("%s: CA connection serverity check error\n", prec->name);
        return 0;
    }


    if(sevr) {         /* if severity is not normal, then just use previous event name */
        if(prevString) {
            if(outString) strcpy(outString, prevString);
        }
        else
            if(outString) strcpy(outString, "Unavailable");

        return 0;
    }

    if(eventNumber >=0 && eventNumber <256 && inString) {   /* normal processing */ 
        if(outString) strcpy(outString, inString+(eventNumber * MAX_STRING_SIZE));
    }
    else
        if(outString) strcpy(outString, "Unavailable");

    return 0;
}

epicsRegisterFunction(aSubEvDescInit);
epicsRegisterFunction(aSubEvDesc);
