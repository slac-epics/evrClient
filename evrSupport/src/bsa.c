/*=============================================================================

  Name: bsa.c

  Abs: This device support for beam synchronous acquisition records via
       EVG/EVR EDEFS
           bsaSecnAvg        - BSA Processing
           bsaSecnInit       - BSA Processing Initialization
           read_bsa          - BSA Record Value Update
           init_bsa_record   - Init BSA Record Device Support
           get_ioint_info    - Get IO Intr pointer
           init_ao_record    - Init AO Record Device Support
           write_ao          - Update Data for BSA Record

  Auth:
  Rev:

  ---------------------------------------------------------------------------*/

#include "copyright_SLAC.h"	/* SLAC copyright comments */

/*-----------------------------------------------------------------------------

  Mod:  12 Feb 2010     S. Hoobler (sonya)

	To set SEVR/STAT on BSA PVs:

	  Structure bsa_ts:
	    (1) Add epicsEnum16 sevr and stat fields.

	  Routine bsaSecnAvg:
	    (1) Add epicsEnum16 secnStat to the argument list.
	    (2) In the code that initializes the average under "if
	    ((bsa_ps->avgcnt == 1) || noAverage)", set sevr and stat
	    of the bsa_ts structure to the input secnStat and secnSevr.
	    (3) In the code that calculates running avg, add logic to
            compute running max of stat and sev

	  Routine read_bsa:
	    (1) Add local variables dstat and dsevr to copy stat and sevr from bsa_ts.
	    (2) In the logic that calls recGblSetSevr, add an "else" that calls
                recGblSetSevr with the local variables.

	 Routine write_ao:
	    (1) Replace dbGetTimeStamp with dbGetField in order to get stat, sevr,
	    and time of input record.
	    (2) In bsaSecnAvg call, replace pao->nsev with severity of input record
	    and add stat of input record.

	 README file of the event module:
	  (1) In the section that describes calling BSA routines directly
	  (II-2), update the API description to include the new argument.


=============================================================================*/

#include <string.h>        /* strcmp */
#include <stdlib.h>        /* calloc */
#include <math.h>          /* sqrt   */

#include "epicsMath.h"        /* for NAN value in case of miss data */
#include "bsaRecord.h"        /* for struct bsaRecord      */
#include "aoRecord.h"         /* for struct aoRecord       */
#include "registryFunction.h" /* for epicsExport           */
#include "epicsExport.h"      /* for epicsRegisterFunction */
#include "epicsTime.h"        /* epicsTimeStamp */
#include "epicsMutex.h"       /* epicsMutexId   */
#include "errlog.h"
#include "dbAccess.h"         /* dbGetTimeStamp */
#include "devSup.h"           /* for dset and DEVSUPFUN    */
#include "devLib.h"           /* for S_dev_noMemory        */
#include "recGbl.h"           /* for recGblSetSevr         */
#include "ellLib.h"           /* linked list    */
#include "dbScan.h"           /* IOSCANPVT      */
#include "alarm.h"            /* INVALID_ALARM  */
#include "evrTime.h"          /* evrTimeGetFromEdef        */
#include "evrPattern.h"       /* EDEF_MAX                  */
#include "bsa.h"              /* prototypes in this file   */
#include "drvEvr.h"

#include "bsaCallbackApi.h"

/* This is the missing counter limit */
#define  EVR_MAX_MISS  (100000000)    /* 1 billion */

/* BSA information for one device, one EDEF */
typedef struct {

  /* Results of Averaging */
  double              val;       /* average value     */
  double              rms;       /* RMS of above      */
  int                 cnt;       /* # in average      */
  int                 readcnt;   /* # total readouts  */
  epicsTimeStamp      time;      /* time of average   */
  unsigned long       nochange;  /* Same time stamp counter */
  unsigned long       noread;    /* Data not read counter   */
  unsigned long       missing;   /* Data miss         */
  int                 readFlag;  /* Data read flag    */
  int                 reset;     /* Reset waveforms   */
  /* Intermediate Values */
  double              avg;       /* average  so far   */
  double              var;       /* variance so far   */
  int                 avgcnt;    /* count    so far   */
  epicsTimeStamp      timeData;  /* latest input time */
  epicsTimeStamp      timeInit;  /* init         time */
  IOSCANPVT           ioscanpvt; /* to process records using above fields */
  epicsEnum16         stat;      /* max status so far */
  epicsEnum16         sevr;      /* max severity so far*/

} bsa_ts;

/* BSA devices */
typedef struct {
  ELLNODE node;
  char    name[PVNAME_STRINGSZ];
  int     noAverage;
  bsa_ts  bsa_as[EDEF_MAX];

} bsaDevice_ts;

typedef struct {
  epicsTimeStamp edefTimeInit_s;
  epicsTimeStamp edefTime_s;
  epicsUInt32    edefAllDone;
  int            edefAvgDone;
  epicsEnum16    edefSevr;
  int            edefIdx;
} BsaChecker_ts;

union BsaChecker_tu;

typedef union BsaChecker_tu {
  BsaChecker_ts        s;
  union BsaChecker_tu *n;
} BsaChecker_tu;

static BsaChecker_tu *bsaCheckerAlloc();
static void           bsaCheckerFree(BsaChecker_tu*);

ELLLIST bsaDeviceList_s;
static epicsMutexId bsaRWMutex_ps = 0;
static epicsMutexId bsaFLMutex_ps = 0;

static BsaChecker_tu *bsaCheckerFL = 0;

int QueueFullCounter = 0;

/* For overloaded IOC, do not insert NaNs, while missing data, if this flag is = 0.*/
static volatile int NaNflag = INSERT_NAN;


/*=============================================================================

  Name: bsaProcessor

  Abs:  Beam Synchronous Acquisition Processor (Averaging and Update)

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        epicsTimieStamp *   secnTime_ps Read       Data timestamp
        double              secnVal     Read       Data value
        epicsEnum16         secnStat    Read       Data status
        epicsEnum16         secnSevr    Read       Data severity
	int                 noAverage   Read       No Averaging for this device
        epicsTimeStamp *    edefTimeInit_ps Read   EDEF init   timestamp
        int                 edefAvgDone Read       EDEF average-done flag
        epicsEnum16         edefSevr    Read       EDEF severity
        bsa_ts *            bsa_ps      Read/Write BSA Structure for this device

  Rem:

  Ret:  0 = OK, -1 = Mutex problem (Not defined).

==============================================================================*/
static int bsaProcessor(epicsTimeStamp *secnTime_ps,
			double		secnVal,
			epicsEnum16	secnStat,
			epicsEnum16     secnSevr,
			int             noAverage,
			epicsTimeStamp *edefTimeInit_ps,
			int		edefAvgDone,
			epicsEnum16     edefSevr,
			bsa_ts         *bsa_ps)
{
    /* Check if the EDEF has initialized and wipe out old values if it has */
    if ((edefTimeInit_ps->secPastEpoch != bsa_ps->timeInit.secPastEpoch) ||
        (edefTimeInit_ps->nsec         != bsa_ps->timeInit.nsec)) {
      bsa_ps->timeInit = *edefTimeInit_ps;
      bsa_ps->avg    = 0.0;
      bsa_ps->var    = 0.0;
      bsa_ps->avgcnt = 0;
      bsa_ps->readcnt= 0;
      if (bsa_ps->readFlag) bsa_ps->noread++;
      bsa_ps->readFlag = 0;
      bsa_ps->reset    = 1;
#ifdef BSA_DEBUG
printf("BSAAVG: NOREAD\n");
#endif
    }
    /* Ignore data that hasn't changed since last time */
    if ((secnTime_ps->secPastEpoch == bsa_ps->timeData.secPastEpoch) &&
        (secnTime_ps->nsec         == bsa_ps->timeData.nsec)) {
      bsa_ps->nochange++;
#ifdef BSA_DEBUG
printf("BSAAVG: NOCHANGE\n");
#endif
    } else {
      bsa_ps->timeData = *secnTime_ps;
      bsa_ps->readcnt++;

#ifdef BSA_DEBUG
printf("secnSevr %d, edefSevr %d\n", secnSevr, edefSevr);
#endif
      /* Include this value in the average if it's OK with the EDEF */
      if (secnSevr < edefSevr) {
	bsa_ps->avgcnt++;

	/* now start the averaging */
	/* first time thru for new cycle; reset previous avg, variance */

	/* compute running avg and variance                      */
	/*        This is translated from REF_RMX_BPM:CUM.F86.   */
	/*                                                       */
	/*        CUM computes VAR as the sample variance:       */
	/*          VAR = (SUMSQ - SUM*SUM/N)/(N-1)              */
	/*          where SUM = sum of the N values, and         */
	/*           SUMSQ = sum of the squares of the N values. */
	/*                                                       */
	/*        Note that CUM's method of computing VAR avoids */
	/*        possible loss of significance.                 */
	/*                                                       */
	/*  Compute running maximum status and severity          */

	if ((bsa_ps->avgcnt == 1) || noAverage) {
          bsa_ps->avgcnt = 1;
	  bsa_ps->avg    = secnVal;
          bsa_ps->var    = 0.0;
	  bsa_ps->stat   = secnStat;
	  bsa_ps->sevr   = secnSevr;
	}
	else {
	  double diff  = secnVal - bsa_ps->avg;
	  bsa_ps->avg += diff/(double)bsa_ps->avgcnt;
      double diff1 = secnVal - bsa_ps->avg;
      bsa_ps->var += diff*diff1;
	  if (secnSevr > bsa_ps->sevr) {
#ifdef BSA_DEBUG
printf("BSAAVG: setting secnSevr %d\n", secnSevr);
#endif
	    bsa_ps->sevr = secnSevr;
	    bsa_ps->stat = secnStat;
	  }
	}
      } /* if good, include in averaging */
    }
    /* Finish up calcs when the average is done and force record processing */
    if (edefAvgDone) { /* values when avg is done */
#ifdef BSA_DEBUG
printf("BSAAVG: done, secnSevr %d (%d avg count)\n", secnSevr, bsa_ps->avgcnt);
#endif
      bsa_ps->val  = bsa_ps->avg;
      bsa_ps->cnt  = bsa_ps->avgcnt;
      bsa_ps->time = bsa_ps->timeData;
      if (bsa_ps->avgcnt <= 1) {
        bsa_ps->rms = 0.0;
	if (bsa_ps->avgcnt <= 0) {
          bsa_ps->stat   = secnStat;
          bsa_ps->sevr   = secnSevr;
	}
      } else {
        bsa_ps->rms = bsa_ps->var/(bsa_ps->avgcnt - 1);
      }
      bsa_ps->avgcnt = 0;
      bsa_ps->avg    = 0;
      if (bsa_ps->ioscanpvt) {
        if (bsa_ps->readFlag) bsa_ps->noread++;
        else                  bsa_ps->readFlag = 1;
        scanIoRequest(bsa_ps->ioscanpvt);
      }
    }
    return 0;
}/*end of bsaProcessor*/

/*=============================================================================

  Name: bsaSecnAvg

  Abs:  Beam Synchronous Acquisition Processing
        Computes BSA device running average and RMS values for all EDEFs
        Computes running maximum status and severity

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        epicsTimeStamp *    secnTime_ps Read       Data timestamp
        double              secnVal     Read       Data value
        epicsEnum16         secnStat    Read       Data status
        epicsEnum16         secnSevr    Read       Data severity
	int                 noAveraging
        void *              dev_ps      Read/Write BSA Device Structure

  Rem:

  Ret:  0 = OK, -1 = Mutex problem or bad status from evrTimeGetFromEdef.

==============================================================================*/

int bsaSecnAvg(epicsTimeStamp *secnTime_ps,
               double          secnVal,
               epicsEnum16     secnStat,
               epicsEnum16     secnSevr,
               int             noAveraging,
               void           *dev_ps)
{
  epicsTimeStamp  edefTimeInit_s, edefTime_s;
  int             edefAvgDone;
  int             idx;
  int             status = 0;
  epicsEnum16     edefSevr;

  if ((!bsaRWMutex_ps) || epicsMutexLock(bsaRWMutex_ps) || (!dev_ps))
    return -1;
  for (idx = 0; idx < EDEF_MAX; idx++) {
    /* Get EDEF information. */
    if (evrTimeGetFromEdef(idx, &edefTime_s, &edefTimeInit_s,
                           &edefAvgDone, &edefSevr)) {
      status = -1;
      continue;
    }
    /* EDEF timestamp must match the data timestamp.
     * And check that for the first acquisition the time is set to 0.
     * Otherwise if those time variables are not ready,
     * you see increasing the diagnostic variable for same timestamp. */
#ifdef BSA_DEBUG
printf("EDEF PID %d, SECN PID %d\n", edefTime_s.nsec&0x1ffff, secnTime_ps->nsec & 0x1ffff);
#endif
    if (((secnTime_ps->secPastEpoch != edefTime_s.secPastEpoch) ||
         (secnTime_ps->nsec         != edefTime_s.nsec)) ||
	((secnTime_ps->secPastEpoch == 0) && (secnTime_ps->nsec  == 0)))  continue;

    /* Process the acquisition for the passed device. This routine gets called when the device time
       is matching with the EDEF time.*/
#ifdef BSA_DEBUG
printf("calling bsaProcessor secnSevr %d\n", secnSevr);
#endif
    bsaProcessor(secnTime_ps, secnVal, secnStat, secnSevr,
		 ((bsaDevice_ts *)dev_ps)->noAverage,
		 &edefTimeInit_s, edefAvgDone, edefSevr,
		 &((bsaDevice_ts *)dev_ps)->bsa_as[idx]);
  }
  epicsMutexUnlock(bsaRWMutex_ps);
  return status;
}

/*=============================================================================

  Name: bsaCheckerAlloc/bsaCheckerFree

  Abs:  maintain free list of BsaChecker_tu unions. These care submitted
        as 'work jobs' to the eventTask.

  Args: alloc: None. free: pointer previously obtained by alloc

  Rem:

  Ret:  memory (alloc), void (free)

==============================================================================*/

static BsaChecker_tu *
bsaCheckerAlloc()
{
BsaChecker_tu *rval;
epicsMutexMustLock( bsaFLMutex_ps );
	if ( (rval = bsaCheckerFL) ) {
		bsaCheckerFL = rval->n;
	}
epicsMutexUnlock( bsaFLMutex_ps );
	if ( ! rval ) {
		rval = malloc( sizeof( *rval ) );
	}
	return rval;
}

static void
bsaCheckerFree(BsaChecker_tu *p)
{
epicsMutexMustLock( bsaFLMutex_ps );
	p->n         = bsaCheckerFL;
	bsaCheckerFL = p;
epicsMutexUnlock( bsaFLMutex_ps );
}


/*=============================================================================

  Name: bsaCheckerWorker

  Abs:  Wrapper for bsaCheckerDevices to be executed by eventTask.

  Args: BsaChecker_tu * -- pointer to argument struct/union

  Rem:

  Ret:  VOID

==============================================================================*/


static void
bsaCheckerWorker(void *arg)
{
BsaChecker_tu *checkArg = (BsaChecker_tu*)arg;

	bsaCheckerDevices(
	    &checkArg->s.edefTimeInit_s,
	    &checkArg->s.edefTime_s,
	    checkArg->s.edefAllDone,
	    checkArg->s.edefAvgDone,
	    checkArg->s.edefSevr,
	    checkArg->s.edefIdx);

	bsaCheckerFree( checkArg );
}


/*=============================================================================
 *
 *     Name: evrBsaMessage
 *
 *     Abs:  evrBsaMessage initialization.
 *
 *     Rem:  Add Message to evrEventTask Queue to Check BSA Data.
 *
=============================================================================*/
static int evrBsaMessage(epicsTimeStamp *edefTimeInit_ps,
                  epicsTimeStamp *edefTime_ps,
                  epicsUInt32    edefAllDone,
                  int            edefAvgDone,
                  epicsEnum16    edefSevr,
                  int            edefIdx)
{
BsaChecker_tu *checkArg = bsaCheckerAlloc();

    checkArg->s.edefTimeInit_s = *edefTimeInit_ps;
    checkArg->s.edefTime_s     = *edefTime_ps;
    checkArg->s.edefAllDone    = edefAllDone;
    checkArg->s.edefAvgDone    = edefAvgDone;
    checkArg->s.edefSevr       = edefSevr;
    checkArg->s.edefIdx        = edefIdx;
    /* If queue is full, then forget about checking BSA for this pulse. */
    return eventTaskTrySendWork( bsaCheckerWorker, (void*) checkArg );
}


/*=============================================================================

  Name: bsaChecker

  Abs:  Beam Synchronous Acquisition Checker.  It is called by the 360hz fiducial
        task right before moving the pipeline. It gives information to bsaCheckerDevices
	that is called by evrEventTask at sligthly lower priority to fill in any missing
        data for any EDEF requesting new data and data for the previous acquisition
	is not yet received.

  Args: None.

  Rem:

  Ret:  0 = OK, -1 = Mutex problem .

==============================================================================*/

void bsaChecker(void *uarg, const BsaTimingData *pBsaData)
{
epicsTimeStamp edefTimeInit_s;
epicsTimeStamp edefTime_s;
epicsUInt32    edefAllDone;
int            edefAvgDone;
epicsEnum16    edefSevr;
int            status = 0;
int            idx;
unsigned long  edefMask;

	if ( INSERT_NAN != NaNflag ) {
		return;
	}

	edefAllDone  = pBsaData->edefAllDoneMask;

	if ( pBsaData->edefActiveMask  || edefAllDone ) {
		for ( idx = 0, edefMask = 1; idx < EDEF_MAX; idx++, edefMask <<= 1 ) {
			/* If EDEF active on the next pulse? Or finished? */
			if ( (pBsaData->edefActiveMask | edefAllDone) & edefMask ) {
				/* Get EDEF information for the last acquistion. */
				if (evrTimeGetFromEdef(idx, &edefTime_s, &edefTimeInit_s, &edefAvgDone, &edefSevr)) {
					status = -1;
					continue;
				}
				/* Get EDEF information regarding the queue called in bsaChecker, it returns -1 if the queue is full. */
				status = evrBsaMessage(&edefTimeInit_s,
						&edefTime_s,
						edefAllDone, edefAvgDone, edefSevr, idx);
				if (status == -1) QueueFullCounter++;
			}/* end of EDEF active on next pulse */
		}/* end of EDEF loop */
	}/* end of any EDEF active on next pulse */
}/*end bsaChecker*/


/*============================================================================
 *
 *   Name: bsaCheckerDevices
 *
 *     Abs:  Beam Synchronous Acquisition Checker.  It is called by the evrEventTask.
 *           It fills in any missing data for any EDEF requesting new data
 *           and data for the previous acquisition is not yet received.
 *
 *     Args: Type                Name             Access     Description
 *         ------------------- -----------       ---------- ----------------------------
 *         epicsTimeStamp *    edefTimeInit_ps    Read       Data timestamp
 *         epicsTimeStamp *    edefTime_ps        Read       Data timestamp
 *         epicsUInt32         edefAllDone        Read	     EDEF flag acquisition
 *         int                 edefAvgDone	  Read       EDEF flag average done
 *         epicsEnum16         edefSevr           Read       EDEF severity
 *	   int		       edefIdx		  Read       EDEF index
 *     Rem:
 *
 *     Ret:  0 = OK, -1 = Mutex problem .
 *
==============================================================================*/
int bsaCheckerDevices(epicsTimeStamp *edefTimeInit_ps,
                      epicsTimeStamp *edefTime_ps,
                      epicsUInt32    edefAllDone,
                      int            edefAvgDone,
                      epicsEnum16    edefSevr,
		      int 	     edefIdx)
{
  bsaDevice_ts  *dev_ps;
  bsa_ts        *bsa_ps;
  /* Now go through all devices and check if they haven't provided data
   *      for the last acquisition. */
  if ((!bsaRWMutex_ps) || epicsMutexLock(bsaRWMutex_ps))
    return -1;
  dev_ps = (bsaDevice_ts *)ellFirst(&bsaDeviceList_s);
  while (dev_ps) {
    /* Fill in invalid data if the last time the device was processed is not the same as
     *        the last requested acquisition */
    bsa_ps = &dev_ps->bsa_as[edefIdx];
    if ((bsa_ps->timeData.secPastEpoch != edefTime_ps->secPastEpoch) ||
        (bsa_ps->timeData.nsec         != edefTime_ps->nsec)) {
      bsaProcessor(edefTime_ps, 0.0, SOFT_ALARM, INVALID_ALARM, dev_ps->noAverage,
                   edefTimeInit_ps, edefAvgDone, edefSevr, bsa_ps);
      /* Update diagnostics counter for missing data. When it reaches the limit, then it gets reset. */
      if (bsa_ps->missing < EVR_MAX_MISS ) {
        bsa_ps->missing++;
      } else {
        bsa_ps->missing = 0;
      }
    }
    dev_ps = (bsaDevice_ts *)ellNext(&dev_ps->node);
  }
  epicsMutexUnlock(bsaRWMutex_ps);
  return 0;
}


/*=============================================================================

  Name: bsaSecnInit

  Abs:  Beam Synchronous Acquisition Processing Initialization for a Device

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        char *              secnName    Read       BSA Device Name
        int                 noAverage   Read       Skip averaging (EVG IOC only)
        void **             dev_pps     Write      BSA Device Structure Pointer

  Rem:

  Ret:  0 = OK, -1 = Mutex lock or memory allocation error

==============================================================================*/

int bsaSecnInit(char  *secnName,
                int    noAverage,
                void **dev_pps)
{
  bsaDevice_ts *dev_ps = 0;

  if ((!bsaRWMutex_ps) || epicsMutexLock(bsaRWMutex_ps))
    return -1;
  /* Check if device name is already registered. */
  dev_ps = (bsaDevice_ts *)ellFirst(&bsaDeviceList_s);
  while(dev_ps) {
    if(strcmp(dev_ps->name, secnName)==0) break;
    dev_ps = (bsaDevice_ts *)ellNext(&dev_ps->node);
  }
  if (!dev_ps) {
    dev_ps = calloc(1,sizeof(bsaDevice_ts));
    if (dev_ps) {
      strcpy(dev_ps->name, secnName);
      ellAdd(&bsaDeviceList_s,&dev_ps->node);
    }
  }
  epicsMutexUnlock(bsaRWMutex_ps);
  *dev_pps = dev_ps;
  if (dev_ps) {
    if (noAverage) dev_ps->noAverage = 1;
    return 0;
  }
  return -1;
}

/*=============================================================================

  Name: bsaInit

  Abs:  Beam Synchronous Acquisition Processing Global Initialization

  Args: None.

  Rem:  Always succeeds (or raises fatal error)

  Ret:  0

==============================================================================*/

static int bsaInit(int pass)
{
	if ( 0 == pass ) {
		if (!bsaRWMutex_ps) {
			bsaRWMutex_ps = epicsMutexMustCreate();
			bsaFLMutex_ps = epicsMutexMustCreate();
			ellInit(&bsaDeviceList_s);
			RegisterBsaTimingCallback(bsaChecker, 0);
		}
	}
  return 0;
}

/*=============================================================================

  Name: read_bsa

  Abs:  Beam Synchronous Acquisition Record Update
        Updates average and RMS values of Secondary
        for PRIM:LOCA:UNIT:$SECN$MDID.VAL

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        bsaRecord *         pbsa        Read/Write BSA Record

  Rem:

  Ret:  0 = OK

==============================================================================*/

static long read_bsa(bsaRecord *pbsa)
{
  bsa_ts *bsa_ps = (bsa_ts *)pbsa->dpvt;
  short reset    = 0;
  int   noread   = 1;
  int   dosqrt   = 0; /* Flag =1 then apply the sqrt to calculate the variance */
  epicsEnum16 dstat = UDF_ALARM;      /* data status */
  epicsEnum16 dsevr = INVALID_ALARM;  /* data severity */

  /* Lock and update */
  if (bsa_ps && bsaRWMutex_ps && (!epicsMutexLock(bsaRWMutex_ps))) {
    if (pbsa->res) {
      pbsa->res        = 0;
      pbsa->nord       = 0;
      bsa_ps->nochange = 0;
      bsa_ps->noread   = 0;
      bsa_ps->missing  = 0;
    }
    if (bsa_ps->readFlag) {
      bsa_ps->readFlag = 0;
      noread           = 0;
      pbsa->val[0] = bsa_ps->val;
      if (bsa_ps->cnt > 1) dosqrt   = 1;
      else if (bsa_ps->cnt <= 0) pbsa->val[0] = epicsNAN;
      pbsa->rms[0]  = bsa_ps->rms;
      pbsa->cnt[0]  = bsa_ps->cnt;
      pbsa->time = bsa_ps->time;
      pbsa->nord = 1;
      pbsa->pid[0]  = PULSEID(bsa_ps->time);
      pbsa->noch = bsa_ps->nochange;
      pbsa->nore = bsa_ps->noread;
      pbsa->miss = bsa_ps->missing;
      pbsa->rcnt = bsa_ps->readcnt;
      dstat      = bsa_ps->stat;
      dsevr      = bsa_ps->sevr;
    }
    if (bsa_ps->reset) {
      bsa_ps->reset = 0;
      reset         = 1;
    }
    epicsMutexUnlock(bsaRWMutex_ps);
    if (dosqrt == 1) pbsa->rms[0] = sqrt(pbsa->rms[0]);
  }
  /* Read alarm if there was nothing to read.
     Soft alarm if there were no valid inputs to the average.
     Else set stat/sevr to max values */
  if (noread) {
    pbsa->val[0]  = epicsNAN;
    pbsa->rms[0]  = 0.0;
    pbsa->cnt[0]  = 0;
    epicsTimeGetEvent(&pbsa->time, 0);
    recGblSetSevr(pbsa,READ_ALARM,INVALID_ALARM);
  } else recGblSetSevr(pbsa,dstat,dsevr);

  /* Reset compress records if requested */
  if (reset) {
    dbPutLink(&pbsa->vres, DBR_SHORT, &reset, 1);
    dbPutLink(&pbsa->rres, DBR_SHORT, &reset, 1);
    dbPutLink(&pbsa->cres, DBR_SHORT, &reset, 1);
    dbPutLink(&pbsa->pres, DBR_SHORT, &reset, 1);
  }
  return 0;
}

static long init_record(dbCommon *prec, int noAverage, DBLINK *link)
{
  if (link->type != INST_IO) {
    errlogPrintf("init_record (%s): INP is not INST_IO\n", prec->name);
    return S_db_badField;
  }
  bsaSecnInit(link->value.instio.string, noAverage, &prec->dpvt);
  if (!prec->dpvt) {
    errlogPrintf("init_record (%s): cannot allocate DPVT\n", prec->name);
    return S_dev_noMemory;
  }
  return 0;
}

static long init_bsa_record(bsaRecord *pbsa)
{
  long status = init_record((dbCommon *)pbsa, pbsa->noav, &pbsa->inp);
  if (status) return status;

  if ((pbsa->edef <= 0) || (pbsa->edef > EDEF_MAX)) {
    errlogPrintf("init_bsa_record (%s): Invalid EDEF %d\n",
                 pbsa->name, pbsa->edef);
    return S_db_badField;
  }
  pbsa->dpvt    = &((bsaDevice_ts *)pbsa->dpvt)->bsa_as[pbsa->edef-1];
  pbsa->pidu[0] = 0;
  return 0;
}

static long get_ioint_info(int cmd, bsaRecord *pbsa, IOSCANPVT *ppvt)
{
  bsa_ts *bsa_ps = (bsa_ts *)(pbsa->dpvt);

  if (bsa_ps) {
      if (bsa_ps->ioscanpvt == 0) scanIoInit(&bsa_ps->ioscanpvt);
      *ppvt = bsa_ps->ioscanpvt;
    } else {
      *ppvt = 0;
    }
    return 0;
}

static long init_ao_record(aoRecord *pao)
{
  return (init_record((dbCommon *)pao, 0, &pao->out));
}

static long write_ao(aoRecord *pao)
{
  long status = 0;


  epicsEnum16      input_status, input_severity;
  epicsTimeStamp   input_timestamp;

/*
  long options = DBR_STATUS | DBR_TIME;
  struct {
	DBRstatus
	DBRtime
  } options_s;
*/


  /* Get the input's STAT and SEVR and timestamp (but don't get value) */
  status = dbGetAlarm(&pao->dol, &input_status, &input_severity);
  if(!status) status = dbGetTimeStamp(&pao->dol, &input_timestamp);
#ifdef BSA_DEBUG
printf("Calling bsaSecnAvg %d; PID %d\n", input_severity, input_timestamp.nsec & 0x1ffff);
#endif
  if(!status) status = bsaSecnAvg(&input_timestamp, pao->val, input_status, input_severity, 0, pao->dpvt);

  if (status) recGblSetSevr(pao,WRITE_ALARM,INVALID_ALARM);
  return status;

}

/*Set Method for the NaNflag used by the ioc */
void setNaN_flag(int flag){
        if ((flag==0) || (flag==1)){
                NaNflag=flag;
        }
        else{ /*The expectation is that is called only once at startup */
                printf("setNaN_flag: illegal value %i for NAN flag. Value must be 0 or 1.",flag);
        }
}
/*Get Method for the NaNflag used by the ioc */
int getNaN_flag(void){
        return   NaNflag;
}


/* Create the device support entry tables */
typedef struct
{
  long        number;
  DEVSUPFUN   report;
  DEVSUPFUN   init;
  DEVSUPFUN   init_record;
  DEVSUPFUN   get_ioint_info;
  DEVSUPFUN   read_write;
  DEVSUPFUN   special_linconv;
} DSET;

DSET devAoBsa =
{
  6,
  NULL,
  NULL,
  init_ao_record,
  NULL,
  write_ao,
  NULL
};

DSET devBsa =
{
  6,
  NULL,
  (DEVSUPFUN)bsaInit,
  init_bsa_record,
  get_ioint_info,
  read_bsa,
  NULL
};

epicsExportAddress(dset,devBsa);
epicsExportAddress(dset,devAoBsa);
