/*=============================================================================
 
  Name: evrTime.c
           evrTimeInit       - Fiducial Processing Initialization
           evrTime           - Fiducial Processing (360Hz)
           evrTimeProcInit   - Fiducial Record Processing Initialization
           evrTimeProc       - Fiducial Record Processing  (360Hz)
           evrTimeDiag       - Fiducial Diagnostics (0.5Hz)
           evrTimeRate       - Rate Calculation for an Event (0.5Hz)
           evrTimeCount      - Update Rate Counter for an Event (ISR)
           evrTimeEvent      - Update Timestamp for an Event (Event Rate)
           evrTimeGetFromPipeline - Get Timestamp from Pipeline
           evrTimeGetFromEdef     - Get Timestamp from EDEF
           evrTimeGet        - Get Timestamp for an Event
           evrTimePutPulseID - Encode Pulse ID into a Timestamp
           evrTimeGetSystem  - Get System Time with Encoded Invalid Pulse ID
           evrTimePatternPutStart - Start New Time/Pattern Update
           evrTimePatternPutEnd   - End   New Time/Pattern Update

  Abs: This file contains all support for evr time processing
       records.
       
  Rem: All functions called by a subroutine record get passed one argument:

         psub                       Pointer to the subroutine record data.
          Use:  pointer
          Type: struct longSubRecord *
          Acc:  read/write
          Mech: reference

         All functions return a long integer.  0 = OK, -1 = ERROR.
         The subroutine record ignores the status returned by the Init
         routines.  For the calculation routines, the record status (STAT) is
         set to SOFT_ALARM (unless it is already set to LINK_ALARM due to
         severity maximization) and the severity (SEVR) is set to psub->brsv
         (BRSV - set by the user in the database though it is expected to
          be invalid).

  Auth:  
  Rev:  
-----------------------------------------------------------------------------*/

#include "copyright_SLAC.h"	/* SLAC copyright comments */
 
/*-----------------------------------------------------------------------------
 
  Mod:  (newest to oldest)  
 
=============================================================================*/

/* c includes */

#include <string.h>           /* for memset                */
#include "subRecord.h"        /* for struct subRecord      */
#include "longSubRecord.h"    /* for struct longSubRecord  */
#include "registryFunction.h" /* for epicsExport           */
#include "epicsExport.h"      /* for epicsRegisterFunction */
#include "epicsTime.h"        /* epicsTimeStamp and protos */
#include "epicsGeneralTime.h" /* generalTimeTpRegister     */
#include "generalTimeSup.h"
#include "epicsMutex.h"       /* epicsMutexId and protos   */
#include "alarm.h"            /* INVALID_ALARM             */
#include "dbScan.h"           /* for post_event            */

#include "mrfCommon.h"        /* MRF_NUM_EVENTS */    
#include "evrMessage.h"       /* EVR_MAX_INT    */    
#include "evrTime.h"       
#include "evrPattern.h"        

#include "bsaCallbackApi.h"

#define  EVR_TIME_OK 0
#define  EVR_TIME_INVALID 1

/* Pattern and timestamp table */
typedef struct {
  evrMessagePattern_ts   pattern_s;
  unsigned long          timeslot;
  unsigned long          patternStatus;
  int                    timeStatus; /* 0=OK; -1=invalid                 */
} evrTimePattern_ts;

/* EVR Timestamp table */
typedef struct {
  epicsTimeStamp      time;   /* epics timestamp:                        */
                              /* 1st 32 bits = # of seconds since 1990   */
                              /* 2nd 32 bits = # of nsecs since last sec */
                              /*           except lower 17 bits = pulsid */
  int                 status; /* 0=OK; -1=invalid                        */
  int                 count;  /* # times this event happened in last 2s  */
} evrTime_ts;

/* EDEF Timestamp table */
typedef struct {
  epicsTimeStamp      time;   /* EDEF epics timestamp:                   */
                              /* 1st 32 bits = # of seconds since 1990   */
                              /* 2nd 32 bits = # of nsecs since last sec */
                              /*           except lower 17 bits = pulsid */
  epicsTimeStamp      timeInit;/* Time of EDEF initialization            */
  int                 avgdone;/* BSA averaging is done for this pulse    */
  epicsEnum16         sevr;   /* Severity at or above which data is not
                                 used in Beam Synchronous Acquisition    */
} evrTimeEdef_ts;

/* Pattern and timestamp pipeline */
static evrTimePattern_ts   evr_as[MAX_EVR_TIME+1];
static evrTimePattern_ts  *evr_aps[MAX_EVR_TIME+1];
extern unsigned long evrFiducialTime;
unsigned long evrActiveFiducialTime = 0;

/* Event definition (BSA) patterns */
static evrTimeEdef_ts      edef_as[EDEF_MAX];

/* Event code timestamps */
static evrTime_ts          eventCodeTime_as[MRF_NUM_EVENTS+1];
/* EVR Time Timestamp RWMutex */
static epicsMutexId        evrTimeRWMutex_ps = 0;

static unsigned long msgCount         = 0; /* # fiducials processed since boot/reset */ 
static unsigned long msgRolloverCount = 0; /* # time msgCount reached EVR_MAX_INT    */ 
static unsigned long samePulseCount   = 0; /* # same pulses                          */
static unsigned long skipPulseCount   = 0; /* # skipped pulses                       */
static unsigned long pulseErrCount    = 0; /* # invalid pulses                       */
static unsigned long fiducialStatus   = EVR_TIME_INVALID;
/* Each IOC will only process records on 2 of 6 time slots.  Default to 1 and 4.  */
/* Other valid combination are 2 and 5 and 3 and 6.                               */
static long firstTimeSlot    = 1;
static long secondTimeSlot   = 4;
static long activeTimeSlot   = 0; /* 1=time slot active on the current pulse*/
static epicsTimeStamp mod720time;

static BsaTimingCallback timingCallback     = 0;
static void             *timingCallbackParm;

/*=============================================================================

  Name: evrTimeGetSystem

  Abs:  Returns system time and status of call
  
  Ret:  -1=Failed; 0 = Success
  
==============================================================================*/

static int evrTimeGetSystem (epicsTimeStamp  *epicsTime_ps, evrTimeId_te id)
{
  if ( epicsTimeGetCurrent (epicsTime_ps) ) return epicsTimeERROR;
  /* Set pulse ID to invalid */
  evrTimePutPulseID(epicsTime_ps, PULSEID_INVALID);

  return epicsTimeOK;
}

/*=============================================================================

  Name: evrTimeGetFromPipeline

  Abs:  Get the evr epics timestamp from the pipeline, defined as:
        1st integer = number of seconds since 1990  
        2nd integer = number of nsecs since last sec, except lower 17 bits=pulsid

        Optionally get the pattern.
		
  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        epicsTimeStamp * epicsTime_ps     Write    Timestamp of this pattern
        evrTimeId_te        id            Read     Timestamp pipeline index;
	                                  0=time associated w this pulse
                                          1,2,3 = time associated w next pulses
                                          4 = last active pulse
        evrModifier_ta  modifier_a        Write    First 6 longwords of the pattern
        epicsUInt32   * patternStatus_p   Write    Pattern Status (see evrPattern.h)
        epicsUInt32   * edefAvgDoneMask_p Write    EDEF average-done mask
        epicsUInt32   * edefMinorMask_p   Write    EDEF minor severity mask
        epicsUInt32   * edefMajorMask_p   Write    EDEF major severity mask

  Rem:  Routine to get the epics timestamp and pattern from the evr timestamp
        table that is populated from incoming broadcast from EVG.
        All outputs are set to zero

  Side: None.

  Ret:  -1=Failed; 0 = Success
==============================================================================*/

int evrTimeGetFromPipeline(epicsTimeStamp  *epicsTime_ps,
                           evrTimeId_te     id,
                           evrModifier_ta   modifier_a, 
                           epicsUInt32     *patternStatus_p,
                           epicsUInt32     *edefAvgDoneMask_p,
                           epicsUInt32     *edefMinorMask_p,
                           epicsUInt32     *edefMajorMask_p)
{
  evrTimePattern_ts *evr_ps;
  int status;
  int idx;

  /* Set all outputs to zero if there is any problem locking the pipeline. */
  if ((id > evrTimeActive)) {
    if (epicsTime_ps) {
      epicsTime_ps->secPastEpoch = 0;
      epicsTime_ps->nsec         = 0;
    }
    if (modifier_a) {
      for (idx = 0; idx < MAX_EVR_MODIFIER; idx++) modifier_a[idx] = 0;
      if (patternStatus_p  ) *patternStatus_p   = 0;
      if (edefAvgDoneMask_p) *edefAvgDoneMask_p = 0;
      if (edefMinorMask_p  ) *edefMinorMask_p   = 0;
      if (edefMajorMask_p  ) *edefMajorMask_p   = 0;
    }
    return epicsTimeERROR;
  }
  epicsMutexMustLock(evrTimeRWMutex_ps);
  /* Read requested time index */
  evr_ps = evr_aps[id];
  if (evr_ps->timeStatus) status = evr_ps->timeStatus;
  else                    status = fiducialStatus;
  if (epicsTime_ps) *epicsTime_ps = evr_ps->pattern_s.time;
  if (modifier_a) {
    for (idx = 0; idx < MAX_EVR_MODIFIER; idx++)
      modifier_a[idx] = evr_ps->pattern_s.modifier_a[idx];
    if (patternStatus_p  ) *patternStatus_p   = evr_ps->patternStatus;
    if (edefAvgDoneMask_p) *edefAvgDoneMask_p = evr_ps->pattern_s.edefAvgDoneMask;
    if (edefMinorMask_p  ) *edefMinorMask_p   = evr_ps->pattern_s.edefMinorMask;
    if (edefMajorMask_p  ) *edefMajorMask_p   = evr_ps->pattern_s.edefMajorMask;
  }
  epicsMutexUnlock(evrTimeRWMutex_ps);
  
  return status;
}

/*=============================================================================

  Name: evrTimeGetFromEdef

  Abs:  Get the evr epics timestamp from EDEF, defined as:
        1st integer = number of seconds since 1990  
        2nd integer = number of nsecs since last sec, except lower 17 bits=pulsid
		
  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        unsigned int        edefIdx       Read     EDEF Index (0 to EDEF_MAX-1)
        epicsTimeStamp *    edefTime_ps   Write    EDEF active timestamp
        epicsTimeStamp *    edefTimeInit_ps Write  EDEF init   timestamp
        int *               edefAvgDone_p Write    EDEF average-done flag
        epicsEnum16  *      edefSevr_p    Write    EDEF severity

  Rem:  Routine to get the epics timestamp and flags from the EDEF timestamp
        table that is populated from incoming broadcast from EVG

  Side: None

  Ret:  -1=Failed; 0 = Success
==============================================================================*/

int evrTimeGetFromEdef    (unsigned int     edefIdx,
                           epicsTimeStamp  *edefTime_ps,
                           epicsTimeStamp  *edefTimeInit_ps,
                           int             *edefAvgDone_p,
                           epicsEnum16     *edefSevr_p)
{  
  if ((edefIdx >= EDEF_MAX)) return epicsTimeERROR;
  /* if the r/w mutex is valid, and we can lock with it, read requested time index */
  epicsMutexMustLock(evrTimeRWMutex_ps);
  *edefTime_ps     = edef_as[edefIdx].time;
  *edefTimeInit_ps = edef_as[edefIdx].timeInit;
  *edefAvgDone_p   = edef_as[edefIdx].avgdone;
  *edefSevr_p      = edef_as[edefIdx].sevr;
  epicsMutexUnlock(evrTimeRWMutex_ps);
  
  return epicsTimeOK;
}

/*=============================================================================

  Name: evrTimeGet

  Abs:  Get the epics timestamp associated with an event code, defined as:
        1st integer = number of seconds since 1990  
        2nd integer = number of nsecs since last sec, except lower 17 bits=pulsid
		
  Args: Type     Name           Access	   Description
        -------  -------	---------- ----------------------------
  epicsTimeStamp * epicsTime_ps write  ptr to epics timestamp to be returned
  unsigned int   eventCode      read   Event code 0 to 255.
	                                  0,1=time associated w this pulse
                                          (event code 1 = fiducial)
                                          1 to 255 = EVR event codes

  Rem:  Routine to get the epics timestamp from the event code timestamp table
        that is populated by the EVR event IRQ handler.  If the IRQ is off or
        the event code has not been sent for a while, the timestamp will be
        very old.

  Side: 

  Ret:  -1=Failed; 0 = Success
==============================================================================*/

int evrTimeGet (epicsTimeStamp  *epicsTime_ps, unsigned int eventCode)
{
  int status;

  /* Hack event code to get pre-bundled general-time behavior */
  if ( (unsigned int)epicsTimeEventBestTime == eventCode )
	eventCode = 0;

#ifdef BSA_DEBUG
printf("evrTimeGet for %d\n", eventCode);
#endif
  
  if ((eventCode > MRF_NUM_EVENTS)) {
    return epicsTimeERROR;
  /* if the r/w mutex is valid, and we can lock with it, read requested time index */
  }
  epicsMutexMustLock(evrTimeRWMutex_ps);
  *epicsTime_ps = eventCodeTime_as[eventCode].time;
  status = eventCodeTime_as[eventCode].status;
#ifdef BSA_DEBUG
printf("evrTimeGet for %d ret PID %d, status %d\n", eventCode, eventCodeTime_as[eventCode].time.nsec & 0x1ffff, status);
#endif
  epicsMutexUnlock(evrTimeRWMutex_ps);
  
  return status; 
}

/*=============================================================================

  Name: evrTimePutPulseID

  Abs:  Puts pulse ID in the lower 17 bits of the nsec field
        of the epics timestamp.
        
  Ret:  0 = Success
  
==============================================================================*/ 
int evrTimePutPulseID (epicsTimeStamp  *epicsTime_ps, unsigned int pulseID)
{
  epicsTime_ps->nsec &= UPPER_15_BIT_MASK;
  epicsTime_ps->nsec |= pulseID;
  if (epicsTime_ps->nsec >= NSEC_PER_SEC) {
    epicsTime_ps->secPastEpoch++;
    epicsTime_ps->nsec -= NSEC_PER_SEC;
    epicsTime_ps->nsec &= UPPER_15_BIT_MASK;
    epicsTime_ps->nsec |= pulseID;
  }
  return epicsTimeOK;
}

/*===============================================
  Wrapper function for generalTime: Temporal
=================================================*/
static int evrTimeGet_gtWrapper(epicsTimeStamp *epicsTime_ps, int eventCode)
{
    return evrTimeGet(epicsTime_ps, (unsigned int)eventCode);
}

static int evrTimeGetSystem_gtWrapper(epicsTimeStamp *epicsTime_ps, int eventCode)
{
    return evrTimeGetSystem(epicsTime_ps, 0);
}




/*=============================================================================

  Name: evrTimeInit

  Abs:  Creates the evrTimeRWMutex_ps read/write mutex 
        and initializes the timestamp arrays.
	The evr timestamp table is initialized to system time, invalid status,
        and invalid pattern. During processing, if a timestamp status goes
	invalid, the time is overwritten to the last good evr timestamp.
        
  Args: Type	            Name        Access	   Description
        ------------------- -----------	---------- ----------------------------
        epicsInt32      firstTimeSlotIn Read       1st timeslot for this IOC (-1,0,1,2,3)
                                                   (0 = don't use 1st timeslot)
        epicsInt32     secondTimeSlotIn Read       2nd timeslot for this IOC (-1,0,4,5,6)
                                                   (0 = don't use 2nd timeslot)
        remark) for 360Hz resolution timestamp, both firstTimeSlotIn and secondTimeSlotIn should be -1

  Side: EVR Time Timestamp table
  pulse pipeline n  , status - 0=OK; -1=invalid
  0 = current pulse (Pn), status
  1 = next (upcoming) pulse (Pn-1), status
  2 = two pulses in the future (Pn-2), status
  3 = three pulses in the future (Pn-3), status

  Status is invalid when
  1) Bootup - System time is entered (as opposed to evr timestamp).
  2) the PULSEID of most recent index is is the same as the previous index.
  3) pattern waveform record invalid - timestamp is last good time
  
  Ret:  -1=Failed; 0 = Success
==============================================================================*/ 
int evrTimeSetSlots(epicsInt32 firstTimeSlotIn, epicsInt32 secondTimeSlotIn)
{
  int timeslotDiff;

  if ((firstTimeSlotIn  >= 0) && (secondTimeSlotIn >= 0) &&
      (firstTimeSlotIn  <= TIMESLOT_MAX) &&
      (secondTimeSlotIn <= TIMESLOT_MAX) &&
      ((firstTimeSlotIn != 0) || (secondTimeSlotIn != 0))) {
    timeslotDiff = firstTimeSlotIn - secondTimeSlotIn;
    if ((firstTimeSlotIn == 0) || (secondTimeSlotIn == 0) ||
        (timeslotDiff ==  TIMESLOT_DIFF) ||
        (timeslotDiff == -TIMESLOT_DIFF)) {
      firstTimeSlot = firstTimeSlotIn;
      secondTimeSlot = secondTimeSlotIn;
    }
  }

  if((firstTimeSlotIn == 0) && (secondTimeSlotIn == 0)) { /* for 360Hz resolution timestamp */
    firstTimeSlot = firstTimeSlotIn;
    secondTimeSlot = secondTimeSlotIn;
  }
  return epicsTimeOK;
}

int evrTimeInit()
{
int idx;
	/* create read/write mutex around evr timestamp table array */
	if (!evrTimeRWMutex_ps) {
		if (evrTimeGetSystem(&mod720time, evrTimeCurrent))
			return epicsTimeERROR;
		/* init patterns in pipeline */
		for (idx=0; idx<MAX_EVR_TIME+1; idx++) {
			memset(&evr_as[idx].pattern_s, 0, sizeof(evrMessagePattern_ts));
			evr_as[idx].pattern_s.time = mod720time;
			evr_as[idx].timeStatus     = epicsTimeERROR;
			evr_aps[idx] = evr_as + idx;
		}
		/* Init EDEF pattern array */
		for (idx=0; idx<EDEF_MAX; idx++) {
			memset(&edef_as[idx], 0, sizeof(evrTimeEdef_ts));
			edef_as[idx].timeInit = mod720time;
			edef_as[idx].sevr = INVALID_ALARM;
		}
		/* init timestamp structures to invalid status & system time*/
		for (idx=0; idx<=MRF_NUM_EVENTS; idx++) {
			eventCodeTime_as[idx].time   = mod720time;
			eventCodeTime_as[idx].status = epicsTimeERROR;
			eventCodeTime_as[idx].count  = 0;
		}

		evrTimeRWMutex_ps = epicsMutexMustCreate();

		/* For IOCs that support iocClock (RTEMS and vxWorks), register
		   evrTimeGet with generalTime so it is used by epicsTimeGetEvent */
#ifdef EVR_DRIVER_SUPPORT
		if(generalTimeRegisterEventProvider("evrTimeGet", 1000, (TIMEEVENTFUN) evrTimeGet_gtWrapper))
			return epicsTimeERROR;

		if(generalTimeRegisterEventProvider("evrTimeGetSystem", 2000, (TIMEEVENTFUN) evrTimeGetSystem_gtWrapper))
			return epicsTimeERROR;
#endif
	}
	return 0;
}
/* For IOCs that don't support iocClock (linux), supply a dummy
   iocClockRegister to keep the linker happy. */
#ifndef EVR_DRIVER_SUPPORT
void iocClockRegister(TIMECURRENTFUN getCurrent,
                      TIMEEVENTFUN   getEvent) 
{
}
#endif

/*=============================================================================

  Name: evrTime

  Abs:  Processes every time the fiducial event code is received @ 360 Hz.
        Performs evr error checking, and then advances 
        the timestamp/pattern table in the evrTime_as array.
 
  Error Checking:

  Pulse ID error (any PULSEID of 0 or non-consecutive PULSEIDs) - 
    Set appropriate counters. Set error flag.
  Set EVR timestamp status to invalid if PULSEIDs are not changing.
  Error advancing EVR timestamps - set error flag used later to disable 
    triggers (EVR) or event codes (EVG). 
		
  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        epicsUInt32         mpsModifier read       MPS pattern modifier

  Rem:  

  Side: Upon entry, pattern is:
        n P0   don't care about n, this was acted upon for beam occuring last 
	       fiducial
        n-1 P1 this is the pattern for this fiducial.
        n-2 P2
        n-3 P3
	Algorithm: 
	During proper operation, every pulse in the pattern should be consecutive;
	differing by "1".  Error until all three pulseids are consecutive.
	Upon error, set error flag that will disable events/triggers.	    

  Ret:  -1=Failed; 0 = Success
   
==============================================================================*/
int evrTime(epicsUInt32 mpsModifier)
{
  int idx;
  evrTimePattern_ts *evr_ps;
  epicsInt32  pulseid, pulseidNext1, pulseidNext2, pulseidDiff;
  unsigned long timeslot;
  unsigned long edefMask;
  BsaTimingData bsaData;
  epicsUInt32   minorMask, majorMask, allDoneMask;
  BsaTimingCallback  cb;
  void              *cbArg;

  /* Keep a count of fiducials and reset before overflow */
  if (msgCount < EVR_MAX_INT) {
    msgCount++;
  } else {
    msgRolloverCount++;
    msgCount = 0;
  }

  epicsMutexMustLock(evrTimeRWMutex_ps);

  if ( (cb = timingCallback) ) {

	cbArg = timingCallbackParm;

	evr_ps = evr_aps[evrTimeNext1];

	bsaData.pulseId         = PULSEID( evr_ps->pattern_s.time );
	bsaData.timeStamp       = evr_ps->pattern_s.time;
	minorMask               = evr_ps->pattern_s.edefMinorMask;
	majorMask               = evr_ps->pattern_s.edefMajorMask;

	bsaData.edefInitMask    = evr_ps->pattern_s.edefInitMask;
	bsaData.edefActiveMask  = evr_ps->pattern_s.modifier_a[MOD5_IDX] & MOD5_EDEF_MASK;
	bsaData.edefAvgDoneMask = evr_ps->pattern_s.edefAvgDoneMask;
	allDoneMask             = (minorMask >> 20) & 0x003ff;
	allDoneMask            |= (majorMask >> 10) & 0xffc00;
	bsaData.edefAllDoneMask = allDoneMask;
	/* BsaCore buffers multiple results and posts to compress when NELM or timeout is reached */
	bsaData.edefUpdateMask  = 0;
	bsaData.edefMinorMask   = minorMask & MOD5_EDEF_MASK;
	bsaData.edefMajorMask   = majorMask & MOD5_EDEF_MASK;

  }

  epicsMutexUnlock(evrTimeRWMutex_ps);

	/* Race condition; if they change the callback here then we are still executing
	 * the old one. Since callbacks are installed only once during init this seems
	 * acceptable.
	 */
	if ( cb ) {
		cb( cbArg, &bsaData );
	}

  epicsMutexMustLock(evrTimeRWMutex_ps);

    fiducialStatus = EVR_TIME_OK;
    /* Advance the evr pattern in the pipeline.  Update MPS
       information (which is not pipelined) into the pattern. */
    evr_ps = evr_aps[evrTimeCurrent];
    for (idx=0;idx<evrTimeNext3;idx++) {
      evr_aps[idx] = evr_aps[idx+1];
      evr_aps[idx]->pattern_s.modifier_a[MOD6_IDX] = mpsModifier;
    }
    evr_aps[evrTimeNext3] = evr_ps;
    evr_aps[evrTimeNext3]->timeStatus = epicsTimeERROR;
    evr_ps = evr_aps[evrTimeCurrent];
#ifdef BSA_DEBUG
printf("Pipeline [0]: %d\n", evr_aps[evrTimeCurrent]->pattern_s.time.nsec & 0x1ffff);
printf("Pipeline [1]: %d\n", evr_aps[evrTimeNext1]->pattern_s.time.nsec & 0x1ffff);
printf("Pipeline [2]: %d\n", evr_aps[evrTimeNext2]->pattern_s.time.nsec & 0x1ffff);
#endif
    /* Update the EDEF array for any initialized or active EDEF -
       this array is used later by BSA processing */
    if (evr_aps[evrTimeNext2]->pattern_s.edefInitMask ||
        evr_ps->pattern_s.modifier_a[MOD5_IDX] & MOD5_EDEF_MASK) {
      for (idx=0;idx<EDEF_MAX;idx++) {
        edefMask = 1 << idx;
        /* EDEF initialized? - check the newest mask so init done ASAP */
        if (evr_aps[evrTimeNext2]->pattern_s.edefInitMask & edefMask) {
          edef_as[idx].timeInit = evr_aps[evrTimeNext2]->pattern_s.time;
        }
        /* EDEF active? - set time and flags used by BSA processing later */
        if (evr_ps->pattern_s.modifier_a[MOD5_IDX] & edefMask) {
#ifdef BSA_DEBUG
printf("Setting EDEF (%d) time PId %d \n", idx, evr_ps->pattern_s.time.nsec & 0x1ffff);
#endif
          edef_as[idx].time = evr_ps->pattern_s.time;
          if    (evr_ps->pattern_s.edefAvgDoneMask & edefMask)
            edef_as[idx].avgdone = 1;
          else
            edef_as[idx].avgdone = 0;
          if      (evr_ps->pattern_s.edefMinorMask & edefMask)
            edef_as[idx].sevr = MINOR_ALARM;
          else if (evr_ps->pattern_s.edefMajorMask & edefMask)
            edef_as[idx].sevr = MAJOR_ALARM;
          else
            edef_as[idx].sevr = INVALID_ALARM;
        }
      }
    }
    pulseidNext2 = PULSEID(evr_aps[evrTimeNext2]->pattern_s.time);
    pulseidNext1 = PULSEID(evr_aps[evrTimeNext1]->pattern_s.time);
    pulseid      = PULSEID(evr_aps[evrTimeCurrent]->pattern_s.time);
    timeslot     = evr_ps->timeslot;
    /* The 3 next pulses must be valid before all is considered OK to go */
    if (evr_aps[evrTimeCurrent]->patternStatus ||
        evr_aps[evrTimeNext1]->patternStatus   ||
        evr_aps[evrTimeNext2]->patternStatus) {
      fiducialStatus = EVR_TIME_INVALID;
      pulseErrCount++;
    /* Diff between first and second and second and third must be 1 */
    /* pulse ID may have rolled over */
    } else {
      pulseidDiff = pulseidNext2 - pulseidNext1;
      if ((pulseidDiff != 1) && (pulseidDiff != -PULSEID_MAX)) {
        fiducialStatus = EVR_TIME_INVALID;
        if (pulseidDiff==0)
          ++samePulseCount; /* same pulse coming into the pipeline */
        else
          ++skipPulseCount; /* skipped pulse (non-consecutive) in the pipeline */
      } else {
        pulseidDiff = pulseidNext1 - pulseid;
        if ((pulseidDiff != 1) && (pulseidDiff != -PULSEID_MAX)) {
          fiducialStatus = EVR_TIME_INVALID;
        }
      }
    }
    if ((timeslot == 0) ||
        (firstTimeSlot == timeslot) || (secondTimeSlot == timeslot) ||
        ((firstTimeSlot == 0) && (secondTimeSlot == 0))) {
      evr_as[evrTimeActive] = *evr_ps;
      evrActiveFiducialTime = evrFiducialTime;
      activeTimeSlot = 1;
    } else {
      activeTimeSlot = 0;
    }
    /* determine if the next 3 pulses are all the same. */
    /* Same pulses means the EVG is not sending timestamps and this forces   
       record timestamps to revert to system time */
    if ((pulseidNext2==pulseidNext1) && (pulseidNext2==pulseid)) {
#ifdef BSA_DEBUG
printf("********* PID MISMATCH ************************************ (PID n %d, n+1 %d, n+2 %d)\n", pulseid, pulseidNext1, pulseidNext2);
#endif
      for (idx=0;idx<evrTimeNext3;idx++)
        evr_aps[idx]->timeStatus = epicsTimeERROR;
      eventCodeTime_as[0].status = epicsTimeERROR;
      eventCodeTime_as[EVENT_FIDUCIAL].status = epicsTimeERROR;
    } else {
      if (activeTimeSlot) {
#ifdef BSA_DEBUG
printf("Storing active pid %d\n", evr_ps->pattern_s.time.nsec & 0x1ffff);
#endif
        eventCodeTime_as[0].time   = evr_ps->pattern_s.time;
        eventCodeTime_as[0].status = evr_ps->timeStatus;
      }
#ifdef BSA_DEBUG
printf("Storing fiducial pid %d\n", evr_ps->pattern_s.time.nsec & 0x1ffff);
#endif
      eventCodeTime_as[EVENT_FIDUCIAL].time   = evr_ps->pattern_s.time;
      eventCodeTime_as[EVENT_FIDUCIAL].status = evr_ps->timeStatus;
    }
  epicsMutexUnlock(evrTimeRWMutex_ps);
  /* If we cannot lock - bad problem somewhere. */
  return epicsTimeOK;
}

/*=============================================================================

  Name: evrTimeProcInit

  Abs:  Initialization for the fiducial processing record.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        longSubRecord *     psub        read       point to subroutine record

  Rem:  Init Subroutine for IOC:LOCA:UNIT:FIDUCIAL

  Side: None.
  
  Ret:  -1=Failed; 0 = Success
==============================================================================*/ 
static int evrTimeProcInit(longSubRecord *psub)
{
  evrTimeSetSlots((epicsInt32)psub->i, (epicsInt32)psub->j);
  /* Register this record for the start of fiducial processing */
  if (evrMessageRegister(EVR_MESSAGE_FIDUCIAL_NAME, 0, (dbCommon *)psub) < 0)
    return epicsTimeERROR;  
  return epicsTimeOK;
}

/*=============================================================================

  Name: evrTimeProc

  Abs:  Record processing every time the fiducial event code is received @ 360 Hz.
        Get data from storage.
		
  Args: Type	            Name        Access	   Description
        ------------------- -----------	---------- ----------------------------
        longSubRecord *     psub        read       point to subroutine record

  Rem:  Subroutine for IOC:LOCA:UNIT:FIDUCIAL

  Side: None.
	    
  Sub Inputs/ Outputs:
   Input/Outputs:
    I - First  Active Time Slot for this IOC (1, 2, 3)
    J - Second Active Time Slot for this IOC (4, 5, 6)   
   Outputs:
    A - Fiducial High Resolution Clock Time (360hz)
    B - Active Time Slot High Resolution Clock Time (120hz)
    L - Enable/Disable flag for current pattern and timestamp update
    VAL = Error Flag

  Ret:  -1=Failed; 0 = Success
   
==============================================================================*/
static int evrTimeProc (longSubRecord *psub)
{
  psub->a = evrFiducialTime;
  psub->b = evrActiveFiducialTime;
  epicsMutexMustLock(evrTimeRWMutex_ps);
    psub->l   = activeTimeSlot?0:1;
    psub->val = fiducialStatus;
    /* See if user wants different time slots */
    if ((psub->i != firstTimeSlot) || (psub->j != secondTimeSlot)) {
      evrTimeSetSlots((epicsInt32)psub->i, (epicsInt32)psub->j);
      psub->i = firstTimeSlot;
      psub->j = secondTimeSlot;
    }
  epicsMutexUnlock(evrTimeRWMutex_ps);
  if (psub->val) return epicsTimeERROR;
  return epicsTimeOK;
}

/*=============================================================================

  Name: evrTimeDiag

  Abs:  Expose counters.
        This subroutine is for status only and should update at a low
        rate like 1Hz.
  
  Inputs:
  R - Counter Reset Flag

  Outputs:
  A  Fiducial Delay Time (us) - due to the epicsEventSignal
  B  Minimum of Fiducial Delay Time (us)
  C  Maximum of Fiducial Delay Time (us)

  D  Absolute Fiducial Delay (us) - based on the evr clock outer
  E  Minimum of Absolute Fiducial Delay (us)
  F  Maximum of Absolute Fiducial Delay (us) 

  G  Absolute start time for the data buffer handling (us)  
  H  Minimum of the start time for the data buffer handling (us)
  I  Maximum of the start time for the data buffer handling (us)
  
  J  Pended message for the evrEvent task
  K  Maximum number of pended message for the evrEventTask

  L Spare

  M  fiducial counter
  N  Number of times M has rolled over
  O  Number of same pulses
  P  Number of skipped pulses
  Q  Spare
  S  Number of invalid pulses
  T  Number of fiducial interrupts
  U  Number of times T has rolled over
  V  Minimum Fiducial Delta Start Time (us)
  W  Maximum Fiducial Delta Start Time (us)
  X  Average Fiducial Processing Time  (us)
  Y  Number of missed fiducials
  Z  Maximum Fiducial Processing Time  (us)
  VAL = Error flag from evrTime
  
  Ret:  -1=Failed; 0 = Success
==============================================================================*/ 

static long evrTimeDiag (longSubRecord *psub)
{
  epicsUInt32  dummy;
  
  psub->val = fiducialStatus;
  psub->m = msgCount;          /* # fiducials processed since boot/reset */
  psub->n = msgRolloverCount;  /* # time msgCount reached EVR_MAX_INT    */
  psub->o = samePulseCount;
  psub->p = skipPulseCount;
  psub->s = pulseErrCount;
  evrMessageCounts(EVR_MESSAGE_FIDUCIAL,
                   &psub->t,&psub->u,&dummy  ,&psub->y,&dummy,
                   &dummy,  &psub->v,&psub->w,&psub->x,&psub->z);
  evrMessageCountsFiducial(EVR_MESSAGE_FIDUCIAL,
                           &psub->a, &psub->b, &psub->c);
  evrMessageCountsClockCounter(EVR_MESSAGE_FIDUCIAL,
                               &psub->d, &psub->e, &psub->f);
  evrMessageCountsClockCounter(EVR_MESSAGE_PATTERN,
                               &psub->g, &psub->h, &psub->i);
  evrMessageCountsQ(EVR_MESSAGE_FIDUCIAL,
                    &psub->j, &psub->k);

  if (psub->r > 0) {
    psub->r           = 0;
    msgCount          = 0;
    msgRolloverCount  = 0;
    samePulseCount    = 0;
    skipPulseCount    = 0;
    pulseErrCount     = 0;
    evrMessageCountReset(EVR_MESSAGE_FIDUCIAL);
  }
  return epicsTimeOK;
}

/*=============================================================================

  Name: evrTimeRate

  Abs:  Calculate rate that an event code is received by the EVR ISR.
        It is assumed this subroutine processes at 0.5hz.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        subRecord *         psub        read       point to subroutine record

  Rem:  Subroutine for IOC:LOCA:UNIT:NAMERATE

  Inputs:
       E - Event Code
     
  Outputs:
       VAL = Rate in Hz
  
  Ret:  -1=Failed; 0 = Success

==============================================================================*/
static long evrTimeRate(subRecord *psub)
{
  int eventCode = psub->e + 0.5;

  if ((eventCode > 0) && (eventCode <= MRF_NUM_EVENTS)) {
    epicsMutexMustLock(evrTimeRWMutex_ps);
      psub->val = eventCodeTime_as[eventCode].count;
      eventCodeTime_as[eventCode].count = 0;
    epicsMutexUnlock(evrTimeRWMutex_ps);
      psub->val /= MODULO720_SECS;
      return epicsTimeOK;
  }
  psub->val = 0.0;
  return epicsTimeERROR;
}

/*=============================================================================

  Name: evrTimeCount

  Abs:  Increment a counter for an event code.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        unsigned int        eventCode    read       Event Code

  Rem:  This routine is called at interrupt level.
  
  Ret:  -1=Failed; 0 = Success

==============================================================================*/
int evrTimeCount(unsigned int eventCode)
{
  if ((eventCode > 0) && (eventCode <= MRF_NUM_EVENTS)) {
    eventCodeTime_as[eventCode].count++;
    return epicsTimeOK;
  }
  return epicsTimeERROR;
}

/*=============================================================================

  Name: evrTimeEvent

  Abs:  Update the event code timestamp and increment a diagnostic counter.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        longSubRecord *     psub        read       point to subroutine record

  Rem:  Subroutine for IOC:LOCA:UNIT:NAMECNT

  Inputs:
       A - Event code
       VAL - Counter that is updated every time the event is received
     
  Outputs:
       VAL - Incremented by 1
  
  Ret:  -1=Failed; 0 = Success

==============================================================================*/
static long evrTimeEvent(longSubRecord *psub)
{
  /* Rollover if value gets too big */
  if (psub->val < EVR_MAX_INT) psub->val++;
  else                         psub->val = 1;
  if ((psub->a <= 0) || (psub->a > MRF_NUM_EVENTS))
    return epicsTimeERROR;
  epicsMutexMustLock(evrTimeRWMutex_ps);
#ifdef BSA_DEBUG
printf("Setting event time (SUB) for %d PID %d\n", psub->a, evr_aps[evrTimeCurrent]->pattern_s.time.nsec & 0x1ffff);
#endif
    eventCodeTime_as[psub->a].time   = evr_aps[evrTimeCurrent]->pattern_s.time;
    eventCodeTime_as[psub->a].status = evr_aps[evrTimeCurrent]->timeStatus;
  epicsMutexUnlock(evrTimeRWMutex_ps);
  return epicsTimeOK;
}

long evrTimeEventProcessing(epicsInt16 eventNum)
{

  epicsMutexMustLock(evrTimeRWMutex_ps);
#ifdef BSA_DEBUG
printf("Setting event time for %d PID %d\n", eventNum, evr_aps[evrTimeCurrent]->pattern_s.time.nsec & 0x1ffff);
#endif
    eventCodeTime_as[eventNum].time   = evr_aps[evrTimeCurrent]->pattern_s.time;
    eventCodeTime_as[eventNum].status = evr_aps[evrTimeCurrent]->timeStatus;
  epicsMutexUnlock(evrTimeRWMutex_ps);
  return epicsTimeOK;
}



/*=============================================================================

  Name: evrTimePatternPutStart

  Abs:  Lock Mutex and Return pointer to newest pattern

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        evrMessagePattern_ts ** pattern_pps Write  Pointer to pattern
        unsigned long **        timeslot_pp Write  Pointer to timeslot
        unsigned long **     patternStatus_pp Write  Pointer to status
        epicsTimeStamp *     mod720time_pps Write  Pointer to mod720 timestamp

  Rem:  The caller MUST call evrTimePatternPutEnd after the pattern is filled in.

  Side: Mutex is left locked.
  
  Ret:  -1=Failed; 0 = Success

==============================================================================*/
int evrTimePatternPutStart(evrMessagePattern_ts **pattern_pps,
                           unsigned long        **timeslot_pp,
                           unsigned long        **patternStatus_pp,
                           epicsTimeStamp       **mod720time_pps)
{
  evrTimePattern_ts *evr_ps;
  
  epicsMutexLock(evrTimeRWMutex_ps);
    evr_ps                 = evr_aps[evrTimeNext3];
    evr_ps->timeStatus     = epicsTimeOK;
    evr_ps->pattern_s.time = evr_aps[evrTimeNext2]->pattern_s.time;
    *pattern_pps           = &evr_ps->pattern_s;
    *timeslot_pp           = &evr_ps->timeslot;
    *patternStatus_pp      = &evr_ps->patternStatus;
    *mod720time_pps        = &mod720time;
    return epicsTimeOK;
}

/*=============================================================================

  Name: evrTimePatternPutEnd

  Abs:  Post modulo 720 event and update timestamp if requested.  Unlock Mutex.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        int                 modulo720Flag read     Modulo 720 Flag

  Rem:  

  Side: Event posted.
  
  Ret:  -1=Failed; 0 = Success

==============================================================================*/
int evrTimePatternPutEnd(int modulo720Flag)
{
  /* Post the modulo-720 sync event if the pattern has that bit set */
  if (modulo720Flag) {
    post_event(EVENT_MODULO720);
    epicsTimeGetCurrent(&mod720time);
  }
  epicsMutexUnlock(evrTimeRWMutex_ps);
  return epicsTimeOK;
}

int RegisterBsaTimingCallback(BsaTimingCallback cb, void *uarg)
{
  epicsMutexMustLock(evrTimeRWMutex_ps);
	timingCallback     = cb;
	timingCallbackParm = uarg;
  epicsMutexUnlock(evrTimeRWMutex_ps);
  return 0;
}

epicsRegisterFunction(evrTimeProcInit);
epicsRegisterFunction(evrTimeProc);
epicsRegisterFunction(evrTimeDiag);
epicsRegisterFunction(evrTimeRate);
epicsRegisterFunction(evrTimeEvent);
