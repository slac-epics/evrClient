/*=============================================================================
 
  Name: evrTime.h

  Abs:  This include file contains external prototypes for EVR 
        timestamp/pattern routines that allow access to the EVR timestamp/pattern
        table (timestamp/pattern received in the EVG broadcast) 

  Auth: 17 NOV-2006, drogind created 
 
-----------------------------------------------------------------------------*/
#include "copyright_SLAC.h"    
/*----------------------------------------------------------------------------- 
  Mod:  (newest to oldest)  
        DD-MMM-YYYY, My Name:
           Changed such and such to so and so. etc. etc.
        DD-MMM-YYYY, Your Name:
           More changes ... The ordering of the revision history 
           should be such that the NEWEST changes are at the HEAD of
           the list.
 
=============================================================================*/

#ifndef INCevrTimeH
#define INCevrTimeH 

#include "bsaCallbackApi.h"

#ifdef __cplusplus
extern "C" {
#endif

#include    "epicsTime.h"          /* epicsTimeStamp       */
#include    "registryFunction.h"   /* REGISTRYFUNCTION     */
  
/* Masks used to decode pulse ID from the nsec part of the timestamp   */
#define UPPER_15_BIT_MASK       (0xFFFE0000)    /* (2^32)-1 - (2^17)-1 */
#define LOWER_17_BIT_MASK       (0x0001FFFF)    /* (2^17)-1            */
/* Pulse ID Definitions */
#define PULSEID(time)           ((time).nsec & LOWER_17_BIT_MASK)
#define PULSEID_BIT17           (0x00020000)    /* Bit up from pulse ID*/
#define PULSEID_INVALID         (0x0001FFFF)    /* Invalid Pulse ID    */
#define PULSEID_MAX             (0x0001FFDF)    /* Rollover value      */
#define PULSEID_RESYNC          (0x0001E000)    /* Resync Pulse ID     */
#define NSEC_PER_SEC            1E9             /* # nsec in one sec   */
  
#define MODULO720_SECS          2               /* # sec for MODULO720 */
#define TIMESLOT_MIN            1               /* Minimum time slot   */
#define TIMESLOT_MAX            6               /* Maximum time slot   */
#define TIMESLOT_DIFF           3               /* Timeslot difference */
  
/* For time ID */
typedef enum {
  evrTimeCurrent=0, evrTimeNext1=1, evrTimeNext2=2, evrTimeNext3=3, evrTimeActive
} evrTimeId_te;
#define MAX_EVR_TIME  4

/* For modifier array */
#define MAX_EVR_MODIFIER  6
typedef epicsUInt32 evrModifier_ta[MAX_EVR_MODIFIER];
  
/* Event codes - see mrfCommon.h for reserved internal event codes      */
#define EVENT_FIDUCIAL          1        /* Fiducial event code         */
#define EVENT_EXTERNAL_TRIG     100      /* External trigger event code */
#define EVENT_MODULO720         121      /* Modulo 720 event code       */
#define EVENT_MPG               122      /* MPG update event code       */
#define EVENT_MODULO36_MIN      201      /* Min modulo 36 event code    */
#define EVENT_MODULO36_MAX      236      /* Max modulo 36 event code    */
#define MODULO36_MAX            36       /* # modulo 36 event codes     */

/* For overloaded IOC, do not insert NaNs, while missing data, if this flag is = 0.*/
#define INSERT_NAN 1
#define NO_INSERT_NAN 0
int
getNaN_flag(void);

  
typedef void (*FIDUCIALFUNCTION)(void *arg);

int evrInitialize         (void);
int evrTimeRegister       (FIDUCIALFUNCTION fiducialFunc,
                           void *           fiducialArg);
int evrTimeGetFromPipeline(epicsTimeStamp  *epicsTime_ps,
                           evrTimeId_te     id,
                           evrModifier_ta   modifier_a, 
                           epicsUInt32     *patternStatus_p,
                           epicsUInt32     *edefAvgDoneMask_p,
                           epicsUInt32     *edefMinorMask_p,
                           epicsUInt32     *edefMajorMask_p);
int evrTimeGetFromEdef    (unsigned int     edefIdx,
                           epicsTimeStamp  *edefTime_ps,
                           epicsTimeStamp  *edefTimeInit_ps,
                           int             *edefAvgDone_p,
                           epicsEnum16     *edefSevr_p);
int evrTimeGet            (epicsTimeStamp  *epicsTime_ps,
                           unsigned int     eventCode);
int evrTimePutPulseID     (epicsTimeStamp  *epicsTime_ps,
                           unsigned int     pulseID);

/* Routines for timing diagnostics */
int evrGetLastFiducial( );
unsigned long long evrGetFiducialTsc();

/* Routines used only by event module and Mpg application */
#ifdef INCevrMessageH
int evrTimeSetSlots       (epicsInt32   firstTimeSlotIn,
                           epicsInt32   secondTimeSlotIn);
int evrTimeInit           ();
int evrTime               (epicsUInt32  mpsModifier);
long evrTimeEventProcessing(epicsInt16 eventNum);
int evrTimeCount          (unsigned int eventCode);
int evrTimePatternPutStart(evrMessagePattern_ts **pattern_pps,
                           unsigned long        **timeslot_pp,
                           unsigned long        **patternStatus_pp,
                           epicsTimeStamp       **mod720time_pps);
int evrTimePatternPutEnd  (int modulo720Flag);
#endif

#ifdef __cplusplus
}
#endif

#endif /*INCevrTimeH*/
