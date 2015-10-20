/*=============================================================================
 
  Name: evrMessage.h

  Abs:  This include file contains definitions and typedefs shared by
        evrMessage.c, devWFevrMessage.c, drvPnet.c, mpgPattern.c, evrPattern.c
        for EVR/PNET message access.

  Auth: 21-Dec-2006, S. Allison
        06-Feb-2007, DRogind added edefAvgDoneMask to evrMessagePattern_ts
 
-----------------------------------------------------------------------------*/
#include "copyright_SLAC.h"    
/*----------------------------------------------------------------------------- 
  Mod:  (newest to oldest)  
 
=============================================================================*/

#ifndef INCevrMessageH
#define INCevrMessageH 

#include    <stddef.h>             /* size_t                 */
#include    "dbCommon.h"           /* dbCommon               */
#include    "epicsTime.h"          /* epicsTimeStamp         */
#include    "epicsTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define  EVR_MESSAGE_PNET_NAME              "PNET"
#define  EVR_MESSAGE_PATTERN_NAME           "PATTERN"
#define  EVR_MESSAGE_DATA_NAME              "DATA"
#define  EVR_MESSAGE_FIDUCIAL_NAME          "FIDUCIAL"
#define  EVR_MESSAGE_PNET                   0
#define  EVR_MESSAGE_PATTERN                1
#define  EVR_MESSAGE_DATA                   2
#define  EVR_MESSAGE_FIDUCIAL               3
#define  EVR_MESSAGE_MAX                    4

#define  EVR_MESSAGE_PATTERN_VERSION        1
#define  EVR_MESSAGE_DATA_VERSION           1
  
/* This number should rollover every 69 days at 360 Hz */
#define  EVR_MAX_INT  (2147483647)    /* 4 byte int - 1 bit for sign */
  
#define  EVR_PNET_MODIFIER_MAX            4 /* Number of PNET modifiers   */
#define  EVR_MODIFIER_MAX                 6 /* Number of pattern modifiers*/
#define  EVR_DATA_MAX                    13 /* Number of data epicsUInt32 */

/* Defines for MODULO720 (2 second) Processing */
#define MODULO720_COUNT 720   /* # of expected pulses for MOD720RESYNC */

#if defined(_X86_) || defined(_X86_64_)
void Get_evrTicksPerUsec_for_X86(void);
#endif
  
/* Waveform header in waveform sent by the EVG and received by the EVR */
typedef struct {
  epicsUInt16 type;
  epicsUInt16 version;
} evrMessageHeader_ts;

/* Waveform sent by the MPG and read by the EVG IOC */
typedef struct {
  epicsUInt32       modifier_a[EVR_PNET_MODIFIER_MAX];
} evrMessagePnet_ts;
  
/* Waveform sent by the EVG and received by the EVR */
typedef struct {
  evrMessageHeader_ts header_s;
  epicsUInt32         modifier_a[EVR_MODIFIER_MAX];
  epicsTimeStamp      time;     /* epics timestamp:                        */
                                /* 1st 32 bits = # of seconds since 1990   */
                                /* 2nd 32 bits = # of nsecs since last sec */
                                /*           except lower 17 bits = pulsid */
  epicsUInt32         edefAvgDoneMask;
  epicsUInt32         edefMinorMask;
  epicsUInt32         edefMajorMask;
  epicsUInt32         edefInitMask;
} evrMessagePattern_ts;
  
typedef union
{
  evrMessagePnet_ts    pnet_s;
  evrMessagePattern_ts pattern_s;
  epicsUInt32          data_a[EVR_DATA_MAX];
} evrMessage_tu;

typedef enum
{
    evrMessageOK,
    evrMessageInpError,
    evrMessageDataNotAvail
    
} evrMessageReadStatus_te;

int evrMessageCreate    (char         *messageName_a, size_t  messageSize);
int evrMessageRegister  (char         *messageName_a, size_t  messageSize,
                         dbCommon     *record_ps);
evrMessageReadStatus_te
    evrMessageRead      (unsigned int  messageIdx, evrMessage_tu *message_pu);
int evrMessageWrite     (unsigned int  messageIdx, evrMessage_tu *message_pu);
int evrMessageProcess   (unsigned int  messageIdx);
int evrMessageClockCounter(unsigned int messageIdx, epicsUInt32 evrClockCounter);
int evrMessageStart     (unsigned int  messageIdx);
int evrMessageLap       (unsigned int  messageIdx);
int evrMessageEnd       (unsigned int  messageIdx);
int evrMessageQ         (unsigned int  messageIdx, int pend);
int evrMessageReport    (unsigned int  messageIdx, char *messageName_a,
                         int interest);
int evrMessageCounts    (unsigned int  messageIdx,
                         epicsUInt32 *updateCount_p,
                         epicsUInt32 *updateCountRollover_p,
                         epicsUInt32 *overwriteCount_p,
                         epicsUInt32 *noDataCount_p,
                         epicsUInt32 *writeErrorCount_p,
                         epicsUInt32 *checkSumErrorCount_p,
                         epicsUInt32 *procTimeStartMin_p,
                         epicsUInt32 *procTimeStartMax_p,
                         epicsUInt32 *procTimeDeltaAvg_p,
                         epicsUInt32 *procTimeDeltaMax_p);
int evrMessageCountsFiducial(unsigned int messageIdx,
                         epicsUInt32 *procTimeDelay_p,
                         epicsUInt32 *procTimeDelayMin_p,
                         epicsUInt32 *procTimeDelayMax_p);
int evrMessageCountsClockCounter(unsigned int messageIdx,
                         epicsUInt32 *absoluteStartTime_p,
                         epicsUInt32 *absoluteStartTimeMin_p,
                         epicsUInt32 *absoluteStartTimeMax_p);
int evrMessageCountsQ(unsigned int messageIdx,
                      epicsUInt32  *qPend_p,
                      epicsUInt32  *qPendMax_p);
int evrMessageCountReset   (unsigned int messageIdx);
int evrMessageCheckSumError(unsigned int messageIdx);
int evrMessageNoDataError  (unsigned int messageIdx);
  
#ifdef __cplusplus
}
#endif

#endif /*INCevrMessageH*/
