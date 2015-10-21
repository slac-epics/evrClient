/*=============================================================================

  Name: evrMessage.c
           evrMessageCreate    - Initialize Message Space
           evrMessageRegister  - Register Reader of the Message
           evrMessageWrite     - Write a Message
           evrMessageRead      - Read  a Message
           evrMessageStart     - Set Start Time of Message Processing
           evrMessageEnd       - Set End   Time of Message Processing
           evrMessageReport    - Report Information about the Message
           evrMessageCounts    - Get   Message Diagnostic Counts
           evrMessageCountReset- Reset Message Diagnostic Counts
           evrMessageCheckSumError - Increment Check Sum Error Counter
           evrMessageIndex     - Get Index of the Message Array

  Abs:  EVR and PNET message space creation and registration.

  Auth: 21-Dec-2006, S. Allison
  Rev:
-----------------------------------------------------------------------------*/

#include "copyright_SLAC.h"     /* SLAC copyright comments */

/*-----------------------------------------------------------------------------

  Mod:  (newest to oldest)

=============================================================================*/

#include <stddef.h>             /* size_t                      */
#include <string.h>             /* strcmp                      */
#include <stdio.h>              /* printf                      */
#ifdef __rtems__
#include <rtems.h>              /* timer routines              */
#include <rtems/timerdrv.h>     /* timer routines              */
#include <bsp.h>                /* BSP*                        */
#if (__RTEMS_MAJOR__ > 4) || (__RTEMS_MAJOR__ == 4 && __RTEMS_MINOR__ >= 9)
#define Timer_initialize benchmark_timer_initialize
#define Read_timer benchmark_timer_read
#endif
#endif

#include "dbAccess.h"           /* dbProcess,dbScan* protos    */
#include "epicsMutex.h"
#include "epicsThread.h"        /* epicsThreadSleep()          */
#include "epicsTime.h"          /* epicsTimeStamp              */
#include "errlog.h"             /* errlogPrintf                */
#include "evrMessage.h"         /* prototypes in this file     */

#define MAX_DELTA_TIME 400000000

#define DBUFF_OLDWAY
#define DBUFF_NEWWAY_

unsigned long evrFiducialTime = 0;

#if defined(DBUFF_NEWWAY)
typedef struct 
{
  evrMessage_tu       message_au[2]; /* Message, double-buffered          */
  dbCommon           *record_ps;     /* Record that processes the message */
  epicsTimeStamp      resetTime_s;   /* Time when counters reset          */
  unsigned long       notRead_a[2];  /* Message not-yet read flag         */
  unsigned long       newData_a[2];  /* New data flag */
  unsigned long       flgRead_a[2];
  unsigned long       updateCount;   
  unsigned long       updateCountRollover;
  unsigned long       overwriteCount;
  unsigned long       writeErrorCount;
  unsigned long       noDataCount;
  unsigned long       checkSumErrorCount;
  unsigned long       procTimeStart;
  unsigned long       procTimeLap;
  unsigned long       procTimeEnd;
  unsigned long       procTimeDeltaStartMax;
  unsigned long       procTimeDeltaStartMin;
  unsigned long       procTimeDeltaMax;
  unsigned long       procTimeDeltaCount;
  unsigned long       procTimeDelayMin;
  unsigned long       procTimeDelayMax;
  unsigned long       procTimeDelay;
  unsigned long       procTimeDelta_a[MODULO720_COUNT];
  unsigned long       absoluteStartTime;
  unsigned long       absoluteStartTimeMin;
  unsigned long       absoluteStartTimeMax;
  unsigned long       qPend;
  unsigned long       qPendMax;

  epicsMutexId        lock;

} evrMessage_ts;
#endif

#if defined(DBUFF_OLDWAY)
typedef struct
{
  evrMessage_tu       message_au[2]; /* Message, double-buffered          */
  dbCommon           *record_ps;     /* Record that processes the message */
  epicsTimeStamp      resetTime_s;   /* Time when counters reset          */
  unsigned long       notRead_a[2];  /* Message not-yet read flag         */
  long                readingIdx; /* Message currently being read, -1 = none */
  unsigned long       newestIdx;     /* Index in double buffer of newest msg */
  unsigned long       fiducialIdx;   /* Index at the time of the fiducial */
  unsigned long       updateCount;
  unsigned long       updateCountRollover;
  unsigned long       overwriteCount;
  unsigned long       writeErrorCount;
  unsigned long       noDataCount;
  unsigned long       checkSumErrorCount;
  unsigned long       procTimeStart;
  unsigned long       procTimeLap;
  unsigned long       procTimeEnd;
  unsigned long       procTimeDeltaStartMax;
  unsigned long       procTimeDeltaStartMin;
  unsigned long       procTimeDeltaMax;
  unsigned long       procTimeDeltaCount;
  unsigned long       procTimeDelayMin;
  unsigned long       procTimeDelayMax;
  unsigned long       procTimeDelay;
  unsigned long       procTimeDelta_a[MODULO720_COUNT];
  unsigned long       absoluteStartTime;
  unsigned long       absoluteStartTimeMin;
  unsigned long       absoluteStartTimeMax;
  unsigned long       qPend;
  unsigned long       qPendMax;

  epicsMutexId        lock;

} evrMessage_ts;
#endif

/* Maintain 4 messages - PNET, PATTERN, DATA (future), FIDUCIAL */
static evrMessage_ts evrMessage_as[EVR_MESSAGE_MAX];

/* Divisor to go from ticks to microseconds */
#ifdef __PPC__
static double        evrTicksPerUsec = 1;
#endif

#ifdef __rtems__
/*
 * From Till Straumann:
 * Macro for "Move From Time Base" to get current time in ticks.
 * The PowerPC Time Base is a 64-bit register incrementing usually
 * at 1/4 of the PPC bus frequency (which is CPU/Board dependent.
 * Even the 1/4 divisor is not fixed by the architecture).
 *
 * 'MFTB' just reads the lower 32-bit of the time base.
 */
#ifdef __PPC__
#define MFTB(var) asm volatile("mftb %0":"=r"(var))
#else
#define MFTB(var) (var)=Read_timer()
#endif
#endif
#if defined(linux) && (!defined(_X86_) && !defined(_X86_64_))
#define MFTB(var)  ((var)=1) /* make compiler happy */
#endif

/*
 *
 * from Kukhee Kim: 
 *
 */

#if defined( _X86_) || defined(_X86_64_)
#if defined(_X86_)
static __inline__ unsigned long long int rdtsc(void)
{
        unsigned long long int x;
        __asm__ volatile (".byte 0x0f, 0x31": "=A" (x));
        return x;
}
#elif defined(_X86_64_)
static __inline__ unsigned long long rdtsc(void)
{
  unsigned hi, lo;
  __asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
  return ( (unsigned long long)lo)|( ((unsigned long long)hi)<<32 );
}
#endif


static double evrTicksPerUsec = 1.5E+3;  /* need to fix it to avoid hardcoding */
#define MFTB(var)  ((var)=(unsigned long) rdtsc())

void  Get_evrTicksPerUsec_for_X86(void)
{
    unsigned long start, stop;

    do {
      MFTB(start);
      epicsThreadSleep(1.);
      MFTB(stop);
    } while(!(stop>start));

    evrTicksPerUsec = (double)(stop-start) * 1.E-6;
    printf("CPU Clock Estimation: %lf MHz\n", evrTicksPerUsec);
}
#endif




/*=============================================================================

  Name: evrMessageIndex

  Abs:  Get the Index into the Message Array

  Args: Type     Name           Access     Description
        -------  -------        ---------- ----------------------------
        char *   messageName_a  Read       Message Name ("PNET", "PATTERN", "DATA")
        size_t   messageSizeIn  Read       Size of a message

  Rem:  The message name is checked for validity and the index into the message
        array is returned.  The message size is also checked.

  Side: None

  Return: index (-1 = Failed)
==============================================================================*/

static int evrMessageIndex(char *messageName_a, int messageSizeIn)
{
  int messageSize;
  int messageIndex;
  
  if (!strcmp(messageName_a, EVR_MESSAGE_PNET_NAME           )) {
    messageSize  = sizeof(unsigned long) * EVR_PNET_MODIFIER_MAX;
    messageIndex = EVR_MESSAGE_PNET;
  } else if (!strcmp(messageName_a, EVR_MESSAGE_PATTERN_NAME )) {
    messageSize  = sizeof(evrMessagePattern_ts);
    messageIndex = EVR_MESSAGE_PATTERN;
  } else if (!strcmp(messageName_a, EVR_MESSAGE_DATA_NAME    )) {
    messageSize  = sizeof(unsigned long) * EVR_DATA_MAX;
    messageIndex = EVR_MESSAGE_DATA;
  } else if (!strcmp(messageName_a, EVR_MESSAGE_FIDUCIAL_NAME)) {
    messageSize  = 0;
    messageIndex = EVR_MESSAGE_FIDUCIAL;
  } else {
    messageSize  = 0;
    messageIndex = -1;
  }
  if (messageIndex < 0) {
    errlogPrintf("evrMessageIndex - Invalid message name %s\n", messageName_a);
  } else if (messageSizeIn != messageSize) {
    errlogPrintf("evrMessageIndex - Invalid message size %d\n", messageSizeIn);
    messageIndex = -1;
  } 
  return messageIndex;
}

/*=============================================================================

  Name: evrMessageCreate

  Abs:  Initialize Message Space

  Args: Type     Name           Access     Description
        -------  -------        ---------- ----------------------------
        char *   messageName_a  Read       Message Name ("PNET", "PATTERN", "DATA")
        size_t   messageSize    Read       Size of a message

  Rem:  The message index is found and the message is initialized.
        The message index is returned if all is OK.

  Side: None.

  Return: -1 = Failed, 0,1,2 = message index
==============================================================================*/

int evrMessageCreate(char *messageName_a, size_t messageSize)
{
  int messageIdx = evrMessageIndex(messageName_a, messageSize);

  /* Find divisor to go from ticks to microseconds
     (coarse resolution good enough). */
#ifdef __rtems__
#ifdef __PPC__
  evrTicksPerUsec = ((double)BSP_bus_frequency/
                     (double)BSP_time_base_divisor)/1000;
#else
  Timer_initialize();
#endif
#endif
  if (messageIdx < 0) return -1;

  memset(&evrMessage_as[messageIdx], 0, sizeof(evrMessage_ts));

#if defined(linux)
  /* probably, we don't need mutex for linux. Let's try without the mutex */
  /*  evrMessage_as[messageIdx].lock = epicsMutexMustCreate();  */
  evrMessage_as[messageIdx].lock = 0;
#else 
  evrMessage_as[messageIdx].lock = 0;
#endif


#if defined(DBUFF_OLDWAY)
  evrMessage_as[messageIdx].readingIdx = -1;
#endif

  evrMessageCountReset(messageIdx);
  return messageIdx;
}

/*=============================================================================

  Name: evrMessageRegister

  Abs:  Register a Message Reader

  Args: Type     Name           Access     Description
        -------  -------        ---------- ----------------------------
        char *   messageName_a  Read       Message Name ("PNET", "PATTERN",
                                                         "DATA", "FIDUCIAL")
        size_t   messageSize    Read       Size of a message
        dbCommon *record_ps     Read       Pointer to Record Structure

  Rem:  The message index is found, the message is checked that
        it's been created, and a check is done that the message hasn't already
        been taken. The message index is returned if all is OK. 

  Side: None

  Return: -1 = Failed, 0,1,2 = message index
==============================================================================*/

int evrMessageRegister(char *messageName_a, size_t messageSize,
                       dbCommon *record_ps)
{
  int messageIdx = evrMessageIndex(messageName_a, messageSize);
  
  if (messageIdx < 0) return -1;
  
  if (evrMessage_as[messageIdx].record_ps) {
    errlogPrintf("evrMessageRegister - Message %s already has a reader\n",
                 messageName_a);
    return -1;
  }
  evrMessage_as[messageIdx].record_ps = record_ps;    
  
  return messageIdx;
}

/*=============================================================================

  Name: evrMessageWrite

  Abs:  Write a Message

  Args: Type     Name           Access     Description
        -------  -------        ---------- ----------------------------
  unsigned int    messageIdx     Read       Index into Message Array
  evrMessage_tu * message_pu     Read       Message to Update

  Rem:  Write a message.

        THIS ROUTINE IS CALLED AT INTERRUPT LEVEL!!!!!!!!!!!!!!!!!!!
        It can also be called at task level by a simulator.

  Side: None.

  Return: 0 = OK, -1 = Failed
==============================================================================*/

#if defined(DBUFF_NEWWAY)
int evrMessageWrite(unsigned int messageIdx, evrMessage_tu * message_pu)
{
  int idx;

  if (messageIdx >= EVR_MESSAGE_MAX) return -1;

  if(evrMessage_as[messageIdx].lock) epicsMutexLock(evrMessage_as[messageIdx].lock);


  /* Double buffer message if not PNET - first find a free message */
  idx = 0;  /* Let's start from buffer0 */
  
  if(messageIdx != EVR_MESSAGE_PNET) {
    if(evrMessage_as[messageIdx].newData_a[0]) idx = 1; /* if it used up the buffer0, then switch to buffer 1 */
  }

  /* if this buffer is being read out by the evrMessageRead, then we got trouble */
  if(evrMessage_as[messageIdx].flgRead_a[idx]) {
      evrMessage_as[messageIdx].writeErrorCount++;
      goto bail;
  }
 
  /* everthing looks good, let's update the buffer */ 
  evrMessage_as[messageIdx].message_au[idx] = *message_pu;
  evrMessage_as[messageIdx].newData_a[idx] = 1;      /* now, this buffer has new data */
  evrMessage_as[messageIdx].newData_a[idx?0:1] = 0;  /* the previous one is old now */

  if (evrMessage_as[messageIdx].notRead_a[idx])      /* Oh! I update, before read out, It is overwitten */
     evrMessage_as[messageIdx].overwriteCount++;
  else
     evrMessage_as[messageIdx].notRead_a[idx] = 1;   /* This buffer is not read out yet */

  bail:
  if(evrMessage_as[messageIdx].lock) epicsMutexUnlock(evrMessage_as[messageIdx].lock);

  return 0;
}
#endif

#if defined(DBUFF_OLDWAY)
int evrMessageWrite(unsigned int messageIdx, evrMessage_tu * message_pu)
{
  int idx;

  if (messageIdx >= EVR_MESSAGE_MAX) return -1;

  /* Double buffer message if not PNET - first find a free message */
  idx = 0;
  if (evrMessage_as[messageIdx].notRead_a[idx] &&
      (messageIdx != EVR_MESSAGE_PNET)) {
    idx = 1;
    if (evrMessage_as[messageIdx].notRead_a[idx]) {
      /* No message is free.  If a message is being read, overwrite the
         other one.  Otherwise, overwrite the message after the fiducial. */
      if (evrMessage_as[messageIdx].readingIdx >= 0)
        idx = evrMessage_as[messageIdx].readingIdx?0:1;
      else
        idx = evrMessage_as[messageIdx].fiducialIdx?0:1;
    } /* end both messages not free */
  }   /* end message 0 is not free  */

  /* Update a message only if it's not currently being read. */
  if (idx != evrMessage_as[messageIdx].readingIdx) {
    /* Update message in holding array */
    evrMessage_as[messageIdx].message_au[idx] = *message_pu;
    evrMessage_as[messageIdx].newestIdx       = idx;
    if (evrMessage_as[messageIdx].notRead_a[idx])
      evrMessage_as[messageIdx].overwriteCount++;
    else
      evrMessage_as[messageIdx].notRead_a[idx] = 1;
  } else {
    evrMessage_as[messageIdx].writeErrorCount++;
  }
  return 0;
}
#endif


/*=============================================================================

  Name: evrMessageProcess

  Abs:  Process a record that processes the Message

  Args: Type     Name           Access     Description
        -------  -------        ---------- ----------------------------
  unsigned int   messageIdx     Read       Index into Message Array

  Rem:  Calls record processing.

  Side: None.

  Return: 0 = OK
==============================================================================*/

int evrMessageProcess(unsigned int messageIdx)
{
  if (messageIdx >= EVR_MESSAGE_MAX) return -1;
  if (!interruptAccept)              return -1;
  
  /* Process record that wants the message. */
  if (evrMessage_as[messageIdx].record_ps) {
    dbScanLock(evrMessage_as[messageIdx].record_ps);
    dbProcess(evrMessage_as[messageIdx].record_ps);
    dbScanUnlock(evrMessage_as[messageIdx].record_ps);
  }
  return 0;
}
/*=============================================================================

  Name: evrMessageRead

  Abs:  Read a Message

  Args: Type     Name           Access     Description
        -------  -------        ---------- ----------------------------
  unsigned int    messageIdx     Read       Index into Message Array
  evrMessage_tu * message_pu     Write      Message

  Rem:  Read a message.  

  Side: None

  Return: 0 = OK, 1 = Input Error, 2 = No Data Available
  
==============================================================================*/

#if defined(DBUFF_NEWWAY)
evrMessageReadStatus_te evrMessageRead(unsigned int  messageIdx,
                                       evrMessage_tu *message_pu)
{
  evrMessageReadStatus_te status;
  int idx;
  
  if (messageIdx >= EVR_MESSAGE_MAX) return evrMessageInpError;
  
 
  if(evrMessage_as[messageIdx].lock) epicsMutexLock(evrMessage_as[messageIdx].lock);
  
  /* check up if there is no data available */
  if (!evrMessage_as[messageIdx].notRead_a[0] && 
      !evrMessage_as[messageIdx].notRead_a[1]) {
    status = evrMessageDataNotAvail;
    evrMessage_as[messageIdx].noDataCount++;
  } else {
    /* yes! we have new data */
    status = evrMessageOK;

    if(evrMessage_as[messageIdx].newData_a[0]) idx =0; /* new data is at buffer0 */
    else                                       idx =1; /* new data is at buffer1 */

    /* check up if fiducial is delayed. If both buffer have not been read, then we can assume
       the delayed fiducial */
    if(evrMessage_as[messageIdx].notRead_a[0] && evrMessage_as[messageIdx].notRead_a[1]) {
        idx = idx?0:1;  /* for the delayed fiducial, we need to read the old data instead of new one */
    }

    if(messageIdx == EVR_MESSAGE_PNET) idx = 0; /* for the PNET, we only use buffer0 */

    evrMessage_as[messageIdx].flgRead_a[idx] = 1; /* I am reading now, don't change anything on my buffer */

    switch (messageIdx) {
      case EVR_MESSAGE_PNET:
        message_pu->pnet_s    = evrMessage_as[messageIdx].message_au[idx].pnet_s;
        break;
      case EVR_MESSAGE_PATTERN:
        message_pu->pattern_s = evrMessage_as[messageIdx].message_au[idx].pattern_s;
        break;
      case EVR_MESSAGE_FIDUCIAL:
        break;
      case EVR_MESSAGE_DATA:
      default:
        status = evrMessageInpError;
        break;
    }

    evrMessage_as[messageIdx].flgRead_a[idx] = 0; /* done to read */
    evrMessage_as[messageIdx].notRead_a[idx] = 0; /* I have read out this buffer */
  }

  if(evrMessage_as[messageIdx].lock) epicsMutexUnlock(evrMessage_as[messageIdx].lock);

  return status;
}
#endif

#if defined(DBUFF_OLDWAY)
evrMessageReadStatus_te evrMessageRead(unsigned int  messageIdx,
                                       evrMessage_tu *message_pu)
{
  evrMessageReadStatus_te status;
  volatile int idx;

  if (messageIdx >= EVR_MESSAGE_MAX) return evrMessageInpError;

  /* Read the message marked by the fiducial. */
  idx = evrMessage_as[messageIdx].fiducialIdx;
  if (!evrMessage_as[messageIdx].notRead_a[idx]) {
    status = evrMessageDataNotAvail;
    evrMessage_as[messageIdx].noDataCount++;
  } else {
    status = evrMessageOK;
    evrMessage_as[messageIdx].readingIdx = idx;
    switch (messageIdx) {
      case EVR_MESSAGE_PNET:
        message_pu->pnet_s    = evrMessage_as[messageIdx].message_au[idx].pnet_s;
        break;
      case EVR_MESSAGE_PATTERN:
        message_pu->pattern_s = evrMessage_as[messageIdx].message_au[idx].pattern_s;
        break;
      case EVR_MESSAGE_FIDUCIAL:
        break;
      case EVR_MESSAGE_DATA:
      default:
        status = evrMessageInpError;
        break;
    }
    evrMessage_as[messageIdx].readingIdx = -1;
    evrMessage_as[messageIdx].notRead_a[idx] = 0;
  }
  return status;
}
#endif


int evrMessageClockCounter(unsigned int messageIdx, epicsUInt32 evrClockCounter)
{
    if(messageIdx >= EVR_MESSAGE_MAX) return -1;

    evrClockCounter += 33; 

    evrMessage_as[messageIdx].absoluteStartTime = evrClockCounter;
    if(evrMessage_as[messageIdx].absoluteStartTimeMin > evrClockCounter)
        evrMessage_as[messageIdx].absoluteStartTimeMin = evrClockCounter;
    if(evrMessage_as[messageIdx].absoluteStartTimeMax < evrClockCounter) 
        evrMessage_as[messageIdx].absoluteStartTimeMax = evrClockCounter;

    return 0;
}

/*=============================================================================

  Name: evrMessageStart

  Abs:  Set Start Time of Message Processing

  Args: Type     Name           Access     Description
        -------  -------        ---------- ----------------------------
  unsigned int    messageIdx     Read       Index into Message Array

  Rem:  None.

  Side: None.

  Return: 0 = OK, -1 = Failed
==============================================================================*/

int evrMessageStart(unsigned int messageIdx)
{
  unsigned long prevTimeStart, deltaTimeStart;
  int idx, oldidx;  

  if (messageIdx >= EVR_MESSAGE_MAX) return -1;

  /* Get time when processing starts */
  prevTimeStart = evrMessage_as[messageIdx].procTimeStart;
  MFTB(evrMessage_as[messageIdx].procTimeStart);
  deltaTimeStart = evrMessage_as[messageIdx].procTimeStart - prevTimeStart;
  if (deltaTimeStart < MAX_DELTA_TIME) {
    if (evrMessage_as[messageIdx].procTimeDeltaStartMax < deltaTimeStart)
      evrMessage_as[messageIdx].procTimeDeltaStartMax = deltaTimeStart;
    if (evrMessage_as[messageIdx].procTimeDeltaStartMin > deltaTimeStart)
      evrMessage_as[messageIdx].procTimeDeltaStartMin = deltaTimeStart;
  }
  
  /* Reset time that processing ends */
  evrMessage_as[messageIdx].procTimeEnd = 0;

  /* Update diagnostics counters */
  if (evrMessage_as[messageIdx].updateCount < EVR_MAX_INT) {
    evrMessage_as[messageIdx].updateCount++;
  } else {
    evrMessage_as[messageIdx].updateCountRollover++;
    evrMessage_as[messageIdx].updateCount = 0;
  }




#if defined(DBUFF_OLDWAY)
  /* Special processing for the fiducial - set PATTERN message to read
     and throw away any old PATTERN messages */
  if (messageIdx == EVR_MESSAGE_FIDUCIAL) {
    evrFiducialTime = evrMessage_as[messageIdx].procTimeStart;
    idx = evrMessage_as[EVR_MESSAGE_PATTERN].newestIdx;
    evrMessage_as[EVR_MESSAGE_PATTERN].fiducialIdx = idx;
    oldidx = idx?0:1;
    if (evrMessage_as[EVR_MESSAGE_PATTERN].readingIdx != oldidx) {
      evrMessage_as[EVR_MESSAGE_PATTERN].notRead_a[oldidx] = 0;
    }
  }
#endif
  

  return 0;
}

int evrMessageLap(unsigned int messageIdx)
{
    unsigned long delayTime;

    if(messageIdx == EVR_MESSAGE_FIDUCIAL) {
        MFTB(evrMessage_as[messageIdx].procTimeLap);
        delayTime = evrMessage_as[messageIdx].procTimeLap - evrMessage_as[messageIdx].procTimeStart;
        evrMessage_as[messageIdx].procTimeDelay = delayTime;
        
        if(delayTime < MAX_DELTA_TIME) {
            if(evrMessage_as[messageIdx].procTimeDelayMax < delayTime) 
                evrMessage_as[messageIdx].procTimeDelayMax = delayTime;
            if(evrMessage_as[messageIdx].procTimeDelayMin > delayTime)
                evrMessage_as[messageIdx].procTimeDelayMin = delayTime;
        }
    }

    

    return 0;   
}


int evrMessageQ(unsigned int messageIdx, int pend)
{
    if(messageIdx==EVR_MESSAGE_FIDUCIAL) {
        evrMessage_as[messageIdx].qPend = pend;
        if(pend > evrMessage_as[messageIdx].qPendMax) 
            evrMessage_as[messageIdx].qPendMax = pend;
    }

    return 0;
}



/*=============================================================================

  Name: evrMessageEnd

  Abs:  Set End Time of Message Processing

  Args: Type     Name           Access     Description
        -------  -------        ---------- ----------------------------
  unsigned int    messageIdx     Read       Index into Message Array

  Rem:  None.

  Side: None.

  Return: 0 = OK, -1 = Failed
==============================================================================*/

int evrMessageEnd(unsigned int messageIdx)
{
  evrMessage_ts *em_ps = evrMessage_as + messageIdx;
  
  if (messageIdx >= EVR_MESSAGE_MAX) return -1;

  /* Get end of processing time */
  MFTB(em_ps->procTimeEnd);

  /* Update array of delta times used later to calc avg.
     Keep track of maximum. */
  em_ps->procTimeDelta_a[em_ps->procTimeDeltaCount] =
    em_ps->procTimeEnd - em_ps->procTimeStart;
  if (em_ps->procTimeDeltaMax <
      em_ps->procTimeDelta_a[em_ps->procTimeDeltaCount]) {
    em_ps->procTimeDeltaMax =
      em_ps->procTimeDelta_a[em_ps->procTimeDeltaCount];
  }

  em_ps->procTimeDeltaCount++;
  if (em_ps->procTimeDeltaCount >= MODULO720_COUNT) {
    em_ps->procTimeDeltaCount = MODULO720_COUNT-1;
  }  
  return 0;
}

/*=============================================================================

  Name: evrMessageReport

  Abs:  Output Report on a Message Message

  Args: Type     Name           Access     Description
        -------  -------        ---------- ----------------------------
  unsigned int    messageIdx     Read       Index into Message Array
        char *    messageName_a  Read       Message Name ("PNET", "PATTERN",
                                                          "DATA", "FIDUCIAL")
        int       interest       Read       Interest level

  Rem:  Printout information about this message space.  

  Side: Output to standard output.

  Return: 0 = OK, -1 = Failed
==============================================================================*/

int evrMessageReport(unsigned int  messageIdx, char *messageName_a,
                     int interest)
{ 
  char timestamp_c[MAX_STRING_SIZE];
  int  idx;
  
  if (messageIdx >= EVR_MESSAGE_MAX) return -1;
  if (interest <= 0) return 0;
  timestamp_c[0]=0;
  epicsTimeToStrftime(timestamp_c, MAX_STRING_SIZE, "%a %b %d %Y %H:%M:%S.%f",
                      &evrMessage_as[messageIdx].resetTime_s);
  printf("%s counters reset at %s\n", messageName_a, timestamp_c);
  printf("Number of attempted message writes/rollover: %lu/%lu\n",
         evrMessage_as[messageIdx].updateCount,
         evrMessage_as[messageIdx].updateCountRollover);
  printf("Number of overwritten messages: %lu\n",
         evrMessage_as[messageIdx].overwriteCount);
  printf("Number of no data available errors: %lu\n",
         evrMessage_as[messageIdx].noDataCount);
  printf("Number of write errors: %lu\n",
         evrMessage_as[messageIdx].writeErrorCount);
  printf("Number of check sum errors: %lu\n",
         evrMessage_as[messageIdx].checkSumErrorCount);
#if defined(__PPC__) || defined(_X86_) || defined(_X86_64_)
  printf("Maximum proc time delta (us) = %lf\n",
         (double)evrMessage_as[messageIdx].procTimeDeltaMax/evrTicksPerUsec);
  printf("Max/Min proc start time deltas (us) = %lf/%lf\n",
         (double)evrMessage_as[messageIdx].procTimeDeltaStartMax/
         evrTicksPerUsec,
         (double)evrMessage_as[messageIdx].procTimeDeltaStartMin/
         evrTicksPerUsec);
#else
  printf("Maximum proc time delta (us) = %lu\n",
         evrMessage_as[messageIdx].procTimeDeltaMax);
  printf("Max/Min proc start time deltas (us) = %lu/%lu\n",
         evrMessage_as[messageIdx].procTimeDeltaStartMax,
         evrMessage_as[messageIdx].procTimeDeltaStartMin);
#endif
  if (interest > 1) {
    int count = 25;
    if (interest > 2) count = MODULO720_COUNT;
    printf("Last %d proc time deltas (us):\n", count);
    for (idx=0; idx<count; idx++) {
#if defined(__PPC__) || defined(_X86_) || defined(_X86_64_)
      printf("  %d: %lf\n", idx,
             (double)evrMessage_as[messageIdx].procTimeDelta_a[idx]/
             evrTicksPerUsec);
#else
      printf("  %d: %lu\n", idx, 
             evrMessage_as[messageIdx].procTimeDelta_a[idx]);
#endif
    }
  }
  return 0;
}

/*=============================================================================

  Name: evrMessageCounts

  Abs:  Return message diagnostic count values
        (for use by EPICS subroutine records)

  Args: Type     Name           Access     Description
        -------  -------        ---------- ----------------------------
  unsigned int   messageIdx            Read    Message Index
  epicsUInt32   * updateCount_p        Write   # times ISR wrote a message
  epicsUInt32   * updateCountRollover_p  Write # times above rolled over
  epicsUInt32   * overwriteCount_p     Write   # times ISR overwrote a message
  epicsUInt32   * noDataCount_p        Write   # times no data was available for a read
  epicsUInt32   * writeErrorCount_p    Write   # times data not written during read
  epicsUInt32   * checkSumErrorCount_p Write   # times message check sum error
  epicsUInt32   * procTimeStartMin_p   Write   Min start time delta (us)
  epicsUInt32   * procTimeStartMax_p   Write   Max start time delta (us)
  epicsUInt32   * procTimeDeltaAvg_p   Write   Avg time for message processing (us)
  epicsUInt32   * procTimeDeltaMax_p   Write   Max time for message processing (us)

  Rem:  The diagnostics count values are filled in if the message index is valid.
  
  Side: None

  Return: 0 = OK, -1 = Failed
==============================================================================*/

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
                         epicsUInt32 *procTimeDeltaMax_p)
{  
  evrMessage_ts *em_ps = evrMessage_as + messageIdx;
  int    idx;

  if (messageIdx >= EVR_MESSAGE_MAX ) return -1;
  *updateCount_p         = em_ps->updateCount;
  *updateCountRollover_p = em_ps->updateCountRollover;
  *overwriteCount_p      = em_ps->overwriteCount;
  *noDataCount_p         = em_ps->noDataCount;
  *writeErrorCount_p     = em_ps->writeErrorCount;
  *checkSumErrorCount_p  = em_ps->checkSumErrorCount;
  *procTimeStartMin_p    = em_ps->procTimeDeltaStartMin;
  *procTimeStartMax_p    = em_ps->procTimeDeltaStartMax;
  *procTimeDeltaMax_p    = em_ps->procTimeDeltaMax;
/* Nearest microsecond for PPC */
#if defined(__PPC__) || defined(_X86_) || defined(_X86_64_)
  *procTimeStartMin_p    = (epicsUInt32)
    (((double)(*procTimeStartMin_p)/evrTicksPerUsec) + 0.5);
  *procTimeStartMax_p    = (epicsUInt32)
    (((double)(*procTimeStartMax_p)/evrTicksPerUsec) + 0.5);
  *procTimeDeltaMax_p    = (epicsUInt32)
    (((double)(*procTimeDeltaMax_p)/evrTicksPerUsec) + 0.5);
  *procTimeDeltaAvg_p    = (epicsUInt32)
    (((double)(*procTimeDeltaAvg_p)/evrTicksPerUsec) + 0.5);
#endif
  *procTimeDeltaAvg_p    = 0;
  if  (em_ps->procTimeDeltaCount > 0) {
    for (idx = 0; idx < em_ps->procTimeDeltaCount; idx++) {
      *procTimeDeltaAvg_p += em_ps->procTimeDelta_a[idx];
    }
#if defined(__PPC__) || defined(_X86_) || defined(_X86_64_)
    *procTimeDeltaAvg_p    = (epicsUInt32)
      ((((double)(*procTimeDeltaAvg_p)/
         (double)em_ps->procTimeDeltaCount)/
        evrTicksPerUsec) + 0.5);
#else
    *procTimeDeltaAvg_p /= em_ps->procTimeDeltaCount;
#endif
    em_ps->procTimeDeltaCount = 0;
  }
  return 0;
}


int evrMessageCountsFiducial(unsigned int messageIdx,
                             epicsUInt32 *procTimeDelay_p,
                             epicsUInt32 *procTimeDelayMin_p,
                             epicsUInt32 *procTimeDelayMax_p)

{
   evrMessage_ts *em_ps = evrMessage_as + messageIdx;

   if(messageIdx != EVR_MESSAGE_FIDUCIAL) return -1;

    *procTimeDelay_p    = em_ps->procTimeDelay;
    *procTimeDelayMin_p = em_ps->procTimeDelayMin;
    *procTimeDelayMax_p = em_ps->procTimeDelayMax;

#if defined(__PPC__) || defined(_X86_) || defined(_X86_64_)
    *procTimeDelay_p = (epicsUInt32) 
        (((double)(*procTimeDelay_p)/evrTicksPerUsec) + 0.5);
    *procTimeDelayMin_p = (epicsUInt32)
        (((double)(*procTimeDelayMin_p)/evrTicksPerUsec) + 0.5);
    *procTimeDelayMax_p = (epicsUInt32)
        (((double)(*procTimeDelayMax_p)/evrTicksPerUsec) + 0.5);
#endif

    return 0;
}


int evrMessageCountsClockCounter(unsigned int messageIdx,
                                 epicsUInt32 *absoluteStartTime_p,
                                 epicsUInt32 *absoluteStartTimeMin_p,
                                 epicsUInt32 *absoluteStartTimeMax_p)
{

    evrMessage_ts *em_ps = evrMessage_as + messageIdx;
    if(messageIdx >= EVR_MESSAGE_MAX) return -1;

    *absoluteStartTime_p    = em_ps->absoluteStartTime;
    *absoluteStartTimeMin_p = em_ps->absoluteStartTimeMin;
    *absoluteStartTimeMax_p = em_ps->absoluteStartTimeMax;

    #define CALC_FUNC(A)  (*A) = (epicsUInt32)(((double)(*A)*(1./119.))+0.5)

    CALC_FUNC(absoluteStartTime_p);
    CALC_FUNC(absoluteStartTimeMin_p);
    CALC_FUNC(absoluteStartTimeMax_p);

    #undef CALC_FUNC

    return 0; 
}

int evrMessageCountsQ(unsigned int messageIdx,
                      epicsUInt32 *qPend,
                      epicsUInt32 *qPendMax)
{
    evrMessage_ts *em_ps = evrMessage_as + messageIdx;
    if(messageIdx >= EVR_MESSAGE_MAX) return -1;

    *qPend    = em_ps->qPend;
    *qPendMax = em_ps->qPendMax;
   
    return 0;
}


/*=============================================================================

  Name: evrMessageCountReset

  Abs:  Reset message message diagnostic count values
        (for use by EPICS subroutine records)

  Args: Type     Name           Access     Description
        -------  -------        ---------- ----------------------------
  unsigned int   messageIdx     Read       Message Index

  Rem:  The diagnostics count values are reset if the message index is valid.
  
  Side: None

  Return: 0 = OK, -1 = Failed
==============================================================================*/

int evrMessageCountReset (unsigned int messageIdx)
{
  if (messageIdx >= EVR_MESSAGE_MAX ) return -1;
  evrMessage_as[messageIdx].updateCount           = 0;
  evrMessage_as[messageIdx].updateCountRollover   = 0;
  evrMessage_as[messageIdx].overwriteCount        = 0;
  evrMessage_as[messageIdx].noDataCount           = 0;
  evrMessage_as[messageIdx].writeErrorCount       = 0;
  evrMessage_as[messageIdx].checkSumErrorCount    = 0;
  evrMessage_as[messageIdx].procTimeDeltaMax      = 0;
  evrMessage_as[messageIdx].procTimeDeltaStartMax = 0;
  evrMessage_as[messageIdx].procTimeDeltaStartMin = MAX_DELTA_TIME;
  evrMessage_as[messageIdx].procTimeDeltaCount    = 0;
  evrMessage_as[messageIdx].procTimeDelayMax      = 0;
  evrMessage_as[messageIdx].procTimeDelayMin      = MAX_DELTA_TIME;
  evrMessage_as[messageIdx].absoluteStartTime     = 0;
  evrMessage_as[messageIdx].absoluteStartTimeMax  = 0;
  evrMessage_as[messageIdx].absoluteStartTimeMin  = MAX_DELTA_TIME;
  evrMessage_as[messageIdx].qPend                 = 0;
  evrMessage_as[messageIdx].qPendMax              = 0;
  /* Save counter reset time for reporting purposes */
  epicsTimeGetCurrent(&evrMessage_as[messageIdx].resetTime_s);
  return 0;
}

/*=============================================================================

  Name: evrMessageCheckSumError

  Abs:  Increment check sum error count

  Args: Type     Name           Access     Description
        -------  -------        ---------- ----------------------------
  unsigned int    messageIdx     Read       Index into Message Array

  Rem:  None.

  Side: None.

  Return: 0 = OK, -1 = Failed
==============================================================================*/

int evrMessageCheckSumError(unsigned int messageIdx)
{  
  if (messageIdx >= EVR_MESSAGE_MAX) return -1;
  evrMessage_as[messageIdx].checkSumErrorCount++;
  return 0;
}

/*=============================================================================

  Name: evrMessageNoDataError

  Abs:  Increment missed fiducial error count

  Args: Type     Name           Access     Description
        -------  -------        ---------- ----------------------------
  unsigned int    messageIdx     Read       Index into Message Array

  Rem:  None.

  Side: None.

  Return: 0 = OK, -1 = Failed
==============================================================================*/

int evrMessageNoDataError(unsigned int messageIdx)
{  
  if (messageIdx >= EVR_MESSAGE_MAX) return -1;
  evrMessage_as[messageIdx].noDataCount++;
  return 0;
}
