/*
 * Not the MRF code, but leaving the plugins that start with devMrfEr... in 
 * epicsExportAddress for the compatibility. The same holds for ErRegisterEventHandler
 * and similar.
 * 
 * 
 * 'eevrma' prefix stands for: Epics EVRMA
 */

/**************************************************************************************************/
/*  If EVR_DEBUG Flag is Defined, Make All Local Routines Globally Callable                       */
/**************************************************************************************************/

#ifdef EVR_DEBUG
#define LOCAL_RTN
#endif

/**************************************************************************************************/
/*  Imported Header Files                                                                         */
/**************************************************************************************************/

#include <epicsStdlib.h>        /* EPICS Standard C library support routines                      */
#include <epicsStdio.h>         /* EPICS Standard C I/O support routines                          */
#include <epicsStdioRedirect.h>
#include <epicsTypes.h>         /* EPICS Architecture-independent type definitions                */
#include <epicsInterrupt.h>     /* EPICS Interrupt context support routines                       */
#include <epicsMutex.h>         /* EPICS Mutex support library                                    */
#include <string.h>

#include <alarm.h>              /* EPICS Alarm status and severity definitions                    */
#include <dbAccess.h>           /* EPICS Database access definitions                              */
#include <dbScan.h>             /* EPICS Database scan routines and definitions                   */
#include <devLib.h>             /* EPICS Device hardware addressing support library               */
#include <devSup.h>             /* EPICS Device support layer structures and symbols              */
#include <ellLib.h>             /* EPICS Linked list support library                              */
#include <errlog.h>             /* EPICS Error logging support library                            */
#include <recGbl.h>             /* EPICS Record Support global routine definitions                */
#include <registryFunction.h>   /* EPICS Registry support library                                 */

#include <erRecord.h>           /* Event Receiver (ER) Record structure                           */
#include <ereventRecord.h>      /* Event Receiver Event (EREVENT) record structure                */
#include <eventRecord.h>        /* Standard EPICS Event Record structure                          */
#include <biRecord.h>           /* Standard EPICS Event Record structure                          */
#include <erDefs.h>             /* Common Event Receiver (ER) definitions                         */

#include <devMrfEr.h>           /* MRF Event Receiver device support layer interface              */
#include "drvEvrma.h"

#include <epicsExport.h>        /* EPICS Symbol exporting macro definitions                       */
#include <iocsh.h>              /* EPICS iocsh support library                                    */


#include <drvSup.h>

#include "linux-evrma.h"

#define ADBG(FORMAT, ...) {printf("DBG: " FORMAT "\n", ## __VA_ARGS__); fflush(stdout);}
#define AINFO(FORMAT, ...) printf("INFO: " FORMAT "\n", ## __VA_ARGS__)
#define AERR(FORMAT, ...) errlogPrintf("ERROR: " FORMAT "\n", ## __VA_ARGS__)

#ifdef DBG_MEASURE_TIME_FROM_IRQ_TO_USER
#include "libevrma_dbg.h"
#endif

epicsInt32 eevrmaDebug = 0;

// needed in erapi.c, for whatever reason
unsigned int erapiDebug;

static ELLLIST eevrmaCardList;                        /* Linked list of ER card structures */
static epicsBoolean eevrmaCardListInitDone = epicsFalse;
static epicsMutexId eevrmaCardListLock;
static epicsMutexId eevrmaConfigureLock;

int          int_showme = 0;
int          event_showme = 0;
int          dbufst_showme =0;



LOCAL_RTN epicsStatus eevrmaInitRecord (erRecord*);
LOCAL_RTN epicsStatus eevrmaProcess    (erRecord*);



epicsStatus eevrmaFinishDrvInit(int AfterRecordInit)
{
	return OK;
}


static ErDsetStruct devMrfEr = {
    5,                                  /* Number of entries in the Device Support Entry Table    */
    (DEVSUPFUN)NULL,                    /* -- No device report routine                            */
    (DEVSUPFUN)eevrmaFinishDrvInit,         /* Driver-Layer routine to complete the hardware init.    */
    (DEVSUPFUN)eevrmaInitRecord,            /* Record initialization routine                          */
    (DEVSUPFUN)NULL,                    /* -- No I/O Interrupt information routine                */
    (DEVSUPFUN)eevrmaProcess                /* Record processing routine                              */
};

epicsExportAddress (dset, devMrfEr);


GLOBAL_RTN
VevrStruct *eevrmaGetVevrStruct (int card)
{
   /*---------------------
    * Local variables
    */
    VevrStruct  *vevr;

   /*---------------------
    * Loop to see if the requested card is in the linked list of known
    * Event Receiver card structures.
    */
    for (vevr = (VevrStruct *)ellFirst(&eevrmaCardList);
         vevr != NULL;
         vevr = (VevrStruct *)ellNext(&vevr->link)) {

        if (card == vevr->Cardno) return vevr;

    }/*end for each card structure on the list*/

   /*---------------------
    * Return NULL pointer if there was no card structure for the requested card.
    */
    return NULL;

}/*end eevrmaGetVevrStruct()*/

void eevrmaRegisterDevEventHandler(VevrStruct *pCard, DEV_EVENT_FUNC EventFunc)
{
	pCard->devEventFunc = EventFunc;
}

void eevrmaRegisterDevErrorHandler(VevrStruct *pCard, DEV_ERROR_FUNC ErrorFunc)
{
	pCard->devErrorFunc = ErrorFunc;
}

void eevrmaRegisterDevDBuffHandler (VevrStruct *pCard, DEV_DBUFF_FUNC DBuffFunc)
{
	pCard->devDBuffFunc = DBuffFunc;
}

LOCAL_RTN
void ErDevEventFunc (VevrStruct *pCard, epicsInt16 EventNum, epicsUInt32 Time)
{
   /*---------------------
    * Invoke the user-defined event handler (if one is defined)
    */
    if (pCard->eventFunc != NULL) {
        (*(USER_EVENT_FUNC)pCard->eventFunc)(pCard->Cardno, EventNum, Time);
	}

}/*end ErDevEventFunc()*/


LOCAL_RTN
void ErDevErrorFunc (VevrStruct *pCard, int ErrorNum)
{
   /*---------------------
    * Local Variables
    */
    int        Card = pCard->Cardno;            /* Card number of the offending board             */
    erRecord  *pRec = (erRecord *)pCard->pRec;  /* Address of this board's ER record              */

   /*---------------------
    * Decide how to handle the error based on the specified error code
    */
    switch (ErrorNum) {

   /*---------------------
    * Receiver Link Frame (Taxi) Error
    */
    case ERROR_TAXI:
        pRec->rxve = 0;
        pRec->taxi = pCard->rxvioCount;
        if(eevrmaDebug) {
            epicsSnprintf (pCard->intMsg, EVR_INT_MSG_LEN,
                           "ER Card %d Receiver Link (Taxi) Error.  Error repetition %d...\n",
                           Card, pCard->rxvioCount);
            epicsInterruptContextMessage (pCard->intMsg);
        }/*end if debug flag is set*/
        break;

   /*---------------------
    * Lost Heartbeat Error
    */
    case ERROR_HEART:
        if(eevrmaDebug) {
            epicsSnprintf (pCard->intMsg, EVR_INT_MSG_LEN,
                           "ER Card %d Lost Heartbeat\n", Card);
            epicsInterruptContextMessage (pCard->intMsg);
        }/*end if debug flag is set*/
        break;

   /*---------------------
    * FIFO Overflow Error
    */
    case ERROR_LOST:
        if(eevrmaDebug) {
            epicsSnprintf (pCard->intMsg, EVR_INT_MSG_LEN,
                           "ER Card %d Event FIFO Overflow\n", Card);
            epicsInterruptContextMessage (pCard->intMsg);
        }/*end if debug flag is set*/
        break;

   /*---------------------
    * Data Stream Checksum Error
    */
    case ERROR_DBUF_CHECKSUM:
        if(eevrmaDebug) {
            epicsSnprintf (pCard->intMsg, EVR_INT_MSG_LEN,
                           "ER Card %d Data Stream Checksum Error\n", Card);
            epicsInterruptContextMessage (pCard->intMsg);
        }/*end if debug flag is set*/
        break;

   /*---------------------
    * Invalid Error Code
    */
    default:
        if(eevrmaDebug) {
          epicsSnprintf (pCard->intMsg, EVR_INT_MSG_LEN,
                         "ER Card %d Invalid Error Code = %d.\n", Card, ErrorNum);
          epicsInterruptContextMessage (pCard->intMsg);
        }
        return;

    }/*end switch on error number*/

   /*---------------------
    * Invoke the user-defined error handler (if defined)
    */
    if (pCard->errorFunc != NULL)
        (*(USER_ERROR_FUNC)pCard->errorFunc) (Card, ErrorNum);

}/*end ErDevErrorFunc()*/


LOCAL_RTN
epicsStatus eevrmaInitRecord (erRecord *pRec)
{
    epicsInt16     Card;        /* Event Receiver card number for this record                     */
    int            i;           /* Loop counter                                                   */
    VevrStruct  *pCard;       /* Pointer to Event Receiver card structure for this record       */

   /*---------------------
    * Output a debug message if the debug flag is set
    */
    if (eevrmaDebug)
        printf ("devMrfEr::ErInitRecord(%s) entered\n", pRec->name);

   /*---------------------
    * Make sure the card number is valid by fetching its card structure
    */
    Card = pRec->out.value.vmeio.card;
    epicsSnprintf(pRec->busd,sizeof(pRec->busd)-1,"Unavailable");
    if (NULL == (pCard = eevrmaGetVevrStruct(Card))) {
        recGblRecordError(S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErInitErRec() Invalid card number");
        return(S_dev_badCard);
    }/*end if invalid card number*/

   /*---------------------
    * Make sure we only have one record for a given ER card
    */
    if (pCard->pRec != NULL) {
        recGblRecordError (S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErInitErRec() onlyone record allowed per card");
        return (S_dev_badCard);
    }/*end if card already initialized*/

	memset(pRec->busd, 0, sizeof(pRec->busd));
	epicsSnprintf(pRec->busd, sizeof(pRec->busd)-1, "VEVR Card %i", pCard->Cardno);

   /*---------------------
    * Finish initializing the event receiver card structure
    */
    pCard->pRec = (void *)pRec;                        /* Set the record address                  */
    eevrmaRegisterDevEventHandler (pCard, ErDevEventFunc); /* Set the device support event handler    */
    eevrmaRegisterDevErrorHandler (pCard, ErDevErrorFunc); /* Set the device support error handler    */

    for (i=0;  i <= EVENT_DELAYED_IRQ;  i++)           /* Initialize the IOSCANPVT structures     */
        scanIoInit (&pCard->IoScanPvt[i]);

   /*---------------------
    * Store the address of the event receiver card structure in the record's DPVT field.
    */
    pRec->dpvt = (void *)pCard;

   /*---------------------
    * Set the Event Receiver with any pre-defined values
    */
    eevrmaProcess (pRec);
    return (0);

}/*end eevrmaInitRecord()*/

static void eevrmaTaxiIrq (VevrStruct   *pCard, int enable)
{
	epicsMutexLock(pCard->cardLock);
	if(enable) {
		evrmaSubscribe(pCard->session, EVRMA_EVENT_ERROR_TAXI);
	} else {
		evrmaUnsubscribe(pCard->session, EVRMA_EVENT_ERROR_TAXI);
	}
	epicsMutexUnlock(pCard->cardLock);
}

static void eevrmaSetPulseGen(VevrStruct *pCard, int Channel, epicsBoolean Enable, 
			epicsUInt32 Delay, epicsUInt32 Width, 
			epicsUInt16 Prescaler, epicsBoolean Pol)
{	
	if(!Enable) {
		Prescaler = 0;
		Width = 0;
	}
	
	epicsMutexLock(pCard->cardLock);
	
	evrmaSetPulseParams(pCard->session, Channel, Prescaler, Delay, Width);
	evrmaSetPulseProperties(pCard->session, Channel, Enable, Pol, 1 << EVR_PULSE_CFG_BIT_TRIGGER);

	epicsMutexUnlock(pCard->cardLock);
	return;
}



LOCAL_RTN
epicsStatus eevrmaProcess (erRecord  *pRec)
{
    VevrStruct   *pCard;
	int pulseCount;
	
   /*---------------------
    * Output a debug message if the debug flag is set.
    */
    if (eevrmaDebug)
        printf ("devEvrma: eevrmaProcess (%s) entered\n", pRec->name);

   /*---------------------
    * Get the card structure.
    * Abort if we don't have a valid card structure.
    */
    if (NULL == (pCard = (VevrStruct *)pRec->dpvt)) {
        pRec->pact = epicsTrue;
        return (-1);
    }/*end if did not have a valid card structure*/

   /*---------------------
    * Lock the card structure while we are processing the record.
    */
    epicsMutexLock (pCard->cardLock);
    
   	pulseCount = evrmaGetPulseCount(pCard->session);
	
	/*
	 * Use:
	 * - the programmable delay (DG) for pulsegens 0..3
	 * - the programmable width (OTP) for pulsegens 4..17 (prescaler fixed to 1)
	 */
	
    if(pulseCount > 0) eevrmaSetPulseGen (pCard, 0, pRec->dg0e, pRec->dg0d, pRec->dg0w, pRec->dg0c, pRec->dg0p);
    if(pulseCount > 1) eevrmaSetPulseGen (pCard, 1, pRec->dg1e, pRec->dg1d, pRec->dg1w, pRec->dg1c, pRec->dg1p);
    if(pulseCount > 2) eevrmaSetPulseGen (pCard, 2, pRec->dg2e, pRec->dg2d, pRec->dg2w, pRec->dg2c, pRec->dg2p);
    if(pulseCount > 3) eevrmaSetPulseGen (pCard, 3, pRec->dg3e, pRec->dg3d, pRec->dg3w, pRec->dg3c, pRec->dg3p);
	
    if(pulseCount > 4) eevrmaSetPulseGen (pCard, 4,  pRec->otp0, pRec->ot0d, pRec->ot0w, 1, pRec->ot0p);
    if(pulseCount > 5) eevrmaSetPulseGen (pCard, 5,  pRec->otp1, pRec->ot1d, pRec->ot1w, 1, pRec->ot1p);
    if(pulseCount > 6) eevrmaSetPulseGen (pCard, 6,  pRec->otp2, pRec->ot2d, pRec->ot2w, 1, pRec->ot2p);
    if(pulseCount > 7) eevrmaSetPulseGen (pCard, 7,  pRec->otp3, pRec->ot3d, pRec->ot3w, 1, pRec->ot3p);
    if(pulseCount > 8) eevrmaSetPulseGen (pCard, 8,  pRec->otp4, pRec->ot4d, pRec->ot4w, 1, pRec->ot4p);
    if(pulseCount > 9) eevrmaSetPulseGen (pCard, 9,  pRec->otp5, pRec->ot5d, pRec->ot5w, 1, pRec->ot5p);
    if(pulseCount > 10) eevrmaSetPulseGen (pCard, 10,  pRec->otp6, pRec->ot6d, pRec->ot6w, 1, pRec->ot6p);
    if(pulseCount > 11) eevrmaSetPulseGen (pCard, 11,  pRec->otp7, pRec->ot7d, pRec->ot7w, 1, pRec->ot7p);
    if(pulseCount > 12) eevrmaSetPulseGen (pCard, 12,  pRec->otp8, pRec->ot8d, pRec->ot8w, 1, pRec->ot8p);
    if(pulseCount > 13) eevrmaSetPulseGen (pCard, 13,  pRec->otp9, pRec->ot9d, pRec->ot9w, 1, pRec->ot9p);
    if(pulseCount > 14) eevrmaSetPulseGen (pCard, 14, pRec->otpa, pRec->otad, pRec->otaw, 1, pRec->otap);
    if(pulseCount > 15) eevrmaSetPulseGen (pCard, 15, pRec->otpb, pRec->otbd, pRec->otbw, 1, pRec->otbp);
    if(pulseCount > 16) eevrmaSetPulseGen (pCard, 16, pRec->otpc, pRec->otcd, pRec->otcw, 1, pRec->otcp);
    if(pulseCount > 17) eevrmaSetPulseGen (pCard, 17, pRec->otpd, pRec->otdd, pRec->otdw, 1, pRec->otdp);   

   /*---------------------
    * Enable or Disable Receive Link Frame Error Interrupts
    */
    eevrmaTaxiIrq (pCard, pRec->rxve);

   /*---------------------
    * Set various record fields with the status of the receive link frame error,
    * the frame error count, and the current FPGA version.  Process error count
    * reset request.
    */
   
	{
		int taxiStatus;
		uint32_t fpgaVersion;
		
		evrmaGetStatus(pCard->session, &fpgaVersion, &taxiStatus);
		
		pRec->plok = !taxiStatus;
		if (pRec->rxvr) {
			pRec->rxvr = 0;
			pCard->rxvioCount = 0;
		}
		pRec->taxi = pCard->rxvioCount;
		pRec->fpgv = fpgaVersion;
	}
 
   /*---------------------
    * Unlock the card mutex, mark the record "processed", and return success
    */
    epicsMutexUnlock (pCard->cardLock);
    pRec->udf = 0;
    return (0);

}/*end eevrmaProcess()*/

LOCAL_RTN epicsStatus eevrmaEventInitRecord (ereventRecord*);
LOCAL_RTN epicsStatus eevrmaEventProcess    (ereventRecord*);

/**************************************************************************************************/
/*  Device Support Entry Table (DSET)                                                             */
/**************************************************************************************************/

static ErDsetStruct devMrfErevent = {
    5,                                  /* Number of entries in the Device Support Entry Table    */
    (DEVSUPFUN)NULL,                    /* -- No device report routine                            */
    (DEVSUPFUN)NULL,                    /* -- No device initialization route                      */
    (DEVSUPFUN)eevrmaEventInitRecord,       /* Record initialization routine                          */
    (DEVSUPFUN)NULL,                    /* -- No I/O Interrupt information routine                */
    (DEVSUPFUN)eevrmaEventProcess           /* Record processing routine                              */
};

epicsExportAddress (dset, devMrfErevent);


LOCAL_RTN
epicsStatus eevrmaEventInitRecord (ereventRecord *pRec)
{
   /*---------------------
    * Local variables
    */
    epicsInt16     Card;        /* Event Receiver card number for this record                     */
    VevrStruct  *pCard;       /* Pointer to the Event Receiver card structure for this record   */

   /*---------------------
    * Output a debug message if the debug flag is set.
    */
    if (eevrmaDebug)
        printf ("devMrfEr::ErEventInitRec(%s)\n", pRec->name);

   /*---------------------
    * Make sure the card number is valid
    * by obtaining the address of its card structure
    */
    Card = pRec->out.value.vmeio.card;
    pRec->dpvt = NULL;
    if (NULL == (pCard = eevrmaGetVevrStruct(Card))) {
        recGblRecordError(S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErEventInitRec() invalid card number in OUT field");
        return (S_dev_badCard);
    }/*end if invalid card number*/

   /*---------------------
    * Initialize the record structure
    */
    pRec->dpvt = (void *)pCard; /* Save the address of the card structure */
    pRec->lenm = -1;      	/* Force setting on first process         */
    pRec->lout = 0;             /* Clear the 'last' event mask            */

    return (0);

}/*end eevrmaEventInitRecord()*/

LOCAL_RTN
epicsStatus eevrmaEventProcess (ereventRecord  *pRec)
{
   /*---------------------
    * Local variables
    */
    epicsBoolean   DebugFlag;			/* True if debug output prints are enabled        */
    VevrStruct  *pCard;                       /* Pointer to Event Receiver card structure       */
  
	int pulseCount;
// 	ADBG("eevrmaEventProcess");
	
   /*---------------------
    * Get the card structure.
    * Abort if we don't have a valid card structure.
    */
    if (NULL == (pCard = (VevrStruct *)pRec->dpvt)) {
        pRec->pact = epicsTrue;
        return (-1);
    }/*end if did not have a valid card structure*/

   /*---------------------
    * If debug output is enabled, display the record's enable status
    * Debug output can be enabled for all records by calling "eevrmaDebugLevel(11)",
    * or on a per-record basis by setting the TPRO field to 11.
    */
    DebugFlag = (pRec->tpro > 10) || (eevrmaDebug > 10);
    if (DebugFlag)
        printf ("ErEventProc(%s) entered.  ENAB = %s\n",
                      pRec->name, pRec->enab?"True":"False");

   /*---------------------
    * Lock the event receiver card structure while we process this record
    */
    epicsMutexLock (pCard->cardLock);

	pulseCount = evrmaGetPulseCount(pCard->session);
		
   /*---------------------
    * If the record is enabled, see if the event number or output mask have changed.
    */
    if (pRec->enab) {

		#define SET_FOR_ONE_PULSGEN(N) \
			if(pulseCount > 0x##N) { \
				evrmaSetPulseRamForEvent(pCard->session, 0x##N, \
				pRec->enm, (pRec->out##N != 0) ? (1 << EVR_PULSE_CFG_BIT_TRIGGER) : 0); \
			}
		
		SET_FOR_ONE_PULSGEN(0);
		SET_FOR_ONE_PULSGEN(1);
		SET_FOR_ONE_PULSGEN(2);
		SET_FOR_ONE_PULSGEN(3);
		SET_FOR_ONE_PULSGEN(4);
		SET_FOR_ONE_PULSGEN(5);
		SET_FOR_ONE_PULSGEN(6);
		SET_FOR_ONE_PULSGEN(7);
		SET_FOR_ONE_PULSGEN(8);
		SET_FOR_ONE_PULSGEN(9);
		SET_FOR_ONE_PULSGEN(a);
		SET_FOR_ONE_PULSGEN(b);
		SET_FOR_ONE_PULSGEN(c);
		SET_FOR_ONE_PULSGEN(d);
		
		 if (pRec->vme  != 0) {
			 evrmaSubscribe(pCard->session, pRec->enm);
		 } else {
			 evrmaUnsubscribe(pCard->session, pRec->enm);
		 }

	} else {
		
		/* The record is disabled, disable interrupt and triggers for the event */
		
		#define CLEAR_FOR_ONE_PULSGEN(N) \
			if(pulseCount > 0x##N) { \
				evrmaSetPulseRamForEvent(pCard->session, 0x##N, \
				pRec->enm, 0); \
			}
		
		CLEAR_FOR_ONE_PULSGEN(0);
		CLEAR_FOR_ONE_PULSGEN(1);
		CLEAR_FOR_ONE_PULSGEN(2);
		CLEAR_FOR_ONE_PULSGEN(3);
		CLEAR_FOR_ONE_PULSGEN(4);
		CLEAR_FOR_ONE_PULSGEN(5);
		CLEAR_FOR_ONE_PULSGEN(6);
		CLEAR_FOR_ONE_PULSGEN(7);
		CLEAR_FOR_ONE_PULSGEN(8);
		CLEAR_FOR_ONE_PULSGEN(9);
		CLEAR_FOR_ONE_PULSGEN(a);
		CLEAR_FOR_ONE_PULSGEN(b);
		CLEAR_FOR_ONE_PULSGEN(c);
		CLEAR_FOR_ONE_PULSGEN(d);
		
		evrmaUnsubscribe(pCard->session, pRec->enm);
		
    }/*end if record is enabled*/

   /*---------------------
    * Unlock the Event Record card structure
    */
    epicsMutexUnlock (pCard->cardLock);
    if (DebugFlag)
        printf ("ErEventProc(%s) I/O operations complete\n", pRec->name);

   /*---------------------
    * Raise the record severity to MAJOR, if the event number is not valid.
    */
    if ((pRec->enm > EVRMA_FIFO_MAX_EVENT_CODE) || (pRec->enm < 0))
        recGblSetSevr (pRec, HW_LIMIT_ALARM, MAJOR_ALARM);

    return (0);

}/*end eevrmaEventProcess()*/


/**************************************************************************************************/
/*                         EPICS Event Record Device Support Routines                             */
/*                                                                                                */

/**************************************************************************************************/
/*  Prototype Definitions for EPICS Event Record Device Support Functions                         */
/**************************************************************************************************/

LOCAL_RTN epicsStatus eevrmaEpicsEventInitRec   (eventRecord*);
LOCAL_RTN epicsStatus eevrmaEpicsEventGetIoScan (int, eventRecord*, IOSCANPVT*);

/**************************************************************************************************/
/*  Device Support Entry Table (DSET)                                                             */
/**************************************************************************************************/

static ErDsetStruct devMrfErEpicsEvent = {
    5,                                  /* Number of entries in the Device Support Entry Table    */
    (DEVSUPFUN)NULL,                    /* -- No device report routine                            */
    (DEVSUPFUN)NULL,                    /* -- No device initialization route                      */
    (DEVSUPFUN)eevrmaEpicsEventInitRec,     /* Record initialization routine                          */
    (DEVSUPFUN)eevrmaEpicsEventGetIoScan,   /* Get I/O interrupt information routine                  */
    (DEVSUPFUN)NULL                     /* -- No Record processing routine                        */
};

epicsExportAddress (dset, devMrfErEpicsEvent);


LOCAL_RTN
epicsStatus eevrmaEpicsEventInitRec (eventRecord *pRec)
{
   /*---------------------
    * Local variables
    */
    int            Card;	/* Event Receiver card number for this record                     */
    int            Event;       /* Event number to trigger this record                            */
    VevrStruct  *pCard;       /* Pointer to the Event Receiver card structure for this record   */

   /*---------------------
    * Extract the Event Receiver card number (card) and the event number (signal)
    * from the record's output link.
    */
    Card = pRec->inp.value.vmeio.card;
    Event = pRec->inp.value.vmeio.signal;

   /*---------------------
    * Output a debug message if the debug flag is set.
    */
    if (eevrmaDebug)
        printf ("ErEpicsEventInitRec(%s) Card %d, Event %d\n",
                      pRec->name, Card, Event);

   /*---------------------
    * Make sure the card number is valid
    * by obtaining the address of its card structure
    */
    pRec->dpvt = NULL;
    if (NULL == (pCard = eevrmaGetVevrStruct(Card))) {
        recGblRecordError(S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErEpicsEventInitRec() invalid card number in INP field");
        return(S_dev_badCard);
    }/*end if card number is invalid*/

   /*---------------------
    * Make sure the event number is in the correct range
    */
    if ((Event < EVRMA_FIFO_MIN_EVENT_CODE) || (Event > EVRMA_FIFO_MAX_EVENT_CODE)) {
        recGblRecordError(S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErEpicsEventInitRec() invalid signal number in INP field");
        return(S_dev_badCard);
    }/*end if event number is invalid*/

   /*---------------------
    * Store the address of the IOSCANPVT structure that corresponds
    * to the requested event.
    */
    pRec->dpvt = (void *) &pCard->IoScanPvt[Event];
    return (0);

}/*end eevrmaEpicsEventInitRec()*/

LOCAL_RTN
epicsStatus eevrmaEpicsEventGetIoScan (int cmd, eventRecord *pRec, IOSCANPVT *pPvt)
{
  return (0);

}/*end eevrmaEpicsEventGetIoScan()*/


/**************************************************************************************************/
/*                         EPICS Binary Input Record Device Support Routines                      */
/*                         to check for taxi violation                                            */

/**************************************************************************************************/
/*  Prototype Definitions for EPICS Binary Input Record Device Support Functions                  */
/**************************************************************************************************/

LOCAL_RTN epicsStatus eevrmaEpicsBiInitRec  (biRecord*);
LOCAL_RTN epicsStatus eevrmaEpicsBiProcess  (biRecord*);

/**************************************************************************************************/
/*  Device Support Entry Table (DSET)                                                             */
/**************************************************************************************************/

static ErDsetStruct devMrfErEpicsBi = {
    5,                                  /* Number of entries in the Device Support Entry Table    */
    (DEVSUPFUN)NULL,                    /* -- No device report routine                            */
    (DEVSUPFUN)NULL,                    /* -- No device initialization routine                    */
    (DEVSUPFUN)eevrmaEpicsBiInitRec,     /* Record initialization routine                          */
    (DEVSUPFUN)NULL,                    /* -- No I/O interrupt information routine                */
    (DEVSUPFUN)eevrmaEpicsBiProcess      /* Record processing routine                              */
};

epicsExportAddress (dset, devMrfErEpicsBi);

LOCAL_RTN
epicsStatus eevrmaEpicsBiInitRec (biRecord *pRec)
{
   /*---------------------
    * Local variables
    */
    int            Card;	/* Event Receiver card number for this record                     */
    VevrStruct  *pCard;       /* Pointer to the Event Receiver card structure for this record   */

   /*---------------------
    * Extract the Event Receiver card number (card) from the record's output link.
    */
    Card = pRec->inp.value.vmeio.card;

   /*---------------------
    * Output a debug message if the debug flag is set.
    */
    if (eevrmaDebug)
        printf ("ErEpicsBiInitRec(%s) Card %d\n",
                      pRec->name, Card);

   /*---------------------
    * Make sure the card number is valid
    * by obtaining the address of its card structure
    */
    pRec->dpvt = NULL;
    if (NULL == (pCard = eevrmaGetVevrStruct(Card))) {
        recGblRecordError(S_dev_badCard, (void *)pRec, 
                          "devMrfEr::ErEpicsBiInitRec() invalid card number in INP field");
        return(S_dev_badCard);
    }/*end if card number is invalid*/

    pRec->dpvt = (void *)pCard; /* Save the address of the card structure */
    return (0);

}/*end eevrmaEpicsBiInitRec()*/


LOCAL_RTN
epicsStatus eevrmaEpicsBiProcess (biRecord  *pRec)
{
   /*---------------------
    * Local variables
    */
    VevrStruct  *pCard;                       /* Pointer to Event Receiver card structure       */
  
   /*---------------------
    * Get the card structure.
    * Abort if we don't have a valid card structure.
    */
    if (NULL == (pCard = (VevrStruct *)pRec->dpvt)) {
        pRec->pact = epicsTrue;
        return (-1);
    }/*end if did not have a valid card structure*/

   /*---------------------
    * Lock the event receiver card structure while we process this record
    */
    epicsMutexLock (pCard->cardLock);

	{
		int taxiStatus;
		uint32_t fpgaVersion;
		
		evrmaGetStatus(pCard->session, &fpgaVersion, &taxiStatus);
		
		pRec->val = !taxiStatus;
	}

	
    pRec->udf = 0;

   /*---------------------
    * Unlock the Event Record card structure
    */
    epicsMutexUnlock (pCard->cardLock);

    return (2);

}/*end eevrmaEpicsBiProcess()*/



/*
 *
 * Register a listener for the event system.  Every time we get an event from
 * the event receiver, we will call the registered listener and pass in the
 * event number received and the tick counter value when that event was
 * received.
 *
 **/
GLOBAL_RTN
epicsStatus ErRegisterEventHandler (int card, USER_EVENT_FUNC eventFunc)
{
    VevrStruct  *vevr;

	if (eevrmaDebug){
        printf ("ErRegisterEventHandler(%d, %p)\n", card, (void *)eventFunc);
    }

    if (NULL == (vevr = eevrmaGetVevrStruct(card))) {
        errlogPrintf ("ErRegisterEventHandler() called with invalid card number (%d)\n", card);
        return (-1);
    }

    vevr->eventFunc = (EVENT_FUNC)eventFunc;
    return (0);
}

/*
 *
 * Register an error handler for the event system. 
 * Every time we get an error (rxvio, etc.) from
 * the event receiver, we will call the registered listener and pass in the
 * event number received and the tick counter value when that event was
 * received.
 *
 **/
GLOBAL_RTN
epicsStatus ErRegisterErrorHandler (int Card, USER_ERROR_FUNC ErrorFunc)
{
    VevrStruct  *pCard;

    if (eevrmaDebug){
        printf ("ErRegisterErrorHandler(%d, %p)\n", Card, (void *)ErrorFunc);
    }

    if (NULL == (pCard = eevrmaGetVevrStruct(Card))) {
        errlogPrintf ("ErRegisterErrorHandler() called with invalid card number (%d)\n", Card);
        return (-1);
    }

    pCard->errorFunc = (ERROR_FUNC)ErrorFunc;
    return(0);
}

/*
 * This function was taken from the old 'event' module where the implementation
 * didn't use any mutexes to protect the pCard->pEr value (the mutex protection
 * lines were commented out). Therefore, this implementation also does not
 * provide any mutex protection and the caller must be carefull to call it
 * in a protected context.
 */
epicsStatus ErGetTicks(int Card, epicsUInt32 *Ticks)
{
	VevrStruct *pCard = eevrmaGetVevrStruct(Card);

	if(pCard == NULL)
		return ERROR;

	if(evrmaGetTimestampLatch(pCard->session, Ticks) < 0) return ERROR;

	return OK;
}


epicsStatus ErGetTemperature(int Card, epicsUInt32 *rawTemp, epicsFloat32 *Temp)
{
	VevrStruct *pCard = eevrmaGetVevrStruct(Card);

	if(pCard == NULL);

	if(evrmaGetTemperature(pCard->session, rawTemp) < 0) return ERROR;


	*Temp = ((epicsFloat32)(*rawTemp) * 503.975)/4096. - 273.15;

	return OK;
}

epicsStatus ErGetMaxTemperature(int Card, epicsUInt32 *rawTemp, epicsFloat32 *Temp)
{
	VevrStruct *pCard = eevrmaGetVevrStruct(Card);

	if(pCard == NULL);

	if(evrmaGetMaxTemperature(pCard->session, rawTemp) < 0) return ERROR;


	*Temp = ((epicsFloat32)(*rawTemp) * 503.975)/4096. - 273.15;

	return OK;
}


LOCAL
registryFunctionRef devMrfErRef [] = {
    {"ErRegisterEventHandler", (REGISTRYFUNCTION)ErRegisterEventHandler},
    {"ErRegisterErrorHandler", (REGISTRYFUNCTION)ErRegisterErrorHandler}
};/*end devMrfErRef[]*/

LOCAL_RTN
void devMrfErRegistrar (void) {
    registryFunctionRefAdd (devMrfErRef, NELEMENTS(devMrfErRef));
}/*end devMrfErRegister()*/

epicsExportRegistrar (devMrfErRegistrar);


// -------------------------- FROM drvLinuxEvr.c --------------------------------


epicsStatus eevrmaDrvReport (int level)
{
	int             NumCards = 0;       /* Number of configured cards we found                    */
	VevrStruct   *pCard;              /* Pointer to card structure                              */


	for (pCard = (VevrStruct *)ellFirst(&eevrmaCardList);
		pCard != NULL;
		pCard = (VevrStruct *)ellNext(&pCard->link)) {
		NumCards++;

		printf ("\n-------------------- VEVR#%d Hardware Report --------------------\n", pCard->Cardno);
		printf ("\n NOTE: This report is not fully supported yet for EVRMA  \n");
		
		{
			int taxiStatus;
			uint32_t fpgaVersion;
		
			evrmaGetStatus(pCard->session, &fpgaVersion, &taxiStatus);
			printf("	Firmware Version = %4.4X.\n", fpgaVersion);
		}
		
		printf ("	%d Frame Errors\n", pCard->rxvioCount);
		
		{
			char cmd[256] = "cat ";
			if(evrmaGetSysfsDevice(pCard->session, cmd + 4, 240) == 0) {
				printf ("	Sys config:"); fflush(stdout);
				strcat(cmd, "/config");
				system(cmd);
				printf("\n");
			}
		}
		
		{
			int pulseCount = evrmaGetPulseCount(pCard->session);
			int i;
			for(i = 0; i < pulseCount; i ++) {
				uint32_t prescaler, delay, width;
				if(evrmaGetPulseParams(pCard->session, i, &prescaler, &delay, &width) == 0) {
					printf ("	%d Pulse gen: prescaler=%d, delay=%d, width=%d\n", i, prescaler, delay, width);
				}
			}
		}
		
	}
	if(!NumCards)
		printf ("  No Event Receiver cards were configured\n");


	return OK;
}

epicsStatus eevrmaDrvInit (void)
{
	return OK;
}


drvet drvMrf200Er =
{
    2,                                  /* Number of entries in the table                         */
    (DRVSUPFUN)eevrmaDrvReport,             /* Driver Support Layer device report routine             */
    (DRVSUPFUN)eevrmaDrvInit                /* Driver Support layer device initialization routine     */
};

epicsExportAddress (drvet, drvMrf200Er);


GLOBAL_RTN
epicsUInt32 eevrmaGetSecondsSR (VevrStruct *pCard)
{
	uint32_t secSh;
	if(evrmaGetSecondsShift(pCard->session, &secSh) < 0) return 0;
	
	return secSh;

}/*end eevrmaGetSecondsSR()*/


#ifdef DBG_MEASURE_TIME_FROM_IRQ_TO_USER

static uint32_t dbg_get_time(EvrmaSession session)
{
	uint32_t val;
	
	if(evrmaGetTimestampLatch(session, &val) < 0) return 0;
	return val;
}

// #define OPTIMIZE_EPICS_CALLBACK_TIME
// #define DONT_CALL_EPICS_CALLBACKS

#endif


static void eevrmaEventHandler(EvrmaSession session, void *handlerArg, int event, 
							   uint8_t *dataEvent, int dataLength)
{
	struct VevrStruct *vevr = (struct VevrStruct *)handlerArg;
	
#ifdef DBG_MEASURE_TIME_FROM_IRQ_TO_USER
	uint32_t start_meas = 0, end_meas = 0;
	int doTimeDebug = 0;
	
	start_meas = dbg_get_time(session);

#endif


#ifdef OPTIMIZE_EPICS_CALLBACK_TIME
#else
	epicsMutexLock(vevr->cardLock);
#endif

	if(int_showme>0) { int_showme--; printf("event: %d\n", event); }
	
	if(event >= EVRMA_FIFO_MIN_EVENT_CODE && event <= EVRMA_FIFO_MAX_EVENT_CODE) {
		
#ifdef DBG_MEASURE_TIME_FROM_IRQ_TO_USER
		doTimeDebug = 1;
#endif
		
		struct evr_data_fifo_event *evData = (struct evr_data_fifo_event *)dataEvent;
		
		if(event_showme>0) { event_showme--; printf("FIFOEvent: %d\n", event); }
		if (vevr->devEventFunc != NULL) {
			
#ifdef DONT_CALL_EPICS_CALLBACKS
#else
			(*vevr->devEventFunc)(vevr, event, evData->timestamp);
#endif
			
		}
		
	} else {
		
		switch(event) {
		case EVRMA_EVENT_DBUF_DATA:
		{
#ifdef DBG_MEASURE_TIME_FROM_IRQ_TO_USER
			doTimeDebug = 1;
#endif
			uint32_t *dataDBuf;
			int dbufLength;

			int dbufStatus = evrmaGetDBuf(session, &dataDBuf, &dbufLength);
			
			if(dbufst_showme>0) { dbufst_showme--; printf("DBUF status: %d, size: %d\n", dbufStatus, dbufLength); }
			
			vevr->DBuffError = epicsFalse;
            if(dbufStatus < 0) {

				static int counterHere = 0;
				static int showFac = 100;
				counterHere ++;
				
				if(counterHere % showFac == 0) {
					ADBG("%dX bad dbufStatus (last=%d)", showFac, dbufStatus);
					if(showFac < 100000)
						showFac *= 2;
				}
				
				vevr->DBuffError = epicsTrue;
                if (vevr->devErrorFunc != NULL) {
#ifdef DONT_CALL_EPICS_CALLBACKS
#else
                    (*vevr->devErrorFunc)(vevr, ERROR_DBUF_CHECKSUM);
#endif
                }
            } else {
				if (vevr->devDBuffFunc != NULL) {
					if(dbufst_showme>0) printf("DBUF size:  %d bytes,  %d u32\n", dbufLength, dbufLength>>2); 
					if(dbufst_showme>0) printf("DBUF header (type/version): %8.8x\n", dataDBuf[0]);
#ifdef DONT_CALL_EPICS_CALLBACKS
#else
					(*vevr->devDBuffFunc)(vevr, dbufLength, dataDBuf);
#endif
				}
			}

			break;
		}
		
		case EVRMA_EVENT_ERROR_TAXI:
            vevr->rxvioCount++;
			ADBG("TAXI err");
            if (vevr->devErrorFunc != NULL) {
#ifdef DONT_CALL_EPICS_CALLBACKS
#else
                (*vevr->devErrorFunc)(vevr, ERROR_TAXI);
#endif
            }
			break;
			
		case EVRMA_EVENT_ERROR_LOST:
			ADBG("LOST err");
			if (vevr->devErrorFunc != NULL) {
#ifdef DONT_CALL_EPICS_CALLBACKS
#else
                (*vevr->devErrorFunc)(vevr, ERROR_LOST);
#endif
            }
			break;
			
		case EVRMA_EVENT_ERROR_HEART:
			ADBG("HEART err");
            if (vevr->devErrorFunc != NULL) {
#ifdef DONT_CALL_EPICS_CALLBACKS
#else
                (*vevr->devErrorFunc)(vevr, ERROR_HEART);
#endif
            }
			break;
			
		case EVRMA_EVENT_DELAYED_IRQ:
            if (vevr->devEventFunc != NULL) {
#ifdef DONT_CALL_EPICS_CALLBACKS
#else
                (*vevr->devEventFunc)(vevr, EVENT_DELAYED_IRQ, 0);
#endif
            }
			break;
		}
		
	}

		
#ifdef OPTIMIZE_EPICS_CALLBACK_TIME
#else
	epicsMutexUnlock(vevr->cardLock);
#endif
	
#ifdef DBG_MEASURE_TIME_FROM_IRQ_TO_USER
	end_meas = dbg_get_time(session);
	
	if(doTimeDebug)
		evrmaTimeDebug(event, dataEvent, 150, start_meas, end_meas);
#endif
}

static int eevrmaConfigure (
    int card,                   /* Logical card number for this VEVR. */
	const char *vevrDevName     /* Name of the Linux device. */
)
{
    struct VevrStruct *vevr;
	
    epicsMutexLock(eevrmaCardListLock);
    /* If not already done, initialize the driver structures */
    if (!eevrmaCardListInitDone) {
        ellInit(&eevrmaCardList);
        eevrmaCardListInitDone = epicsTrue;
    }
    epicsMutexUnlock(eevrmaCardListLock);

    epicsMutexLock(eevrmaConfigureLock);
    for (vevr = (VevrStruct *) ellFirst(&eevrmaCardList); vevr != NULL; vevr = (VevrStruct *) ellNext(&vevr->link)) {
        if (vevr->Cardno == card) {
            errlogPrintf("%s: Card number %d has already been configured\n", __func__, card);
            epicsMutexUnlock(eevrmaConfigureLock);
            return ERROR;
        }
    }

    /* Fill in the minimum of the driver structure for driver data structures management */
    vevr = (struct VevrStruct *) malloc(sizeof(struct VevrStruct));
    if (vevr == NULL) {
        errlogPrintf("%s@%d(malloc): failed.\n", __func__, __LINE__);

LUnlockAndError:

		epicsMutexUnlock(eevrmaConfigureLock);
		return ERROR;
    }
    memset(vevr, 0, sizeof(struct VevrStruct));

    vevr->Cardno = card;
    vevr->cardLock = epicsMutexCreate();
    if (vevr->cardLock == 0) {
        errlogPrintf("%s@%d(epicsMutexCreate): failed.\n", __func__, __LINE__);
LFreeAndError:
        free(vevr);
        goto LUnlockAndError;
    }

    ellAdd(&eevrmaCardList, &vevr->link);

    /* Now that the card is registered there is no chance that configure will go through
     again if called with the same card number: we can release the mutex */
    epicsMutexUnlock(eevrmaConfigureLock);
	
	
    epicsMutexLock(vevr->cardLock);

    ADBG("--------------------------- will evrmaOpenSession");
	
	// NOTE: here we have a potential problem. We pass eevrmaEventHandler's
	// argument 'vevr' before it has its 'session' set.
	// eevrmaEventHandler must be prepared for that. If eevrmaEventHandler has
	// a epicsMutexLock(vevr->cardLock) at the start it will stop and that's good.
    vevr->session = evrmaOpenSession(vevrDevName, eevrmaEventHandler, vevr);
    
	if(vevr->session == NULL) {
		
		AERR("evrmaOpenSession failed");
		goto LFreeAndError;
	}
	
	// TODO: Who calls the evrmaCloseSession() ?

    epicsMutexUnlock(vevr->cardLock);

	return OK;
}

static void eevrmaDebugLevel(epicsInt32 level)
{
	eevrmaDebug = level;   /* Set the new debug level */
	return;
}

void eevrmaSubscribeDBuff (VevrStruct *pCard, epicsBoolean enable)
{
	epicsMutexLock(pCard->cardLock);
	if(enable) {
		evrmaSubscribe(pCard->session, EVRMA_EVENT_DBUF_DATA);
	} else {
		evrmaUnsubscribe(pCard->session, EVRMA_EVENT_DBUF_DATA);
	}
	epicsMutexUnlock(pCard->cardLock);
}


/**************************************************************************************************/
/*                              EPICS iocsh extension                                             */
/*                                                                                                */

/* iocsh command: eevrmaConfigure */
LOCAL const iocshArg eevrmaConfigureArg0 = {"Card"       , iocshArgInt};
LOCAL const iocshArg eevrmaConfigureArg1 = {"vevrName", iocshArgString};
LOCAL const iocshArg *const eevrmaConfigureArgs[2] = {
       &eevrmaConfigureArg0,
       &eevrmaConfigureArg1
      };
LOCAL const iocshFuncDef    eevrmaConfigureDef     = {"eevrmaConfigure", 2, eevrmaConfigureArgs};

LOCAL_RTN void eevrmaConfigureCall(const iocshArgBuf * args)
{
	if(eevrmaConfigure(args[0].ival, args[1].sval) == ERROR) {
		AERR("Can't eevrmaConfigure");
		abort();
	}
}

/* iocsh command: eevrmaDebugLevel */
LOCAL const iocshArg eevrmaDebugLevelArg0 = {"Level" , iocshArgInt};
LOCAL const iocshArg *const eevrmaDebugLevelArgs[1] = {&eevrmaDebugLevelArg0};
LOCAL const iocshFuncDef eevrmaDebugLevelDef = {"ErDebugLevel", 1, eevrmaDebugLevelArgs};

LOCAL_RTN void eevrmaDebugLevelCall(const iocshArgBuf * args)
{
 eevrmaDebugLevel((epicsInt32)args[0].ival);
}

/* iocsh command: eevrmaDrvReport */
LOCAL const iocshArg ErDrvReportArg0 = {"Level" , iocshArgInt};
LOCAL const iocshArg *const ErDrvReportArgs[1] = {&ErDrvReportArg0};
LOCAL const iocshFuncDef ErDrvReportDef = {"eevrmaDrvReport", 1, ErDrvReportArgs};

LOCAL_RTN void ErDrvReportCall(const iocshArgBuf * args)
{
 eevrmaDrvReport((epicsInt32)args[0].ival);
}

/* Registration APIs */
LOCAL void drvMrfErRegister()
{
 /* Initialize global variables */
 eevrmaCardListLock = epicsMutexCreate();
 eevrmaConfigureLock = epicsMutexCreate();
 /* register APIs */
 iocshRegister( &eevrmaConfigureDef, eevrmaConfigureCall );
 iocshRegister( &eevrmaDebugLevelDef, eevrmaDebugLevelCall );
 iocshRegister( &ErDrvReportDef, ErDrvReportCall );
}
epicsExportRegistrar(drvMrfErRegister);
epicsExportAddress(int, int_showme);
epicsExportAddress(int, event_showme);
epicsExportAddress(int, dbufst_showme);

// -------------------------- END OF FROM drvLinuxEvr.c --------------------------------



// added to be able to compile evrLab

static void drvMrfRegister() {
}
epicsExportRegistrar(drvMrfRegister);


static void evrEEPROMFixupRegister() {
}
epicsExportRegistrar(evrEEPROMFixupRegister);
