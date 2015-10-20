#ifdef USE_EVRMA
#error Must not be used in the EVRMA
#endif

/*
 * This code is a linux implementation of the mrfApp EVR driver code
 *
 * Copyright 2010, Stanford University
 * Authors:
 *  Remi Machet <rmachet@slac.stanford.edu>
 *  Bruce Hill <bhill@slac.stanford.edu>
 *
 * Released under the GPLv2 licence <http://www.gnu.org/licenses/gpl-2.0.html>
 */

#define  EVR_DRIVER_SUPPORT_MODULE   /* Indicates we are in the driver support module environment */
#include <signal.h>
#include "drvMrfEr.h"
#undef EVR_MAX_BUFFER
#include <epicsExport.h>        /* EPICS Symbol exporting macro definitions                       */
#include <registryFunction.h>   /* EPICS Registry support library                                 */
#include <epicsStdio.h>
#include <epicsExit.h>
#include <epicsStdlib.h>        /* EPICS Standard C library support routines                      */
#include <epicsStdioRedirect.h>
#include <errlog.h>
#include <iocsh.h>              /* EPICS iocsh support library                                    */
#include <drvSup.h>
#include <string.h>
#include <errno.h>
#include <endian.h>
#include <unistd.h>
#include <byteswap.h>
#include "erapi.h"

#define DEVNODE_NAME_BASE  "/dev/er"
#define DEVNODE_NAME_BASE1 "/dev/er3"

#ifdef EVENT_CLOCK_SPEED
    #define FR_SYNTH_WORD   EVENT_CLOCK_SPEED
#else
    #warning EVENT_CLOCK_SPEED not defined default to 124.950 MHz
    #define FR_SYNTH_WORD   CLOCK_124950_MHZ
#endif

/**************************************************************************************************/
/*                              Private types                                                     */
/*                                                                                                */


int          int_showme = 0;
int          event_showme = 0;
int          dbufst_showme =0;

#define TOTAL_EVR_PULSES 10
enum outputs_mapping_id
{
 PULSE_GENERATOR_0 = 0,
 PULSE_GENERATOR_1 = 1,
 PULSE_GENERATOR_2 = 2,
 PULSE_GENERATOR_3 = 3,
 PULSE_GENERATOR_4 = 4,
 PULSE_GENERATOR_5 = 5,
 PULSE_GENERATOR_6 = 6,
 PULSE_GENERATOR_7 = 7,
 PULSE_GENERATOR_8 = 8,
 PULSE_GENERATOR_9 = 9,
 DBUS_0 = 32,
 DBUS_1 = 33,
 DBUS_2 = 34,
 DBUS_3 = 35,
 DBUS_4 = 36,
 DBUS_5 = 37,
 DBUS_6 = 38,
 DBUS_7 = 39,
 PRESCALER_0 = 40,
 PRESCALER_1 = 41,
 PRESCALER_2 = 42,
 HIZ     = 61,
 LOGIC_1 = 62,
 LOGIC_0 = 63,
 UNUSED = 63,
};

#define TOTAL_FP_CHANNELS 8
#define TOTAL_TB_CHANNELS 32
#define TOTAL_UO_CHANNELS 16

#define NUM_TRIGGER_EVENTS 0
enum transition_board_channel
{
 DELAYED_PULSE_0 = 0,
/*
 DELAYED_PULSE_1 = 1,
 DELAYED_PULSE_2 = 2,
 DELAYED_PULSE_3 = 3,
*/
 TRIGGER_EVENT_0 = 4,
/*
 TRIGGER_EVENT_1 = 5,
 TRIGGER_EVENT_2 = 6,
 TRIGGER_EVENT_3 = 7,
 TRIGGER_EVENT_4 = 8,
 TRIGGER_EVENT_5 = 9,
 TRIGGER_EVENT_6 = 10,
*/
 OTP_DBUS_0 = TRIGGER_EVENT_0 + NUM_TRIGGER_EVENTS,
/*
 OTP_DBUS_1 = 12,
 OTP_DBUS_2 = 13,
 OTP_DBUS_3 = 14,
 OTP_DBUS_4 = 15,
 OTP_DBUS_5 = 16,
 OTP_DBUS_6 = 17,
 OTP_DBUS_7 = 18,
 OTP_8 = 19,
 OTP_9 = 20,
 OTP_10 = 21,
 OTP_11 = 22,
 OTP_12 = 23,
 OTP_13 = 24,
*/
 OTL_0 = 25,
/*
 OTL_1 = 26,
 OTL_2 = 27,
 OTL_3 = 28,
 OTL_4 = 29,
 OTL_5 = 30,
 OTL_6 = 31,
*/
 UNUSED_CH = 255,
};

#define TOTAL_PULSE_GENERATORS 10

struct PulseInfo
{
 epicsBoolean DBusEnable; /* Set if the DBus is used instead of Pulse */
 epicsBoolean Enable;  /* Should a pulse generator be used */
 epicsUInt32  Delay;  /* Desired delay */
 epicsUInt32  Width;  /* Desired width */
 epicsBoolean Pol;  /* Desired polarity */
};

struct LinuxErCardStruct
{
 struct ErCardStruct    ErCard;
 /* Backplane channels on the old firmware, those now share
    a common set of resources, we need to keep track of them */
 struct PulseInfo    OTP[EVR_NUM_OTP];
 /* we cache the Map for each channel to go faster */
 enum outputs_mapping_id   tb_channel[TOTAL_TB_CHANNELS];
 enum transition_board_channel fp_channel[TOTAL_FP_CHANNELS];
 enum outputs_mapping_id   pulse_irq;
};
#ifdef __compiler_offsetof
#define offsetof(TYPE,MEMBER) __compiler_offsetof(TYPE,MEMBER)
#else
#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#endif
#define ercard_to_linuxercard(pCard) \
    ((struct LinuxErCardStruct *)(((char *)pCard) - offsetof(struct LinuxErCardStruct, ErCard)))

#define EVR_IRQ_OFF      0x0000         /* Turn off All Interrupts                                */
#define EVR_IRQ_ALL      0x001f         /* Enable All Interrupts                                  */
#define EVR_IRQ_TELL     0xffff         /* Return Current Interrupt Enable Mask                   */

/**************************************************************************************************/
/*                              Global variables                                                  */
/*                                                                                                */

static ELLLIST ErCardList;                        /* Linked list of ER card structures */
static epicsBoolean bErCardListInitDone = epicsFalse;
static epicsMutexId ErCardListLock;
static epicsMutexId ErConfigureLock;


/**************************************************************************************************/
/*                              Private APIs                                                      */
/*                                                                                                */
static void BlockSIGIO(void)
{
    static int once_flag = 0;
    sigset_t set;

    if(once_flag) return;  /* if it has been done, nothing to do it again */

    once_flag = 1;
    /* Blocking SIGIO signal for the _MAIN_ thread */
    sigemptyset(&set);
    sigaddset(&set, SIGIO);
    pthread_sigmask(SIG_BLOCK, &set, NULL);
}

static char *FormFactorToString(int formFactor) {
    switch (formFactor) {
    case CPCI_EVR:
        return "cPCI_EVR";
    case PMC_EVR:
        return "PMC_EVR";
    case VME_EVR:
        return "VME_EVR";
    case MCOR_EVR:
        return "MCOR_EVR";
    case PCIE_EVR:
        return "PCIE_EVR";
    default:
        return "Invalid";
    }
}

static int ErGetFormFactor(volatile struct MrfErRegs *pEr) {
    int id = (be32_to_cpu(pEr->FPGAVersion) >> 24) & 0x0F;
    switch (id) {
    case 0x0:
        return CPCI_EVR;
    case 0x1:
        return PMC_EVR;
    case 0x2:
        return VME_EVR;
    case 0xF:
        return MCOR_EVR;
    case 0x7:
        return PCIE_EVR;
    default:
        return ERROR;
    }
}

int find_free_pulsegen(struct LinuxErCardStruct *pLinuxErCard)
{
 enum outputs_mapping_id pulse;
 int id;

 for(pulse = PULSE_GENERATOR_4; pulse <= PULSE_GENERATOR_9; pulse++) {
  if(pLinuxErCard->pulse_irq == pulse) {
   if (ErDebug >=1 )
	printf("%s: pulse %i used by pulse_irq\n", __func__, pulse);
   continue;
  }
  for(id = 0; id < TOTAL_TB_CHANNELS; id++)
   if(pLinuxErCard->tb_channel[id] == pulse)
    break;
  /* If no channel uses this pulse it is available */
  if(id != TOTAL_TB_CHANNELS) {
   if (ErDebug >=1 )
	printf("%s: pulse %i used by tb_channel %i\n", __func__, pulse, id);
   continue;
  }
  if ( ErDebug >= 1 )
   printf( "%s: Pulse %d is available.\n", __func__, pulse - PULSE_GENERATOR_0 );
  return pulse-PULSE_GENERATOR_0;
 }
 return -1;
}

/* To ensure compatibility with old firmwares the front-panel outputs
   are copies of the transition board outputs. This API takes care of
   producing this behavior and must be called every time a transition
   board output is re-mapped */
int update_fp_map(struct LinuxErCardStruct *pLinuxErCard, enum transition_board_channel channel)
{
 int fp, count = 0;
 
 /* just in case someone would call this API with a value
    that is valid for FP but has no meaning as a TB output */
 if(channel >= 32)
  return 0;
 for(fp = 0; fp < TOTAL_FP_CHANNELS; fp++) {
  if(pLinuxErCard->fp_channel[fp] == channel) {
   if(pLinuxErCard->ErCard.FormFactor == PMC_EVR)
   {
    if ( ErDebug >= 1 )
     printf( "%s: Calling EvrSetFPOutMap for ch %d to pulse %d\n", __func__,
       channel, pLinuxErCard->tb_channel[channel] );
    EvrSetFPOutMap(pLinuxErCard->ErCard.pEr, fp, pLinuxErCard->tb_channel[channel]);
   }
   if(pLinuxErCard->ErCard.FormFactor == CPCI_EVR) {
    if ( ErDebug >= 1 )
     printf( "%s: Calling EvrSetUnivOutMap for ch %d to pulse %d\n", __func__,
       channel, pLinuxErCard->tb_channel[channel] );
    EvrSetUnivOutMap(pLinuxErCard->ErCard.pEr, fp, pLinuxErCard->tb_channel[channel]);
   }
   count++;
  }
 }
 return count;
}

epicsUInt16 ErEnableIrq_nolock (ErCardStruct *pCard, epicsUInt16 Mask)
{
 epicsUInt16 ret;
 
 if(Mask == EVR_IRQ_TELL) {
  ret = (epicsUInt16)pCard->IrqVector;
 } else if (Mask == EVR_IRQ_OFF) {
  pCard->IrqVector = EvrIrqEnable(pCard->pEr, 0);
  ret = OK;
 } else {
  pCard->IrqVector = EvrIrqEnable(pCard->pEr, Mask | EVR_IRQ_MASTER_ENABLE);
  ret = OK;
 }
 return ret;
}

/**************************************************************************************************
|* ErIrqHandler () -- Event Receiver Interrupt Service Routine
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|* The following interrupt conditions are handled:
|*
|*    o Receiver Link Error:
|*      Also known as a "TAXI" error (for historical reasons).
|*      Reset the RXVIO bit the the Control/Status register, increment the the count of receive
|*      errors in the Event Receiver Card Structure, and report the error to the device-support layer's
|*      interrupt error reporting routine.
|*
|*    o Lost Heartbeat Error:
|*      Reset the HRTBT bit in the Control/Status register and report the error to the
|*      device-support layer's interrupt error reporting routine.
|*
|*    o Event FIFO Not Empty:
|*      Reset the IRQFL bit in the Control/Status register and extract the queued events
|*      and their timestamps from the FIFO.  In order to prevent long spin-loops at interrupt
|*      level, there is a maximum number of events that will be extracted per interrupt.  This
|*      value is specified by the EVR_FIFO_EVENT_LIMIT symbol defined in the "drvMrfEr.h" header
|*      file. For each event extracted from the FIFO, the device-support layer's event handling
|*      routine is called with the extracted event number and timestamp.
|*
|*    o FIFO Overflow Error:
|*      Reset the FF bit in the Control/Status register and report the error to the
|*      device-support layer's interrupt error reporting routine.
|*
|*    o Delayed Interrupt Condition:
|*      Reset the DIRQ bit in the Control/Status register. Invoke the device-support layer's
|*      event handling routine with the special "Delayed Interrupt" event code.
|*
|*    o Data Buffer Ready Condition:
|*      Check for and report checksum errors to the device-support layer's interrupt error
|*      reporting routine.
|*      If device support has registered a data buffer listener, copy the data buffer into
|*      the card structure and invoke the device-support data listener.
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o Note that the IRQ handler is called by all card the same way
|*   it has to figure out why and who generated an interrupt
|*
\**************************************************************************************************/
int irqCount = 0;
void ErIrqHandler(int signal) {
    struct ErCardStruct *pCard;
    struct MrfErRegs *pEr;
    int flags, i;

    epicsMutexLock(ErCardListLock);
    for (pCard = (ErCardStruct *) ellFirst(&ErCardList); pCard != NULL; pCard = (ErCardStruct *) ellNext(&pCard->Link)) {
        epicsMutexLock(pCard->CardLock);
        if (pCard->IrqLevel != 1) {
            epicsMutexUnlock(pCard->CardLock);
            continue;
        }
        pEr = pCard->pEr;
        flags = EvrGetIrqFlags(pEr);

        if(int_showme>0) { int_showme--; printf("IrqFlag: %8.8x\n", flags); }

        if (flags & ~EVR_IRQFLAG_FIFOFULL) {
            EvrClearIrqFlags(pEr, (flags & ~EVR_IRQFLAG_FIFOFULL));
        }

        irqCount++;

        if (flags & EVR_IRQFLAG_EVENT) {
            struct FIFOEvent fe;
            for (i = 0; i < EVR_FIFO_EVENT_LIMIT; i++) {
                if (EvrGetFIFOEvent(pEr, &fe) < 0) {
                    break;
                }
                if(event_showme>0) { event_showme--; printf("FIFOEvent: %d\n", fe.EventCode); }
                if (pCard->DevEventFunc != NULL) {
                    (*pCard->DevEventFunc)(pCard, fe.EventCode, fe.TimestampLow);
                }
            }
        }

        if (flags & EVR_IRQFLAG_DATABUF) {
            int databuf_sts = EvrGetDBufStatus(pEr);
            if(dbufst_showme>0) { dbufst_showme--; printf("DBUF status: %8.8x\n", databuf_sts); }
            if (databuf_sts & (1 << C_EVR_DATABUF_CHECKSUM)) {
                if (pCard->DevErrorFunc != NULL) {
                    (*pCard->DevErrorFunc)(pCard, ERROR_DBUF_CHECKSUM);
                }
            }
            if (pCard->DevDBuffFunc != NULL) {
                pCard->DBuffSize = (databuf_sts & ((1 << (C_EVR_DATABUF_SIZEHIGH + 1)) - 1));
                if(dbufst_showme>0) printf("DBUF size:  %d bytes,  %d u32\n", pCard->DBuffSize, pCard->DBuffSize>>2); 
                for (i = 0; i < pCard->DBuffSize >> 2; i++) {
                    pCard->DataBuffer[i] = be32_to_cpu(pEr->Databuf[i]);
                }
                if(dbufst_showme>0) printf("DBUF header (type/version): %8.8x\n", pCard->DataBuffer[0]);

                EvrReceiveDBuf(pEr, 1); // That means we only re-enable it if someone cares (DevDBuffFunc set)
                (*pCard->DevDBuffFunc)(pCard, pCard->DBuffSize, pCard->DataBuffer);
            }
        }

        if (flags & EVR_IRQFLAG_PULSE) {
            if (pCard->DevEventFunc != NULL) {
                (*pCard->DevEventFunc)(pCard, EVENT_DELAYED_IRQ, 0);
            }
        }
        if (flags & EVR_IRQFLAG_HEARTBEAT) {
            if (pCard->DevErrorFunc != NULL) {
                (*pCard->DevErrorFunc)(pCard, ERROR_HEART);
            }
        }
        if (flags & EVR_IRQFLAG_FIFOFULL) {
            /* Clear and re-enable the FIFO and if applicable report the error */
            EvrClearFIFO(pEr);
            EvrClearIrqFlags(pEr, EVR_IRQFLAG_FIFOFULL);
            if (pCard->DevErrorFunc != NULL) {
                (*pCard->DevErrorFunc)(pCard, ERROR_LOST);
            }
        }
        if (flags & EVR_IRQFLAG_VIOLATION) {
            pCard->RxvioCount++;
            if (pCard->DevErrorFunc != NULL) {
                (*pCard->DevErrorFunc)(pCard, ERROR_TAXI);
            }
        }

        if (flags) {
            /* EvrClearIrqFlags(pEr, flags); */
            if (pCard->FormFactor == MCOR_EVR) {
                EvrIrqFdHandled(pCard->Slot);
            } else {
                EvrIrqHandled(pCard->Slot);
            }
            epicsMutexUnlock(pCard->CardLock);
            epicsMutexUnlock(ErCardListLock);
            return;
        }
        epicsMutexUnlock(pCard->CardLock);
    }
    epicsMutexUnlock(ErCardListLock);
    errlogPrintf("%s: called but no interrupt found.\n", __func__);
    return;
}
 

/**************************************************************************************************
|* ErConfigure () -- Event Receiver Card Configuration Routine
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      Card        =  (int)         Logical card number for this Event Receiver card.
|*      CardAddress =  (epicsUInt32) Starting address for this card's register map.
|*      IrqVector   =  (epicsUInt32) Interrupt vector for this card.  
|*      IrqLevel    =  (epicsUInt32) VME interrupt request level for this card.
|*      FormFactor  =  (int)         Specifies VME or PMC version of the card.
|*
|*-------------------------------------------------------------------------------------------------
|* IMPLICIT INPUTS:
|*      CardName    =  (char *)      ASCII string identifying the Event Receiver Card.
|*      CfgRegName
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      status      =  (int)         Returns OK if the routine completed successfully.
|*                                   Returns ERROR if the routine could not configure the
|*                                   requested Event Receiver card.
|*
\**************************************************************************************************/

static int ErConfigure (
    int Card,                   /* Logical card number for this Event Receiver card             */
    epicsUInt32 CardAddress,    /* IGNORED: Starting address for this card's register map       */
    epicsUInt32 IrqVector,      /* if VME_EVR, Interrupt request vector, if PMC_EVR set to zero */
    epicsUInt32 IrqLevel,       /* if VME_EVR, Interrupt request level. if PMC_EVR set to zero  */
    int FormFactor)             /* cPCI or PMC form factor                                      */
{
    int ret;
    int fdEvr;
    int actualFormFactor;
    char strDevice[strlen(DEVNODE_NAME_BASE) + 3];
    char strDevice1[strlen(DEVNODE_NAME_BASE1) + 3];
    struct LinuxErCardStruct *pLinuxErCard;
    struct ErCardStruct *pCard;
    struct MrfErRegs *pEr;
    u32 FPGAVersion;

    BlockSIGIO();

    epicsMutexLock(ErCardListLock);
    /* If not already done, initialize the driver structures */
    if (!bErCardListInitDone) {
        ellInit(&ErCardList);
        bErCardListInitDone = epicsTrue;
    }
    epicsMutexUnlock(ErCardListLock);

    epicsMutexLock(ErConfigureLock);
    for (pCard = (ErCardStruct *) ellFirst(&ErCardList); pCard != NULL; pCard = (ErCardStruct *) ellNext(&pCard->Link)) {
        if (pCard->Cardno == Card) {
            errlogPrintf("%s: Card number %d has already been configured\n", __func__, Card);
            epicsMutexUnlock(ErConfigureLock);
            return ERROR;
        }
    }

    /* Look for the EVR */
    ret = snprintf(strDevice, strlen(DEVNODE_NAME_BASE) + 3, DEVNODE_NAME_BASE "%c3", Card + 'a');
    if (ret < 0) {
        errlogPrintf("%s@%d(snprintf): %s.\n", __func__, __LINE__, strerror(-ret));
        epicsMutexUnlock(ErConfigureLock);
        return ERROR;
    }

    ret = snprintf(strDevice1, strlen(DEVNODE_NAME_BASE1) + 3, DEVNODE_NAME_BASE1 "%c3", Card + 'a');
    if (ret < 0) {
        errlogPrintf("%s@%d(snprintf): %s.\n", __func__, __LINE__, strerror(-ret));
        epicsMutexUnlock(ErConfigureLock);
        return ERROR;
    }

    printf("Try EvrOpen, device = %s\n", strDevice);
    fdEvr = EvrOpen(&pEr, strDevice);
    if (fdEvr < 0) {
        errlogPrintf("%s@%d(EvrOpen): %s.\n", __func__, __LINE__, strerror(errno));

        printf("Try EvrOpen, device = %s\n", strDevice1);
        fdEvr = EvrOpen(&pEr, strDevice1);

        if (fdEvr < 0) {
            errlogPrintf("%s@%d(EvrOpen): %s.\n", __func__, __LINE__, strerror(errno));
            epicsMutexUnlock(ErConfigureLock);
            return ERROR;
        }
    }

    /* Check the firmware version */
    FPGAVersion = be32_to_cpu(pEr->FPGAVersion);
    printf("EVR Found with Firmware Revision 0x%04X\n", FPGAVersion);
    switch (FPGAVersion) {
    case PMC_EVR_FIRMWARE_REV_LINUX1:
    case PMC_EVR_FIRMWARE_REV_LINUX2:
    case PMC_EVR_FIRMWARE_REV_LINUX3:
    case PMC_EVR_FIRMWARE_REV_LINUX4:
    case PCIE_EVR_FIRMWARE_REV_LINUX1:
    case PCIE_EEVR_FIRMWARE_REV_LINUX1:
        break;
    case PMC_EVR_FIRMWARE_REV_VME1:
        fprintf( stderr, "\nErConfigure ERROR: This PMC EVR has firmware for a linux based system\n"
                "and cannot be used under RTEMS!\n");
        EvrClose(fdEvr);
        epicsMutexUnlock(ErConfigureLock);
        return ERROR;
    default:
        printf("ErConfigure: WARNING: Unknown firmware revision on PMC EVR!\n");
        break;
    }

    /* Check the hardware signature for an EVR */
    if ((FPGAVersion >> 28) != 0x1) {
        errlogPrintf("%s: invalid hardware signature: 0x%08x.\n", __func__, FPGAVersion);
        EvrClose(fdEvr);
        epicsMutexUnlock(ErConfigureLock);
        return ERROR;
    }

    actualFormFactor = ErGetFormFactor(pEr);
    if (FormFactor != actualFormFactor) {
        printf("Configured for %s form factor, but has %s form factor.\n", FormFactorToString(FormFactor), FormFactorToString(actualFormFactor));
        errlogPrintf("%s: wrong form factor %d, signature is 0x%08x.\n", __func__, FormFactor, FPGAVersion);
        EvrClose(fdEvr);
        epicsMutexUnlock(ErConfigureLock);
        return ERROR;
    }
    printf("Found a %s\n", FormFactorToString(FormFactor));

    /* Fill in the minimum of the driver structure for driver data structures management */
    pLinuxErCard = (struct LinuxErCardStruct *) malloc(sizeof(struct LinuxErCardStruct));
    if (pLinuxErCard == NULL) {
        errlogPrintf("%s@%d(malloc): failed.\n", __func__, __LINE__);
        EvrClose(fdEvr);
        epicsMutexUnlock(ErConfigureLock);
        return ERROR;
    }
    memset(pLinuxErCard, 0, sizeof(struct LinuxErCardStruct));
    pCard = &pLinuxErCard->ErCard;
    pCard->Cardno = Card;
    pCard->CardLock = epicsMutexCreate();
    if (pCard->CardLock == 0) {
        errlogPrintf("%s@%d(epicsMutexCreate): failed.\n", __func__, __LINE__);
        free(pCard);
        EvrClose(fdEvr);
        epicsMutexUnlock(ErConfigureLock);
        return ERROR;
    }
    ellAdd(&ErCardList, &pCard->Link);
    /* Now that the card is registered there is no chance that configure will go through
     again if called with the same card number: we can release the mutex */
    epicsMutexUnlock(ErConfigureLock);

    /* Finish filling the driver structure and configuring the hardware,
     if this fails we cannot release the linked list link, instead we
     we set Cardno to an invalid value */
    epicsMutexLock(pCard->CardLock);
    pCard->pEr = (void *) pEr;
    pCard->Slot = fdEvr; /* we steal this irrelevant field */
    pCard->FormFactor = FormFactor;
    /* Set the clock divider. For now it is a fixed value */
    EvrSetFracDiv(pEr, FR_SYNTH_WORD);
    ErEnableIrq_nolock(pCard, EVR_IRQ_OFF);
    if (FormFactor == MCOR_EVR) {
        EvrIrqFdAssignHandler(fdEvr, ErIrqHandler);
    } else {
        EvrIrqAssignHandler(pEr, fdEvr, ErIrqHandler);
    }
    pCard->IrqLevel = 1; /* Tell the interrupt handler this interrupt is enabled */
    epicsMutexUnlock(pCard->CardLock);
    ErResetAll(pCard);

    /* Backwards compatibility: old firmware had optical signal always
     * looped back to the TX so we enable that here...
     */
    pEr->Control |= be32_to_cpu((1 << C_EVR_CTRL_RXLOOPBACK));
    if (FPGAVersion == PCIE_EVR_FIRMWARE_REV_LINUX1) {
        EvrOutputEnable(pEr, 1);
    }
    return OK;
}

/**************************************************************************************************/
/*                              Public APIs                                                       */
/*                                                                                                */

/**************************************************************************************************
|* ErCheckTaxi () -- Check to See if We Have A Receive Link Framing Error (TAXI)
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|* 
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      status    = (epicsBoolean)   True if a framing error was present.
|*                                   False if a framing error was not present.
|*
\**************************************************************************************************/
epicsBoolean ErCheckTaxi(ErCardStruct *pCard)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	epicsBoolean ret = epicsFalse;
	
	epicsMutexLock(pCard->CardLock);
	if (EvrGetViolation(pEr, 1))
		ret = epicsTrue;
	epicsMutexUnlock(pCard->CardLock);
	return ret;
}

/**************************************************************************************************
|* ErDebugLevel () -- Set the Event Receiver Debug Flag to the Desired Level.
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      Level  = (epicsInt32) New debug level.
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* Debug Level Definitions:
|*    0: No debug output produced.
|*    1: Messages on entry to driver and device initialization routines
|*       Messages when the event mapping RAM is changed.
|*   10: All previous messages, plus:
|*       - Changes to OTP gate parameters.
|*       - Message on entry to interrupt service routine
|*   11: All previous messages, plus:
|*       - Messages describing events extracted from the event FIFO and their timestamps
|*
|* The device-support layer will add additional debug outputs to the above list.
|*
\**************************************************************************************************/
void ErDebugLevel(epicsInt32 level)
{
	ErDebug = level;   /* Set the new debug level */
	return;
}

/**************************************************************************************************
|* ErEnableIrq () -- Enable or Disable Event Receiver Interrupts
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      Mask      = (epicsUInt16)    New value to write to the interrupt enable register.
|*                                   In addition to setting a specified bitmask into the
|*                                   interrupt enable register, three additional special
|*                                   codes are defined for the Mask parameter:
|*                                     - EVR_IRQ_OFF:  Disable all interrupts.
|*                                     - EVR_IRQ_ALL:  Enable all interrupts.
|*                                     - EVR_IRQ_TELL: Return (but do not change) the current
|*                                                     value of the interrupt enable register.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      Mask_or_Status = (epicsUInt16) If the value of the "Mask" parameter is "EVR_IRQ_TELL",
|*                                     return the current mask of enabled interrupts.
|*                                     Otherwise, return OK.
|*
\**************************************************************************************************/
epicsUInt16 ErEnableIrq (ErCardStruct *pCard, epicsUInt16 Mask)
{
	epicsUInt16 ret;
	
	epicsMutexLock(pCard->CardLock);
	ret = ErEnableIrq_nolock(pCard, Mask);
	epicsMutexUnlock(pCard->CardLock);
	return ret;
}

/**************************************************************************************************
|* ErEnableDBuff () -- Enable or Disable the Data Stream
|*-------------------------------------------------------------------------------------------------
|* CALLING SEQUENCE:
|*      ErEnableDBuff (pCard, Enable);
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      Enable    = (epicsBoolean)   If true, enable data stream transmission
|*                                   If false, disable data stream transmission
|*
\**************************************************************************************************/
void ErEnableDBuff(ErCardStruct *pCard, epicsBoolean Enable)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	
	epicsMutexLock(pCard->CardLock);
	if(Enable == epicsTrue) {
		EvrSetDBufMode(pEr, 1);
		EvrReceiveDBuf(pEr, 1);
	} else {
		EvrReceiveDBuf(pEr, 0);
		EvrSetDBufMode(pEr, 0);
	}
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErEnableRam () -- Enable the Selected Event Mapping RAM
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
|*      RamNumber = (int)            Which Event Mapping RAM (1 or 2) we should enable.
|*                                   If 0, disable all Event Mapping.
|*
\**************************************************************************************************/
void ErEnableRam(ErCardStruct *pCard, int RamNumber)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	
	epicsMutexLock(pCard->CardLock);
	EvrMapRamEnable(pEr, RamNumber-1, 1);
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErFinishDrvInit () -- Complete the Event Receiver Board Driver Initialization
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      AfterRecordInit = (int)          0 if the routine is being called before record
|*                                         initialization has started.
|*                                       1 if the routine is being called after record
|*                                         initialzation completes.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      Always returns OK
|*
\**************************************************************************************************/
epicsStatus ErFinishDrvInit(int AfterRecordInit)
{
	return OK;
}

/**************************************************************************************************
|* ErDBuffIrq () -- Enable or Disable Event Receiver "Data Buffer Ready" Intrrupts
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      Enable    = (epicsBoolean)   If true, enable the Data Buffer Ready interrupt.
|*                                   If false, disable the Data Buffer Ready interrupt.
|*
\**************************************************************************************************/
void ErDBuffIrq(ErCardStruct *pCard, epicsBoolean Enable)
{
	int mask;
	
	epicsMutexLock(pCard->CardLock);
	mask = pCard->IrqVector;
	if (Enable)
		mask |= EVR_IRQFLAG_DATABUF;
	else
		if (pCard->IrqVector & EVR_IRQFLAG_DATABUF)
			mask ^= EVR_IRQFLAG_DATABUF;	
	ErEnableIrq_nolock(pCard, mask);
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErEventIrq () -- Enable or Disable Event Receiver Event Interrupts
|*-------------------------------------------------------------------------------------------------
|*
|* This routine will enable or disable the "Event FIFO" interrupt on the specified
|* Event Receiver card.  The "FIFO-Full" interrupt is also enabled/disabled at the same time.
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      Enable    = (epicsBoolean)   If true, enable the event FIFO interrupt.
|*                                   If false, disable the event FIFO interrupt.
|*
\**************************************************************************************************/
void ErEventIrq(ErCardStruct *pCard, epicsBoolean Enable)
{
	int mask;
	
	epicsMutexLock(pCard->CardLock);
	mask = pCard->IrqVector;
	if (Enable) {
		mask |= EVR_IRQFLAG_EVENT | EVR_IRQFLAG_FIFOFULL;
	} else {
		if (pCard->IrqVector & EVR_IRQFLAG_EVENT)
			mask ^= EVR_IRQFLAG_EVENT;
		if (pCard->IrqVector & EVR_IRQFLAG_FIFOFULL)
			mask ^= EVR_IRQFLAG_FIFOFULL;
	}
	ErEnableIrq_nolock(pCard, mask);
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErFlushFifo () -- Flush the Event FIFO
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
\**************************************************************************************************/
void ErFlushFifo(ErCardStruct *pCard)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	int mask;

	epicsMutexLock(pCard->CardLock);
	mask=pCard->IrqVector;
	ErEnableIrq_nolock(pCard, mask & (~EVR_IRQFLAG_FIFOFULL));
	EvrClearFIFO(pEr);
	EvrClearIrqFlags(pEr, EVR_IRQFLAG_FIFOFULL);
	ErEnableIrq_nolock(pCard, mask);
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErGetCardStruct () -- Retrieve a Pointer to the Event Receiver Card Structure
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      Card        = (epicsInt16)     Card number to get the Event Receiver card structure for.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      pCard       = (ErCardStruct *) Pointer to the requested card structure, or NULL if the
|*                                     requested card was not successfully configured.
|*
\**************************************************************************************************/
ErCardStruct *ErGetCardStruct(int Card)
{
	ErCardStruct  *pCard;
	for (pCard = (ErCardStruct *)ellFirst(&ErCardList);
		pCard != NULL;
		pCard = (ErCardStruct *)ellNext(&pCard->Link))
		if (pCard->Cardno == Card)
			return pCard;
	return NULL;
}

/**************************************************************************************************
|* ErGetFpgaVersion () -- Return the Event Receiver's FPGA Version
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|* 
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      version   = (epicsUInt16)    The FPGA version of the requested Event Receiver Card.
|*
\**************************************************************************************************/
epicsUInt16 ErGetFpgaVersion(ErCardStruct *pCard)
{	
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	epicsUInt32 version;

	if ( pEr == NULL )
		errlogPrintf( "%s: NULL EVR structure pointer\n", __func__ );

	/* no need for a lock, this is a read only register */
	version = be32_to_cpu(pEr->FPGAVersion);
	return ((version >> 16) & 0xFF00) | (version & 0xFF);
}

/**************************************************************************************************
|* ErGetSecondsSR () -- Return the Event Receiver's Unlatched Seconds Register
|*-------------------------------------------------------------------------------------------------
|*
|* Read the unlatched timestamp "seconds" from the Event Receiver's SecondsSR register.
|*
|*-------------------------------------------------------------------------------------------------
|* CALLING SEQUENCE:
|*      version = ErGetSecondsSR (pCard);
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|* 
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      seconds  = (epicsUInt16)    The timestamp "seconds" of the requested Event Receiver Card.
|*
\**************************************************************************************************/

GLOBAL_RTN
epicsUInt32 ErGetSecondsSR (ErCardStruct *pCard)
{
    struct MrfErRegs	*	pEr = (struct MrfErRegs *)pCard->pEr;
	
	/* no need for a lock, this is a read only register */
	epicsUInt32		secondsSR = be32_to_cpu(pEr->SecondsShift);
	return secondsSR;

}/*end ErGetSecondsSR()*/

/**************************************************************************************************
|* ErGetRamStatus () -- Return the Enabled/Disabled Status of the Requested Event Mapping RAM
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      RamNumber = (int)            Which Event Map RAM (1 or 2) to check.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      enabled   = (epicsBoolean)   True if the specified Event Map RAM is enabled.
|*                                   False if the specified Event Map RAM is not enabled.
|*
\**************************************************************************************************/
epicsBoolean ErGetRamStatus(ErCardStruct *pCard, int RamNumber)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	epicsUInt32 ctrl;
	
	epicsMutexLock(pCard->CardLock);
	ctrl = be32_to_cpu(pEr->Control);
	epicsMutexUnlock(pCard->CardLock);
	return ((ctrl>>C_EVR_CTRL_MAP_RAM_SELECT) & 1) == (RamNumber-1) ? 
			epicsTrue : epicsFalse;
}

/**************************************************************************************************
|* ErGetTicks () -- Return the Current Value of the Event Counter
|*-------------------------------------------------------------------------------------------------
|*
|* Returns the current value of the "Event Counter", which can represent either:
|*  a) Accumulated timestamp events (Event Code 0x7C),
|*  b) Clock ticks from bit 4 of the distributed data bus, or
|*  c) Scaled event clock ticks.
|* 
|*-------------------------------------------------------------------------------------------------
|* CALLING SEQUENCE:
|*      status = ErGetTicks (Card, &Ticks);
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      Card   = (int)           Card number of the Event Receiver board to get the event
|*                               the counter from.
|*
|*-------------------------------------------------------------------------------------------------
|* OUTPUT PARAMETERS:
|*      Ticks  = (epicsUInt32 *) Pointer to the Event Receiver card structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      status = (epicsStatus)   Returns OK if the operation completed successfully.
|*                               Returns ERROR if the specified card number was not registered.
|*
\**************************************************************************************************/
epicsStatus ErGetTicks(int Card, epicsUInt32 *Ticks)
{
	struct MrfErRegs *pEr;
	ErCardStruct *pCard = ErGetCardStruct(Card);

	if(pCard == NULL)
		return ERROR;
	pEr = (struct MrfErRegs *)pCard->pEr;
	/* epicsMutexLock(pCard->CardLock); */
        *Ticks = (epicsUInt32)EvrGetTimestampLatch(pEr);
	//*Ticks = (epicsUInt32)EvrGetTimestampCounter(pEr);
	/* epicsMutexUnlock(pCard->CardLock); */
	return OK;
}

/**************************************************************************************************
|* ErMasterEnableGet () -- Returns the State of the  Event Receiver Master Enable Bit
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard  = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      state  = (epicsBoolean )  True if the card is enabled.
|*                                False if the card is disabled.
|*
\**************************************************************************************************/
epicsBoolean ErMasterEnableGet(ErCardStruct *pCard)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	epicsBoolean ret;
	
	epicsMutexLock(pCard->CardLock);
	if (EvrGetEnable(pEr)) ret = epicsTrue;
		else ret = epicsFalse;
	epicsMutexUnlock(pCard->CardLock);
	return ret;
}

/**************************************************************************************************
|* ErMasterEnableSet () -- Turn the Event Receiver Master Enable Bit On or Off
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard   = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
|*      Enable  = (epicsBoolean ) True if we are to enable the card.
|*                                False if we are to disable the card.
|*
\**************************************************************************************************/
void ErMasterEnableSet(ErCardStruct *pCard, epicsBoolean Enable)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	
	epicsMutexLock(pCard->CardLock);
	EvrEnable(pEr, Enable == epicsTrue ? 1 : 0);
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErRegisterDevEventHandler () -- Register a Device-Support Level Event Handler
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *)   Pointer to the Event Receiver card structure for the card
|*                                     we are registering with.
|*
|*      EventFunc = (DEV_EVENT_FUNC *) Address of the device-support layer event function.
|*
\**************************************************************************************************/
void ErRegisterDevEventHandler(ErCardStruct *pCard, DEV_EVENT_FUNC EventFunc)
{
	pCard->DevEventFunc = EventFunc;
}

/**************************************************************************************************
|* ErRegisterDevErrorHandler () -- Register a Device-Support Level Error Handler
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *)   Pointer to the Event Receiver card structure for the card
|*                                     we are registering with.
|*
|*      ErrorFunc = (DEV_ERROR_FUNC *) Address of the device-support layer error function.
|*
\**************************************************************************************************/
void ErRegisterDevErrorHandler(ErCardStruct *pCard, DEV_ERROR_FUNC ErrorFunc)
{
	pCard->DevErrorFunc = ErrorFunc;
}

/**************************************************************************************************
|* ErRegisterDevDBuffHandler () -- Register a Device-Support Level Data Buffer Handler
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *)   Pointer to the Event Receiver card structure for the card
|*                                     we are registering with.
|*
|*      DBuffFunc = (DEV_DBUFF_FUNC *) Address of the device-support layer data buffer function.
|*
\**************************************************************************************************/
void ErRegisterDevDBuffHandler (ErCardStruct *pCard, DEV_DBUFF_FUNC DBuffFunc)
{
	pCard->DevDBuffFunc = DBuffFunc;
}

int EvrSetOutMap(volatile struct MrfErRegs *pEr, int output, int map)
{
	switch ( ErGetFormFactor(pEr) ) {
		default:  break;
		case PCIE_EVR:
			return ( output < TOTAL_UO_CHANNELS ) ? EvrSetUnivOutMap(pEr, output, map) : -1;

	}
	return EvrSetTBOutMap(pEr, output, map);
}

/**************************************************************************************************
|* ErResetAll () -- Reset the Event Receiver Card
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|* o Disable all card interrupt sources.
|* o Disable event output mapping.
|* o Flush the event FIFO.
|* o Clear delay, width, and polarity of all Programmable Width (OTP) outputs.
|* o Clear delay, width, prescaler, and polarity of all Programmable Delay (DG) outputs.
|* o Clear the delay and prescaler values for the delayed interrupt.
|* o Disable all Output Level (OTP) outputs.
|* o Disable all Trigger Event Pulse (TRG) outputs.
|* o Disable all Distributed Data Bus outputs.
|* o Clear the event clock pre-scaler.
|* o Disable all front-panel gate outputs.
|* o Disable data stream transmission.
|* o Clear all Control/Status register condition flags
|* o Clear the event output mapping RAM's
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard  = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
\**************************************************************************************************/
void ErResetAll(ErCardStruct *pCard)
{	
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	struct LinuxErCardStruct *pLinuxErCard = ercard_to_linuxercard(pCard);
	int i,j;

	ErMasterEnableSet(pCard, epicsFalse);
	epicsMutexLock(pCard->CardLock);
	ErEnableIrq_nolock(pCard, EVR_IRQ_OFF);
	for(i = 0; i < EVR_MAPRAMS; i++)
		EvrMapRamEnable(pEr, i, 0);
	for(i = 0; i < EVR_NUM_OTP; i++) {
		if(i < 8)
			pLinuxErCard->OTP[i].DBusEnable = epicsTrue;
		else
			pLinuxErCard->OTP[i].DBusEnable = epicsFalse;
		pLinuxErCard->OTP[i].Enable = epicsFalse;
		pLinuxErCard->OTP[i].Delay = 0;
		pLinuxErCard->OTP[i].Width = 0;
		pLinuxErCard->OTP[i].Pol = 0;
	}
	for(i = 0; i < TOTAL_EVR_PULSES; i++) {
		EvrSetPulseParams(pEr, i, 0, 0, 0);
		EvrSetPulseProperties(pEr, i, 1, 0, 0, 0, 0);
	}
	pLinuxErCard->pulse_irq = UNUSED;
	EvrSetPulseIrqMap(pEr, pLinuxErCard->pulse_irq);
	for(i=0; i < TOTAL_TB_CHANNELS; i++) {
		pLinuxErCard->tb_channel[i] = UNUSED;
		EvrSetTBOutMap(pEr, i, pLinuxErCard->tb_channel[i]);
	}
	for(i=0; i < TOTAL_UO_CHANNELS; i++) {
		EvrSetUnivOutMap(pEr, i, HIZ);
	}
	EvrSetTimestampDivider(pEr, 0);
	for(i=0; i < TOTAL_FP_CHANNELS; i++) {
		pLinuxErCard->fp_channel[i] = UNUSED_CH;
		EvrSetFPOutMap(pEr, i, pLinuxErCard->fp_channel[i]);
	}
	EvrReceiveDBuf(pEr, 0);
	EvrSetDBufMode(pEr, 0);
	for(i = 0; i < EVR_MAPRAMS; i++)
		for(j = 0; j < sizeof(pEr->MapRam[0])/sizeof(u32); j++)
			((u32 *)pEr->MapRam[0])[j] = 0;
	EvrClearFIFO(pEr);
	EvrClearIrqFlags(pEr, EVR_IRQFLAG_DATABUF | EVR_IRQFLAG_PULSE | EVR_IRQFLAG_EVENT
				| EVR_IRQFLAG_HEARTBEAT | EVR_IRQFLAG_FIFOFULL | EVR_IRQFLAG_VIOLATION);
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErSetDg () -- Set Parameters for a Programmable Delay (DG) Pulse
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      Channel   = (int)            The DG channel (0-3) that we wish to set.
|*      Enable    = (epicsBoolean)   True if we are to enable the selected DG channel.
|*                                   False if we are to disable the selected DG channel
|*      Delay     = (epicsUInt32)    Desired delay for the DG channel.
|*      Width     = (epicsUInt32)    Desired width for the DG channel.
|*      Prescaler = (epicsUInt16)    Prescaler countdown applied to delay and width.
|*      Polarity  = (epicsBoolean)   0 for normal polarity (high true)
|*                                   1 for reverse polarity (low true)
|* 
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o This routine expects to be called with the Event Receiver card structure locked.
|*
\**************************************************************************************************/
void ErSetDg(ErCardStruct *pCard, int Channel, epicsBoolean Enable, 
			epicsUInt32 Delay, epicsUInt32 Width, 
			epicsUInt16 Prescaler, epicsBoolean Pol)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	struct LinuxErCardStruct *pLinuxErCard = ercard_to_linuxercard(pCard);
	enum outputs_mapping_id map;
	int pulse;

	if ( ErDebug >= 1 )
		printf( "%s: EVR %d-%d %s DG %d: pre=%u, del=%u, wid=%u, pol=%s.\n",
				__func__, pCard->Cardno, pCard->Slot,
				( Enable ? "Enable" : "Disable" ),
				Channel, Prescaler, Delay, Width,
				( Pol ? "Nml" : "Inv" )	);

	if( Channel < 0 || Channel >= EVR_NUM_DG ) {
		errlogPrintf("%s: invalid parameter: Channel = %d.\n", __func__, Channel);
		return;
	}

	epicsMutexLock(pCard->CardLock);
	map = pLinuxErCard->tb_channel[DELAYED_PULSE_0 + Channel];
	if(Enable) {
		/* If the channel is already using a pulse generator we keep it 
		   otherwise we get a new one */
		if((map < PULSE_GENERATOR_0) || (map > PULSE_GENERATOR_9)) {
			pulse = Channel; /* pulse = find_free_pulsegen(pLinuxErCard); */
			/* Check for error */
			if (pulse < 0) {
				epicsMutexUnlock(pCard->CardLock);
				errlogPrintf("%s: No free pulse generators! Wrong template?\n", __func__);
				return;
			}
			map = pulse + PULSE_GENERATOR_0;
			pLinuxErCard->tb_channel[DELAYED_PULSE_0 + Channel] = map;
			EvrSetOutMap(pEr, DELAYED_PULSE_0 + Channel, map);
			/* If a front panel uses the same settings as this TB port we update */
			update_fp_map(pLinuxErCard, DELAYED_PULSE_0 + Channel);
		} else {
			pulse = map - PULSE_GENERATOR_0;
		}
		EvrSetPulseParams(pEr, pulse, Prescaler, Delay, Width);
		EvrSetPulseProperties(pEr, pulse, Pol, 0, 0, 1, 1);
	} else {
		if(map == UNUSED) {
			epicsMutexUnlock(pCard->CardLock);
			return;
		}
		EvrSetOutMap(pEr, DELAYED_PULSE_0 + Channel, UNUSED);
		pLinuxErCard->tb_channel[DELAYED_PULSE_0 + Channel] = UNUSED;
		update_fp_map(pLinuxErCard, DELAYED_PULSE_0 + Channel);
	}

	if ( ErDebug >= 2 )
		EvrDumpPulses( pEr, 10 );

	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErSetDirq () -- Set Parameters for the Delayed Interrupt.
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
|*      Enable    = (epicsBoolean)   True if we are to enable the delayed interrupt.
|*                                   False if we are to disable the delayed interrupt.
|*
|*      Delay     = (epicsUInt16)    Desired delay for the delayed interrupt.
|*
|*      Prescaler = (epicsUInt16)    Prescaler countdown applied to the delayed interrupt.
|* 
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o This routine expects to be called with the Event Receiver card structure locked.
|*
\**************************************************************************************************/
void ErSetDirq(ErCardStruct *pCard, epicsBoolean Enable, epicsUInt16 Delay, epicsUInt16 Prescaler)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	struct LinuxErCardStruct *pLinuxErCard = ercard_to_linuxercard(pCard);
	int pulse;

	if ( ErDebug >= 1 )
		printf( "%s: EVR %d-%d %s D-IRQ: del=%u, pre=%u.\n",
				__func__, pCard->Cardno, pCard->Slot,
				( Enable ? "Enable" : "Disable" ),
				Delay, Prescaler );

	epicsMutexLock(pCard->CardLock);
	if(Enable) {
		if((pLinuxErCard->pulse_irq < PULSE_GENERATOR_0) || (pLinuxErCard->pulse_irq > PULSE_GENERATOR_9)) {
			pulse = find_free_pulsegen(pLinuxErCard);
			/* Check for error */
			if (pulse < 0) {
				epicsMutexUnlock(pCard->CardLock);
				errlogPrintf("%s: No free pulse generators! Wrong template?\n", __func__);
				return;
			}
			pLinuxErCard->pulse_irq = pulse + PULSE_GENERATOR_0;
			EvrSetPulseIrqMap(pEr, pLinuxErCard->pulse_irq);
		} else {
			pulse = pLinuxErCard->pulse_irq - PULSE_GENERATOR_0;
		}
		EvrSetPulseParams(pEr, pulse, Prescaler, Delay, 0);
		EvrSetPulseProperties(pEr, pulse, 1, 0, 0, 1, 1);
	} else {
		if(pLinuxErCard->pulse_irq != UNUSED)
			EvrSetPulseIrqMap(pEr, pLinuxErCard->pulse_irq);
		pLinuxErCard->pulse_irq = UNUSED;
	}		
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErSetFPMap () -- Set Front Panel Signal Output Map
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
|*      Port      = (int)            Output port to be configured.  Ports 0-3 are TTL outputs.
|*                                   Ports 4 and 5 are NIM outputs.
|*
|*      Map       = (epicsUInt16)    Mapping value for which signal should be mapped to the
|*                                   selected port. (See definitions in the "drvMrfEr.h" header
|*                                   file).
|* 
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      status    = (epicsStatus)    Returns OK (success) if the call succeeded.
|*                                   Returns ERROR (failure) if the input parameters were illegal.
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o This routine expects to be called with the Event Receiver card structure locked.
|*
\**************************************************************************************************/
epicsStatus ErSetFPMap(ErCardStruct *pCard, int Port, epicsUInt16 Map)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	struct LinuxErCardStruct *pLinuxErCard = ercard_to_linuxercard(pCard);

	/* An interesting feature of the EVR firmware is that if you access
	   a non-existant port, it will actually write into an existing port
	   (in that case it ignores the upper part of the address) =>
	   we reject any port above 3 which is the limit for PMC, but may
	   not for other cards */
	if((pCard->FormFactor == PMC_EVR) && (Port>=3))
		return ERROR;
	epicsMutexLock(pCard->CardLock);
	/* This is tricky: with FW 230 the Map has changed. The old map
	   was just a way to redirect one of the backplane channel on 
	   the front panel, so we use this approach here too */
	pLinuxErCard->fp_channel[Port] = Map;
	if(Map < 32) {
		/* Maps under 32 are copies of TB output on the old firmware */
		update_fp_map(pLinuxErCard, DELAYED_PULSE_0 + Map);
	} else {
		/* Maps after 32 are the same on both firmware */
		EvrSetFPOutMap(pEr, Port, Map);
	}
	epicsMutexUnlock(pCard->CardLock);
	return OK;
}

/**************************************************************************************************
|* ErSetOtb () -- Enable or Disable the Selected Distributed Bus Channel
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard   = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      Channel = (int)           The distributed bus channel (0-7) that should be enabled or
|*                                disabled.
|*      Enable  = (epicsBoolean ) True if we are to enable the selected bus channel.
|*                                False if we are to disable the selected bus channel.
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o This routine expects to be called with the Event Receiver card structure locked.
|*
\**************************************************************************************************/
void ErSetOtb(ErCardStruct *pCard, int Channel, epicsBoolean Enable)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	struct LinuxErCardStruct *pLinuxErCard = ercard_to_linuxercard(pCard);

	if ( ErDebug >= 1 )
		printf( "%s: EVR %d-%d %s OTB %d\n",
				__func__, pCard->Cardno, pCard->Slot,
				( Enable ? "Enable" : "Disable" ),
				Channel );

	if(Enable) {
		epicsMutexLock(pCard->CardLock);
		EvrSetOutMap(pEr, OTP_DBUS_0 + Channel, DBUS_0 + Channel);
		pLinuxErCard->tb_channel[OTP_DBUS_0 + Channel] = DBUS_0 + Channel;
		pLinuxErCard->OTP[Channel].DBusEnable = epicsTrue;
		update_fp_map(pLinuxErCard, OTP_DBUS_0 + Channel);
		epicsMutexUnlock(pCard->CardLock);
	} else {
		pLinuxErCard->OTP[Channel].DBusEnable = epicsFalse;
		ErSetOtp(	pCard, Channel, pLinuxErCard->OTP[Channel].Enable,
					pLinuxErCard->OTP[Channel].Delay, 
					pLinuxErCard->OTP[Channel].Width,
					pLinuxErCard->OTP[Channel].Pol );
	}
	return;
}

/**************************************************************************************************
|* ErSetOtl () -- Enable or Disable the Selected Level Output Channel
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard   = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      Channel = (int)           The level output channel (0-6) that should be enabled or
|*                                disabled.
|*      Enable  = (epicsBoolean ) True if we are to enable the selected level output channel
|*                                False if we are to disable the selected level output channel.
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o This routine expects to be called with the Event Receiver card structure locked.
|*
\**************************************************************************************************/
void ErSetOtl(ErCardStruct *pCard, int Channel, epicsBoolean Enable)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	struct LinuxErCardStruct *pLinuxErCard = ercard_to_linuxercard(pCard);
	enum outputs_mapping_id map;
	int pulse;
	
	if(Channel >= EVR_NUM_OTL) {
		errlogPrintf("%s: invalid parameter: Channel = %d.\n", __func__, Channel);
		return;
	}
	
	epicsMutexLock(pCard->CardLock);
	map = pLinuxErCard->tb_channel[OTL_0 + Channel];
	if(Enable) {
		/* If the channel is already using a pulse generator we keep it 
		   otherwise we get a new one */
		if((map >= PULSE_GENERATOR_0) && (map <= PULSE_GENERATOR_9)) {
			epicsMutexUnlock(pCard->CardLock);
			return;
		}
		pulse = find_free_pulsegen(pLinuxErCard);
		/* Check for error */
		if (pulse < 0) {
			epicsMutexUnlock(pCard->CardLock);
			errlogPrintf("%s: No free pulse generators! Wrong template?\n", __func__);
			return;
		}
		map = pulse + PULSE_GENERATOR_0;
		pLinuxErCard->tb_channel[OTL_0 + Channel] = map;
		EvrSetPulseParams(pEr, pulse, 1, 0, 0);
		EvrSetPulseProperties(pEr, pulse, 1, 1, 1, 0, 1);
		EvrSetOutMap(pEr, OTL_0 + Channel, map);
		/* If a front panel uses the same settings as this TB port we update */
		update_fp_map(pLinuxErCard, OTL_0 + Channel);
	} else {
		if(map == UNUSED) {
			epicsMutexUnlock(pCard->CardLock);
			return;
		}
		EvrSetOutMap(pEr, OTL_0 + Channel, UNUSED);
		epicsMutexLock(pCard->CardLock);
		pLinuxErCard->tb_channel[OTL_0 + Channel] = UNUSED;
		epicsMutexUnlock(pCard->CardLock);
		update_fp_map(pLinuxErCard, OTL_0 + Channel);
	}		
	epicsMutexUnlock(pCard->CardLock);
	return;
}


/**	ErSetOtp
 *	Enable or disable a Programmable Width (OTP) channel
 ***********************************************************************************************/
void ErSetOtp(
    ErCardStruct                  *pCard,       /* Pointer to Event Receiver card structure       */
    int                            Channel,     /* Channel number (0-13) of the OTP channel		  */
    epicsBoolean                   Enable,      /* Enable/Disable flag                            */
    epicsUInt32                    Delay,       /* Desired delay                                  */
    epicsUInt32                    Width,       /* Desired width                                  */
    epicsBoolean                   Pol)         /* Desired polarity                               */
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	struct LinuxErCardStruct *pLinuxErCard = ercard_to_linuxercard(pCard);
	enum outputs_mapping_id map;
	int pulse;

	if ( ErDebug >= 1 )
		printf( "%s: EVR %d-%d %s OTP%d: del=%u, wid=%u, pol=%s.\n",
				__func__, pCard->Cardno, pCard->Slot,
				( Enable ? "Enable" : "Disable" ),
				Channel, Delay, Width,
				( Pol ? "Nml" : "Inv" )	);

	if( Channel < 0 || Channel >= EVR_NUM_OTP ) {
		errlogPrintf("%s: invalid parameter: Channel = %d.\n", __func__, Channel);
		return;
	}
	epicsMutexLock(pCard->CardLock);

	/* save parameters */
	pLinuxErCard->OTP[Channel].Enable = Enable;
	pLinuxErCard->OTP[Channel].Delay = Delay;
	pLinuxErCard->OTP[Channel].Width = Width;
	pLinuxErCard->OTP[Channel].Pol = Pol;
	
	if(pLinuxErCard->OTP[Channel].DBusEnable == epicsFalse) {
		map = pLinuxErCard->tb_channel[OTP_DBUS_0 + Channel];
		if(Enable) {
			/* If the channel is already using a pulse generator we keep it 
			   otherwise we get a new one */
			if((map < PULSE_GENERATOR_0) || (map > PULSE_GENERATOR_9)) {
				pulse = find_free_pulsegen(pLinuxErCard);
				/* Check for error */
				if (pulse < 0) {
					epicsMutexUnlock(pCard->CardLock);
					errlogPrintf("%s: No free pulse generators! Wrong template?\n", __func__);
					return;
				}
				map = pulse + PULSE_GENERATOR_0;
				pLinuxErCard->tb_channel[OTP_DBUS_0 + Channel] = map;
				EvrSetOutMap(pEr, OTP_DBUS_0 + Channel, map);
				/* If a front panel uses the same settings as this TB port we update */
				update_fp_map(pLinuxErCard, OTP_DBUS_0 + Channel);
			} else {
				pulse = map - PULSE_GENERATOR_0;
			}
			EvrSetPulseParams(pEr, pulse, 1, Delay, Width);
			EvrSetPulseProperties(pEr, pulse, Pol, 0, 0, 1, 1);
		} else {
			if(map == UNUSED) {
				epicsMutexUnlock(pCard->CardLock);
				return;
			}
			EvrSetOutMap(pEr, OTP_DBUS_0 + Channel, UNUSED);
			pLinuxErCard->tb_channel[OTP_DBUS_0 + Channel] = UNUSED;
			update_fp_map(pLinuxErCard, OTP_DBUS_0 + Channel);
		}		
	}
	epicsMutexUnlock(pCard->CardLock);
	return;
}

void ErSetTrg(ErCardStruct *pCard, int Channel, epicsBoolean Enable)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	struct LinuxErCardStruct *pLinuxErCard = ercard_to_linuxercard(pCard);
	enum outputs_mapping_id map;
	int pulse;

	if ( ErDebug >= 1 )
		printf( "%s: EVR %d-%d %s Trg %d.\n",
				__func__, pCard->Cardno, pCard->Slot,
				( Enable ? "Enable" : "Disable" ),
				Channel );
	
	if ( Channel < 0 || Channel >= EVR_NUM_TRG ) {
		errlogPrintf("%s: invalid parameter: Channel = %d.\n", __func__, Channel);
		return;
	}

    if ( Channel >= NUM_TRIGGER_EVENTS ) {
		if ( ErDebug >= 1 )
			printf( "%s: EVR %d-%d requested channel %i >= configured trigger events (%i)\n",
			        __func__, pCard->Cardno, pCard->Slot, Channel, NUM_TRIGGER_EVENTS );
		return;
	}

	epicsMutexLock(pCard->CardLock);
	map = pLinuxErCard->tb_channel[TRIGGER_EVENT_0 + Channel];
	if(Enable) {
		/* If the channel is already using a pulse generator we keep it 
		   otherwise we get a new one */
		if((map >= PULSE_GENERATOR_0) && (map <= PULSE_GENERATOR_9)) {
			epicsMutexUnlock(pCard->CardLock);
			return;
		}
		pulse = find_free_pulsegen(pLinuxErCard);
		/* Check for error */
		if (pulse < 0) {
			epicsMutexUnlock(pCard->CardLock);
			errlogPrintf("%s: No free pulse generators! Wrong template?\n", __func__);
			return;
		}
		map = pulse + PULSE_GENERATOR_0;
		EvrSetPulseParams(pEr, pulse, 0, 0, 0);
		EvrSetPulseProperties(pEr, pulse, 1, 0, 0, 1, 1);
		EvrSetOutMap(pEr, TRIGGER_EVENT_0 + Channel, map);
		pLinuxErCard->tb_channel[TRIGGER_EVENT_0 + Channel] = map;
		/* If a front panel uses the same settings as this TB port we update */
		update_fp_map(pLinuxErCard, TRIGGER_EVENT_0 + Channel);
	} else {
		if(map == UNUSED) {
			epicsMutexUnlock(pCard->CardLock);
			return;
		}
		EvrSetOutMap(pEr, TRIGGER_EVENT_0 + Channel, UNUSED);
		pLinuxErCard->tb_channel[TRIGGER_EVENT_0 + Channel] = UNUSED;
		update_fp_map(pLinuxErCard, TRIGGER_EVENT_0 + Channel);
	}		
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErSetTickPre () -- Set the Tick Counter Prescaler
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard   = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
|*      Counter = (epicsUInt16)    If non-zero, the tick counter will count event-clock
|*                                 ticks scaled by this value.
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o This routine expects to be called with the Event Receiver card structure locked.
|*
\**************************************************************************************************/
void ErSetTickPre(ErCardStruct *pCard, epicsUInt16 Counts)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;

/*	epicsMutexLock(pCard->CardLock); */
	EvrSetTimestampDivider(pEr, Counts);
/*	epicsMutexUnlock(pCard->CardLock); */
	return;
}

/**************************************************************************************************
|* ErTaxiIrq () -- Enable or Disable Event Receiver "TAXI" Violation Interrupts
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*      Enable    = (epicsBoolean)   If true, enable the Receive Link Violation interrupt.
|*                                   If false, disable the Receive Link Violation interrupt.
|*
\**************************************************************************************************/
void ErTaxiIrq(ErCardStruct *pCard, epicsBoolean Enable)
{
	int mask;
	
	epicsMutexLock(pCard->CardLock);
	mask = pCard->IrqVector;
	if (Enable)
		mask |= EVR_IRQFLAG_VIOLATION;
	else
		if (pCard->IrqVector & EVR_IRQFLAG_VIOLATION)
			mask ^= EVR_IRQFLAG_VIOLATION;	
	ErEnableIrq_nolock(pCard, mask);
	epicsMutexUnlock(pCard->CardLock);
	return;
}

/**************************************************************************************************
|* ErProgramRam () -- Load the Selected Event Mapping RAM
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
|*      RamBuf    = (epicsUInt16 *)  Pointer to the array of event output mapping words that we
|*                                   are to write to the requested mapping RAM
|*
|*      RamNumber = (int)            Which Event Mapping RAM (1 or 2) we should write.
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o This routine expects to be called with the Event Receiver card structure locked.
|* WARNING:
|* o Assume all outputs have been configured first!!!
|*
\**************************************************************************************************/
void ErProgramRam(ErCardStruct *pCard, epicsUInt16 *RamBuf, int RamNumber)
{
	struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
	struct LinuxErCardStruct *pLinuxErCard = ercard_to_linuxercard(pCard);
	int code;
	epicsUInt32 ctrl;

	if ((RamNumber < 1) || (RamNumber > EVR_MAPRAMS)) {
		errlogPrintf("%s: RAM %d is not valid.\n", __func__, RamNumber);
		return;
	}
	epicsMutexLock(pCard->CardLock);
	ctrl = be32_to_cpu(pEr->Control);
	if (((ctrl>>C_EVR_CTRL_MAP_RAM_SELECT) & 1) == (RamNumber-1)) {
		epicsMutexUnlock(pCard->CardLock);
		errlogPrintf("%s: RAM %d is active.\n", __func__, RamNumber);
		return;
	}
	for(code=0; code < EVR_MAX_EVENT_CODE; code++) {
		struct MapRamItemStruct ramloc = { 0 };
		int map;

		if(code == 0x70)
			ramloc.IntEvent |= 1<<C_EVR_MAP_SECONDS_0;
		else if(code == 0x71)
			ramloc.IntEvent |= 1<<C_EVR_MAP_SECONDS_1;
		else if(code == 0x7A)
			ramloc.IntEvent |= 1<<C_EVR_MAP_HEARTBEAT_EVENT;
		else if(code == 0x7B)
			ramloc.IntEvent |= 1<<C_EVR_MAP_RESETPRESC_EVENT;
		else if(code == 0x7C)
			ramloc.IntEvent |= 1<<C_EVR_MAP_TIMESTAMP_CLK;
		else if(code == 0x7D)
			ramloc.IntEvent |= 1<<C_EVR_MAP_TIMESTAMP_RESET;
		if(RamBuf[code] & (1<<15))
			ramloc.IntEvent |= 1<<C_EVR_MAP_SAVE_EVENT;
		if(RamBuf[code] & (1<<14))
			ramloc.IntEvent |= 1<<C_EVR_MAP_LATCH_TIMESTAMP;
		for(map=0; map < 14; map++) {
			if(RamBuf[code] & (1<<map)) {
				enum outputs_mapping_id func;
				func = pLinuxErCard->tb_channel[OTL_0+(map>>1)];
				if((func>=PULSE_GENERATOR_0) && (func <= PULSE_GENERATOR_9)) {
					if(map & 1)
						ramloc.PulseSet |= 1<<(func-PULSE_GENERATOR_0);
					else
						ramloc.PulseClear |= 1<<(func-PULSE_GENERATOR_0);
				}
				func = pLinuxErCard->tb_channel[OTP_DBUS_0+map];
				if((func>=PULSE_GENERATOR_0) && (func <= PULSE_GENERATOR_9))
					ramloc.PulseTrigger |= 1<<(func-PULSE_GENERATOR_0);
				if(map < EVR_NUM_DG) {
					func = pLinuxErCard->tb_channel[DELAYED_PULSE_0+map];
					if((func>=PULSE_GENERATOR_0) && (func <= PULSE_GENERATOR_9))
						ramloc.PulseTrigger |= 1<<(func-PULSE_GENERATOR_0);
				}
			}
		}
		pEr->MapRam[RamNumber-1][code].IntEvent = be32_to_cpu(ramloc.IntEvent);
		pEr->MapRam[RamNumber-1][code].PulseSet = be32_to_cpu(ramloc.PulseSet);
		pEr->MapRam[RamNumber-1][code].PulseClear = be32_to_cpu(ramloc.PulseClear);
		pEr->MapRam[RamNumber-1][code].PulseTrigger = be32_to_cpu(ramloc.PulseTrigger);
	}
	epicsMutexUnlock(pCard->CardLock);
}

/**************************************************************************************************
|* ErUpdateRam () -- Load and Activate a New Event Map
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      pCard     = (ErCardStruct *) Pointer to the Event Receiver card structure.
|*
|*      RamBuf    = (epicsUInt16 *)  Pointer to the array of event output mapping words that we
|*                                   will write to the chosen Event Mapping RAM.
|*
|*-------------------------------------------------------------------------------------------------
|* NOTES:
|* o This routine expects to be called with the Event Receiver card structure locked.
|*
\**************************************************************************************************/
void ErUpdateRam(ErCardStruct *pCard, epicsUInt16 *RamBuf)
{
	int ram;

	if(ErGetRamStatus(pCard, 1))
		ram = 2;
	else ram = 1;
	ErProgramRam(pCard, RamBuf, ram);
	ErEnableRam(pCard, ram);
}

/**************************************************************************************************
|* ErDrvReport () -- Event Receiver Driver Report Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine gets called by the EPICS "dbior" routine.  For each configured Event Receiver
|* card, it display's the card's slot number, firmware version, hardware address, interrupt
|* vector, interupt level, enabled/disabled status, and number of event link frame errors
|* seen so far.
|*
|*-------------------------------------------------------------------------------------------------
|* INPUT PARAMETERS:
|*      level       = (int)     Indicator of how much information is to be displayed
|*                              (currently ignored).
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      Always returns OK;
|*
\**************************************************************************************************/
epicsStatus ErDrvReport (int level)
{
	int             NumCards = 0;       /* Number of configured cards we found                    */
	ErCardStruct   *pCard;              /* Pointer to card structure                              */


	for (pCard = (ErCardStruct *)ellFirst(&ErCardList);
		pCard != NULL;
		pCard = (ErCardStruct *)ellNext(&pCard->Link)) {
		struct MrfErRegs *pEr = (struct MrfErRegs *)pCard->pEr;
		NumCards++;

		printf ("\n-------------------- EVR#%d Hardware Report --------------------\n", pCard->Cardno);
		printf("	Form factor %s.\n", FormFactorToString( ErGetFormFactor(pEr) ) );
		printf("	Firmware Version = %4.4X.\n", ErGetFpgaVersion(pCard));
		printf ("	Address = %p.\n", pCard->pEr);
		printf ("	%s,  ", ErMasterEnableGet(pCard)? "Enabled" : "Disabled");
		printf ("	%d Frame Errors\n", pCard->RxvioCount);
		EvrDumpStatus( pEr );
		EvrDumpPulses(		pEr, 10 );
		EvrDumpFPOutMap(	pEr, 3 );
		EvrDumpTBOutMap(	pEr, 3 );
                printf("        MapRam 1\n");
                EvrDumpMapRam(pEr, 0);
                printf("        MapRam 2\n");
                EvrDumpMapRam(pEr,1);
	}
	if(!NumCards)
		printf ("  No Event Receiver cards were configured\n");


	return OK;
}

/**************************************************************************************************
|* ErShutdownFunc () -- Disable Event Receiver Interrupts on Soft Reboot
|*-------------------------------------------------------------------------------------------------
|*
|* This is an "exit handler" which is invoked when the IOC is soft rebooted.
|* It is enabled in the driver initialization routine (ErDrvInit) via a call to "epicsAtExit".
|*
|*-------------------------------------------------------------------------------------------------
|* IMPLICIT INPUTS:
|*      ErCardList  = (ELLLIST) Linked list of all configured Event Receiver card structures.
|*
\**************************************************************************************************/
void ErShutdownFunc (void *arg)
{
	ErCardStruct  *pCard;

	/* Loop to close all cards */
	for (pCard = (ErCardStruct *)ellFirst(&ErCardList);
		pCard != NULL;
		pCard = (ErCardStruct *)ellNext(&pCard->Link)) {
		close(pCard->Slot);
	}
}

/**************************************************************************************************
|* ErDrvInit () -- Driver Initialization Routine
|*-------------------------------------------------------------------------------------------------
|*
|* This routine is called from the EPICS iocInit() routine. It gets called prior to any of the
|* device or record support initialization routines.
|*
|*-------------------------------------------------------------------------------------------------
|* FUNCTION:
|*    o Disable any further calls to the ErConfigure routine.
|*    o Add a hook into EPICS to disable Event Receiver card interrupts in the event of a
|*      soft reboot.
|*
|*-------------------------------------------------------------------------------------------------
|* RETURNS:
|*      Always returns OK;
|*
\**************************************************************************************************/
epicsStatus ErDrvInit (void)
{
	epicsAtExit (&ErShutdownFunc, NULL);

	return OK;
}

/**************************************************************************************************/
/*                              EPICS records and PVs                                             */
/*                                                                                                */


drvet drvMrf200Er =
{
    2,                                  /* Number of entries in the table                         */
    (DRVSUPFUN)ErDrvReport,             /* Driver Support Layer device report routine             */
    (DRVSUPFUN)ErDrvInit                /* Driver Support layer device initialization routine     */
};

epicsExportAddress (drvet, drvMrf200Er);


/**************************************************************************************************/
/*                              EPICS iocsh extension                                             */
/*                                                                                                */

/* iocsh command: ErConfigure */
LOCAL const iocshArg ErConfigureArg0 = {"Card"       , iocshArgInt};
LOCAL const iocshArg ErConfigureArg1 = {"CardAddress", iocshArgInt};
LOCAL const iocshArg ErConfigureArg2 = {"IrqVector"  , iocshArgInt};
LOCAL const iocshArg ErConfigureArg3 = {"IrqLevel"   , iocshArgInt};
LOCAL const iocshArg ErConfigureArg4 = {"FormFactor"    , iocshArgInt};
LOCAL const iocshArg *const ErConfigureArgs[5] = {
       &ErConfigureArg0,
       &ErConfigureArg1,
       &ErConfigureArg2,
       &ErConfigureArg3,
       &ErConfigureArg4
      };
LOCAL const iocshFuncDef    ErConfigureDef     = {"ErConfigure", 5, ErConfigureArgs};

LOCAL_RTN void ErConfigureCall(const iocshArgBuf * args)
{
 ErConfigure(args[0].ival, (epicsUInt32)args[1].ival,
   (epicsUInt32)args[2].ival, (epicsUInt32)args[3].ival,
   args[4].ival);
}

/* iocsh command: ErDebugLevel */
LOCAL const iocshArg ErDebugLevelArg0 = {"Level" , iocshArgInt};
LOCAL const iocshArg *const ErDebugLevelArgs[1] = {&ErDebugLevelArg0};
LOCAL const iocshFuncDef ErDebugLevelDef = {"ErDebugLevel", 1, ErDebugLevelArgs};

LOCAL_RTN void ErDebugLevelCall(const iocshArgBuf * args)
{
 ErDebugLevel((epicsInt32)args[0].ival);
}

/* iocsh command: ErDrvReport */
LOCAL const iocshArg ErDrvReportArg0 = {"Level" , iocshArgInt};
LOCAL const iocshArg *const ErDrvReportArgs[1] = {&ErDrvReportArg0};
LOCAL const iocshFuncDef ErDrvReportDef = {"ErDrvReport", 1, ErDrvReportArgs};

LOCAL_RTN void ErDrvReportCall(const iocshArgBuf * args)
{
 ErDrvReport((epicsInt32)args[0].ival);
}

/* Registration APIs */
LOCAL void drvMrfErRegister()
{
 /* Initialize global variables */
 ErCardListLock = epicsMutexCreate();
 ErConfigureLock = epicsMutexCreate();
 /* register APIs */
 iocshRegister( &ErConfigureDef, ErConfigureCall );
 iocshRegister( &ErDebugLevelDef, ErDebugLevelCall );
 iocshRegister( &ErDrvReportDef, ErDrvReportCall );
}
epicsExportRegistrar(drvMrfErRegister);
epicsExportAddress(int, int_showme);
epicsExportAddress(int, event_showme);
epicsExportAddress(int, dbufst_showme);
