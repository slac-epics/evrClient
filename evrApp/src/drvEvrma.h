#ifndef DRV_EVRMA_H
#define DRV_EVRMA_H

#include <mrfCommon.h>          /* MRF Event system constants and definitions                     */
#include "libevrma.h"

/**************************************************************************************************/
/*  Error Numbers Passed To Event Receiver ERROR_FUNC Routines                                    */
/**************************************************************************************************/

#define ERROR_TAXI                    1        /* Taxi (receive link) violation                   */
#define ERROR_HEART                   2        /* Lost the system heart beat                      */
#define ERROR_LOST                    3        /* Events were lost                                */
#define ERROR_DBUF_CHECKSUM           5        /* Data stream checksum error                      */

/*---------------------
 * Define the maximum length of a debug message generated from interrupt level
 */
#define EVR_INT_MSG_LEN         256

struct VevrStruct;

/*---------------------
 * Dummy type definitions for user-defined event, error, and data buffer call-back functions.
 */
typedef void (*EVENT_FUNC) (void);
typedef void (*ERROR_FUNC) (void);
typedef void (*DBUFF_FUNC) (void);

/*---------------------
 * Function prototypes for the device-support error, event, and data buffer ready
 * notification functions.
 */
typedef void (*DEV_EVENT_FUNC)  (struct VevrStruct *vevr, epicsInt16 EventNum, epicsUInt32 Ticks);
typedef void (*DEV_ERROR_FUNC)  (struct VevrStruct *vevr, int ErrorNum);
typedef void (*DEV_DBUFF_FUNC)  (struct VevrStruct *vevr, epicsInt16 Size, void *Buffer);

struct VevrStruct {
    ELLNODE         link;                   /* Linked list node structure                         */
    void           *pRec;                   /* Pointer to the ER record                           */

    // these three names start with uppercase because they are used from the old code.
    epicsInt16      Cardno;                 /* Logical card number                                */
    epicsBoolean    DBuffError;             /* True if there was a data buffer error              */
    IOSCANPVT       IoScanPvt  [EVENT_DELAYED_IRQ+1];/* Event-based record processing structures  */

    epicsMutexId    cardLock;               /* Mutex to lock acces to the card                    */
    
    EvrmaSession    session;
	
	DEV_EVENT_FUNC  devEventFunc;			/* Pointer to device-support event handling routine   */
    DEV_ERROR_FUNC  devErrorFunc;           /* Pointer to device-support error handling routine   */
    DEV_DBUFF_FUNC  devDBuffFunc;           /* Pointer to device-support data buffer ready rtn.   */

    EVENT_FUNC      eventFunc;              /* Pointer to user-registered event handling rtn.     */
    ERROR_FUNC      errorFunc;              /* Pointer to user-registered error handling rtn.     */
    DBUFF_FUNC      dBuffFunc;              /* Pointer to user-registered data buffer ready rtn.  */
	
	int rxvioCount;
    char            intMsg     [EVR_INT_MSG_LEN];    /* Buffer for interrupt debug messages       */
} ;


/*---------------------
 * Event Receiver card structure prototype
 */
typedef struct VevrStruct VevrStruct;


VevrStruct  *eevrmaGetVevrStruct (int);
epicsUInt32       eevrmaGetSecondsSR (VevrStruct*);


void              eevrmaRegisterDevDBuffHandler (VevrStruct*, DEV_DBUFF_FUNC);

/**
 * This function replaces two old functions: ErEnableDBuff + ErDBuffIrq
 */
void              eevrmaSubscribeDBuff (VevrStruct*, epicsBoolean);

/* This function does not reference the VevrStruct but the card num.
 * Therefore it should have been in  devMrfEr.h from the very start.
 * Therefore, not renaming to evrmaXxxx.
 * It is needed (called from the evrSupport).
 */
epicsStatus    ErGetTicks (int, epicsUInt32*);


#endif /* DRV_EVRMA_H */

