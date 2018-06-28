/*=============================================================================

  Name: devBsaCore.c

  Abs: Device support for BSA record. This is an adapter of the legacy
       BSA system to the backend of BsaCore.
  Auth:
  Rev:

  ---------------------------------------------------------------------------*/

#include "copyright_SLAC.h"	/* SLAC copyright comments */

#include <bsaRecord.h>
#include <devSup.h>
#include <epicsExport.h>
#include <evrPattern.h>
#include <errlog.h>
#include <dbAccess.h>
#include <BsaApi.h>
#include <bsaCallbackApi.h>
#include <stdlib.h>
#include <alarm.h>
#include <recGbl.h>
#include <dbScan.h>
#include <cantProceed.h>
#include <envDefs.h>
#include <initHooks.h>
#include <ellLib.h>

static void onInit(BsaChannel, const epicsTimeStamp *, void *);
static void onResult(BsaChannel, BsaResult, unsigned, void *);
static void onAbort(BsaChannel, const epicsTimeStamp *, int, void *);

struct BsaSimpleDataSinkStruct theSink = {
	OnInit:   onInit,
	OnResult: onResult,
	OnAbort:  onAbort
};

typedef struct BsaRecNode_ {
	ELLNODE           n;
	struct bsaRecord *prec;
} *BsaRecNode;

static ELLLIST precList = ELLLIST_INIT;

typedef struct BsaDpvt {
	BsaResult results;
	unsigned  numResults;
	int       reset;
} BsaDpvt;

static ENV_PARAM doRegisterCb = {
	name:  "EVENT_REGISTER_BSA_CALLBACK",
	pdflt: "1"
};

static void initHook(initHookState state)
{
BsaChannel ch;
BsaRecNode precNode;
bsaRecord  *prec;

	if ( initHookAfterIocRunning == state ) {
		/* IOC is up; it is OK for BSA to scan records */

		while ( (precNode = (BsaRecNode)ellGet( &precList )) ) {
			prec = precNode->prec;
			free( precNode );

			if ( ! (ch = BSA_FindChannel( prec->inp.value.instio.string )) ) {
				errlogPrintf("initHook(bsa,%s): unable to locate BSA channel for channel %s/edef %d\n", prec->name, prec->inp.value.instio.string, prec->edef);
				continue;
			}
			if ( BSA_AddSimpleSink( ch, prec->edef - 1, &theSink, prec, prec->nelm ) ) {
				errlogPrintf("init_record(bsa,%s): unable to add BSA sink for channel %s/edef %d\n", prec->name, prec->inp.value.instio.string, prec->edef);
			}
		}
	}
}

static long init(int pass)
{
long v;
	if ( 0 == pass ) {
		if ( envGetLongConfigParam(&doRegisterCb, &v) || v ) {
			if ( BSA_TimingCallbackRegister( RegisterBsaTimingCallback ) ) {
				errlogPrintf("BSA: unable to register timing callback!\n");
				return -1;
			}
		}
		initHookRegister( initHook );
	}
	return 0;
}

static long init_record(bsaRecord *prec)
{
BsaChannel ch   = 0;
BsaDpvt   *pvt;
BsaRecNode precNode;

long       rval = S_db_badField;

	/* Hack so the legacy BSA database can be used unchanged... */
	prec->scan = SCAN_PASSIVE;

	if ( prec->inp.type != INST_IO ) {
		errlogPrintf("init_record(bsa,%s): INP is not constant (but %d)\n", prec->name, prec->inp.type);
		goto bail;
	}
	if ( prec->edef <= 0 || prec->edef > EDEF_MAX ) {
		errlogPrintf("init_record(bsa,%s): Invalid EDEF %d\n", prec->name, prec->edef);
		goto bail;
	}
	if ( ! (ch = BSA_CreateChannel( prec->inp.value.instio.string )) ) {
		errlogPrintf("init_record(bsa,%s): unable to create BSA channel %s\n", prec->name, prec->inp.value.instio.string);
		goto bail;
	}
	pvt             = (BsaDpvt*) mallocMustSucceed( sizeof(*pvt), "devBsaCore: unable to allocate memory for DPVT" );
	pvt->numResults = 0;
	pvt->results    = 0;
	pvt->reset      = 0;
	prec->dpvt      = pvt;

	/* We cannot activate the BSA sink yet. The callbacks do record processing and this is 'legal' only
	 * after the IOC is fully initialized (which is not yet the case when this code executes).
	 * Thus, we defer activation of the sink to an 'initHook' which executes later...
	 */

	precNode        = (BsaRecNode) mallocMustSucceed( sizeof(*precNode), "devBsaCore: unable to allocate memory for initHook" );
	precNode->prec  = prec;

	ellAdd( &precList, &precNode->n );

	rval            = 0;

bail:
	if ( rval ) {
		if ( ch ) {
			BSA_ReleaseChannel( ch );
		}
		prec->pact = TRUE;
	}
	return rval;
}

static long read_bsa(bsaRecord *prec)
{
BsaDpvt  *pvt = (BsaDpvt*) prec->dpvt;
unsigned  i;
unsigned  maxSevr = INVALID_ALARM;
unsigned  maxStat = UDF_ALARM;
BsaResult result;

short reset = pvt->reset ? 1 : 0;

	if ( prec->res ) {
		prec->res  = 0;
		prec->nord = 0;
	}

	if ( reset || prec->res ) {
		prec->rcnt = 0;
		prec->miss = 0;
	}

	prec->noch    = 0;
	prec->nore    = 0;

	if ( (prec->nord = pvt->numResults) > 0 ) {
		prec->time = pvt->results[pvt->numResults-1].timeStamp;
		maxSevr    = 0;
		maxStat    = 0;
	}

	for ( i=0, result=pvt->results; i<pvt->numResults; i++, result++ ) {
		if ( 0 == (prec->cnt[i] = result->count) ) {
			prec->val[i] = 0./0.;
			prec->rms[i] = 0./0.;
		} else {
			prec->val[i] = result->avg;
			prec->rms[i] = result->rms;
		}
		prec->pid [i]    = result->pulseId;
		prec->pidu[i]    = (result->pulseId >> 32);
		prec->miss      += result->missed;
		prec->rcnt      += result->count;
		if ( result->sevr > maxSevr ) {
			maxSevr = result->sevr;
			maxStat = result->stat;
		}
	}

	recGblSetSevr( prec, maxStat, maxSevr );

	if ( reset ) {
		dbPutLink(&prec->vres, DBR_SHORT, &reset, 1);
		dbPutLink(&prec->rres, DBR_SHORT, &reset, 1);
		dbPutLink(&prec->cres, DBR_SHORT, &reset, 1);
		dbPutLink(&prec->pres, DBR_SHORT, &reset, 1);
		pvt->reset = 0;
	}

	return 0;
}

static void onInit(BsaChannel channel, const epicsTimeStamp *initTime, void *closure)
{
bsaRecord *prec = (bsaRecord*)closure;
BsaDpvt   *pvt  = (BsaDpvt*) prec->dpvt;

	/* Assume we can atomically set and don't have to lock the record */
	pvt->reset = 1;
}

static void onResult(BsaChannel channel, BsaResult results, unsigned numResults, void *closure)
{
bsaRecord *prec = (bsaRecord*)closure;
BsaDpvt   *pvt  = (BsaDpvt*) prec->dpvt;

dbScanLock( (dbCommon*)prec );

	pvt->results    = results;
	pvt->numResults = numResults;

	dbProcess( (dbCommon*)prec );

	pvt->results    = 0;
	pvt->numResults = 0;

dbScanUnlock( (dbCommon*)prec );

	if ( results ) {
		BSA_ReleaseResults( results );
	}
}

static void onAbort(BsaChannel channel, const epicsTimeStamp *timeStamp, int status, void *closure)
{
	/* Unused */
}

struct {
	long number;
	DEVSUPFUN report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_bsa;
	DEVSUPFUN special_linconv;
} devBsa = {
	6,
	NULL,
	init,
	init_record,
	NULL,
	read_bsa,
	NULL
};

epicsExportAddress(dset, devBsa);
