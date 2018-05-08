/*=============================================================================
 
  Name: devAoBsaCore.c

  Abs: Backwards-compatibility device support for AO.
       Lets the record forward data directly in to the BsaCore.
  Auth:  
  Rev:  

  ---------------------------------------------------------------------------*/

#include "copyright_SLAC.h"	/* SLAC copyright comments */

#include <devSup.h>
#include <aoRecord.h>
#include <epicsExport.h>
#include <evrTime.h>
#include <stdio.h>
#include <BsaApi.h>
#include <errlog.h>
#include <recGbl.h>
#include <alarm.h>
#include <dbLink.h>
#include <dbScan.h>

#undef  BSA_DEBUG

static long init_record(aoRecord *pao)
{
BsaChannel ch;

	if ( pao->dol.type != DB_LINK && pao->dol.type != CA_LINK ) {
		errlogPrintf("devAoBsaCore (init_record) (%s) DOL must be PV link\n", pao->name);
		pao->pact = TRUE;
		return -1;
	}
	ch = BSA_CreateChannel( pao->dol.value.pv_link.pvname );
	pao->dpvt = ch;
	return 0;
}

static long spc_linconv(aoRecord *pao, int after)
{
	return 0;
}

static long write_ao(aoRecord *pao)
{
BsaChannel     ch = (BsaChannel)pao->dpvt;
epicsTimeStamp t;
epicsEnum16    stat, sevr;
long           status;

	if ( (status = dbGetAlarm( &pao->dol, &stat, &sevr )) ) {
		goto bail;
	}
	if ( (status = dbGetTimeStamp( &pao->dol, &t )) ) {
		goto bail;
	}
	if ( (status = BSA_StoreData( ch, t, pao->val, stat, sevr )) ) {
		goto bail;
	}

bail:
	if ( status ) {
		recGblSetSevr( pao, WRITE_ALARM, INVALID_ALARM );
	}
	return status;
}

static long report(aoRecord *pao)
{
	printf("Device support for BSA Core legacy (record-based) data source\n");
	return 0;
}

struct {
	long number;
	DEVSUPFUN report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN write_ao;
	DEVSUPFUN special_linconv;
	
} devAoBsa = {
	6,
	(DEVSUPFUN) report,
	NULL,
	(DEVSUPFUN) init_record,
	NULL,
	(DEVSUPFUN) write_ao,
	(DEVSUPFUN) spc_linconv
};

epicsExportAddress(dset, devAoBsa);
