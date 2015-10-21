/*=============================================================================
 
  Name: evrModifier5.c - Pattern Bit Manipulation Routines
        evrModifer5     - Modifier 5 Creation using EDEF Check Bits
        evrModifer5Bits - Get EDEF Check Bits out of Modifier 5

  Abs: This file contains all subroutine support for evr Pattern processing
       records.
       
  Rem: All functions called by the subroutine record get passed one argument:

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

#include "longSubRecord.h"    /* for struct longSubRecord  */
#include "registryFunction.h" /* for epicsExport           */
#include "epicsExport.h"      /* for epicsRegisterFunction */
#include "evrPattern.h"       /* MOD5_NOEDEF_MASK,EDEF_MAX */

/*=============================================================================

  Name: evrModifier5

  Abs:  Create modifier 5 by adding EDEF enable bits from the pattern
        check records into the rest of Modifer5.

		
  Args: Type	            Name        Access	   Description
        ------------------- -----------	---------- ----------------------------
        longSubRecord *     psub        read       point to subroutine record

  Rem:  Subroutine for MODIFIER5 simulation

  Side:

  Sub Inputs/ Outputs:
   Inputs:
    A-T - Pattern Check Results (for 20 EDEFs)
    U   - Modifier5 without EDEF bits
   Outputs:   
    VAL = Modifier5
  Ret:  0

==============================================================================*/
static long evrModifier5(longSubRecord *psub)
{ 
  epicsUInt32 *check_p = &psub->a;
  epicsUInt32  value;
  int          edefIdx;

  value = psub->u & MOD5_NOEDEF_MASK;
  for (edefIdx = 0; edefIdx < EDEF_MAX; edefIdx++, check_p++) {
    value |= ((*check_p) << edefIdx);
  }
  psub->val = value;
  return 0;
}

/*=============================================================================

  Name: evrModifier5Bits

  Abs:  Get EDEF Check Bits out of Modifier 5
		
  Args: Type	            Name        Access	   Description
        ------------------- -----------	---------- ----------------------------
        longSubRecord *     psub        read       point to subroutine record

  Rem:  Subroutine for IOC:LOCA:UNIT:MODIFIER5

  Side:

  Sub Inputs/ Outputs:
   Inputs:
    V = Modifier5
    
   Outputs:   
    A-T - Pattern Check Results (for 20 EDEFs)
    U   - Modifier5 without EDEF bits
    VAL = Modifier5
  Ret:  0

==============================================================================*/
static long evrModifier5Bits(longSubRecord *psub)
{ 
  epicsUInt32 *check_p = &psub->a;
  int         edefIdx;
  
  psub->val = psub->v;
  psub->u   = psub->v & MOD5_NOEDEF_MASK;
  for (edefIdx = 0; edefIdx < EDEF_MAX; edefIdx++, check_p++) {
    if (psub->v & (1 << edefIdx)) *check_p = 1;
    else                          *check_p = 0;
  }
  return 0;
}
epicsRegisterFunction(evrModifier5);
epicsRegisterFunction(evrModifier5Bits);
