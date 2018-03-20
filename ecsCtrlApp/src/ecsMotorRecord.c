/* recEcsMotor.c - Record Support Routines for Motor records */
/*
 *		Original Author:	William Rambold
 *		Date: 97/10/07
 *
 *	Experimental Physics and Industrial Control System (EPICS)
 *
 *	Copyright 1995, the University of Chicago Board of Governors.
 *
 *	This software was produced under U.S. Government contract:
 *	(W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *	Developed by
 *		The Beamline Controls and Data Acquisition Group
 *		Experimental Facilities Division
 *		Advanced Photon Source
 *		Argonne National Laboratory
 *
 *	Co-developed with
 *		The Controls and Computing Group
 *		Accelerator Systems Division
 *		Advanced Photon Source
 *		Argonne National Laboratory
 *
 *
 * Modification Log:
 * -----------------
 * 0.0  97/10/07	wnr		initial development from motor record
 * 1.0  98/04/27        wnr             gamma release
 *
 */
#define VERSION 1.0

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <dbDefs.h>
#include <dbAccess.h>
#include <dbFldTypes.h>
#include <dbEvent.h>
#include <devSup.h>
#include <alarm.h>
#include <recSup.h>
#include <recGbl.h>

#include <menuCarstates.h>

#include "ecsMotorRecord.h"
#include "choiceEcsMotor.h"
#include "devEcsAbDf1.h"

/*** Debugging variables, macros ***/
/*
 *  Define a macro to print debugging information. If the current debugging
 *  level set by the DBUG field is
 *  greater than or equal to the debugging threshold given to the macro then 
 *  the given information message string is sent to the logging task.
 *
 */

static const char * const tsfmt = "%Y-%m-%d %H:%M:%S.%09f";

#define Debug(l,FMT,V) if (l <= pmr->dbug)                               \
                      {                                                   \
                            epicsTimeStamp ts;                            \
                            char timebuff[40];                            \
                            epicsTimeGetCurrent(&ts);                     \
                            epicsTimeToStrftime(timebuff, sizeof(timebuff), tsfmt, &ts);          \
                            printf  ("%s: "FMT,                           \
                                    epicsThreadGetNameSelf(),             \
                                    timebuff,                             \
                                    pmr->name,                            \
                                    V);                                   \
                      }

/*** Forward references ***/

//static void     alarm(struct ecsMotorRecord * pmr);
/* alarm() confilcts with system library -- need to change the name here */
static void     emrAlarm(struct ecsMotorRecord * pmr);
static void     monitor(struct ecsMotorRecord * pmr);
static void     post_MARKed_fields(struct ecsMotorRecord * pmr, unsigned short mask);
static long     initLinks(struct ecsMotorRecord * pmr);
static long     readInputLinks(struct ecsMotorRecord * pmr);

/*** Record Support Entry Table (RSET) functions. ***/

#define report         NULL
#define initialize     NULL
static long            init_record(struct ecsMotorRecord * pmr, int pass);
static long            process(struct ecsMotorRecord * pmr);
static long            special(struct dbAddr * paddr, int after);
static long            get_value(struct ecsMotorRecord * pmr, struct valueDes * pvdes);
#define cvt_dbaddr     NULL
#define get_array_info NULL
#define put_array_info NULL
static long            get_units(struct dbAddr * paddr, char *units);
#define get_enum_str   NULL
#define get_enum_strs  NULL
#define put_enum_str   NULL
static long            get_precision(struct dbAddr * paddr, long *precision);
static long            get_graphic_double(struct dbAddr * paddr, struct dbr_grDouble * pgd);
static long            get_control_double(struct dbAddr * p, struct dbr_ctrlDouble * pcd);
static long            get_alarm_double(struct dbAddr * paddr, struct dbr_alDouble * pad);

/* record support entry table */
struct rset     ecsMotorRSET = {
	RSETNUMBER,
	report,
	initialize,
	init_record,
	process,
	special,
	get_value,
	cvt_dbaddr,
	get_array_info,
	put_array_info,
	get_units,
	get_precision,
	get_enum_str,
	get_enum_strs,
	put_enum_str,
	get_graphic_double,
	get_control_double,
	get_alarm_double
};

/* device support entry table */
struct ecsMotorDset {
	long            number;
	DEVSUPFUN       dev_report;
	DEVSUPFUN       init;
	DEVSUPFUN       init_record;
	DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       processType;
	DEVSUPFUN       processRecord;
};





/******************************************************************************
	init_record()

Called twice after an EPICS database has been loaded, and then never called
again.
*******************************************************************************/
static long
init_record(struct ecsMotorRecord * pmr, int pass) {
	struct ecsMotorDset *pdset = (struct ecsMotorDset *) (pmr->dset);
	long status = 0;

	Debug(DBUG_MAX, "<%s> %s init_record pass = %d\n", pass);

	/* Do nothing on the first pass */
	if (pass == 0)
		return (0);

	/* Check that we have a device-support entry table. */
	if (!(pdset = (struct ecsMotorDset *) (pmr->dset))) {
		recGblRecordError(S_dev_noDSET, (void *) pmr,
				  "motor: init_record");
		return (S_dev_noDSET);
	}

	/* Check that DSET has pointers to functions we need. */
	if ((pdset->number < 6) ||
	    (pdset->processType == NULL) ||
	    (pdset->processRecord == NULL)) {
		recGblRecordError(S_dev_missingSup, (void *) pmr,
				  "motor: init_record");
		return (S_dev_missingSup);
	}

	/* Call device support to initialize itself and the driver */
	if (pdset->init_record) {
	  Debug(DBUG_MAX, "<%s> %s:init_record calling dset->init_record%c\n", ' ');
	  status = (*pdset->init_record) (pmr);
	  Debug(DBUG_MAX, "<%s> %s:init_record dset->init_record returns %ld\n", status);
	  if (status) return (status);
	}

	/* Initialize all database links */
	status = initLinks (pmr);
	if (status)
	  return (status);

	/* Initialize miscellaneous control fields. */
       	pmr->vers = VERSION;
       	pmr->mmap = 0;
       	pmr->udf = TRUE;

	pmr->dmov = FALSE;
	MARK(M_DMOV);
	pmr->movn = FALSE;
	MARK(M_MOVN);
	pmr->dsta = menuCarstatesIDLE;
	MARK(M_DSTA);
	pmr->mip = 0;
	MARK(M_MIP);
	pmr->msta = 0;
	MARK(M_MSTA);
	pmr->lvio = 0;
	MARK(M_LVIO);
	pmr->pp = TRUE;

	/* and update the marked fields */
	post_MARKed_fields(pmr, DBE_VALUE);

	return (0);
}


/******************************************************************************
	process()

Called for the following reasons:

1) Someone poked our .proc field, or some other field that is marked
'process-passive' in the motorRecord.ascii file.  In this case, we
read the input links and dive directly into device support.

2) Device support triggered an asynchronous callback.   This is done so that
the fields changed during device processing are posted as they change.   Input
links are not read during callback processing. 

3) A change in the fault status was detected.   This is treated like 
a callback in that it was not directly requested by the operator.


*******************************************************************************/
static long
process(struct ecsMotorRecord * pmr)
{
	struct ecsMotorDset *pdset = (struct ecsMotorDset *) (pmr->dset);
	long recordProcessType;
	long status, nRequest=1;

	if (pmr->pact)
		return (0);

	Debug(DBUG_MAX, "<%s> %s:process begin%c\n", ' ');
	pmr->pact = TRUE;

	/* 
	 * Ask the device support to determine wether or not this
	 * processing was a result of an operator request i.e. mode
	 * change.   If so it will be necessary to mark the record as
	 * busy and read all of the input links before processing.
	 */
	recordProcessType = (*pdset->processType) (pmr);

	if (recordProcessType == PROCESS_NORMAL) {
	  Debug(DBUG_MAX, "<%s> %s:process external process request%c\n", ' ');

	  /* Clear the last error and indicate that the record is busy */
	  pmr->mess[0] = '\0';
	  MARK(M_MESS);
	  pmr->dsta = menuCarstatesBUSY;
	  MARK(M_DSTA);
#if 0
	  status = recGblPutFastLink (&(pmr->dstl), (void *) pmr, &(pmr->dsta));
	  if (status) return (status);
#endif

	  /* Check input links for new operating conditions */
	  status = readInputLinks (pmr);
	  if (status) return (status);
	}

	/*
	 * No other epics interface stuff to do so let device
	 * support code take over from here.
	 */
	Debug(DBUG_FULL, "<%s> %s:invoke device processing%c\n", ' ');
	status = (*pdset->processRecord) (pmr, recordProcessType);
	if (status) {
	  pmr->dsta = menuCarstatesERROR;
	  pmr->pp = TRUE;
	}

	Debug(DBUG_FULL, "<%s> %s:device processing return %ld\n", status);

	/* if Post Process request perform epics completion tasks */
	if (pmr->pp) {
	  Debug(DBUG_MAX, "<%s> %s:process: post process entry%c\n", ' ');
	  pmr->pp = FALSE;

	  /* If an error was detected send the associated message, otherwise 
	   * we just go back into the idle state.
	   */
	  if (pmr->dsta == menuCarstatesERROR) {
#if 0
	     status = recGblPutLinkValue (&(pmr->msgl),(void *) pmr, DBR_STRING, pmr->mess, &nRequest);
	     if (status) return status;
#endif
	    /* Modified for GEM7. The status returned by dbPutLink should
	     * not be checked because, according to the gensub's code, it
	     * returns -1 if the record in not connected.
	     */
	    status = dbPutLink (&(pmr->msgl), DBR_STRING, pmr->mess, nRequest);

	  }
	  else if (pmr->dsta == menuCarstatesBUSY) {
	    pmr->dsta = menuCarstatesIDLE;
	    MARK(M_DSTA);
	  }

#if 0
	  status = recGblPutFastLink (&(pmr->dstl),(void *) pmr, &(pmr->dsta));
	  if (status) return status;
#endif
	  /* Modified for GEM7. The status returned by dbPutLink should
	   * not be checked because, according to the gensub's code, it
	   * returns -1 if the record in not connected
	   */
	  status = dbPutLink (&(pmr->dstl), DBR_LONG, &(pmr->dsta), nRequest);

	  /* fire off the forward link and record the time */
	  recGblFwdLink(pmr);	
	  recGblGetTimeStamp(pmr);
	}

	/* Update alarms and trigger motors before leaving */
	emrAlarm(pmr);
	monitor (pmr);

      	pmr->pact = FALSE;
	return (status);
}




/******************************************************************************
	special()
*******************************************************************************/
static long
special(struct dbAddr *paddr, int after) {

	struct ecsMotorRecord *pmr = (struct ecsMotorRecord *) paddr->precord;
	
	/* no special processing supported, if we are here, it's bad news !! */
	Debug(DBUG_MAX, "<%s> %s:special after = %d\n", after);
	if (after) recGblDbaddrError (S_db_badChoice, paddr, "ecsMotor: special");

	return (0);
}


/******************************************************************************
	get_value()
*******************************************************************************/
static long
get_value(struct ecsMotorRecord * pmr, struct valueDes * pvdes)
{
	Debug(DBUG_MAX, "<%s> %s:get_value: entry%c\n", ' ');
	pvdes->field_type = DBF_DOUBLE;
	pvdes->no_elements = 1;
	pvdes->pvalue = &pmr->val;
	return (0);
}

/******************************************************************************
	get_units()
*******************************************************************************/
static long
get_units(struct dbAddr * paddr, char *units)
{
	struct ecsMotorRecord *pmr = (struct ecsMotorRecord *) paddr->precord;

	Debug(DBUG_MAX, "<%s> %s:get_units entry%c\n", ' ');
	strncpy (units, pmr->egu, DB_UNITS_SIZE);
	units[DB_UNITS_SIZE-1] = '\0';

	return (0);
}

/******************************************************************************
	get_graphic_double()
*******************************************************************************/
static long
get_graphic_double(struct dbAddr * paddr, struct dbr_grDouble * pgd)
{
	struct ecsMotorRecord *pmr = (struct ecsMotorRecord *) paddr->precord;

	Debug(DBUG_MAX, "<%s> %s:get_graphic_double entry%c\n", ' ');

	if (paddr->pfield == (void *) &pmr->val ||
	    paddr->pfield == (void *) &pmr->mpos ) {
	  pgd->upper_disp_limit = pmr->hopr;
	  pgd->lower_disp_limit = pmr->lopr;
	}
	else recGblGetGraphicDouble(paddr, pgd);

	return (0);
}

/******************************************************************************
	get_control_double()
*******************************************************************************/
static long
get_control_double(struct dbAddr * paddr, struct dbr_ctrlDouble * pcd)
{
	struct ecsMotorRecord *pmr = (struct ecsMotorRecord *) paddr->precord;

	Debug(DBUG_MAX, "<%s> %s:get_control_double entry%c\n", ' ');

	if (paddr->pfield == (void *) &pmr->val ||
	    paddr->pfield == (void *) &pmr->mpos ) {
	  pcd->upper_ctrl_limit = pmr->hlm;
	  pcd->lower_ctrl_limit = pmr->llm;
	}
	else
	  recGblGetControlDouble(paddr, pcd);
	
	return (0);
}

/******************************************************************************
	get_precision()
*******************************************************************************/
static long
get_precision(struct dbAddr * paddr, long *precision)
{
	struct ecsMotorRecord *pmr = (struct ecsMotorRecord *) paddr->precord;

	Debug(DBUG_MAX, "<%s> %s:get_precision entry%c\n", ' ');

	if (paddr->pfield == (void *) &pmr->val ||
	    paddr->pfield == (void *) &pmr->mpos) 
	  *precision = pmr->prec;

	else
	  recGblGetPrec(paddr, precision);
 
	return (0);
}


/******************************************************************************
	get_alarm_double()
*******************************************************************************/
static long
get_alarm_double(struct dbAddr * paddr, struct dbr_alDouble * pad)
{
	struct ecsMotorRecord *pmr = (struct ecsMotorRecord *) paddr->precord;

	Debug(DBUG_MAX, "<%s> %s:get_alarm_double entry%c\n", ' ');

	if (paddr->pfield == (void *) &pmr->val ||
	    paddr->pfield == (void *) &pmr->mpos) { 
	  pad->upper_alarm_limit = pmr->hihi;
	  pad->upper_warning_limit = pmr->high;
	  pad->lower_warning_limit = pmr->low;
	  pad->lower_alarm_limit = pmr->lolo;
	} 
	else
	  recGblGetAlarmDouble(paddr, pad);

	return (0);
}


/******************************************************************************
	alarm()
*******************************************************************************/
static void
emrAlarm(struct ecsMotorRecord * pmr) {
  short rangeFlag;

	Debug(DBUG_MAX, "<%s> %s:alarm entry%c\n", ' ');
	if (pmr->udf == TRUE) {
	  recGblSetSevr(pmr, UDF_ALARM, INVALID_ALARM);
	  return;
	}

	if ((pmr->val > pmr->hihi || pmr->mpos > pmr->hihi) &&
	    recGblSetSevr(pmr, HIHI_ALARM, pmr->hhsv))
	  return;
	if ((pmr->val > pmr->high || pmr->mpos > pmr->high) &&
	    recGblSetSevr(pmr, HIGH_ALARM, pmr->hsv))
	  return;
	if ((pmr->val < pmr->low || pmr->mpos < pmr->low) &&
	    recGblSetSevr(pmr, LOW_ALARM, pmr->lsv))
	  return;
	if ((pmr->val < pmr->lolo || pmr->mpos < pmr->lolo) &&
	    recGblSetSevr(pmr, HIHI_ALARM, pmr->hhsv))
	  return;

	rangeFlag = (pmr->val > pmr->hlm || pmr->val < pmr->llm) ? TRUE : FALSE;
	if (rangeFlag ^ pmr->lvio) {
	  pmr->lvio = rangeFlag;
	  MARK(M_LVIO);
	}

	return;
}


/******************************************************************************
	monitor()
*******************************************************************************/
static void
monitor(struct ecsMotorRecord * pmr)
{
	unsigned short  monitorMask;
	
	Debug(DBUG_MAX, "<%s> %s:monitor: entry%c\n", ' ');
	monitorMask = recGblResetAlarms(pmr);

	/* Only post value field if it exceeds defined deadband */
	UNMARK(M_VAL);

#if 0
	if (abs(pmr->val - pmr->mlst) > pmr->mdel) {
#endif
	/* Changed abs() with fabs() for GEM7.
	 */
	if (fabs(pmr->val - pmr->mlst) > pmr->mdel) {
	  monitorMask |= DBE_VALUE;
	  pmr->mlst = pmr->val;
	}

	if (fabs(pmr->val - pmr->alst) > pmr->adel) {
	  monitorMask |= DBE_LOG;
	  pmr->alst = pmr->val;
	}

	/* send out monitors connected with value field */
	if (monitorMask) {
	  db_post_events(pmr, &pmr->val, monitorMask);
	}

	/* and now catch anything else that has changed */
	post_MARKed_fields (pmr, DBE_VALUE);
	return;
}


/******************************************************************************
	post_MARKed_fields()
*******************************************************************************/
static void
post_MARKed_fields(struct ecsMotorRecord * pmr, unsigned short mask)
{
	Debug(DBUG_MAX, "<%s> %s:post_MARKed_fields entry = %zx\n", pmr->mmap);

	if (MARKED(M_VAL)) {
		db_post_events(pmr, &pmr->val, mask);
		UNMARK(M_VAL);
	}
	if (MARKED(M_VELO)) {
		db_post_events(pmr, &pmr->velo, mask);
		UNMARK(M_VELO);
	}
	if (MARKED(M_MODE)) {
		db_post_events(pmr, &pmr->mode, mask);
		UNMARK(M_MODE);
	}
	if (MARKED(M_SIMM)) {
		db_post_events(pmr, &pmr->simm, mask);
		UNMARK(M_SIMM);
	}
	if (MARKED(M_RRBV)) {
		db_post_events(pmr, &pmr->rrbv, mask);
		UNMARK(M_RRBV);
	}
	if (MARKED(M_MPOS)) {
		db_post_events(pmr, &pmr->mpos, mask);
		UNMARK(M_MPOS);
	}
	if (MARKED(M_DBUG)) {
		db_post_events(pmr, &pmr->dbug, mask);
		UNMARK(M_DBUG);
	}
	if (MARKED(M_LLS)) {
		db_post_events(pmr, &pmr->lls, mask);
		UNMARK(M_LLS);
	}
	if (MARKED(M_HLS)) {
		db_post_events(pmr, &pmr->hls, mask);
		UNMARK(M_HLS);
	}
	if (MARKED(M_FLT)) {
		db_post_events(pmr, &pmr->flt, mask);
		UNMARK(M_FLT);
	}
	if (MARKED(M_HSTA)) {
		db_post_events(pmr, &pmr->hsta, mask);
		UNMARK(M_HSTA);
	}
	if (MARKED(M_DMOV)) {
		db_post_events(pmr, &pmr->dmov, mask);
		UNMARK(M_DMOV);
	}
	if (MARKED(M_MOVN)) {
		db_post_events(pmr, &pmr->movn, mask);
		UNMARK(M_MOVN);
	}
	if (MARKED(M_MIP)) {
		db_post_events(pmr, &pmr->mip, mask);
		UNMARK(M_MIP);
	}
	if (MARKED(M_HLM)) {
		db_post_events(pmr, &pmr->hlm, mask);
		UNMARK(M_HLM);
	}
	if (MARKED(M_LLM)) {
		db_post_events(pmr, &pmr->llm, mask);
		UNMARK(M_LLM);
	}
	if (MARKED(M_LVIO)) {
		db_post_events(pmr, &pmr->lvio, mask);
		UNMARK(M_LVIO);
	}
	if (MARKED(M_MESS)) {
		db_post_events(pmr, &pmr->mess, mask);
		UNMARK(M_MESS);
	}
	if (MARKED(M_MSTA)) {
		db_post_events(pmr, &pmr->msta, mask);
		UNMARK(M_MSTA);
	}
	if (MARKED(M_DSTA)) {
		db_post_events(pmr, &pmr->dsta, mask);
		UNMARK(M_DSTA);
	}
	if (MARKED(M_STOP)) {
		db_post_events(pmr, &pmr->stop, mask);
		UNMARK(M_STOP);
	}
	if (MARKED(M_MDBD)) {
		db_post_events(pmr, &pmr->mdbd, mask);
		UNMARK(M_MDBD);
	}

	UNMARK_ALL;

	return;
}

/******************************************************************************
	initLinks()
*******************************************************************************/
static long
initLinks (struct ecsMotorRecord * pmr) {
  long status = 0;

	Debug(DBUG_MAX, "<%s> %s:initLinks: entry%c\n", ' ');

#if 0
	/*
	 * .dol (Desired Output Location) is a struct containing either a
	 * link to some other field in this database, or a constant intended
	 * to initialize the .val field.  If the latter, get that initial
	 * value and apply it.
	 */
	if (pmr->dol.type == CONSTANT) {
	  pmr->udf = FALSE;
	  pmr->val = pmr->dol.value.value;
	}
	else {
	  status = recGblInitFastInLink (&(pmr->dol), (void *) pmr, DBR_DOUBLE, "VAL");
	  if (status) return (status);
	}
#endif
	/*
	 * .dol (Desired Output Location) is a struct containing either a
	 * link to some other field in this database, or a constant intended
	 * to initialize the .val field.  If the latter, get that initial
	 * value and apply it.
	 */
	/* Modified fo GEM7 (from the stepper motor record code). The call
	 * to recGblInitFastInLink() was removed because is no longer needed.
	 */
	if (pmr->dol.type == CONSTANT) {
	    if (recGblInitConstantLink(&pmr->dol,DBF_FLOAT,&pmr->val))
		pmr->udf = FALSE;
	}

	/* Initialize the input links used to recover operating modes */
	if (pmr->dbgl.type != CONSTANT) {
#if 0
	  /* Removed for GEM7 since they are no longer needed. The status
	   * returned by dbPutLink should not be checked because, according
	   * to the gensub's code, it returns -1 if the record in not
	   * connected
	   */
	  status = recGblInitFastInLink (&(pmr->dbgl),(void *) pmr, DBR_ENUM, "DBUG");
	  if (status) return (status);
#endif
	}

	if (pmr->siml.type != CONSTANT) {
#if 0
	  /* Removed for GEM7 since they are no longer needed. The status
	   * returned by dbPutLink should not be checked because, according
	   * to the gensub's code, it returns -1 if the record in not
	   * connected
	   */
	  status = recGblInitFastInLink (&(pmr->siml),(void *) pmr, DBR_ENUM, "SIMM");
	  if (status) return (status);
#endif
	}

	/* Initialize the status output links */
	if (pmr->dstl.type != CONSTANT) {
#if 0
	  /* Removed for GEM7 since they are no longer needed. The status
	   * returned by dbPutLink should not be checked because, according
	   * to the gensub's code, it returns -1 if the record in not
	   * connected
	   */
	  status = recGblInitFastOutLink (&(pmr->dstl),(void *) pmr, DBR_LONG, "DSTA");
	  if (status) return (status);
#endif
	}

#if 0
	  /* Removed for GEM7 since they are no longer needed. The status
	   * returned by dbPutLink should not be checked because, according
	   * to the gensub's code, it returns -1 if the record in not
	   * connected
	   */
	if (pmr->msgl.type == PV_LINK) {
	  status = dbCaAddOutlink (&(pmr->msgl),(void *) pmr, "MESS");
	  if (status) return (status);
	}
#endif

	/* and we are done */
	return status;
}


/******************************************************************************
	readInputLinks()
*******************************************************************************/
static long
readInputLinks (struct ecsMotorRecord * pmr) {
  double dval;
  long status = 0;
  unsigned short sval;

	Debug(DBUG_MAX, "<%s> %s:readInputLinks: entry%c\n", ' ');

	/* Check all input links for new operating conditions */

         if (pmr->dol.type != CONSTANT) {
#if 0
            status = recGblGetFastLink (&(pmr->dol),(void *) pmr, &dval);
	    if (status) return (status);
#endif
	    /* Modified for GEM7.
	     */
	    status = dbGetLink(&(pmr->dol), DBR_DOUBLE, &dval, 0, 0);
	    if (status) return (status);

	    if (pmr->val != dval) {
	         Debug(DBUG_FULL, "<%s> %s:process new position = %f\n", dval);
	         pmr->val = dval;
	         MARK(M_VAL);
	    }
	 }

         if (pmr->siml.type != CONSTANT) {
#if 0
	    status = recGblGetFastLink (&(pmr->siml), (void *) pmr,&sval);
 	    if (status) return (status);
#endif
	    /* Modified for GEM7.
	     */
	    status = dbGetLink(&(pmr->siml), DBR_USHORT, &sval, 0, 0);
 	    if (status) return (status);

	     if (pmr->simm != sval) {
	         Debug(DBUG_FULL, "<%s> %s:process new simulation level = %d\n", sval);
	         pmr->simm = sval;
	         MARK(M_SIMM);
	     }
	 }

         if (pmr->dbgl.type != CONSTANT) {
#if 0
	      status = recGblGetFastLink ( &(pmr->dbgl), (void *) pmr, &sval);
	      if (status) return (status);
#endif

	    /* Modified for GEM7.
	     */
	    status = dbGetLink(&(pmr->dbgl), DBR_USHORT, &sval, 0, 0);
 	    if (status) return (status);

	      if (pmr->dbug != sval) {
	         Debug(DBUG_FULL, "<%s> %s:process new debug level = %d\n", sval);
	         pmr->dbug = sval;
	         MARK(M_DBUG);
	      }
	 }

	return status;
}
