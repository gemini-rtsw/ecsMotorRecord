
/*
 *      Original Author:   William Rambold
 *      Date: 97/10/07
 *
 *   Experimental Physics and Industrial Control System (EPICS)
 *
 *   Copyright 1995, the University of Chicago Board of Governors.
 *
 *   This software was produced under U.S. Government contract:
 *   (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *   Developed by
 *      The Beamline Controls and Data Acquisition Group
 *      Experimental Facilities Division
 *      Advanced Photon Source
 *      Argonne National Laboratory
 *
 *   Co-developed with
 *      The Controls and Computing Group
 *      Accelerator Systems Division
 *      Advanced Photon Source
 *      Argonne National Laboratory
 *
 *
 * Modification Log:
 * -----------------
 * 0.0  97/10/07   wnr      initial development from motor record
 * 1.0  98/04/27        wnr             gamma release
 *
 */
#define VERSION 2.0

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <dbDefs.h>
#include <dbAccess.h>
#include <dbFldTypes.h>
#include <dbEvent.h>
#include <recSup.h>
#include <devSup.h>
#include <devLib.h>
#include <alarm.h>
#include <recGbl.h>
#include <ellLib.h>
#include <epicsMutex.h>
#include <epicsExport.h>
#include <iocsh.h>

#include <postfix.h>

#include <menuCarstates.h>

#include "ecsMotorRecordChoice.h"
#include "ecsMotorRecordFields.h"
#include "ecsMotorRecordDefs.h"

#define GEN_SIZE_OFFSET
#include <ecsMotorRecord.h>
#undef GEN_SIZE_OFFSET

/*** Debugging variables, macros ***/
/*
 *  Define a macro to print debugging information. If the current debugging
 *  level set by the DBUG field is
 *  greater than or equal to the debugging threshold given to the macro then 
 *  the given information message string is sent to the logging task.
 *
 */


#define MESSAGE_BUFFER_SIZE 40


static ELLLIST         ecsMotorRecordScanList;
static epicsThreadId   ecsMotorRecordTaskId;

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

/* handshake bit masks */
#define INPUT_MASK(vms) ((vms)?\
			(PWR_BIT|PWR_ACK_BIT|DRV_ACK_BIT|POS_ACK_BIT|IN_POS_BIT|VEL_ACK_BIT):\
			(PWR_BIT|PWR_ACK_BIT|DRV_ACK_BIT|POS_ACK_BIT|IN_POS_BIT))
#define OUTPUT_MASK(vms) ((vms)?\
			 (NEW_POS_BIT|NEW_VEL_BIT|DRV_BIT):\
			 (NEW_POS_BIT|DRV_BIT))

/*** Forward references ***/

//static void     alarm(struct ecsMotorRecord * pmr);
/* alarm() confilcts with system library -- need to change the name here */
static void     emrAlarm(struct ecsMotorRecord * pmr);
static void     monitor(struct ecsMotorRecord * pmr);
static void     post_MARKed_fields(struct ecsMotorRecord * pmr, unsigned short mask);
static long     initLinks(struct ecsMotorRecord * pmr);
static long     readInputLinks(struct ecsMotorRecord * pmr);
static long	auxiliary_process(struct ecsMotorRecord *pmr, long processType);

/*** Record Support Entry Table (RSET) functions. ***/

#define report         NULL
#define initialize     NULL
static long            init_record(struct ecsMotorRecord * pmr, int pass);
static long            process(struct ecsMotorRecord * pmr);
static long            special(struct dbAddr * paddr, int after);
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

static int var01 = 5;
static int var02 = 5;
static int var03 = 5;

/* record support entry table */
rset     ecsMotorRSET = {
   RSETNUMBER,
   report,
   initialize,
   init_record,
   process,
   special,
   NULL,
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
epicsExportAddress(rset, ecsMotorRSET);

// No device support!
#if 0
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
#endif

/*
* Internal control structure for ecsMotorRecord support. 
* This holds pointers to the current state of the system control variables.
*/
typedef struct {
    ELLNODE      node;                  /* processing scan node defintion */
    struct       ecsMotorRecord *pmr;   /* pointer to ecsMotor record */

    /* we don't need these any more */
    // absWordIO    *pReadHskPriv;         /* callback for handshake read */
    // absWordIO    *pWriteHskPriv;        /* callback for handshake write */
    // absWordIO    *pEncoderPriv;         /* callback for position read */
    // absWordIO    *pWritePosPriv;        /* callback for position write */
    // absWordIO    *pWriteVelPriv;        /* callback for velocity write */

    epicsMutexId mutexSem;              /* mutual exclusion semaphore */
    short        operatingMode;         /* current operating mode */
    long         lastMode;              /* saved command */
    double       lastVal;               /* saved value */
    long         currentMode;           /* current command */
    double       currentTarget;         /* current value */
    double       currentVelocity;       /* current velocity */
    long         deferredMode;          /* deferred command */
    double       deferredTarget;        /* deferred value */
    long         simMode;               /* simulation mode */
    double       simTarget;             /* simulation mode position target */
    double       simVelocity;           /* simulation mode velocity */
    double       simPosition;           /* simulation mode device position */
    //unsigned     handshake;           /* latest handshake word */
    unsigned     handshake_in;          /* latest handshake input word */
    unsigned     handshake_out;         /* latest handshake output word */
    unsigned     encoder;               /* latest encoder position */
    unsigned     fault;                 /* latest fault condition */
    unsigned     pattern;               /* latest output bit pattern */
    short        inPosition;            /* latest in-position state */
    short        callbackFlags;         /* callback activity */
    short        timeoutActive;         /* timeout flag */
    long         timeoutTimer;          /* timeout tick counter */
    } ecsMotorRecordPriv;

/* state machine forward declarations */
static long idleState (struct ecsMotorRecord *);
static long startingState (struct ecsMotorRecord *);
static long movingState (struct ecsMotorRecord *);
static long stoppingState (struct ecsMotorRecord *);
/* not used any more (pgx)
static long poweringState (struct ecsMotorRecord *);
static long depoweringState (struct ecsMotorRecord *);
*/
static long writingState (struct ecsMotorRecord *);
static long verifyingState (struct ecsMotorRecord *);
static long checkFaultState (struct ecsMotorRecord *);
static long failingState (struct ecsMotorRecord *);

static void setHandshakeBits (struct ecsMotorRecord *pmr, unsigned int pattern);
static void resetHandshakeBits (struct ecsMotorRecord *pmr, unsigned int pattern);
static void updateInputBits (struct ecsMotorRecord *pmr);
static double positionFromEncoder(struct ecsMotorRecord *pmr);

inline char *timeStamp() {
    epicsTimeStamp ts;
    static char timebuff[40];
    epicsTimeGetCurrent(&ts);
    epicsTimeToStrftime(timebuff, sizeof(timebuff), tsfmt, &ts);
    return timebuff;
}

/* Function positionFromEncoder
 *
 * Calculate the device position limiting the raw encoder value to less than 60000
 * counts (negative values in the PLC side) to prevent problems when the encoders
 * go below zero. This is needed to avoid problems with the high level systems
 * (e.g. TCS). The scaling factor is applied at this point as well.
 */
static double positionFromEncoder(struct ecsMotorRecord *pmr)
{
    long limval;

    limval = (pmr->rrbv > 60000) ? 0 : pmr->rrbv;
    return (double) limval / pmr->psca;
}

/*
* Function ecsMotorRecordScanTask
*
* This function is spawned as a task to control
* the internal processing of ecsMotor records.   The
* device private structures are used directly to access
* the associated records.
*/
/* We can probably get rid of this, and just let each ecsMotorRecord scan
 * independently on a standard EPICS scan record scan list (mdw)
 */
static void ecsMotorRecordScanTask(void *p) {
   ecsMotorRecordPriv *pPriv;
   struct ecsMotorRecord *pmr = NULL;
   struct rset *pRset;
   int inPosition;

   /* if we delete this scan task, most of the below will have
      to moved to the process() routine. (mdw) */
   while (TRUE) {

      /* For each record in the list .. */
      pPriv = (ecsMotorRecordPriv *) ellFirst(&ecsMotorRecordScanList);
      while (pPriv) {
         pmr = (struct ecsMotorRecord *) pPriv->pmr;
         pRset = (struct rset *) pmr->rset;

         Debug(DBUG_MAX, "<%s> %s:\n------------ scanTask:entry ------------%c\n", ' ');

         /* adjust the timeout timer */
         if (pPriv->timeoutActive) {
            pPriv->timeoutTimer -= 1;
	    //printf ("- %ld\n", pPriv->timeoutTimer);
            if (pPriv->timeoutTimer <= 0) {
               pPriv->callbackFlags |= TIMEOUT_OVER;
               pPriv->timeoutActive = FALSE;
            }
         }

         /* handle simulated motion here */
         if (pPriv->simMode == SIMM_FULL) {
            if (pmr->mode == MODE_VMOVE) {
               pPriv->simPosition += pPriv->simVelocity;
               pPriv->callbackFlags |= SIMM_UPDATE;
            }
            else if (pPriv->simPosition != pPriv->simTarget) {
               pPriv->simPosition += (pPriv->simPosition < pPriv->simTarget) ?
				      pPriv->simVelocity : -pPriv->simVelocity;

               /* within one velocity step is close enough */
               if (abs(pPriv->simTarget - pPriv->simPosition) < pPriv->simVelocity)
                  pPriv->simPosition = pPriv->simTarget;
 
               /* signal that position has changed */
               pPriv->callbackFlags |= SIMM_UPDATE;
            }
         }

	/* Handle changes in the input handshake.
	 */
	if (pmr->hinp != pPriv->handshake_in) {
	    Debug(DBUG_FULL, "<%s> %s:detected hinp change from 0x%x\n", pPriv->handshake_in);
	    Debug(DBUG_FULL, "<%s> %s:detected hinp change to 0x%x\n", pmr->hinp);
	    pPriv->handshake_in = pmr->hinp;
	    inPosition = ((pmr->hinp & IN_POS_BIT) != 0);
	    if (inPosition != pPriv->inPosition) {
		pPriv->inPosition = inPosition;
	    }
	    updateInputBits(pmr);
	    pPriv->callbackFlags |= DATA_READ;
	    MARK(M_HINP);
	}

	/* Handle changes in the output handshake.
	 */
       if (pmr->hsta != pPriv->handshake_out) {
	    //printf ("%s: output handshake change %x -> %x\n", timeStamp(), pPriv->handshake_out, pmr->hsta);
	    Debug(DBUG_FULL, "<%s> %s:detected hsta change to 0x%x\n", pmr->hsta);
	    pPriv->handshake_out = pmr->hsta;
	    pPriv->callbackFlags |= DATA_WRITE;
	    MARK(M_HSTA);
       }

	/* Handle changes in the encoder.
	 */
	if (pmr->rrbv != pPriv->encoder) {
	    Debug(DBUG_FULL, "<%s> %s:detected encoder change to %d\n", pmr->rrbv);
	    pPriv->encoder = pmr->rrbv;
	    pPriv->callbackFlags |= DATA_READ;
            MARK(M_RRBV);
	}

	/* Handle changes in the position demand feedback.
	 */
	if (pmr->msta == MOTOR_WRITING && (pmr->pdfb == pmr->rpos)) {
	    pPriv->callbackFlags |= DATA_WRITE;
	    MARK(M_PDFB);
	}

         /* if an asynchronous callback has occurred since the
          * last pass force the record to process.   If the record is
          * busy ... no mattah' ... defer 'till next pass.
          */
         if (pPriv->callbackFlags) {

	    Debug(DBUG_FULL, "<%s> %s:detected callback flags change 0x%x\n", pPriv->callbackFlags);

            /* defer processing if the record is busy */
            if (!pmr->pact) {

               /* recover asynchronous info */
               epicsMutexLock(pPriv->mutexSem);
               if (pPriv->callbackFlags & DATA_WRITE) MARK(M_WRITE);
               if (pPriv->callbackFlags & TIMEOUT_OVER) MARK(M_TIMEOUT);
               if (pPriv->callbackFlags & DATA_ERROR) MARK(M_ERROR);

               if (!pmr->simm && pmr->dmov != pPriv->inPosition) {
                  pmr->dmov = pPriv->inPosition;
                  MARK(M_DMOV);
               }

               /* clear flags and indicate that this is an internal processing */
               pPriv->callbackFlags = 0;
               MARK(M_RESCAN);
               epicsMutexUnlock(pPriv->mutexSem);

               /* and then process the record */
               Debug(DBUG_MAX, "<%s> %s:Scan Task re-processing record%c\n",' ');
               dbScanLock ((struct dbCommon *) pmr);
               (*pRset->process) (pmr);
               dbScanUnlock ((struct dbCommon *) pmr);
            }
               else {
                   Debug(DBUG_FULL, "<%s> %s:Scan Task .. record busy%c\n",' ');
               }
         }
         pPriv = (ecsMotorRecordPriv  *) ellNext (&pPriv->node);
	 Debug(DBUG_MAX, "<%s> %s:Scan Task .. end of record loop%c\n", ' ');
      }
      epicsThreadSleep(1.0/ECS_SCAN_RATE);
      // epicsThreadSleep(1.0);
   }
}

/*
 * Function setTimeout
 * 
 * Start or adjust the process timeout counter.
 *
 */
static long
setTimeout (struct ecsMotorRecord *pmr, long time) {
    ecsMotorRecordPriv *pPriv = pmr->dpvt;
    long status = S_ECS_OK;

/* shouldn't this be protected by the private data mutex? (mdw) */
    if (time != TIMEOUT_OFF) {
	//printf ("setTimeout, time=%ld\n", time);
        Debug(DBUG_FULL, "<%s> %s:set timeout to %ld ticks \n", time);
        pPriv->timeoutTimer = time;
        pPriv->timeoutActive = TRUE;
    }
    else {
        Debug(DBUG_FULL, "<%s> %s:cleared timeout%c\n", ' ');
        pPriv->timeoutActive = FALSE;
        }

     status = S_ECS_OK;

    return status;
}

/*
 * Function recordError 
 * 
 * Write an error message to the record message field (pmr->mess)
 *
 */
static long recordError (struct ecsMotorRecord *pmr, char *msg, long status) {

  Debug(DBUG_FULL, "<%s> %s:recordError ..%s.. \n", msg);

  /* clear any timeouts */
  setTimeout (pmr, TIMEOUT_OFF);

/* protect with private data mutex? (mdw) */
  /* save the error in the record message field*/
  if (!strlen(pmr->mess)) {
    strncpy (pmr->mess, msg, MESSAGE_BUFFER_SIZE);
    MARK(M_MESS);
  }

  /* mark the CAR state as error if it is not there already*/
  if (pmr->dsta == menuCarstatesBUSY) {
    pmr->dsta = menuCarstatesERROR;
    MARK(M_DSTA);
  }

  /* if this is a "hard" error, switch to the failing state */
  if (status != S_ECS_OK && status != S_ECS_VALUE_REJECT) {
    if (pmr->msta != MOTOR_FAILING && pmr->msta != MOTOR_FAULT)
      failingState (pmr);
  }

  return status;
}

/*
 * Function ecsMotorRecordPocessType
 * 
 * Determine what caused the record to be processed
 * (This had been part of the ABDF1 device support)
 */
static long ecsMotorRecordProcessType(struct ecsMotorRecord *pmr) {
   ecsMotorRecordPriv *pPriv = pmr->dpvt;

   Debug(DBUG_FULL, "<%s> %s:ecsMotorRecordProcessType:entry%c\n", ' ');

/* protect with private data mutex? (mdw) */
   /* Internal scan requests are detected here. */
   if (MARKED(M_RESCAN)) {
       Debug(DBUG_MAX, "<%s> %s:ecsMotorRecordProcessType:internal%c\n", ' ');
       return PROCESS_INTERNAL;
   }

   /* "button" fields such as stop and fault are detected here */
   /* PGX: should be add a check for pmr->hinp here? */
   if (pmr->stop || pmr->flt != pPriv->fault) {
       Debug(DBUG_MAX, "<%s> %s:ecsMotorRecordProcessType:button%c\n", ' ');
       return PROCESS_BUTTON;
   }

    /* Otherwise it is a normal external procesing request */
    Debug(DBUG_MAX, "<%s> %s:ecsMotorRecordProcessType:normal%c\n", ' ');
    return PROCESS_NORMAL;
}


/*
 * Function processModeChange
 * 
 * Called whenever the operator requests an operating
 * mode change.
 */

static long processModeChange (struct ecsMotorRecord *pmr) {
   ecsMotorRecordPriv *pPriv = pmr->dpvt;
   long status = S_ECS_OK;

   Debug(DBUG_FULL, "<%s> %s:processModeChange:entry%c\n", ' ');

   if (pmr->mode == pPriv->currentMode) return S_ECS_OK;

   /* process depending on requested mode */
   switch (pmr->mode) {
      case MODE_STOP:
	  Debug(DBUG_MAX, "<%s> %s:processModeChange:stop%c\n", ' ');
          status = stoppingState (pmr);
          break;

      case MODE_VMOVE:
	  Debug(DBUG_MAX, "<%s> %s:processModeChange:vmove%c\n", ' ');
         if (!(pmr->vms)) {
            status = recordError (pmr, "Velocity mode not supported", S_ECS_OK);
            return (status);
         }
      case MODE_MOVE:
      case MODE_PARK:
	  Debug(DBUG_MAX, "<%s> %s:processModeChange:move and park%c\n", ' ');
         status = writingState (pmr);
         break;

      case MODE_PAUSE:
	  Debug(DBUG_MAX, "<%s> %s:processModeChange:pause%c\n", ' ');
         if (pPriv->currentMode == MODE_MOVE) {
            status = stoppingState (pmr);
         }
         else {
            recordError (pmr, "Can not pause this move", S_ECS_OK);
         }
         break;

         case MODE_RESUME:
	    Debug(DBUG_MAX, "<%s> %s:processModeChange:resume%c\n", ' ');
            if (pPriv->currentMode == MODE_PAUSE) {
            status = startingState (pmr);
            pmr->mode = MODE_MOVE;
         }
         else {
            recordError (pmr, "Not in PAUSE mode", S_ECS_OK);
         }
         break;

      default:
         recordError (pmr, "Unsupported mode", S_ECS_OK);
         break;
    }
    pPriv->currentMode = pmr->mode;
    return status;
}

/*
 * Function setHandshakeBits
 * 
 * Set one or more bits in the output handshake.
 * This routine is intended to be called from the state machine functions.
 *
 */
static void setHandshakeBits (struct ecsMotorRecord *pmr, unsigned int pattern)
{
    long input_mask, output_mask;

    input_mask = INPUT_MASK(pmr->vms);
    output_mask = OUTPUT_MASK(pmr->vms);

    printf ("\n %s setHandshakeBits1: im=0x%lx om=0x%lx\n", pmr->name, input_mask, output_mask);
    printf ("setHandshakeBits2: hi=0x%x, ho=0x%x, p=0x%x\n", pmr->hinp, pmr->hsta, pattern);

    pmr->hsta = (pmr->hinp & input_mask) | ((pmr->hsta | pattern) & output_mask);
    pmr->pp = TRUE;
    MARK(M_HSTA);

    printf ("setHandshakeBits3: hi=0x%x, ho=0x%x\n", pmr->hinp, pmr->hsta);
}

/*
 * Function resetHandshakeBits
 * 
 * Reset (clear) one or more bits in the output handshake.
 * This routine is intended to be called from the state machine functions.
 *
 */
static void resetHandshakeBits (struct ecsMotorRecord *pmr, unsigned int pattern)
{
    long input_mask, output_mask;

    input_mask = INPUT_MASK(pmr->vms);
    output_mask = OUTPUT_MASK(pmr->vms);

    printf ("\n %s resetHandshakeBits1: im=0x%lx om=0x%lx\n", pmr->name, input_mask, output_mask);
    printf ("resetHandshakeBits2: hi=0x%x, ho=0x%x, p=0x%x\n", pmr->hinp, pmr->hsta, pattern);
    
    pmr->hsta = (pmr->hinp & input_mask) | ((pmr->hsta & ~pattern) & output_mask);
    pmr->pp = TRUE;
    MARK(M_HSTA);

    printf ("resetHandshakeBits3: hi=0x%x, ho=0x%x\n", pmr->hinp, pmr->hsta);
}

/*
 * Function updateInputBits
 * 
 * Update changes in the handshake input bits into the handshake output.
 * This routine is intended to be called from the scan task when a change in one
 * of the input handshake bits (HINP) is detected and needs to be propagated to
 * the output handshake (HSTA) to keep both words syncronized.
 *
 */
static void updateInputBits (struct ecsMotorRecord *pmr)
{
    long input_mask, output_mask;

    input_mask = INPUT_MASK(pmr->vms);
    output_mask = OUTPUT_MASK(pmr->vms);

    printf ("\n %s updateHandShakeBits1: im=0x%lx om=0x%lx\n", pmr->name, input_mask, output_mask);
    printf ("updateHandShakeBits2: hi=0x%x, ho=0x%x\n", pmr->hinp, pmr->hsta);

    pmr->hsta = (pmr->hinp & input_mask) | (pmr->hsta & output_mask);
    pmr->pp = TRUE;
    MARK(M_HSTA);

    printf ("updateHandShakeBits3: hi=0x%x, ho=0x%x\n", pmr->hinp, pmr->hsta);
}

/*
 * Function checkFaultState
 * 
 * Look at the external FAULT line to see if it is safe to proceed.
 *
 */
static long checkFaultState (struct ecsMotorRecord *pmr) {
    ecsMotorRecordPriv *pPriv = pmr->dpvt;
    long switchMask, status = S_ECS_OK;

    Debug(DBUG_FULL, "<%s> %s:checkFaultState:entry%c\n", ' ');

    /* Interlocks are most important */
    if (pmr->flt & INTERLOCK) {
        status = recordError (pmr, "Hardware Interlock Detected", S_ECS_FAULT);
    }
    /* PLC must be online */
    if (pmr->flt & PLC_OFFLINE) {
        status = recordError (pmr, "Carousel PLC off or in local mode", S_ECS_FAULT);
    }
    /* ...and alive */
    if (pmr->flt & SYSTEM_DEAD) {
        status = recordError (pmr, "Carousel PLC heartbeat stopped", S_ECS_FAULT);
    }

#if 0  /* we need to figure out how to get IO status from external links for this to wiork (mdw) */
    /* check for data error during AB communications */
    if (MARKED(M_ERROR)) {
        status = recordError (pmr, "AB communications error", S_ECS_FAULT);
    }
    if (MARKED(M_ERROR)) {
        commErrCount++;         /* ignore and count errors */
    }
#endif

    /* look for servo faults */
    if (pmr->flt & DEVICE_FAILURE) {
        status = recordError (pmr, "Actuator fault detected", S_ECS_FAULT);
    }

    /* look for encoder faults */
    if (pmr->flt & ENCODER_FAULT) {
        status = recordError (pmr, "Encoder fault detected", S_ECS_FAULT);
    }

    /* look for skew faults */
    if (pmr->flt & SKEWING) {
        status = recordError (pmr,
                        "Bottom shutter skew fault detected", S_ECS_FAULT);
    }

    /* look for upper limit switch activation */
     if (pmr->flt & OPEN_OVER_LIMIT) {
         switchMask = ECS_OVER_LSW;
         status = recordError (pmr, "Open direction hard limit encountered", S_ECS_FAULT);
     }
     else if (pmr->flt & OPEN_EOT_LIMIT) switchMask = ECS_END_LSW;
     else if (pmr->flt & OPEN_SLOW_LIMIT) switchMask = ECS_SLOW_LSW;
     else switchMask = ECS_NO_LSW;

     if (pmr->hls != switchMask) {
         pmr->hls = switchMask;
         MARK(M_HLS);
    }

     /* look for lower limit switch activation */
    if (pmr->flt & CLOSE_OVER_LIMIT) {
         switchMask = ECS_OVER_LSW;
         status = recordError (pmr, "Close direction hard limit encountered", S_ECS_FAULT);
     }
     else if (pmr->flt & CLOSE_EOT_LIMIT) switchMask = ECS_END_LSW;
     else if (pmr->flt & CLOSE_SLOW_LIMIT) switchMask = ECS_SLOW_LSW;
     else switchMask = ECS_NO_LSW;

     if (pmr->lls != switchMask) {
         pmr->lls = switchMask;
         MARK(M_LLS);
    }

    pPriv->fault = pmr->flt;
     return status;
}

/*
 * Function idleState
 * 
 * A new position or velocity word has been written to the device and
 * has been accepted.   If the device is not already moving start it
 * now.
 */
static long idleState (struct ecsMotorRecord *pmr) {
   long status = S_ECS_OK;

   Debug(DBUG_FULL, "<%s> %s:idleState%c\n", ' ');

   /* Nothing special on entry */
   if (pmr->msta != MOTOR_IDLE) {
      pmr->msta = MOTOR_IDLE;
      MARK(M_MSTA);
   }

   /* keep an eye on the drive power */
   /* REL-2623: This is no longer relevant when in IDLE, so we just comment it out
                 (RCardenes)
   if (!(pmr->hsta & PWR_ACK_BIT)) {
      status = recordError (pmr, "Unexpected drive powerdown", S_ECS_HSK_SYNC);
      return (status);
   }
   */

   /* update status words to indicate device idle */
   pmr->mip = MIP_STOPPED;
   MARK(M_MIP);
   pmr->movn = FALSE;
   MARK(M_MOVN);

   pmr->pp = TRUE;

   return (status);
}

#if 0
/*
 * Function poweringState
 * 
 * This routine is not used any more. The code was left here for reference (pgx)
 *
 * A power up request has been received.  If the motor
 * power is off then turn it back on and wait for the power
 * on acknowledge to be detected.  The acknowledge
 * must be recieved within the specified time or a timeout
 * error is generated.   Note that this function is called
 * many times during this process.
 */
static long poweringState (struct ecsMotorRecord *pmr) {
   ecsMotorRecordPriv *pPriv = pmr->dpvt;
   long status = S_ECS_OK;

   Debug(DBUG_FULL, "<%s> %s:poweringState%c\n", ' ');

   /* Upon entry clean up handshake bits and make sure that the power is on */
   if (pmr->msta != MOTOR_POWERING) {
      pmr->msta = MOTOR_POWERING;
      MARK(M_MSTA);

#if 0
      if (pmr->hsta != (PWR_BIT | PWR_ACK_BIT)) {
         writeHandshake (pmr, PWR_BIT);
         setTimeout (pmr, ECS_PWR_TMO);
      }
#endif
      if (pmr->hinp != (PWR_BIT | PWR_ACK_BIT)) {
	 setHandshakeBits (pmr, PWR_BIT);
         setTimeout (pmr, ECS_PWR_TMO);
      }
   }

   /* If the timeout has expired regester a fault */
   if (MARKED(M_TIMEOUT)) {
      status = recordError (pmr, "Device did not power up in time", S_ECS_TIMEOUT);
      return status;
   }

   /* Wait for a power on acknowledge */
#if 0
   if (pmr->hsta != (PWR_BIT | PWR_ACK_BIT)) {
      return (S_ECS_OK);
   }
#endif
   if (pmr->hinp != (PWR_BIT | PWR_ACK_BIT)) {
      return (S_ECS_OK);
   }

   /* Got it, make target equal current positon and move to the IDLE state */         
   setTimeout (pmr, TIMEOUT_OFF);
   pPriv->currentTarget = pmr->val = pmr->mpos;
   status = idleState (pmr);
   return (status);
}
#endif

/*
 * Function writingState
 * 
 * The operator has requested a change to either the target position
 * or velocity.   Write the new value and once the write is acknowledged
 * set the position or velocity valid bit.   This function is called
 * more than once in this process.
 */
static long
writingState (struct ecsMotorRecord *pmr) {
    //ecsMotorRecordPriv *pPriv = pmr->dpvt;
    unsigned impliedDecimal = 0;
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%s> %s:writingState%c\n", ' ');
    //printf ("writingState, hinp=%x, val=%f, rpos=%d, pdfb=%d\n", pmr->hinp, pmr->val, pmr->rpos, pmr->pdfb);

    /* 
     * If we are stationary and in position then do not
     * act on any targets within the pre-defined deadband area.
     */
    if (pmr->dmov && !pmr->movn &&
        pmr->mode == MODE_MOVE &&
        (fabs(pmr->mpos - pmr->val) < pmr->mdbd)) {
        pmr->pp = TRUE;
        return S_ECS_OK;
    }

    /* Handle parking when the device is already there or it cannot
     * move because it already reached the limits. It is not possible
     * to compare for equality of the demand and the current position
     * since there is normally an error in positioning. The device
     * tolerance is used instead (PG 25/Aug/99).
     */
    if (!pmr->movn && pmr->mode == MODE_PARK) {
        if (((pmr->lls > ECS_SLOW_LSW) && (pmr->val < pmr->mpos)) ||
            ((pmr->hls > ECS_SLOW_LSW) && (pmr->val > pmr->mpos)) ||
            (fabs (pmr->mpos - pmr->val) < pmr->mdbd))
          return (idleState (pmr));
        /*return (depoweringState (pmr)); AWE: control power externally */
    }

    /* trap requested motion into limit switches */
    if (((pmr->lls > ECS_SLOW_LSW) && (pmr->val < pmr->mpos)) ||
        ((pmr->hls > ECS_SLOW_LSW) && (pmr->val > pmr->mpos))) {
      status = recordError (pmr, "Attempt to move into a limit switch", S_ECS_OK);
      return S_ECS_VALUE_REJECT;
    }

    /* Upon entry scale to integer and write the new value */
    if (pmr->msta != MOTOR_WRITING) {
         pmr->msta = MOTOR_WRITING;
            MARK(M_MSTA);

        if (pmr->mode == MODE_VMOVE) {
	    /* velocity demand not supported */
            Debug(DBUG_MIN, "<%s> %s:velocity word (disabled) write %d\n", impliedDecimal);
            impliedDecimal = (unsigned) (pmr->velo * pmr->vsca);
        }
        else {
            /* this is now our raw position demand */
            Debug(DBUG_MIN, "<%s> %s:position word write %d\n", impliedDecimal);
            impliedDecimal = (unsigned) (pmr->val * pmr->psca);
            pmr->rpos = impliedDecimal;
            pmr->pp = TRUE;
	    MARK(M_RPOS);
        }
        setTimeout (pmr, ECS_WRITE_TMO);
        return status;
    }

    /* If the timeout has expired register a fault */
    if (MARKED(M_TIMEOUT)) {
        status = recordError (pmr, "Vel/Pos write failure", S_ECS_TIMEOUT);
        return status;
    }

    /* Wait for the write acknowledgement */
    if (!(MARKED(M_WRITE))) return status;

    /* Value has been written, set the validation bit */
    //writeHandshake (pmr, pmr->hsta | ((pmr->mode == MODE_VMOVE) ? NEW_VEL_BIT : NEW_POS_BIT));
    setHandshakeBits (pmr, NEW_POS_BIT);
    setTimeout (pmr, ECS_PVOK_TMO);

    /* and proceed to the validation state */
     status = verifyingState (pmr);
    return (status);
}

/*
 * Function verifyingState
 * 
 * A new position or velocity word has been written to the device.
 * We now wait for an acknowledgement to come from the AB system.
 * If this is not received within the allowed time it is assumed that
 * the position was not accepted and the write is aborted.
 */
static long
verifyingState (struct ecsMotorRecord *pmr) {
    ecsMotorRecordPriv *pPriv = pmr->dpvt;
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%s> %s:verifyingState%c\n", ' ');
    //printf ("verifyingState, hinp=%x, rpos=%d\n", pmr->hinp, pmr->rpos);

    /* Nothing special on entry */
    if (pmr->msta != MOTOR_CHECKING) {
        pmr->msta = MOTOR_CHECKING;
        MARK(M_MSTA);
    }

    /* If the timeout expired then the controller has rejected the value */
    if (MARKED(M_TIMEOUT)) {
            Debug(DBUG_MIN, "<%s> %s:Value rejected%c\n", ' ');
        status = recordError (pmr, "Value not accepted!", S_ECS_VALUE_REJECT);
        // writeHandshake (pmr, pmr->hsta & ((pmr->mode == MODE_VMOVE) ? ~NEW_VEL_BIT : ~NEW_POS_BIT));
        resetHandshakeBits (pmr, NEW_POS_BIT);
        status = (pmr->mip == MIP_STOPPED) ? idleState(pmr) : movingState(pmr);
        return (status);
    }

    /* Otherwise keep waiting for acknowledgement */
    if (!(pmr->hinp & ((pmr->mode == MODE_VMOVE) ? VEL_ACK_BIT : POS_ACK_BIT))) {
         return (status);
    }

    /* Proceed to the starting state (this will clear the validation bit) */
    Debug(DBUG_MIN, "<%s> %s:Value acepted%c\n", ' ');
    if (pmr->mode == MODE_VMOVE) pPriv->currentVelocity = pmr->velo;
    else pPriv->currentTarget = pmr->val;

    status = startingState (pmr);
    return (status);
}

/*
 * Function startingState
 * 
 * A new position or velocity word has been written to the device and
 * has been accepted.   If the device is not already moving start it
 * now.
 */
static long
startingState (struct ecsMotorRecord *pmr) {
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%s> %s:startingState%c\n", ' ');
    printf ("startingState, hinp=%x\n", pmr->hinp);

    /* Clear the new position bit. This is necessary because the new driver
     * keeps updating this bit and competing with the PLC.
     */
    resetHandshakeBits (pmr, NEW_POS_BIT);

    if (pmr->msta != MOTOR_STARTING) {
        pmr->msta = MOTOR_STARTING;
        MARK(M_MSTA);

        if (!(pmr->hinp & PWR_ACK_BIT)) {
            status = recordError (pmr, "Unexpected drive powerdown", S_ECS_HSK_SYNC);
            return (status);
        }

        //writeHandshake (pmr, DRV_BIT | (pmr->hsta & DRV_ACK_BIT));
	printf ("%s: startingState, setting drive enable\n", timeStamp());
        setHandshakeBits (pmr, DRV_BIT);
        setTimeout (pmr, ECS_START_TMO);
        return (S_ECS_OK);
    }

    /* Timeout means the device will not start */
    if (MARKED(M_TIMEOUT)) {
	//printf ("%s: startingState, drive ack timeout\n", timeStamp());
        status = recordError (pmr, "Device did not start in time", S_ECS_TIMEOUT);
        return status;
    }

    /* Wait for motor to start */
    if (!(pmr->hinp & DRV_ACK_BIT)) {
        return (S_ECS_OK);
    }
    //printf ("%s: startingState, detected drive ack\n", timeStamp());

    /* Drive is enabled succesfully, go on to moving state */
    setTimeout (pmr, TIMEOUT_OFF);
    status = movingState (pmr);
    return (status);
}

/*
 * Function movingState
 * 
 * A move has been sucessfully started.   Keep an eye on the
 * power, enable and inPosition bits.
 */
static long
movingState (struct ecsMotorRecord *pmr) {
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%s> %s:movingState%c\n", ' ');
    //printf ("movingState, hinp=%x\n", pmr->hinp);

    /* Upon entry update the status words */
    if (pmr->msta != MOTOR_MOVING) {
        pmr->msta = MOTOR_MOVING;
        MARK(M_MSTA);
        pmr->movn = TRUE;
        MARK(M_MOVN);
        pmr->mip = ((pmr->mode == MODE_PARK) ? MIP_PARKING : MIP_MOVING);
        MARK(M_MIP);
    }

    /* Reached the desired position, stop the motion. Ignore the case when
     * the drive is disabled when parking, since the PLC will disable the
     * device when the EOT is reached. This does not conform with the ICD,
     * but modifying the PLC was too complicated (PG 14/Jul/99).
     * PARK command won't work when an EOT swicth is reached.
     */
    if (pmr->dmov || pmr->stop) {
      pmr->stop = 0;
      status = stoppingState(pmr);
    }
    else if (!(pmr->hinp & DRV_ACK_BIT)) {
        status = stoppingState (pmr);
        if (pmr->mode != MODE_PARK) {
            status = recordError (pmr, "PLC has disabled drive, motion failed", S_ECS_HSK_SYNC);
        }
    }
    return status;
}

/*
 * Function stoppingState
 * 
 * A STOP request has been received.  Terminate the motion in 
 * progress.   Set the value field to the current position. 
  */
static long
stoppingState (struct ecsMotorRecord *pmr) {
    ecsMotorRecordPriv *pPriv = pmr->dpvt;
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%s> %s:stoppingState%c\n", ' ');
    //printf ("stoppingState, hinp=%x\n", pmr->hinp);

    /* Disable drive on entry. Make sure that the drive is disabled
     * either when the drive enable or the drive acknowledge bits are
     * zero. This is because the PLC can disable the drive is a limit
     * switch is reached, which is the case when parking (PG 26/Jul/99).
     */
    if (pmr->msta != MOTOR_STOPPING) {
        pmr->msta = MOTOR_STOPPING;
        MARK(M_MSTA);
        pmr->mip = MIP_STOPPING;
        MARK (M_MIP);

        if ((pmr->hinp & DRV_BIT) || (pmr->hinp & DRV_ACK_BIT)) {
            resetHandshakeBits (pmr, DRV_BIT);
            setTimeout (pmr, ECS_STOP_TMO);
            return status;
        }
    }

    /* Timeout means the device will not stop */
    if (MARKED(M_TIMEOUT)) {
        status = recordError (pmr, "Device did not stop in time", S_ECS_TIMEOUT);
        return status;
    }

    /* Wait for drive to be disabled */
    if (pmr->hinp & DRV_ACK_BIT) {
        return (status);
    }

    /* Drive now disabled */
    setTimeout (pmr, TIMEOUT_OFF);
    pmr->movn = FALSE;
    MARK(M_MOVN);
    pmr->mip = MIP_STOPPED;
    MARK(M_MIP);

    /* what we do next depends on the operating mode */
     switch (pmr->mode) {
        case MODE_RESUME:
        case MODE_MOVE:
        case MODE_RESET:
        case MODE_STOP:
            pPriv->currentTarget = pmr->val = pmr->mpos;
            MARK(M_VAL);
            status = idleState (pmr);
            break;

        case MODE_PAUSE:
            pmr->dsta = menuCarstatesPAUSED;
            MARK(M_DSTA);
            status = idleState (pmr);
            break;

        case MODE_PARK:
            status = idleState (pmr);
            /*status = depoweringState (pmr); AWE: control power externally */
            break;

        default:
            status = recordError (pmr, "Unexpected motor stop", S_ECS_OK);
            break;
    }
    return (status);
}

#if 0
/*
 * Function depoweringState
 * 
 * A power-down request has been recived.  Shut off drive power.
 *
 * This routine is not used any more. The code was left here for reference (pgx)
 */
static long
depoweringState (struct ecsMotorRecord *pmr) {
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%s> %s:depoweringState%c\n", ' ');

    /* Upon entry shut down the power. Make sure that the drive is disabled
     * as well. This could not be the case when the PLC disables the drive
     * internally because a limit switch was reached (PG 26/Jul/99).
     */
    if (pmr->msta != MOTOR_DEPOWERING) {
        pmr->msta = MOTOR_DEPOWERING;
        MARK(M_MSTA);

        //writeHandshake (pmr, pmr->hsta & (~DRV_BIT & ~PWR_BIT));
        resetHandshakeBits (pmr, DRV_BIT | PWR_BIT);
        setTimeout (pmr, ECS_PWR_TMO);
        return status;
    }

    /* Timeout means that power will not shut off */
    if (MARKED(M_TIMEOUT)) {
        status = recordError (pmr, "can not shut down power", S_ECS_FAULT);
        return status;
    }

    /* Wait for power to shut down */
#if 0
    if (pmr->hsta) {
        return (status);
    }
#endif
    if (pmr->hinp) {
        return (status);
    }

    /* device has been parked, indicate this */
    setTimeout (pmr, TIMEOUT_OFF);
    pmr->mip = MIP_OFFLINE;
    MARK(M_MIP);
    pmr->dsta = menuCarstatesIDLE;
    MARK(M_MIP);
    pmr->msta = MOTOR_OFF;
    MARK(M_MSTA);
    pmr->dmov = FALSE;
    MARK(M_DMOV);
    pmr->pp = TRUE;
    return status;
}
#endif

/*
 * Function failingState
 * 
 * Error handler
 */
static long
failingState (struct ecsMotorRecord *pmr) {
   ecsMotorRecordPriv *pPriv = pmr->dpvt;
   long status = S_ECS_OK;

   Debug(DBUG_FULL, "<%s> %s:failingState%c\n", ' ');

   /* shut down any move in progress */
   // writeHandshake (pmr, pmr->hsta & ~DRV_BIT);
   resetHandshakeBits (pmr, DRV_BIT);

   /* drive has stopped due to a serious fault, indicate this */
   pmr->val = pPriv->currentTarget = pmr->mpos;
   MARK(M_VAL);
   pmr->mode = MODE_STOP;
   MARK(M_MODE);
   pmr->mip = MIP_ERROR;
   MARK(M_MIP);
   pmr->msta = MOTOR_FAULT;
   MARK(M_MSTA);
   pmr->pp = TRUE;
   return (status);
}

/*
 * Function processStateChange
 * 
 * A change in the handshaking bits has been
 * detected or a value write has been completed. 
 * Handle this depending on the current operating
 * state of the system. 
 * This is the primary control task for the sequencing system.
 */
static long processStateChange (struct ecsMotorRecord *pmr) {
   long status = S_ECS_OK;

#if 0
   if (pmr->msta != MOTOR_IDLE) {
      // tempLog2 ("processStateChange %s: msta=%d\n", pmr->name, pmr->msta);
   }
#endif

   Debug(DBUG_FULL, "<%s> %s:processStateChange:entry%c\n", ' ');

   /* Get on with the serious business of handling state changes 
    * based on the motor and handshake states.  Handshake changes
    * are only expected during the transitions from one stable state
    * to another.   In most of the cases we are waiting 
    * for the current handshake bits to match the target pattern
    * saved in the device private structure before making
    * a state transition
    */

   switch (pmr->msta) {
      case MOTOR_OFF:
      case MOTOR_FAULT:
         status = recordError (pmr, "Device offline or failed, must INIT",
                                S_ECS_VALUE_REJECT);
         break;
      case MOTOR_POWERING:
         /* status = poweringState (pmr); not used any more (pgx) */
         break;
      case MOTOR_IDLE:
         status = idleState (pmr);
         break;
      case MOTOR_WRITING:
         status = writingState(pmr);
         break;
      case MOTOR_CHECKING:
         status = verifyingState (pmr);
         break;
      case MOTOR_STARTING:
         status = startingState (pmr);
         break;
      case MOTOR_MOVING:
         status = movingState (pmr);
         break;
      case MOTOR_STOPPING:
         status = stoppingState (pmr);
         break;
      case MOTOR_DEPOWERING:
         /* status = depoweringState (pmr); not used any more (pgx) */
         break;
      case MOTOR_FAILING:
         status = failingState (pmr);
         break;

   } /* end switch(pmr->msta) */
   return (status);
}

/*
 * Function processInitialize
 * 
 * An INITIALIZE command has been received.   Stop the device
 * and reinitialize internal information.   If the power is off
 * it will be turned on now.
 */
static long processInitialize (struct ecsMotorRecord *pmr) {
   ecsMotorRecordPriv *pPriv = pmr->dpvt;
   long status = S_ECS_OK;

   Debug(DBUG_FULL, "<%s> %s:processInitialize:entry%c\n", ' ');

   /* all faults must be cleared first */
   status = checkFaultState (pmr);
   if (status) return status;

   /* reset new position and drive enable bits */
   resetHandshakeBits (pmr, NEW_POS_BIT | DRV_BIT);

   pmr->dsta = menuCarstatesBUSY;
   MARK(M_DSTA);
   pmr->mip = MIP_OFFLINE;
   MARK(M_MIP);

   pPriv->currentVelocity = pmr->velo;
   pPriv->currentMode = pmr->mode;

   pmr->mode = MODE_STOP;
   MARK(M_MODE);
   pPriv->currentMode = MODE_STOP;

   /* AWE Controlling Power externally now */
   /* Insure that the power is on */
   /* status = poweringState (pmr);*/

   status = idleState (pmr);
   return (status);
}

/******************************************************************************
   init_record()

Called twice after an EPICS database has been loaded, and then never called
again.
*******************************************************************************/
static long
init_record(struct ecsMotorRecord * pmr, int pass) {
   // struct ecsMotorDset *pdset = (struct ecsMotorDset *) (pmr->dset);
   ecsMotorRecordPriv *pPriv;
   long status = 0;

   Debug(DBUG_FULL, "<%s> %s init_record pass = %d\n", pass);

   /* Do nothing on the first pass */
   if (pass == 0)
      return (0);

   if (!ecsMotorRecordTaskId) {

      /* initialize the ecsMotor scan list */
      ellInit(&ecsMotorRecordScanList);

      /* start the ecsMotor control task */
      ecsMotorRecordTaskId = epicsThreadCreate(
           "ecsMotorRecord",                                /* thread name */
           epicsThreadPriorityMedium,                       /* priority    */
           epicsThreadGetStackSize(epicsThreadStackMedium), /* stack size  */
           (EPICSTHREADFUNC)ecsMotorRecordScanTask,         /* thread entry point */
           (void *)NULL);                                   /* paramter to thread function */

      if (!ecsMotorRecordTaskId) {
         status = S_dev_noMemory;
         recGblRecordError (status, pmr, __FILE__ ":can not spawn control task");
         return status;
        }
   }

   /* Allocate space for a private structure */
   pPriv = malloc (sizeof(ecsMotorRecordPriv));
   if (!pPriv) {
      status = S_dev_noMemory;
      recGblRecordError (status, pmr, __FILE__ ":no room for record  private");
      return status;
   }

   /* Create the MUTEX semaphore to protect the private structure
    * during asynchronous callback access */

   pPriv->mutexSem = epicsMutexCreate();

   /* This is where the ABDF1 device layer had set up the I/O links */

   /* 
    * Initialize the control structure then save it as the device private structure.
    * Initialize all of the internal status words.   Clearing the handshake 
    * word forces the system to update it as soon as the reading system
    * is enabled.
    */
   pPriv->pmr = pmr;
   pPriv->operatingMode = NORMAL_MODE;
   pPriv->currentMode = MODE_STOP;
   pPriv->currentTarget= 0;
   //pPriv->handshake = 0;
   pPriv->handshake_in = 0;
   pPriv->handshake_out = 0;
   pPriv->encoder = 0;
   pPriv->fault = 0;
   pPriv->inPosition = 0;
   pPriv->callbackFlags = 0;
   pPriv->timeoutActive = FALSE;

   pmr->dpvt = (void *) pPriv;

   UNMARK_ALL
   pmr->mess[0] = '\0';
   MARK(M_MESS);
   pmr->dsta = menuCarstatesIDLE;
   MARK(M_DSTA);
   pmr->movn = FALSE;
   MARK(M_MOVN);
   pmr->dmov = FALSE;
   MARK(M_DMOV);
   pmr->mip = MIP_OFFLINE;
   MARK(M_MIP);
   pmr->msta = MOTOR_OFF;
   MARK(M_MSTA);
   pmr->hsta = 0;
   MARK(M_HSTA);
   pmr->pp = FALSE;

   /* Add the private structure to the internal scan list */
   ellAdd(&ecsMotorRecordScanList, &pPriv->node);

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

/*
 * Function processSimulation
 * 
 * In simulation mode the actual hardware is not
 * accessed but various states of realism are
 * invoked to test the rest of the system.
 */
static long
processSimulation (struct ecsMotorRecord *pmr, long processType) {
  ecsMotorRecordPriv *pPriv = pmr->dpvt;
  long status = S_ECS_OK;

  Debug(DBUG_FULL, "<%s> %s:processSimulation:entry%c\n", ' ');

  /* Ignore simulation requests if the device is being used */
  if (pmr->simm &&
      pmr->msta != MOTOR_IDLE &&
      pmr->msta != MOTOR_OFF &&
      pmr->msta != MOTOR_FAULT) {
    Debug(DBUG_MIN, "<%s> %s:device busy, ignoring simulation request%c\n", ' ');
    return (FALSE);
  }

  /* Ignore asynchronous processing requests if in simulation mode */
  if (pmr->simm && processType == PROCESS_BUTTON) {
    Debug(DBUG_MIN, "<%s> %s:device simulated, ignoring async request%c\n", ' ');
    pmr->stop = 0;
    pPriv->fault = pmr->flt;
    pmr->pp = TRUE;
    return TRUE;
  }


  /* what happens next depends on the current simulation mode */
  switch (pmr->simm) {
  case SIMM_NONE:
    /* if we are leaving simulation mode regain control of the system */
    if (pPriv->simMode != SIMM_NONE) {
      pPriv->simMode = SIMM_NONE;
      pmr->mpos = (double) pmr->rrbv / pmr->psca;
      MARK(M_MPOS);

      /* if power is off insure that device is offline */
      if (!(pmr->hinp & PWR_ACK_BIT)) {
        resetHandshakeBits (pmr, DRV_BIT);
        pmr->val = pPriv->currentTarget = pmr->mpos;
        MARK(M_VAL);
        pmr->mode = MODE_STOP;
        MARK(M_MODE);
        pmr->mip = MIP_OFFLINE;
        MARK(M_MIP);
        pmr->msta = MOTOR_OFF;
        MARK(M_MSTA);
        pmr->pp = TRUE;
        return TRUE;
      }

      /* otherwise reset device to regain control */
      else {
        if (pmr->msta == MOTOR_FAULT) {
          pmr->mip = MIP_ERROR;
          MARK(M_MIP);
        }
        if (IN_POS_BIT) {
          pmr->dmov = TRUE;
          MARK(M_DMOV);
        pmr->mode = MODE_RESET;
        }
      }
    }

    /* then return FALSE to indicate motions are not being simulated */
    return FALSE;
    break;
  case SIMM_VSM:
    /* 
     * Setup simulation mode on first process.
     */
    if (pPriv->simMode != SIMM_VSM) {
      pPriv->simMode = SIMM_VSM;
      pmr->pp = TRUE;
    }

    /* 
     * Virtual simulation mode .. simply go busy for
     * a short while then idle again.
     */
    else if (MARKED(M_TIMEOUT)) {
      pmr->pp = TRUE;
    }
    else {
      setTimeout (pmr, ECS_SIM_TMO);
    }
    break;

  case SIMM_FAST:
    /* 
     * Setup simulation mode on first process.
     */
    if (pPriv->simMode != SIMM_FAST) {
      pmr->mip = MIP_STOPPED;
      MARK(M_MIP);
      pPriv->simMode = SIMM_FAST;
      pmr->pp = TRUE;
    }

    /* 
     * Fast simulation mode ... simply go busy for
     * a short while then fake move completion.
     */
    else if (MARKED(M_TIMEOUT)) {
      switch (pmr->mode) {
      case MODE_MOVE:
      case MODE_PARK:
        pmr->mpos = pmr->val;
        MARK(M_MPOS);
        pmr->dmov = TRUE;
        MARK(M_DMOV);
        pmr->mip = (pmr->mode == MODE_MOVE) ? MIP_STOPPED : MIP_OFFLINE;
        MARK(M_MIP);
        break;

      case MODE_VMOVE:
        pmr->dsta = menuCarstatesERROR;
        MARK(M_DSTA);
        strcpy (pmr->mess, "Velocity moves not supported");
        MARK(M_MESS);
        pmr->mip = MIP_ERROR;
        MARK(M_MIP);
        break;

      default:
        pmr->dmov = TRUE;
        MARK(M_DMOV);
        pmr->mip = MIP_STOPPED;
        MARK(M_MIP);
        break;
      }
      pmr->pp = TRUE;
    }
    else {
      setTimeout (pmr, ECS_SIM_TMO);
    }
    break;

  case SIMM_FULL:
    /* save device configuraton information when entering simulation mode */
    if (pPriv->simMode != SIMM_FULL) {
      pPriv->simTarget = pPriv->simPosition = pmr->mpos;
      pPriv->simVelocity = ECS_SIM_VEL / ECS_SCAN_RATE;
      pPriv->simMode = SIMM_FULL;
      pmr->mip = MIP_STOPPED;
      MARK(M_MIP);
      pmr->pp = TRUE;
    }
   /* 
     * Internal callback to  mimic the action of a device without
     * actually accessing the hardware
     */
    else if (processType == PROCESS_INTERNAL) {
      switch (pmr->mode) {
      case MODE_VMOVE:
        break;
      case MODE_PARK:
        if (pPriv->simPosition == pPriv->simTarget) {
          pmr->mip = MIP_OFFLINE;
          MARK(M_MIP);
          pmr->dmov = FALSE;
          MARK(M_DMOV);
          pmr->pp = TRUE;
        }
        break;
      default:
        if (pPriv->simPosition == pPriv->simTarget) {
          pmr->mip = MIP_STOPPED;
          MARK(M_MIP);
          pmr->dmov = TRUE;
          MARK(M_DMOV);
          pmr->pp = TRUE;
        }
        break;
      }
      pmr->mpos = pPriv->simPosition;
      MARK(M_MPOS);
    }
    /* handle external operating mode change requests here */
    else {
      switch (pmr->mode) {
      case MODE_RESET:
        pPriv->simTarget = pPriv->simPosition = pmr->mpos;
        pPriv->simVelocity = ECS_SIM_VEL / ECS_SCAN_RATE;
        pmr->mip = MIP_STOPPED;
        MARK(M_MIP);
        pmr->dmov = TRUE;
        MARK(M_DMOV);
        break;
      case MODE_PARK:
      case MODE_MOVE:
        if (pPriv->simTarget == pmr->val) {
          setTimeout (pmr, ECS_SIM_TMO);
        }
        else {
          pPriv->simTarget = pmr->val;
          pPriv->simVelocity = ECS_SIM_VEL / ECS_SCAN_RATE;
          pmr->mip = (pmr->mode == MODE_PARK) ? MIP_PARKING : MIP_MOVING;
          MARK(M_MIP);
          pmr->dmov = FALSE;
          MARK(M_DMOV);
        }
        break;
      case MODE_VMOVE:
        if (pmr->vms) {
          pPriv->simVelocity = pmr->velo / ECS_SCAN_RATE;
          pmr->mip = MIP_MOVING;
          MARK(M_MIP);
          pmr->dmov = FALSE;
          MARK(M_DMOV);
        }
        else {
          status = S_ECS_BAD_INPUT;
          pmr->dsta = menuCarstatesERROR;
          MARK(M_DSTA);
          strcpy (pmr->mess, "Velocity moves not supported");
          MARK(M_MESS);
          pmr->pp = TRUE;
        }
        break;
      default:
        pPriv->simTarget = pPriv->simPosition;
        pmr->dmov = TRUE;
        MARK(M_DMOV);
        setTimeout (pmr, ECS_SIM_TMO);
        break;
      }
    }
    break;
    /* none of the above, must be an error */
  default:
    status = recordError (pmr, "invalid simulation mode", S_ECS_OK);
    break;
  }
  return TRUE;
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
static long process(struct ecsMotorRecord * pmr)
{
   // struct ecsMotorDset *pdset = (struct ecsMotorDset *) (pmr->dset);
   
   long recordProcessType;
   long status, nRequest=1;

   if (pmr->pact)
      return (0);

   Debug(DBUG_FULL, "<%s> %s:process begin%c\n", ' ');
   pmr->pact = TRUE;

   /* 
    * Ask the device support to determine wether or not this
    * processing was a result of an operator request i.e. mode
    * change.   If so it will be necessary to mark the record as
    * busy and read all of the input links before processing.
    */
   // recordProcessType = (*pdset->processType) (pmr);
   recordProcessType = ecsMotorRecordProcessType(pmr);

   if (recordProcessType == PROCESS_NORMAL) {
     Debug(DBUG_FULL, "<%s> %s:process external process request%c\n", ' ');

     /* Clear the last error and indicate that the record is busy */
     pmr->mess[0] = '\0';
     MARK(M_MESS);
     pmr->dsta = menuCarstatesBUSY;
     MARK(M_DSTA);

     /* Check input links for new operating conditions */
     status = readInputLinks (pmr);
     if (status) {
	 Debug(DBUG_FULL, "<%s> %s:exiting after readInputs%c\n", ' ');
	 pmr->pact = FALSE;	/* pgx: added to prevent a deadlock */
	 return (status);
     }
   }

   /*
    * Now call what it used to be the device support process routine.
    * It was left as a separate routine because it has several return
    * calls that would break the logic if we insert the code here (pgx).
    */
   Debug(DBUG_FULL, "<%s> %s:invoke device processing%c\n", ' ');
   status = auxiliary_process(pmr, recordProcessType);
   if (status) {
     pmr->dsta = menuCarstatesERROR;
     pmr->pp = TRUE;
   }

   Debug(DBUG_FULL, "<%s> %s:auxiliary processing returned %ld\n", status);

   /* if Post Process request perform epics completion tasks */
   if (pmr->pp) {
      Debug(DBUG_FULL, "<%s> %s:process: post process entry%c\n", ' ');
      pmr->pp = FALSE;

      /* If an error was detected send the associated message, otherwise 
       * we just go back into the idle state.
       */
      if (pmr->dsta == menuCarstatesERROR) {

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

      /* Modified for GEM7. The status returned by dbPutLink should
       * not be checked because, according to the gensub's code, it
       * returns -1 if the record in not connected
       */
      status = dbPutLink (&(pmr->dstl), DBR_LONG, &(pmr->dsta), nRequest);

      /* fire off the forward link and record the time */
      Debug(DBUG_FULL, "<%s> %s:firing forward link%c\n", ' ');
      recGblFwdLink(pmr);   
      recGblGetTimeStamp(pmr);
   }   

   /* Update alarms and trigger motors before leaving */
   emrAlarm(pmr);
   monitor (pmr);

   pmr->pact = FALSE;

   Debug(DBUG_FULL, "<%s> %s:process end%c\n", ' ');

   return (status);
}

/*
 * Function auxiliary_process
 * 
 * This is the routine that used to be called directly by the ecsMotorRecord to handle
 * all device related functions. This can be a result of either an external process
 * request or an internal callback. The device can also be simulated in three levels
 * of realism. Note that if a new operating mode is recieved while the system is
 * starting or stopping a motion the command will be deferred until the system is stable.
 */
long auxiliary_process(struct ecsMotorRecord *pmr, long recordProcessType)
{
    ecsMotorRecordPriv *pPriv = pmr->dpvt;
    long status = S_ECS_OK;

   Debug(DBUG_FULL, "<%s> %s:auxiliary_process:entry%c\n", ' ');

   /* Force a device position update when a RESET is received. This will
    * make the record to update the device position with the right scaling
    * factors after a reboot or init is performed. The device position is
    * limited (see comment below) (PG 16/Jul/99).
    */
   if (pmr->mode == MODE_RESET) {
      pmr->mpos = positionFromEncoder(pmr);
      MARK(M_MPOS);
   }

   if (processSimulation (pmr, recordProcessType)) return status;

   /* Internal callbacks and control buttons get handled here */
   if (recordProcessType == PROCESS_INTERNAL || recordProcessType == PROCESS_BUTTON) {

      /* Update device position if required. The device position is
       * limited to numbers less than 60000 (negative values in the
       * PLC side) to prevent problems when the encoders go below zero.
       * The latter is needed to avoid problems in the high level
       * systems, specifically the TCS (PG 16/Jul/99).
       */
      if (MARKED(M_RRBV)) {
	  pmr->mpos = positionFromEncoder(pmr);
          MARK(M_MPOS);
      }

      /* Keep an eye out for device failures */
      status = checkFaultState(pmr);

      /* Then process the system state if it is safe to do so */
      if (!status ) status = processStateChange (pmr);

      /* get rid of stop requests made while device was not moving */
      pmr->stop = 0;

      /* end of internal callback process. */
      /* if there are no deferred commands return normally */
      if (status || pPriv->operatingMode != DEFERRED_MODE) return (status);
   }

   /* External requests get handled here */
   Debug(DBUG_FULL, "<%s> %s:auxiliary_process:external requests%c\n", ' ');


   /* Always accept an initialization request */
   if (pmr->mode == MODE_RESET) {
      Debug(DBUG_MIN, "<%s> %s:Executing command reset %d\n", pmr->mode);
      status = processInitialize (pmr);
      return status;
   }

   /* Defer operator requests received during state transitions, we will be back*/
   if (pmr->msta == MOTOR_POWERING   ||
       pmr->msta == MOTOR_WRITING    ||
       pmr->msta == MOTOR_CHECKING   ||
       pmr->msta == MOTOR_STARTING   ||
       pmr->msta == MOTOR_STOPPING   ||
       pmr->msta == MOTOR_DEPOWERING ||
       pmr->msta == MOTOR_FAILING) {
      if (recordProcessType == PROCESS_NORMAL) {
         Debug(DBUG_MIN, "<%s> %s:In transition, deferring command %d\n", pmr->mode);
         pPriv->operatingMode = DEFERRED_MODE;
         pPriv->deferredMode = pmr->mode;
         pmr->mode = pPriv->lastMode;
         pPriv->deferredTarget = pmr->val;
         pmr->val = pPriv->lastVal;
      }
   }
   
      /* If this is a deferred execution recover command and value */
      if (pPriv->operatingMode == DEFERRED_MODE) {
         Debug(DBUG_MIN, "<%s> %s:Recovering deferred value for command %d\n", pmr->mode);
         pmr->mode = pPriv->deferredMode;
         pmr->val = pPriv->deferredTarget;
         pPriv->operatingMode = NORMAL_MODE;
         pmr->pp = FALSE;
      }

      /* Save the input mode and value */
      Debug(DBUG_MIN, "<%s> %s:Executing command %d\n", pmr->mode);
      pPriv->lastMode = pmr->mode;
      pPriv->lastVal = pmr->val;

      /* Trap requests made while the device is offline or in a fault state  */
      if (pmr->msta == MOTOR_OFF || pmr->msta == MOTOR_FAULT) {
        status = recordError (pmr, "Must INIT system before using", S_ECS_OK);
      }

      /* Here is where we start action based a new operating mode*/
      else if (pmr->mode != pPriv->currentMode)
        status = processModeChange (pmr);

      /* Position and velocity updates are valid during motion */
      else if ((pmr->mode == MODE_MOVE && pmr->val != pPriv->currentTarget) ||
             (pmr->mode == MODE_PARK && pmr->val != pPriv->currentTarget) ||
             (pmr->mode == MODE_VMOVE && pmr->velo != pPriv->currentVelocity))
         status = writingState (pmr);

      /* This is totally bogus but forces a delay to insure that the CAR state change
       * is visible to the outside world when no action was required.
       */
      if (pmr->msta == MOTOR_IDLE || pmr->msta == MOTOR_FAULT || pmr->msta == MOTOR_OFF) {
         status = S_ECS_OK;
         pmr->pp = FALSE;
         setTimeout (pmr, ECS_SIM_TMO);
      }

   Debug(DBUG_FULL, "<%s> %s:auxiliary processing end %ld\n", status);

   return status;
}

/******************************************************************************
   special()
*******************************************************************************/
static long special(struct dbAddr *paddr, int after) {

   struct ecsMotorRecord *pmr = (struct ecsMotorRecord *) paddr->precord;
   
   /* no special processing supported, if we are here, it's bad news !! */
   Debug(DBUG_MAX, "<%s> %s:special after = %d\n", after);
   if (after) recGblDbaddrError (S_db_badChoice, paddr, "ecsMotor: special");

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
   Debug(DBUG_FULL, "<%s> %s:post_MARKed_fields entry, mmap = %x\n", pmr->mmap);

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
   if (MARKED(M_RPOS)) {
      db_post_events(pmr, &pmr->rpos, mask);
      UNMARK(M_MDBD);
   }
   if (MARKED(M_HINP)) {
      db_post_events(pmr, &pmr->hinp, mask);
      UNMARK(M_HINP);
   }
   if (MARKED(M_PDFB)) {
      db_post_events(pmr, &pmr->pdfb, mask);
      UNMARK(M_PDFB);
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

   Debug(DBUG_FULL, "<%s> %s:initLinks: entry%c\n", ' ');

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

   Debug(DBUG_FULL, "<%s> %s:readInputLinks: entry%c\n", ' ');


/* Here we will need to add reading the Handshake and encoder position links 
   that had formely been read by the ABDF1 driver (mdw)  */
//         status = dbGetLink(&pmr->posi, DBR_USHORT, &pmr->encoder, 0,0);  
//        status = dbGetLink(&pmr->hski, DBR_USHORT, &pmr->, 0,0);  

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

epicsExportAddress(int, var01);
epicsExportAddress(int, var02);
epicsExportAddress(int, var03);


/*********************************************************************

  original change notes from devEcsAbDf1.c placed here for reference

  $Log: devEcsAbDf1.c,v $
  Revision 1.9  2017/06/15 02:06:40  gemvx
  Fixed the 'Unexpected drive powerdown' message (REL-2623)

  Revision 1.8  2015/05/09 02:12:32  gemvx
  Don't set the pwr bit for a move .. this is being controlled externally to the motor interface

  Revision 1.7  2015/03/19 02:06:57  gemvx
  remove powering during initialize and depowering after parking

  Revision 1.6  2008/03/06 01:12:21  gemvx
  reverted to the original V2-7-2

  Revision 1.4  2003/04/04 23:28:05  pedro
  Removed AB comm error and replaced with counter.

  Revision 1.3  2002/06/21 15:48:56  pedro
  Port to EPICS 3.13.4.

  Revision 1.2  2001/06/21 16:30:34  pedro
  Modified the way PARK is handled when the device is already there. Added
  skewing detection.

  Revision 1.2  1999/04/29 22:02:22  pedro
  Added temporary error logging routine t trap communication errors

  Revision 1.1.1.1  1998/06/30 03:43:20  ajf
  Initial creation of the Gemini ECS Repository


***********************************************************************/
