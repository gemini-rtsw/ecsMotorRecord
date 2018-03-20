/* devEcsAbDf1.c - Allen Bradley DF1 Serial Protocol Device Support Routines */
/*
 *      Original Author:        Jeff Hill 
 *       GEMINI EcsMotor support added:    William Rambold
 *
 */

/*
 * Format of the instrument type addresses accepted by analog records
 * <serial device address> = "<file name> <station number> <word address>"
 *
 * Format of the instrument type addresses accepted by binary records
 * <serial device address> = "<file name> <station number> 
 *                    <word address> <bit number>"
 *
 * Format of the instrument type addresses accepted by ecsMotor records
 * <serial device address> = "<file name> <station number> 
 *                    <handshake word address> <encoder word address>"
 *
 * (where a word address is a byte address divided by two)
 * (where bits are number with the ls bit is bit 0 and the ms bit is bit 15)
 * (where a file name is frequently the device name of a serial port
 *     ex: /tyCo/2 (see iosDevShow under vxWorks) )
 */

/*        version 1.0   1998/05/01  wnr   gamma release
 *        version 1.01  1998/06/05  wnr   fixed velocity moves in FULL simulation,
 *                            forced drive disable in failing state and removed
 *                            spurious debug message on scan task initialization.
 */
 
/*
 * ANSI C
 */
#include    <stdlib.h>
#include    <string.h>

/*
 * EPICS OSI
 */
#include    <epicsEvent.h>
#include    <epicsThread.h>
#include    <epicsTime.h>
#include    <epicsPrint.h>
#include    <math.h>

/*
 * EPICS
 */
#include    <epicsAssert.h>
#include    <alarm.h>
#include    <cvtTable.h>
#include    <dbDefs.h>
#include    <dbAccess.h>
#include    <recSup.h>
#include    <devSup.h>
#include    <dbScan.h>
#include    <link.h>
#include    <recGbl.h>


#include    <aiRecord.h>
#include    <aoRecord.h>
#include    <biRecord.h>
#include    <boRecord.h>
#include    <mbbiRecord.h>
#include    <mbbiDirectRecord.h>
#include    <mbboRecord.h>
#include    <mbboDirectRecord.h>
#include    <devLib.h>

/*
 * Device specific
 */
#include     <drvAbDf1.h>
#include     <devAbDf1.h>
#include     <devEcsAbDf1.h> 
#include     <ecsMotorRecord.h>
#include     <ecsMotor.h>
#include     <ecsABinterface.h>
#include     <choiceEcsMotor.h>
#include     <menuCarstates.h>

#define MESSAGE_BUFFER_SIZE 40

#define S_devAbDf1_OK         0
#define S_devAbDf1_dontConvert       2

#define devInitPassBeforeDevInitRec 0
#define devInitPassAfterDevInitRec 1

static unsigned devAbDf1GetToken(const char **, char *, unsigned);
static void moDevWriteDoneAbDf1 (absWordIO *, long);
static void moDevReadDoneAbDf1 (absWordIO *, long);




/*** Debugging variables, macros ***/

#ifdef NODEBUG
#define Debug(l,FMT,V) ;
#else
//#define Debug(l,FMT,V) if (l <= pmr->dbug) logMsg (FMT, (int) tickGet(), (int) pmr->name, (int) V, 0, 0, 0);
#define Debug(l,FMT,V) if (l <= pmr->dbug) errlogPrintf(FMT, 0, pmr->name,  (int)V);
#endif

static void tempLog0 (char *fmt)
{
}
static void tempLog1 (char *fmt, char *arg1)
{
}
static void tempLog2 (char *fmt, char *arg1, long arg2)
{
}
static void tempLog3 (char *fmt, char *arg1, long arg2, long arg3)
{
}
static void tempLog4 (char *fmt, char *arg1, long arg2, long arg3, long arg4)
{
}
static void tempLog5 (char *fmt, char *arg1, long arg2, long arg3, long arg4,
		      long arg5)
{
}

/*
 * Variables used to count AB comm errors.
 */
static long	      commErrCount;
static epicsTimeStamp commErrTime;




#if 0                /* what a great way to obfuscate things!   */
typedef long moDevReport(struct ecsMotorRecord *prec, unsigned level);
typedef long moDevInit (unsigned pass);
typedef long moDevInitRec (struct ecsMotorRecord * prec);
typedef long moDevGetIoIntInfo(int cmd, struct ecsMotorRecord *prec, IOSCANPVT *pPvt);
typedef long moDevProcessType(struct ecsMotorRecord *prec);
typedef long moDevProcessRecord(struct ecsMotorRecord *prec, long processType);
struct mo_dev_sup{
        long            number;
    moDevReport    *report;
    devInit        *init;
    moDevInitRec    *initRec;
    moDevGetIoIntInfo *getIoIntInfo;
        moDevProcessType *processType;
    moDevProcessRecord *process;
};
LOCAL moDevInitRec moDevInitRecAbDf1;
LOCAL moDevGetIoIntInfo moDevGetIoIntInfoAbDf1;
LOCAL moDevProcessType moDevProcessTypeAbDf1;
LOCAL moDevProcessRecord moDevProcessRecordAbDf1;
#endif


/* let's do it this way instead */
static long moDevInitAbDf1(unsigned pass);
static long moDevInitRecAbDf1 (struct ecsMotorRecord *pmr);
static long moDevGetIoIntInfoAbDf1 (int cmd, struct ecsMotorRecord *pmr, IOSCANPVT *pPvt); 
static long moDevProcessTypeAbDf1 (struct ecsMotorRecord *pmr); 
static long moDevProcessRecordAbDf1 (struct ecsMotorRecord *pmr, long processType);

struct {                  /* Device Support Entry Table */
   long        number;
   DEVSUPFUN   report;
   DEVSUPFUN   init;
   DEVSUPFUN   initRec;
   DEVSUPFUN   getIoIntInfo;
   DEVSUPFUN   processType;
   DEVSUPFUN   process;
} devMoAbDf1 = {
    6,
    NULL,
    moDevInitAbDf1,
    moDevInitRecAbDf1,
    moDevGetIoIntInfoAbDf1,
    moDevProcessTypeAbDf1,
    moDevProcessRecordAbDf1
};










static long processModeChange (struct ecsMotorRecord *);
static long processStateChange (struct ecsMotorRecord *);
static long processInitialize (struct ecsMotorRecord *);
static long processSimulation (struct ecsMotorRecord *, long);
static long checkFaultState (struct ecsMotorRecord *);
static long writeHandshake (struct ecsMotorRecord *, unsigned);
static long recordError (struct ecsMotorRecord *, char *, long);
static long setTimeout (struct ecsMotorRecord *, long);
static int moDevAbDf1ScanTask();
 
static long poweringState (struct ecsMotorRecord *);
static long idleState (struct ecsMotorRecord *);
static long writingState (struct ecsMotorRecord *);
static long verifyingState (struct ecsMotorRecord *);
static long startingState (struct ecsMotorRecord *);
static long movingState (struct ecsMotorRecord *);
static long stoppingState (struct ecsMotorRecord *);
static long depoweringState (struct ecsMotorRecord *);
static long failingState (struct ecsMotorRecord *);

/*
* Internal control structure for ecsMotorRecord support. 
* This holds pointers to the AbDf1 serial control structures
* and the current state of the system control variables.
*/
typedef struct {
    ELLNODE      node;                  /* processing scan node defintion */
    struct       ecsMotorRecord *pmr;   /* pointer to ecsMotor record */
    absWordIO    *pReadHskPriv;         /* callback for handshake read */
    absWordIO    *pWriteHskPriv;        /* callback for handshake write */
    absWordIO    *pEncoderPriv;         /* callback for position read */
    absWordIO    *pWritePosPriv;        /* callback for position write */
    absWordIO    *pWriteVelPriv;        /* callback for velocity write */

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
    unsigned     handshake;             /* latest handshake word */
    unsigned     encoder;               /* latest encoder position */ 
    unsigned     fault;                 /* latest fault condition */
    short        inPosition;            /* latest in-position state */
    short        callbackFlags;         /* callback activity */
    short        timeoutActive;         /* timeout flag */
    long         timeoutTimer;          /* timeout tick counter */ 
    } ecsMotorPriv;

/*
 * ecsMotor internal scan list
 */
static ELLLIST    ecsMotorScanList;
static epicsThreadId  ecsTaskId;



/*
 * devAbDf1InitPrivate()
 *
 * if doBitNo is true then we ask for the bit number field
 */
static long 
devAbDf1InitPrivate(struct link *pLink, void *pRec, 
                    epicsUInt16 *pBitNo, absWordIO **ppPriv)
{
    int        status;
    //char        fileName[sizeof(pLink->value)];
    absWordIO    *pPriv;

    if (pLink->type != INST_IO) {
        status = S_db_badField;
        recGblRecordError (status, pRec,
                __FILE__": Address type must be type \"instrument\"");
        return status;
    }

    pPriv = calloc(1,sizeof(*pPriv));
    if (!pPriv) {
        status = S_dev_noMemory;
        recGblRecordError (status, pRec,
                __FILE__ ": no room for device private");
        return status;
    }

    /*
     *
     * 2018 ECS RTEMS Refactor the following  #if 0  block...
     *
     * ....Probably use the AbDf1 CCB Driver:
     *          drvAbDf1.c: drvAbDf1SetupIO( pLink, ...)
     *
     *     Study the following data types needed from AbDf1 CCB:
     *     1. drvAbDf1FuncTable
     *     2. struct abDf1ElemIO
     *     3. struct drvAbDf1ElemIO
     *
     */
#if 0
    status = parseAbDf1Address(
            pLink->value.instio.string,
            fileName,
            sizeof(fileName),
            &pPriv->stationNumber,
            &pPriv->wordAddr,
            pBitNo);
    if (status) {
        recGblRecordError (status, pRec,
                __FILE__ ": Syntax error in type \"instrument\" address");
        return status;
    }

    status = drvAbDf1CreateLink(fileName, &pPriv->linkId);
#endif
    
    if (status) {
        recGblRecordError (status, pRec,
                __FILE__ ": failed to create serial link");
        return status;
    }
    pPriv->pPriv = pRec;
    *ppPriv = pPriv;

    scanIoInit(&pPriv->ioScanPvt);

    return status;
}

/*
 * Consider for 2018 RTEMS Refactor................
 * devInitAbDf1()
 * 
 *  20180125 MDW Rename and gut all but error count and time initialization
 *               then call the CCB version of devInitAbDf1()
 */

static long moDevInitAbDf1(unsigned pass)
{
	commErrCount = 0;
        epicsTimeGetCurrent(&commErrTime);

        return devInitAbDf1(pass);                   /* call the CCB routine */
}



/*
 * Function moDevInitRecAbDf1
 *
 * Setup Allen Bradley support for the EcsMotorRecord 
 *
 */
static long moDevInitRecAbDf1 (struct ecsMotorRecord *pmr) {
    long status;
    epicsUInt16 encoderWord;
    ecsMotorPriv *pPriv;

    /*
     * set up the ecsMotor scan task.... but only once
     */
    if (!ecsTaskId) {

         /* initialize the ecsMotor scan list */
        ellInit(&ecsMotorScanList);

         /* start the ecsMotor control task */
        ecsTaskId = epicsThreadCreate(
           "ecsMotor",                                      /* thread name */
           epicsThreadPriorityMedium,                       /* priority    */
           epicsThreadGetStackSize(epicsThreadStackMedium), /* stack size  */
           (EPICSTHREADFUNC)moDevAbDf1ScanTask,             /* thread entry point */
           (void *)NULL);                                   /* paramter to thread function */

#if 0
        ecsTaskId = taskSpawn(
            "ecsMotor",            /* task name */
            80,                    /* priority */
            VX_FP_TASK,            /* options - none */
            0x1000,                /* stack size 4096 */
            moDevAbDf1ScanTask,    /* task entry point */
            0,0,0,0,0,0,0,0,0,0);  /* invocation args 1 to 10 */
#endif

        if (!ecsTaskId) {
	  status = S_dev_noMemory;
	  recGblRecordError (status, pmr, __FILE__ ":can not spawn control task");
	  return status;
        }
    }
  
    /* Allocate space for a device private structure */
    pPriv = malloc (sizeof(ecsMotorPriv));
    if (!pPriv) {
        status = S_dev_noMemory;
        recGblRecordError (status, pmr, __FILE__ ":no room for device private");
        return status;
    }

    /* Create the MUTEX semaphore to protect the private structure
     * during asynchronous callback access */
     
     //pPriv->mutexSem = semMCreate (SEM_Q_PRIORITY | SEM_INVERSION_SAFE);
     pPriv->mutexSem = epicsMutexCreate();
     
    /* Setup handshake word reading channel */
    status = devAbDf1InitPrivate (&pmr->out, pmr, &encoderWord, &pPriv->pReadHskPriv);
    if (status) return (status);
    
    status = drvAbDf1SetupAnalogIn (pPriv->pReadHskPriv);
    if (status) {
        recGblRecordError (status, pmr, __FILE__ ":failed to set up handshake scan");
        return (status);
    }
  
    pPriv->pReadHskPriv->pCB = moDevReadDoneAbDf1;
  
    /* Setup handshake word writing channel */
    status = devAbDf1InitPrivate (&pmr->out, pmr, &encoderWord, &pPriv->pWriteHskPriv);
    if (status) return (status);
  
    status = drvAbDf1SetupAnalogOut (pPriv->pWriteHskPriv);
    if (status) {
        recGblRecordError (status, pmr, __FILE__ ":failed to create hsk serial link");
        return (status);
    }
        
    pPriv->pWriteHskPriv->pCB = moDevWriteDoneAbDf1;
    
    /* 
     * Setup position word writing channel.
     * The position word will be the next slot after the handshake channel.
     */  
    
    status = devAbDf1InitPrivate (&pmr->out, pmr, &encoderWord, &pPriv->pWritePosPriv);
    if (status) return (status);
  
    pPriv->pWritePosPriv->wordAddr = pPriv->pWritePosPriv->wordAddr + ECS_POS_OFFSET; 

    status = drvAbDf1SetupAnalogOut (pPriv->pWritePosPriv);
    if (status) {
        recGblRecordError (status, pmr, __FILE__ ":failed to create posn serial link");
        return (status);
    }
        
    pPriv->pWritePosPriv->pCB = moDevWriteDoneAbDf1;
  
    /* 
    * Setup velocity word writing channel if supported in this instance.
    * The velocity and postion words will be the next two slots after the handshake.                     
     */
    if (pmr->vms) {
        status = devAbDf1InitPrivate (&pmr->out, pmr, &encoderWord, &pPriv->pWriteVelPriv);
        if (status) return (status);
    
        pPriv->pWriteVelPriv->wordAddr = pPriv->pWriteVelPriv->wordAddr + ECS_POS_OFFSET;
        pPriv->pWritePosPriv->wordAddr = pPriv->pWriteVelPriv->wordAddr + ECS_POS_OFFSET;
    
        status = drvAbDf1SetupAnalogOut (pPriv->pWriteVelPriv);
        if (status) {
            recGblRecordError (status, pmr, __FILE__ ":failed to create serial link");
            return (status);
	}
        
        pPriv->pWriteVelPriv->pCB = moDevWriteDoneAbDf1;
    }

    /* Setup device position word reading channel. Replace handshake word
    * with encoder word before setting up the channel.
    */
    status = devAbDf1InitPrivate (&pmr->out, pmr, &encoderWord, &pPriv->pEncoderPriv);
    if (status) return (status);
    
    pPriv->pEncoderPriv->wordAddr = encoderWord;
    
    status = drvAbDf1SetupAnalogIn (pPriv->pEncoderPriv);
    if (status) {
        recGblRecordError (status, pmr, __FILE__ ":failed to set up encoder scan");
        return (status);
    }
  
    pPriv->pEncoderPriv->pCB = moDevReadDoneAbDf1;

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
    pPriv->handshake = 0;
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
    ellAdd(&ecsMotorScanList, &pPriv->node); 

    /* and we are done */
    return (status);
}


/*
 * Function moDevGetIoIntInfoAbDf1
 * 
 * Return IO Interrupt scan info for ecsMotorRecord
 *
 */
 
static long moDevGetIoIntInfoAbDf1 (int cmd, struct ecsMotorRecord *pmr, IOSCANPVT *pPvt) {
    ecsMotorPriv *pRecPriv = pmr->dpvt;
    absWordIO *pPriv = (absWordIO *) pRecPriv->pReadHskPriv;
    
     if (!pPriv) return S_devAbDf1_OK;
           
    *pPvt = pPriv->ioScanPvt;
    return S_ECS_OK;
    }

/*
 * Function moDevProcessTypeAbDf1
 * 
 * Determine what caused the record to be processed
 *
 */
 
static long moDevProcessTypeAbDf1 (struct ecsMotorRecord *pmr) {
  ecsMotorPriv *pPriv = pmr->dpvt;

  Debug(DBUG_FULL, "<%d> %s:moDevProcessTypeAbDf1:entry%c\n", ' ');

  /* Internal scan requests are detected here. */
  if (MARKED(M_RESCAN)) return PROCESS_INTERNAL;

  /* "button" fields such as stop and fault are detected here */
  if (pmr->stop ||
      pmr->flt != pPriv->fault) return PROCESS_BUTTON;

  /* Otherwise it is a normal external procesing request */
  return PROCESS_NORMAL;
}


/*
 * Function moDevProcessRecordAbDf1
 * 
 * Called directly by the ecsMotorRecord to handle all device
 * related functions.   This can be a result of either an external
 * process request or an internal callback.   The device can also
 * be simulated in three levels of realism.   Note that if a new
 * operating mode is recieved while the system is starting or
 * stopping a motion the command will be deferred until the system
 * is stable.
 *
 */

static long moDevProcessRecordAbDf1 (struct ecsMotorRecord *pmr, long processType) {
    ecsMotorPriv *pPriv = pmr->dpvt;
    long status = S_ECS_OK;
    long	limval;

    Debug(DBUG_FULL, "<%d> %s:moDevProcessRecordAbDf1:entry%c\n", ' ');


    /* Force a device position update when a RESET is received. This will
     * make the record to update the device position with the right scaling
     * factors after a reboot or init is performed. The device position is
     * limited (see comment below) (PG 16/Jul/99).
     */
    if (pmr->mode == MODE_RESET) {
         limval = (pmr->rrbv > 60000) ? 0 : pmr->rrbv;
	 pmr->mpos = (double) limval / pmr->psca;
	 MARK(M_MPOS);
    }

    /* In simulation mode handle everything seperatly */
    if (processSimulation (pmr, processType)) return status;
                                
     /* Internal callbacks and control buttons get handled here */
     if (processType == PROCESS_INTERNAL || processType == PROCESS_BUTTON) {
 
       /* Update device position if required. The device position is
	* limited to numbers less than 60000 (negative values in the
	* PLC side) to prevent problems when the encoders go below zero.
	* The latter is needed to avoid problems in the high level
	* systems, specifically the TCS (PG 16/Jul/99).
	*/
       if (MARKED(M_RRBV)) {
	 limval = (pmr->rrbv > 60000) ? 0 : pmr->rrbv;
	 pmr->mpos = (double) limval / pmr->psca;
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
                                                 
    /* Always accept an initialization request */
    if (pmr->mode == MODE_RESET) {   
      Debug(DBUG_MIN, "<%d> %s:Executing command %d\n", pmr->mode);
      status = processInitialize (pmr);
      return status;
    }
    
    /* Defer operator requests received during state transitions, we will be back*/
    if (pmr->msta == MOTOR_POWERING ||
        pmr->msta == MOTOR_WRITING ||
	pmr->msta == MOTOR_CHECKING ||
        pmr->msta == MOTOR_STARTING ||
        pmr->msta == MOTOR_STOPPING ||
        pmr->msta == MOTOR_DEPOWERING ||
        pmr->msta == MOTOR_FAILING) {
      if (processType == PROCESS_NORMAL) {
	Debug(DBUG_MIN, "<%d> %s:In transition, deferring command %d\n", pmr->mode);
	pPriv->operatingMode = DEFERRED_MODE;
	pPriv->deferredMode = pmr->mode;
	pmr->mode = pPriv->lastMode;
	pPriv->deferredTarget = pmr->val;
	pmr->val = pPriv->lastVal;
      }
      return (status);
    }

    /* If this is a deferred execution recover command and value */
    if (pPriv->operatingMode == DEFERRED_MODE) {
    Debug(DBUG_MIN, "<%d> %s:Recovering deferred value for command %d\n", pmr->mode);
      pmr->mode = pPriv->deferredMode;
      pmr->val = pPriv->deferredTarget;
      pPriv->operatingMode = NORMAL_MODE;
      pmr->pp = FALSE;
    }

    /* Save the input mode and value */
    Debug(DBUG_MIN, "<%d> %s:Executing command %d\n", pmr->mode);
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

    /* all done, return */
    return (status);
}

/*
 * Function processModeChange
 * 
 * Called whenever the operator requests an operating
 * mode change.
 */

static long
processModeChange (struct ecsMotorRecord *pmr) {
    ecsMotorPriv *pPriv = pmr->dpvt;
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%d> %s:processModeChange:entry%c\n", ' ');

tempLog3 ("processModeChange %s: mode=%d, hsta=%x", pmr->name, pmr->mode, pmr->hsta);

    if (pmr->mode == pPriv->currentMode) return S_ECS_OK;
    

    /* process depending on requested mode */
    switch (pmr->mode) {
        case MODE_STOP:
tempLog1 ("processModeChange %s: STOP, to stoppingState", pmr->name);
            status = stoppingState (pmr);
            break;

        case MODE_VMOVE:
            if (!(pmr->vms)) {
                status = recordError (pmr, "Velocity mode not supported", S_ECS_OK);
                return (status);
                }
        case MODE_MOVE:
        case MODE_PARK:
tempLog1 ("processModeChange %s: MOVE & PARK, to writingState", pmr->name);
            status = writingState (pmr);
            break;

        case MODE_PAUSE:
            if (pPriv->currentMode == MODE_MOVE) {
tempLog1 ("processModeChange %s: PAUSE, to stoppingState", pmr->name);
                status = stoppingState (pmr);
            }
            else {
                recordError (pmr, "Can not pause this move", S_ECS_OK);
            }
            break;
             
        case MODE_RESUME:
            if (pPriv->currentMode == MODE_PAUSE) {
tempLog1 ("processModeChange %s: RESUME, to startingState", pmr->name);
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
 * Function processInitialize
 * 
 * An INITIALIZE command has been received.   Stop the device
 * and reinitialize internal information.   If the power is off
 * it will be turned on now.
 */

static long
processInitialize (struct ecsMotorRecord *pmr) {
    ecsMotorPriv *pPriv = pmr->dpvt; 
    long status = S_ECS_OK;
                          
    Debug(DBUG_FULL, "<%d> %s:processInitialize:entry%c\n", ' ');

    /* all faults must be cleared first */
    status = checkFaultState (pmr);
    if (status) return status;

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

/*
 * Function processStateChange
 * 
 * A change in the Allen Bradley handshaking bits has been
 * detected or a value write has been completed. 
 * Handle this depending on the current operating
 * state of the system. 
 * This is the primary control task for the sequencing system.
 */

static long
processStateChange (struct ecsMotorRecord *pmr) {
    long status = S_ECS_OK;

if (pmr->msta != MOTOR_IDLE) {
tempLog2 ("processStateChange %s: msta=%d\n", pmr->name, pmr->msta);
}

    Debug(DBUG_FULL, "<%d> %s:processStateChange:entry%c\n", ' ');
                                                           
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
            status = poweringState (pmr);
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
            status = depoweringState (pmr);
            break;
        case MOTOR_FAILING:
            status = failingState (pmr);
            break;

    } /* end switch(pmr->msta) */     

    return (status);
}


        
/*
 * Function poweringState
 * 
 * A power up request has been received.  If the motor
 * power is off then turn it back on and wait for the power
 * on acknowledge to be detected.  The acknowledge
 * must be recieved within the specified time or a timeout
 * error is generated.   Note that this function is called
 * many times during this process.
 */

static long
poweringState (struct ecsMotorRecord *pmr) {
    ecsMotorPriv *pPriv = pmr->dpvt;
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%d> %s:poweringState%c\n", ' '); 
 
tempLog3 ("poweringState %s: msta=%d, hsta=%x\n", pmr->name, pmr->msta, pmr->hsta);

    /* Upon entry clean up handshake bits and make sure that the power is on */
    if (pmr->msta != MOTOR_POWERING) {
        pmr->msta = MOTOR_POWERING;
        MARK(M_MSTA);              
     
        if (pmr->hsta != (PWR_BIT | PWR_ACK_BIT)) {
            writeHandshake (pmr, PWR_BIT);
            setTimeout (pmr, ECS_PWR_TMO);
        } 
    }

    /* If the timeout has expired regester a fault */
    if (MARKED(M_TIMEOUT)) {
        status = recordError (pmr, "Device did not power up in time", S_ECS_TIMEOUT);
        return status;
    }

    /* Wait for a power on acknowledge */
    if (pmr->hsta != (PWR_BIT | PWR_ACK_BIT)) { 
        return (S_ECS_OK);
    }
    
    /* Got it, make target equal current positon and move to the IDLE state */         
    setTimeout (pmr, TIMEOUT_OFF);
    pPriv->currentTarget = pmr->val = pmr->mpos;
    status = idleState (pmr);
    return (status);
}

/*
 * Function idleState
 * 
 * A new position or velocity word has been written to the device and
 * has been accepted.   If the device is not already moving start it
 * now.
 */

static long
idleState (struct ecsMotorRecord *pmr) {
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%d> %s:idleState%c\n", ' ');
 
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
    ecsMotorPriv *pPriv = pmr->dpvt;
    unsigned impliedDecimal;
    long status = S_ECS_OK; 

    Debug(DBUG_FULL, "<%d> %s:writingState%c\n", ' ');

tempLog4 ("writingState %s: mode=%d, dmov=%d, movn=%d\n", pmr->name, pmr->mode, pmr->dmov, pmr->movn);

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
            impliedDecimal = (unsigned) (pmr->velo * pmr->vsca);
             status = drvAbDf1WriteAnalog (pPriv->pWriteVelPriv, impliedDecimal);
                if (status) {
                status = recordError (pmr, "ECS velocity write failure", status);
                return (status);
            }
            Debug(DBUG_MIN, "<%d> %s:velocity word write %d \n", impliedDecimal);
        }
        else {
            impliedDecimal = (unsigned) (pmr->val * pmr->psca);
                status = drvAbDf1WriteAnalog (pPriv->pWritePosPriv, impliedDecimal);
            if (status) {
                status = recordError (pmr, "ECS position write failure", status);
                   return (status);
            }
            Debug(DBUG_MIN, "<%d> %s:position word write %d \n", impliedDecimal);
        }
        setTimeout (pmr, ECS_WRITE_TMO); 
        return status;
    }
        
    /* If the timeout has expired regester a fault */
    if (MARKED(M_TIMEOUT)) {
        status = recordError (pmr, "Vel/Pos write failure", S_ECS_TIMEOUT);
        return status;
    }

    /* Wait for the write acknowledgement */
    if (!(MARKED(M_WRITE))) return status;                 

    /* Value has been written, set the validation bit */ 
    writeHandshake (pmr, pmr->hsta | ((pmr->mode == MODE_VMOVE) ? NEW_VEL_BIT : NEW_POS_BIT));
    setTimeout (pmr, ECS_WRITE_TMO); 
            
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
    ecsMotorPriv *pPriv = pmr->dpvt;
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%d> %s:verifyingState%c\n", ' '); 

tempLog3 ("verifyingState %s: msta=%d, hsta=%x\n", pmr->name, pmr->msta, pmr->hsta);

    /* Nothing special on entry */
    if (pmr->msta != MOTOR_CHECKING) {
        pmr->msta = MOTOR_CHECKING;
        MARK(M_MSTA);
    }

    /* If the timeout expired then the controller has rejected the value */
    if (MARKED(M_TIMEOUT)) {
            Debug(DBUG_MIN, "<%d> %s:Value rejected%c\n", ' '); 
        status = recordError (pmr, "Value not accepted!", S_ECS_VALUE_REJECT);
        writeHandshake (pmr, pmr->hsta &
                ((pmr->mode == MODE_VMOVE) ? ~NEW_VEL_BIT : ~NEW_POS_BIT));
        status = (pmr->mip == MIP_STOPPED) ? idleState(pmr) : movingState(pmr);
        return (status);
    }                                                       
    
    /* Otherwise keep waiting for acknowledgement */         
    if (!(pmr->hsta & ((pmr->mode == MODE_VMOVE) ? VEL_ACK_BIT : POS_ACK_BIT))) {
         return (status);
    }

    /* Proceed to the starting state (this will clear the validation bit) */
    Debug(DBUG_MIN, "<%d> %s:Value acepted%c\n", ' '); 
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
   
    /* Upon entry enable the drive if it is not already on */
    Debug(DBUG_FULL, "<%d> %s:startingState%c\n", ' '); 

tempLog2 ("startingState %s: msta=%d\n", pmr->name, pmr->msta);

    if (pmr->msta != MOTOR_STARTING) {
        pmr->msta = MOTOR_STARTING;
        MARK(M_MSTA);

        if (!(pmr->hsta & PWR_ACK_BIT)) {
            status = recordError (pmr, "Unexpected drive powerdown", S_ECS_HSK_SYNC);
            return (status);
        }

        writeHandshake (pmr, DRV_BIT | (pmr->hsta & DRV_ACK_BIT));
        setTimeout (pmr, ECS_PVOK_TMO); 
        return (S_ECS_OK);
    }

    /* Timeout means the device will not start */
    if (MARKED(M_TIMEOUT)) {
        status = recordError (pmr, "Device did not start in time", S_ECS_TIMEOUT);
        return status;
    }

    /* Wait for motor to start */
    if (!(pmr->hsta & DRV_ACK_BIT)) {
        return (S_ECS_OK); 
      }

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

    Debug(DBUG_FULL, "<%d> %s:movingState%c\n", ' ');

tempLog5 ("movingState %s: msta=%d, dmov=%d, stop=%d, hsta=%x\n", pmr->name, pmr->msta, pmr->dmov, pmr->stop, pmr->hsta);

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
    else if (!(pmr->hsta & DRV_ACK_BIT)) {
        status = stoppingState (pmr);
tempLog2 ("movingState %s, after stopping call: mode=%d\n", pmr->name, pmr->mode);
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
    ecsMotorPriv *pPriv = pmr->dpvt;
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%d> %s:stoppingState%c\n", ' ');

tempLog4 ("stoppingState %s: mode=%d, msta=%d, hsta=%x\n", pmr->name, pmr->mode, pmr->msta, pmr->hsta);

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
#if 0
        if (pmr->hsta & DRV_ACK_BIT) {	/* OLD */
#endif
        if ((pmr->hsta & DRV_BIT) || (pmr->hsta & DRV_ACK_BIT)) {
            status = writeHandshake (pmr, pmr->hsta & ~DRV_BIT);
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
    if (pmr->hsta & DRV_ACK_BIT) {
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




/*
 * Function depoweringState
 * 
 * A power-down request has been recived.  Shut off drive power.
 */

static long
depoweringState (struct ecsMotorRecord *pmr) {
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%d> %s:depoweringState%c\n", ' ');

tempLog3 ("depoweringState %s: msta=%d, hsta=%x\n", pmr->name, pmr->msta, pmr->hsta);

    /* Upon entry shut down the power. Make sure that the drive is disabled
     * as well. This could not be the case when the PLC disables the drive
     * internally because a limit switch was reached (PG 26/Jul/99).
     */ 
    if (pmr->msta != MOTOR_DEPOWERING) {
        pmr->msta = MOTOR_DEPOWERING;
        MARK(M_MSTA);
    
tempLog2 ("depoweringState %s: shutting power off %x\n", pmr->name,  pmr->hsta & ~PWR_BIT);

#if 0
        writeHandshake (pmr, pmr->hsta & ~PWR_BIT);	/* OLD */
#endif
        writeHandshake (pmr, pmr->hsta & (~DRV_BIT & ~PWR_BIT));
        setTimeout (pmr, ECS_PWR_TMO);
        return status;
    }

    /* Timeout means that power will not shut off */
    if (MARKED(M_TIMEOUT)) {
        status = recordError (pmr, "can not shut down power", S_ECS_FAULT);
        return status;
    }

    /* Wait for power to shut down */
    if (pmr->hsta) {
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

/*
 * Function failingState
 * 
 * Error handler
 */
static long
failingState (struct ecsMotorRecord *pmr) {
    ecsMotorPriv *pPriv = pmr->dpvt;
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%d> %s:failingState%c\n", ' ');

tempLog2 ("failingState %s: hsta=%x\n", pmr->name, pmr->hsta);

    /* shut down any move in progress */
    writeHandshake (pmr, pmr->hsta & ~DRV_BIT);

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
 * Function setTimeout
 * 
 * Start or adjust the process timeout counter.
 *
 */

static long
setTimeout (struct ecsMotorRecord *pmr, long time) {
    ecsMotorPriv *pPriv = pmr->dpvt;
    long status = S_ECS_OK;
    
    if (time != TIMEOUT_OFF) {
        Debug(DBUG_FULL, "<%d> %s:set timeout to %d ticks \n", time);
        pPriv->timeoutTimer = time;
        pPriv->timeoutActive = TRUE;
    }
    else {
        Debug(DBUG_FULL, "<%d> %s:cleared timeout%c\n", ' ');
        pPriv->timeoutActive = FALSE;
        }

     status = S_ECS_OK;

    return status;
    }

/*
 * Function writeHandshake
 * 
 * Write a new handshake pattern to the AB PLC.
 *
 */
static long 
writeHandshake (struct ecsMotorRecord *pmr, unsigned pattern) {
    ecsMotorPriv *pPriv = pmr->dpvt;
    long status = S_ECS_OK;

    Debug(DBUG_FULL, "<%d> %s:handshake word write %x \n", pattern);

tempLog3 ("writeHandshake %s: dmov=%d, pattern=%d\n", pmr->name, pmr->dmov, pattern);

    if (pmr->dmov) pattern |= IN_POS_BIT;
        
    status = drvAbDf1WriteAnalog (pPriv->pWriteHskPriv, pattern);
    if (status) {
        recordError (pmr, "ECS handshake write failure", status);    
    }
    return (status);
}


/*
 * Function checkFaultState
 * 
 * Look at the external FAULT line to see if it is safe to proceed.
 *
 */

static long 
recordError (struct ecsMotorRecord *pmr, char *msg, long status) {
    
  Debug(DBUG_FULL, "<%d> %s:recordError ..%s.. \n", msg);
  
  /* clear any timeouts */
  setTimeout (pmr, TIMEOUT_OFF);

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
 * Function checkFaultState
 * 
 * Look at the external FAULT line to see if it is safe to proceed.
 *
 */

static long
checkFaultState (struct ecsMotorRecord *pmr) {
    ecsMotorPriv *pPriv = pmr->dpvt;
    long switchMask, status = S_ECS_OK;

    Debug(DBUG_FULL, "<%d> %s:checkFaultState:entry%c\n", ' ');

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

#if 0
    /* check for data error during AB communications */
    if (MARKED(M_ERROR)) {
        status = recordError (pmr, "AB communications error", S_ECS_FAULT);
    }
#endif
    if (MARKED(M_ERROR)) {
	commErrCount++;		/* ignore and count errors */
    }
    
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
* Function moDevReadDoneAbDf1
*
* This callback function is invoked whenever the Allen Bradley interface
* detects a bit change.   Since activity on any channel invokes all
* callbacks  a check is made to see if our word has changed before
* requesting that the record be processed.
*/
static void
moDevReadDoneAbDf1 (absWordIO *pIO, long status)
{
    struct ecsMotorRecord *pmr = (struct ecsMotorRecord *) pIO->pPriv;
    ecsMotorPriv *pPriv = (ecsMotorPriv *) pmr->dpvt;
    unsigned rval;
    short inPosition;

    Debug(DBUG_MAX, "<%d> %s:moDevReadDoneAbDf1:entry%c\n", ' ');

    /* Trap AB read errors */
    if (status) {
      epicsMutexLock(pPriv->mutexSem);
      pPriv->callbackFlags |= DATA_ERROR;
      epicsMutexUnlock(pPriv->mutexSem);
      Debug(DBUG_MIN, "<%d> %s:moDevReadDoneAbDf1:invocation error %d\n", status);
      return;
    }

    /* Trap the first callback after initialization */            
    if (!pIO->oneIOReq) {
      pIO->oneIOReq = TRUE;
      epicsMutexLock(pPriv->mutexSem);
      pPriv->callbackFlags |= FIRST_SCAN;
      epicsMutexUnlock(pPriv->mutexSem);
      return;
   }

    /* Read the input word for this channel*/
    status = drvAbDf1ReadAnalog(pIO, &rval);                 
    if (status) {
      epicsMutexLock(pPriv->mutexSem);
      pPriv->callbackFlags |= DATA_ERROR;
      epicsMutexUnlock(pPriv->mutexSem);
      Debug(DBUG_MIN, "<%d> %s:moDevReadDoneAbDf1:read error %d\n", status);
      return;
    }

    /* Handle encoder value changes */
    if (pIO == (absWordIO *) pPriv->pEncoderPriv) {
        if (rval != pPriv->encoder) {
	  epicsMutexLock(pPriv->mutexSem);
	  pPriv->encoder = rval;
	  pPriv->callbackFlags |= DATA_READ;
	  epicsMutexUnlock(pPriv->mutexSem);
	  Debug(DBUG_MAX, "<%d> %s:Encoder word update %d \n", rval);
        }
    }
        
    /* Handle handshake bit changes */                
    else if (pIO == (absWordIO *) pPriv->pReadHskPriv) {    
        inPosition = ((rval & IN_POS_BIT) != 0);
        rval &= ~IN_POS_BIT;

         /* look for a change in the "in position" state */
        if (inPosition != pPriv->inPosition) {
	  epicsMutexLock(pPriv->mutexSem);
	  pPriv->inPosition = inPosition;
	  pPriv->callbackFlags |= DATA_READ;
	  epicsMutexUnlock(pPriv->mutexSem);
	  Debug(DBUG_FULL, "<%d> %s:In Position bit change %x \n", inPosition);
        }

        /* look for other changes in the handshake state */            
        if (rval != pPriv->handshake) { 
	  epicsMutexLock(pPriv->mutexSem);
	  pPriv->handshake = rval;
	  pPriv->callbackFlags |= DATA_READ;
	  epicsMutexUnlock(pPriv->mutexSem);
	  Debug(DBUG_FULL, "<%d> %s:Handshake word read %x \n", rval);
         }
       }

    else {
      epicsMutexLock(pPriv->mutexSem);
      pPriv->callbackFlags |= DATA_ERROR;
      epicsMutexUnlock(pPriv->mutexSem);
      Debug(DBUG_MIN, "<%d> %s:moDevReadDoneAbDf1:invalid channel callback %d\n", status);
    }
    
    return;
}

/*
* Function moDevWriteDoneAbDf1
*
* This function is called whenever the Allen Bradley interface
* writes a new handshake, position or velocity word.  Mark
* the handshaking field to trigger handshake processing.
*/
static void
moDevWriteDoneAbDf1 (absWordIO *pIO, long status)
{
    struct ecsMotorRecord *pmr = (struct ecsMotorRecord *) pIO->pPriv;
    ecsMotorPriv *pPriv = (ecsMotorPriv *) pmr->dpvt;

    Debug(DBUG_FULL, "<%d> %s:moDevWriteDoneAbDf1:entry%c\n", ' ');

    if (status) {
      Debug(DBUG_MIN, "<%d> %s:moDevWriteDoneAbDf1:AB error %d\n", status);
        pPriv->callbackFlags |= DATA_ERROR;
    }

    /* Indicate that the last write operation has completed and
    * therefore handshake processing action is required
    */
    epicsMutexLock(pPriv->mutexSem);
    pPriv->callbackFlags |= DATA_WRITE;
    epicsMutexUnlock(pPriv->mutexSem);
    Debug(DBUG_FULL, "<%d> %s:data write acknowledge\n", ' ');
           
    return;
}

/*
* Function moDevAbDf1ScanTask
*
* This function is spawned as a task to control
* the internal processing of ecsMotor records.   The
* device private structures are used directly to access
* the associated records.
*/
static int moDevAbDf1ScanTask() {
    ecsMotorPriv *pPriv;
    struct ecsMotorRecord *pmr = NULL;
    struct rset *pRset;
                                              
    while (TRUE) {
        /* For each record in the list .. */
        pPriv = (ecsMotorPriv *) ellFirst(&ecsMotorScanList);
        while (pPriv) {
	  pmr = (struct ecsMotorRecord *) pPriv->pmr;
	  pRset = (struct rset *) pmr->rset;
	  Debug(DBUG_MAX, "<%d> %s:scanTask:entry%c\n", ' ');
   
	  /* adjust the timeout timer */
	  if (pPriv->timeoutActive) {
	    pPriv->timeoutTimer -= 1;
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
	      pPriv->simPosition += (pPriv->simPosition < pPriv->simTarget) ? pPriv->simVelocity : -pPriv->simVelocity;
	      /* within one velocity step is close enough */
	      if (abs(pPriv->simTarget - pPriv->simPosition) < pPriv->simVelocity)
		pPriv->simPosition = pPriv->simTarget;
	      /* signal that position has changed */
	      pPriv->callbackFlags |= SIMM_UPDATE;
	    }
	  }
            
            /*
             * if an asynchronous callback has occurred since the
             * last pass force the record to process.   If the record is
             * busy ... no mattah' ... defer 'till next pass.
             */
            if (pPriv->callbackFlags) {
                           
                /* defer processing if the record is busy */                
                if (!pmr->pact) {

		  /* recover asynchronous info */
		  epicsMutexLock(pPriv->mutexSem);
		  if (pPriv->callbackFlags & DATA_WRITE) MARK(M_WRITE);
		  if (pPriv->callbackFlags & TIMEOUT_OVER) MARK(M_TIMEOUT);
		  if (pPriv->callbackFlags & DATA_ERROR) MARK(M_ERROR);
		  if (pmr->hsta != pPriv->handshake) {
		    pmr->hsta = pPriv->handshake;
		    MARK(M_HSTA);
		  }
		  if (pmr->rrbv != pPriv->encoder) {
		    pmr->rrbv = pPriv->encoder;
		    MARK(M_RRBV);
		  }
		  if (!pmr->simm && pmr->dmov != pPriv->inPosition) {
		    pmr->dmov = pPriv->inPosition;
		    MARK(M_DMOV);
		  }

		  /* clear flags and indicate that this is an internal processing */
		  pPriv->callbackFlags = 0;
		  MARK(M_RESCAN);
		  epicsMutexUnlock(pPriv->mutexSem);
                    
		  /* and then process the record */
		  Debug(DBUG_MAX, "<%d> %s:Scan Task re-processing record:%c\n",' ');
		  dbScanLock ((struct dbCommon *) pmr);
		  (*pRset->process) (pmr);
		  dbScanUnlock ((struct dbCommon *) pmr);		  
                }
		else {
                    Debug(DBUG_FULL, "<%d> %s:Scan Task .. record busy:%c\n",' ');
		}
            }
            pPriv = (ecsMotorPriv  *) ellNext (&pPriv->node);
        }
        taskDelay(sysClkRateGet()/ECS_SCAN_RATE);
    }
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
  ecsMotorPriv *pPriv = pmr->dpvt; 
  long status = S_ECS_OK;

  Debug(DBUG_FULL, "<%d> %s:processSimulation:entry%c\n", ' ');

  /* Ignore simulation requests if the device is being used */
  if (pmr->simm && 
      pmr->msta != MOTOR_IDLE &&
      pmr->msta != MOTOR_OFF &&
      pmr->msta != MOTOR_FAULT) {
    Debug(DBUG_MIN, "<%d> %s:device busy, ignoring simulation request%c\n", ' ');
    return (FALSE);
  }

  /* Ignore asynchronous processing requests if in simulation mode */
  if (pmr->simm && processType == PROCESS_BUTTON) {
    Debug(DBUG_MIN, "<%d> %s:device simulated, ignoring async request%c\n", ' ');
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
      if (!(pmr->hsta & PWR_ACK_BIT)) {
	writeHandshake (pmr, pmr->hsta & ~DRV_BIT);
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

void printErrCount ()
{
	double elapsed;
        epicsTimeStamp timeNow;

        epicsTimeGetCurrent(&timeNow);
        elapsed = epicsTimeDiffInSeconds(&timeNow, &commErrTime);

	printf ("%ld AB comm errors in %f seconds (%g hours) of operation\n",
		commErrCount, elapsed, elapsed / 3600.0);
}

/*********************************************************************
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

