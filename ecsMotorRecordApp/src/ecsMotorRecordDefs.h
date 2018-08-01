/* ecsMotor.h */
/* Interface definitions for ecsMotorRecord support */

#ifndef ECSMOTOR_H
#define ECSMOTOR_H

/* Simulation Levels */
#define SIMM_NONE 0   /* system operates normally                           */
#define SIMM_VSM  1   /* virtual, command parse but no actions              */
#define SIMM_FAST 2   /* command parse with actions but not realistic times */
#define SIMM_FULL 3   /* as FAST but with realistic response times          */

/* Define the error state return codes */
#define S_ECS_OK		0
#define S_ECS_STATE_FAULT	1
#define S_ECS_HSK_SYNC		2
#define S_ECS_TIMEOUT		3 
#define S_ECS_FAULT		4
#define S_ECS_VALUE_REJECT	5 
#define S_ECS_BAD_INPUT		6
#define S_ECS_SIM_RESET         7  

/* Definitions for the MIP field */
#define MIP_STOPPED    		0
#define MIP_STOPPING		1
#define MIP_MOVING     		2
#define MIP_PARKING         	3
#define MIP_OFFLINE    		4
#define MIP_ERROR      		5
 
/* Definitions for the MSTA field */                   
#define MOTOR_OFF       	0
#define MOTOR_POWERING		1
#define MOTOR_IDLE	       	2
#define MOTOR_WRITING           3
#define MOTOR_CHECKING		4
#define MOTOR_STARTING		5
#define MOTOR_MOVING		6
#define MOTOR_STOPPING		7
#define MOTOR_DEPOWERING       	8
#define MOTOR_FAILING		9 
#define MOTOR_FAULT	       	10

/* Definitions for the FLT field */
#define OPEN_SLOW_LIMIT   	        0x0001
#define OPEN_EOT_LIMIT 		        0x0002
#define OPEN_OVER_LIMIT		        0x0004
#define CLOSE_SLOW_LIMIT   	        0x0008
#define CLOSE_EOT_LIMIT 		0x0010
#define CLOSE_OVER_LIMIT		0x0020
#define DEVICE_WARNING		        0x0040
#define DEVICE_FAILURE 	  	        0x0080
#define ENCODER_FAULT	  	        0x0100
#define PLC_OFFLINE		        0x0200
#define SYSTEM_DEAD   		        0x0400
#define INTERLOCK                       0x0800
#define SKEWING				0x1000

/* Definitions for the HLS/LLS field */
#define ECS_NO_LSW                 0
#define ECS_SLOW_LSW               1
#define ECS_END_LSW                2
#define ECS_OVER_LSW     	   3
 
/* Background task scan rate in scans/second */
#define ECS_SCAN_RATE		5	/* 0.2 seconds per scan */

/* Define the internal simulation mode velocity (units/second) */
#define ECS_SIM_VEL		1.0

/* Processing type definitions */
#define PROCESS_NORMAL          0
#define PROCESS_INTERNAL        1
#define PROCESS_BUTTON          2

/* Define the callback activity flags */
#define DATA_READ		0x01
#define DATA_WRITE	       	0x02
#define TIMEOUT_OVER		0x04
#define DATA_ERROR		0x08
#define FIRST_SCAN	       	0x10
#define SIMM_UPDATE             0x20

/* System operating modes */
#define NORMAL_MODE             0
#define DEFERRED_MODE           1

/* internal flags for async communications */
#define M_ERROR         	0x10000000
#define M_WRITE         	0x20000000
#define M_TIMEOUT       	0x40000000
#define M_RESCAN        	0x80000000

/*******************************************************************************
Support for keeping track of which record fields have been changed, so we can
eliminate redundant db_post_events() without having to think, and without having
to keep lots of "last value of field xxx" fields in the record.  The idea is
to say...

	MARK(M_XXXX);

when you mean...

	db_post_events(pmr, &pmr->xxxx, monitor_mask);

Before leaving, you have to call post_MARKed_fields() to actually post the
field to all listeners.  monitor() does this.

	--- NOTE WELL ---
	The macros below assume that the variable "pmr" exists and points to a
	motor record, like so:
		struct ecsMotorRecord *pmr;
	No check is made in this code to ensure that this really is true.
*******************************************************************************/
#define M_VAL			0x00000001
#define M_VELO			0x00000002
#define M_MODE			0x00000004
#define M_SIMM			0x00000008
#define M_LLS			0x00000010
#define M_HLS			0x00000020
#define M_FLT			0x00000040
#define M_MESS			0x00000080
#define M_RRBV          	0x00000100
#define M_MPOS          	0x00000200
#define M_MSTA			0x00000400
#define M_HSTA			0x00000800
#define M_DMOV			0x00001000
#define M_MOVN			0x00002000
#define M_MIP			0x00004000
#define M_TDIR			0x00008000
#define M_HLM			0x00010000
#define M_LLM			0x00020000
#define M_LVIO			0x00040000
#define M_DSTA          	0x00080000
#define M_STOP          	0x00100000
#define M_MDBD          	0x00200000
#define M_DBUG         	        0x00400000
#define M_RPOS         	        0x00800000

/* funky macros to handle the above */
#define MARK(a)	                pmr->mmap |= (a);
#define MARKED(a)		pmr->mmap&(a)
#define UNMARK(a)		pmr->mmap &= ~(a);
#define UNMARK_ALL		pmr->mmap = 0;

/*
 * const fileds are initialized by drvAbDf1 - otherwise they
 * are initialized by device support.
 * (here const is used to document privacy)
 *
 * NOTE:
 * Outputs transactions occur on demand. Input transactions occur
 * at a rate determined by the driver because it is more efficient
 * to scan multiple input signals in one DF1 block IO transaction.
 * Because of this the asynch IO callback is called for outputs 
 * when the associated transaction completes and is called for
 * inputs unsolicited _every_ time that an DF1 block IO read 
 * transaction completes.
 */


/*  
 * Allen Bradley related interface  definitions
 */


/* 
 * Timeouts associated with PLC handshake transitions. 
 * Timeouts expressed in number of seconds before timeout error generated. 
*/

#define TIMEOUT_OFF    	0                      /* cancel the tiemout timer */
//#define ECS_PWR_TMO    	30 * ECS_SCAN_RATE     /* max PWR on/off to PWR_ACK on/off this is part of BFO*/
#define ECS_WRITE_TMO  	20 * ECS_SCAN_RATE     /* max time for value write callback */
#define ECS_PVOK_TMO   	40 * ECS_SCAN_RATE     /* max time NEW_POS/VEL to POS/VEL_ACK */
#define ECS_STOP_TMO   	20 * ECS_SCAN_RATE     /* max time DRV off to DRV_ACK off */
#define ECS_START_TMO  	40 * ECS_SCAN_RATE     /* max time DRV on to DRV_ACK on */
#define ECS_SIM_TMO     ECS_SCAN_RATE / 2      /* VSM mode busy time */

/* AB handshake word bit definitions for the ECS controller. 
 * Note that these are two sets, one for devices that support 
 * position mode only and one for devices that support both  
 * position and velocity modes.
 * This is totally stupid but is required by the Allen Bradley
 * code written for the Gemini enclosure control system!
*/

#define PV_PWR_ACK     	0x0001    /* drive power status bit */
#define PV_VEL_ACK     	0x0002    /* velocity word acknowledge bit */
#define PV_DRV_ACK     	0x0004    /* drive enable status bit */
#define PV_POS_ACK     	0x0008    /* position word acknowledge bit */
#define PV_IN_POS      	0x0010    /* device at target position status bit */
#define PV_PWR		0x0020    /* device power control bit */
#define PV_NEW_POS     	0x0040    /* position word valid bit */
#define PV_DRV		0x0080    /* device drive control bit */
#define PV_NEW_VEL     	0x0100    /* velocity word valid bit */

#define P_PWR_ACK     	0x0001    /* drive power status bit */
#define P_DRV_ACK     	0x0002    /* drive enable status bit */
#define P_POS_ACK     	0x0004    /* position word acknowledge bit */
#define P_IN_POS     	0x0008    /* device at target position status bit */
#define P_PWR     	0x0010    /* device power control bit */
#define P_DRV		0x0020    /* device drive control bit */
#define P_NEW_POS     	0x0040    /* velocity word valid bit */ 
 
/* If having two handshake maps is stupid then the following macros are even more
 * so!   Unfortunatly it is the best way to hide this stuff from the 
 * person trying to understand how the device support code works. 
 * Note that these macros assume that there has been declared a pointer
 * to an ECS record structure named "pmr" as follows:
 *		struct ecsMotorRecord *pmr
 * and that this pointer points to the record being processed!
 */
#define PWR_BIT		((pmr->vms) ? PV_PWR : P_PWR)
#define PWR_ACK_BIT	((pmr->vms) ? PV_PWR_ACK : P_PWR_ACK)
#define DRV_BIT		((pmr->vms) ? PV_DRV : P_DRV)
#define DRV_ACK_BIT	((pmr->vms) ? PV_DRV_ACK : P_DRV_ACK)
#define NEW_POS_BIT	((pmr->vms) ? PV_NEW_POS : P_NEW_POS)
#define POS_ACK_BIT	((pmr->vms) ? PV_POS_ACK : P_POS_ACK)
#define IN_POS_BIT	((pmr->vms) ? PV_IN_POS : P_IN_POS)
#define PV_WRITE_BIT	PV_WRITE
#define NEW_VEL_BIT	PV_NEW_VEL
#define VEL_ACK_BIT	PV_VEL_ACK


#endif
