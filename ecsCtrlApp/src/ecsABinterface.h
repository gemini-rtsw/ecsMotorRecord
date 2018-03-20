/* ecsABinterface.h 
 * Include file for all Allen Bradley related interface
 * definitions
 */

#include <devEcsAbDf1.h>


/* Offset from handshake word to locate position word */
#define ECS_POS_OFFSET  1

/* 
 * Timeouts associated with PLC handshake transitions. 
 * Timeouts expressed in number of seconds before timeout error generated. 
*/

#define TIMEOUT_OFF    	0                      /* cancel the tiemout timer */
#define ECS_PWR_TMO    	30 * ECS_SCAN_RATE     /* max PWR on/off to PWR_ACK on/off */
#define ECS_WRITE_TMO  	5 * ECS_SCAN_RATE      /* max time for value write callback */
#define ECS_PVOK_TMO   	5 * ECS_SCAN_RATE      /* max time NEW_POS/VEL to POS/VEL_ACK */
#define ECS_STOP_TMO   	10 * ECS_SCAN_RATE     /* max time DRV off to DRV_ACK off */
#define ECS_START_TMO  	10 * ECS_SCAN_RATE     /* max time DRV on to DRV_ACK on */
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

