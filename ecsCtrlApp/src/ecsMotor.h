/* ecsMotor.h */
/* Interface definitions for ecsMotorRecord device support */

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
 

