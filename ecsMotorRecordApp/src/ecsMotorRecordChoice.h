/*  choiceEcsMotor.h */

/*
 *      Author:
 *      Date:
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *      Initial development by:
 *              The Controls and Automation Group (AT-8)
 *              Ground Test Accelerator
 *              Accelerator Technology Division
 *              Los Alamos National Laboratory
 *
 *      Co-developed with
 *              The Controls and Computing Group
 *              Accelerator Systems Division
 *              Advanced Photon Source
 *              Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01  97-10-07	wnr	Initial Definition
 */

#ifndef INCchoiceEcsMotorh
#define INCchoiceEcsMotorh 1
#define REC_ECSMOTOR_MODE	0
#define REC_ECSMOTOR_SIM	1
#define REC_ECSMOTOR_DBUG	2
/* for record-support use */
#define MODE_STOP	0
#define MODE_PARK	1
#define MODE_MOVE	2
#define MODE_RESET	3
#define MODE_PAUSE	4
#define MODE_RESUME	5
#define MODE_VMOVE	6
#define SIM_NONE	0
#define SIM_VSM		1
#define SIM_FAST	2
#define SIM_FULL	3
#define DBUG_NONE	0
#define	DBUG_MIN	1
#define	DBUG_FULL	2
#define DBUG_MAX        3
#endif

