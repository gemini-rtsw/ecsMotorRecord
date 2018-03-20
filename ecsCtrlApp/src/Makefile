TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#=============================
# Build a support module called <APPNAME>

LIBRARY_IOC_RTEMS += ecsCtrl

DBDINC += ecsMotorRecord

# <APPNAME>.dbd will be created and installed
DBD += ecsCtrl.dbd

# ecsCtrl.dbd will be made up from these files:
ecsCtrl_DBD += base.dbd
#ecsCtrl_DBD += devEcsMotor.dbd
ecsCtrl_DBD += ecsMotorRecord.dbd

INC += ecsMotor.h
INC += choiceEcsMotor.h
INC += ecsMotorFields.h
INC += ecsABinterface.h
INC += devEcsAbDf1.h
 

# List local .dbd files here
# ecsCtrl_DBD += xxx.dbd

# Include dbd files for external support modules:
ecsCtrl_DBD += AbDf1.dbd

# specify all source files to be compiled
# including sequencer (.st) source files
ecsCtrl_SRCS += ecsMotorRecord.c
ecsCtrl_SRCS += devEcsAbDf1.c
#ecsCtrl_SRCS += xxx.st

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

