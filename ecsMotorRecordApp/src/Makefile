TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#=============================
# Build a support module called <APPNAME>


DBDINC += ecsMotorRecord

DBD += ecsMotorRecordApp.dbd   # Needed for TDCT
ecsMotorRecordApp_DBD += ecsMotorRecord.dbd

LIBRARY_IOC += ecsMotorRecord

# specify all source files to be compiled
# including sequencer (.st) source files
ecsMotorRecord_SRCS += ecsMotorRecord.c

ecsMotorRecord_LIBS += $(EPICS_BASE_IOC_LIBS)

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

