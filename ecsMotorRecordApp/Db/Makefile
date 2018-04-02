TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
# Find any .sym files in the Db directory
vpath %.sym ..
vpath %-edb.def ..
vpath %-tdct.cfg ..
vpath %.db $(SYS_DB_PATHS)

# Install database files created from .sch files
# using TDCT (or just install .db files) into <top>/db.
# TDCT_CFG can be redefined here if needed
#TDCT_CFG = ../tdct.cfg

# Make depends on all .sch files
#DEPSCHS = $(wildcard ../*.sch)

# Add target .db files below here. Each must
# correspond to a .sch file
# DB += xxx.db


TEMPLATES_DIR = schematics
TEMPLATES += ecsMotorControl.sch ecsMotorControl.sym



# If .sym files are in this support module,
# simply install them into <top>/db.
#DB += $(wildcard ../*.sym)
DB += ../eecsmotor.sym
DB += $(wildcard ../*-edb.def)
DB += $(wildcard ../*-tdct.cfg)
vpath %.db $(SYS_DB_PATHS)


include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

$(COMMON_DIR)/%.db: %.db
	cp $< $@



clean::myclean
myclean:
	$(RM) *.dat *.bak 

