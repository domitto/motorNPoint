TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = nPoint
# nPoint.dbd will be created and installed
DBD += nPoint.dbd

# nPoint.dbd will be made up from these files:
nPoint_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
nPoint_DBD += asyn.dbd
nPoint_DBD += drvAsynSerialPort.dbd
#endif
#ifdef FTDI
nPoint_DBD += drvAsynFTDIPort.dbd
#endif

nPoint_DBD += motorSupport.dbd
nPoint_DBD += NPointMotorSupport.dbd
nPoint_DBD += LC400MotorSupport.dbd

# Add all the support libraries needed by this IOC
nPoint_LIBS += NPointMotor
nPoint_LIBS += motor
#ifdef ASYN
nPoint_LIBS += asyn
#endif
#ifdef SNCSEQ
nPoint_LIBS += seq pv
#endif

# nPoint_registerRecordDeviceDriver.cpp derives from nPoint.dbd
nPoint_SRCS += nPoint_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
nPoint_SRCS_DEFAULT += nPointMain.cpp
nPoint_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#nPoint_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
nPoint_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

