TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += NPointMotor

ifeq ($(DRV_FTDI),YES)
  SRC_DIRS += drvAsynFTDI
  NPointMotor_SRCS += ftdiDriver.cpp drvAsynFTDIPort.cpp
  NPointMotor_SYS_LIBS += usb-1.0
  INC += drvAsynFTDIPort.h
  ifeq ($(DRV_FTDI_USE_LIBFTDI1),YES)
    NPointMotor_SYS_LIBS += ftdi1
    USR_CXXFLAGS += -DHAVE_LIBFTDI1
  else
    NPointMotor_SYS_LIBS += ftdi
  endif
  DBD += drvAsynFTDIPort.dbd
endif


# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += NPointMotorSupport.dbd
DBD += LC400MotorSupport.dbd

# The following are compiled and added to the Support library
NPointMotor_SRCS += C300MotorDriver.cpp
NPointMotor_SRCS += LC400MotorDriver.cpp

NPointMotor_LIBS += motor
NPointMotor_LIBS += asyn
NPointMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

