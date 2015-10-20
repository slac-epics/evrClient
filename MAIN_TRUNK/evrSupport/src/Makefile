TOP=../..

include $(TOP)/configure/CONFIG

#=============================
# build a library

# for gdb/ddd debugger:
USR_CFLAGS_solaris-sparc-gnu += -g
USR_CFLAGS_linux-x86 += -g
# for debug print statements and slc-awareness
USR_CFLAGS += -DDEBUG_PRINT
# Use EVR driver routines
USR_CFLAGS_RTEMS += -DEVR_DRIVER_SUPPORT

ifeq ($(EVR_DRIVER_SUPPORT_LINUX), YES)
USR_CFLAGS_Linux += -DEVR_DRIVER_SUPPORT

USR_CFLAGS_Linux += -DUSE_EVRMA
USR_CPPFLAGS += $(EVRMA_INCLUDE)
USR_CPPFLAGS += $(KERNEL_MODULE_EVRMA_INCLUDE)
evrma_DIR = $(EVRMA_LIB)
LIB_LIBS_Linux += evrma
evrSupport_LIBS += evrma


endif

SRC_DIRS += $(TOP)/evrApp/src

INC+=evrTime.h
INC+=evrPattern.h
INC+=evrMessage.h
INC+=bsa.h

DBDINC += longSubRecord
DBDINC += bsaRecord

DBD += evrSupport.dbd
DBD += evrEvInvariantDelay.dbd
DBD += evrEvDesc.dbd
DBD += evrPattern.dbd
DBD += evrModifier5.dbd
DBD += evrTime.dbd
DBD += bsa.dbd
DBD += drvEvr.dbd

LIBRARY_IOC = evrSupport

evrSupport_SRCS += evrEvInvariantDelay.c
evrSupport_SRCS += evrEvDesc.c
evrSupport_SRCS += evrPattern.c
evrSupport_SRCS += evrModifier5.c
evrSupport_SRCS += evrTime.c
evrSupport_SRCS += evrMessage.c
evrSupport_SRCS += bsa.c
evrSupport_SRCS += longSubRecord.c
evrSupport_SRCS += bsaRecord.c
evrSupport_SRCS += drvEvr.c
#evrSupport_LIBS += generalTime
evrSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES

#----------------------------------------
#  ADD RULES AFTER THIS LINE

