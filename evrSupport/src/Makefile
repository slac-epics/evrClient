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
ifeq ($(HAVE_BSA_CORE),YES)
LIBRARY_IOC += evrSupportLegacy
LIBRARY_IOC += eventBsaSupport
endif

EVR_SUP_COM_SRCS += evrEvInvariantDelay.c
EVR_SUP_COM_SRCS += evrEvDesc.c
EVR_SUP_COM_SRCS += evrPattern.c
EVR_SUP_COM_SRCS += evrModifier5.c
EVR_SUP_COM_SRCS += evrTime.c
EVR_SUP_COM_SRCS += evrMessage.c
EVR_SUP_COM_SRCS += longSubRecord.c
EVR_SUP_COM_SRCS += bsaRecord.c
EVR_SUP_COM_SRCS += drvEvr.c


evrSupport_SRCS       += $(EVR_SUP_COM_SRCS)

ifeq ($(HAVE_BSA_CORE),YES)
evrSupportLegacy_SRCS += $(EVR_SUP_COM_SRCS)
evrSupportLegacy_SRCS += bsa.c
evrSupportLegacy_LIBS += $(EPICS_BASE_IOC_LIBS)
evrSupport_SRCS       += devBsaCore.c
evrSupport_SRCS       += devAoBsaCore.c

eventBsaSupport_SRCS  += bsaRecord.c
eventBsaSupport_SRCS  += devBsaCore.c
eventBsaSupport_SRCS  += devAoBsaCore.c
eventBsaSupport_LIBS  += $(EPICS_BASE_IOC_LIBS)
else
evrSupport_SRCS       += bsa.c
endif


#evrSupportLegacy_LIBS += generalTime
evrSupport_LIBS       += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES

#----------------------------------------
#  ADD RULES AFTER THIS LINE

