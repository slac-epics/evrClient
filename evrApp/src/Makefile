TOP = ../..
include $(TOP)/configure/CONFIG

#------------------------------------------------------------------------------
# $(TIMING)/evrApp/src/Makefile
#
# Author:  Ernest Williams
# Date:    17 October 2016
#
#------------------------------------------------------------------------------
# This file contains the EPICS make rules and definitions for making and
# installing the driver support modules for the Micro-Research Finland
# Series 230, 300, and SLAC-PCIe-EVR event receiver modules.
#
# These drivers were modified to integrate in with CosyLab's: 
# Event receiver userspace library and kernel module. 
# This package now supports event module sharing.
#------------------------------------------------------------------------------
# Modification History:
#
# 17 October 2016   ELW     Original Release
#
#------------------------------------------------------------------------------

#--------------------------------
# Set up Compiler Flags
#--------------------------------

#USR_CFLAGS += -pedantic
USR_CFLAGS += -DDEBUG_PRINT

USR_CFLAGS += -DUSE_EVRMA
USR_CPPFLAGS += $(EVRMA_INCLUDE)
USR_CPPFLAGS += $(KERNEL_MODULE_EVRMA_INCLUDE)

#-----------------------
# Install include files
#-----------------------
INC += mrfCommon.h
INC += debugPrint.h
INC += devMrfEr.h
INC += drvMrfEr.h

#-----------------------------------
# Build the record support modules
# for the Event Receiver Only
# ----------------------------------
CUSTOM_RECORDS += erRecord
CUSTOM_RECORDS += ereventRecord

DBDINC += $(CUSTOM_RECORDS)

#-----------------------------
# Install DBD files
# for the Event Receiver Only
# ----------------------------
#
DBD += drvMrfEr.dbd
DBD += devMrfEr.dbd
devMrfErLib_DBD += base.dbd
devMrfErLib_DBD += devMrfEr.dbd

# ============================================
# Building EPICS Device Support Library
# ============================================
LIBRARY_IOC_Linux += devMrfEr

devMrfEr_OBJS += devEvrma

devMrfEr_OBJS += erRecord
devMrfEr_OBJS += ereventRecord

devMrfErLib_OBJS += $(devMrfEr_OBJS)
devMrfErLib_SRCS += devMrfErLib_registerRecordDeviceDriver.cpp

# ============================================
# Building EPICS Driver Support Library
# ============================================
#
LIBRARY_IOC_Linux += drvMrf
drvMrf_OBJS_Linux += evrIrqHandlerThread
evrma_DIR = $(EVRMA_LIB)
LIB_LIBS_Linux += evrma

### can this be for EVRMA, too?
#drvMrf_OBJS_Linux += drvLinux


#=======================================================================#
#                               RULES                                   #
#=======================================================================#

#---------------------
# Generic EPICS build rules
#
include $(TOP)/configure/RULES
 
