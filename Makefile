#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG

DIRS += configure
DIRS += evrApp
DIRS += evrSupport
#DIRS += testIoc

include $(TOP)/configure/RULES_TOP
