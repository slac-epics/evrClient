TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
#

# Rules for installing ChannelWatcher files:
CFG +=  CONFIG_RESTORE

RESTORES += evrDevTrig.cwConfig
RESTORES += evrIocTrig.cwConfig
RESTORES += evrPMXXTrig.cwConfig
RESTORES += evrTrig.req.template


include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

