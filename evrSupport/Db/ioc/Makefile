TOP=../../..
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
# Device Simulation
DB += bsaBPMSTest.db
DB += bsaTOROTest.db
DB += bsaBLENTest.db
DB += bsaWIRETest.db
DB += bsaPMTTest.db
DB += bsaFARCTest.db

# Device edef records
DB += bsaGENEdef.db

DB += bsaBPMSEdef.db
DB += bsaBPMSCavityEdef.db
DB += bsaTOROEdef.db
DB += bsaFARCEdef.db
DB += bsaBLENEdef.db
DB += bsaPMTEdef.db
DB += bsaWIREEdef.db
DB += bsaAMPLEdef.db
DB += bsaPHASEdef.db
DB += bsaPWREdef.db
DB += bsaEnergyEdef.db

# Files for easier IOC deployment, with reasonable defaults
# for each system when $SYS is defined
DB += Bsa.db
DB += Pattern.db
DB += VME-trig.db
DB += VME-TTB-trig.db
DB += PMC-trig.db
DB += PCI-trig.db
DB += EvrPmc.db
DB += EvrPci.db

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

