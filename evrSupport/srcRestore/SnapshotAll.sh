#!/bin/bash
DATE=$(date +"%Y%m%d%H%M")

echo "Use: $0 ioc-name"
echo "e.g. $0 ioc-in20-ls01"
echo "Save the timing settings from this IOC that must be restored correctly"
echo "after a reboot. Values will be saved to a time-stamped file"

allIocs=$(ls $EPICS_IOCS)
#echo "# events"  > $1-event.cwConfig

    for anioc in $allIocs;
      do
	echo "Making EVR snapshot of " $anioc
        ./PreUpgradeSnapshot.sh $anioc
    done
