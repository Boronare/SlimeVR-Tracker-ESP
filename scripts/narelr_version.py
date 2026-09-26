# Firmware version for the Naremotion LR board: NareLR<YYMMDD> of the build day.
# Used from platformio.ini as `!python scripts/narelr_version.py`; comes after
# get_git_commit.py's FIRMWARE_VERSION, so it is the one that sticks.
# `--name` prints just the name (tools/narelr_updater/build_updater.bat).
import datetime
import sys

name = "NareLR%s" % datetime.date.today().strftime("%y%m%d")
if "--name" in sys.argv:
    print(name)
else:
    print("-DFIRMWARE_VERSION='\"%s\"'" % name)
