# Firmware version for the Naremotion LR board: NareLR<YYMMDD> of the build day.
# Used from platformio.ini as `!python scripts/narelr_version.py`; comes after
# get_git_commit.py's FIRMWARE_VERSION, so it is the one that sticks.
import datetime

print("-DFIRMWARE_VERSION='\"NareLR%s\"'" % datetime.date.today().strftime("%y%m%d"))
