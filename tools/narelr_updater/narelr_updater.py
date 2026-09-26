# -*- coding: utf-8 -*-
"""NareLR updater: flash the bundled Narae LR firmware over the tracker's USB (CH340).

Built by build_updater.bat into NareLR<YYMMDD>_updater.exe with the firmware
inside.  Writes the application at 0x0 only -- no full erase -- so the Wi-Fi
settings stay.  Calibration from an older firmware may not carry over; the
tracker then runs its factory boot (magnetometer check, gyro calibration):
leave it still for about 15 s after the update.
"""
import os
import sys
import time

import esptool
import serial.tools.list_ports as list_ports

VERSION = "@VERSION@"  # filled in by build_updater.bat

# USB-serial bridges these boards use: CH340, CH9102, CP210x
USB_SERIAL_IDS = {(0x1A86, 0x7523), (0x1A86, 0x55D4), (0x10C4, 0xEA60)}


def resource(name):
    base = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base, name)


def find_ports():
    return [p for p in list_ports.comports() if (p.vid, p.pid) in USB_SERIAL_IDS]


def pick_port():
    shown = False
    while True:
        ports = find_ports()
        if len(ports) == 1:
            return ports[0].device
        if len(ports) > 1:
            print("트래커가 여러 개 연결돼 있습니다:")
            for i, p in enumerate(ports, 1):
                print("  %d) %s  %s" % (i, p.device, p.description))
            while True:
                s = input("업데이트할 번호를 입력하세요: ").strip()
                if s.isdigit() and 1 <= int(s) <= len(ports):
                    return ports[int(s) - 1].device
        if not shown:
            print("트래커를 USB로 연결해 주세요... (종료: Ctrl+C)")
            shown = True
        time.sleep(1.0)


def flash(port, fw, baud):
    esptool.main([
        "--chip", "esp8266", "--port", port, "--baud", str(baud),
        "--before", "default_reset", "--after", "hard_reset",
        "write_flash", "0x0", fw,
    ])


def main():
    print("나래트래커 LR 펌웨어 업데이트  (%s)" % VERSION)
    print("-" * 48)
    fw = resource("firmware.bin")
    if not os.path.exists(fw):
        print("펌웨어 파일이 exe 안에 없습니다. 다시 빌드해 주세요.")
        return 1
    if "--check" in sys.argv:
        # Build check: what would be written and where, without writing.
        print("펌웨어 %d 바이트, esptool %s" % (os.path.getsize(fw), esptool.__version__))
        print("연결된 포트: %s" % (", ".join(p.device for p in find_ports()) or "없음"))
        return 0
    port = pick_port()
    print("포트 %s 에 %s 를 씁니다. 끝날 때까지 케이블을 뽑지 마세요.\n" % (port, VERSION))
    for baud in (921600, 115200):
        try:
            flash(port, fw, baud)
            print("\n완료했습니다. 트래커가 다시 시작됩니다.")
            print("보정값이 없으면 처음 약 15초 동안 검사와 자이로 보정을 하니 가만히 두세요.")
            print("LED가 쉬지 않고 빠르게 깜빡이면 자력계 불량입니다.")
            return 0
        except Exception as e:  # noqa: BLE001 - report and retry slower
            print("\n%d bps로 실패했습니다: %s" % (baud, e))
            if baud != 115200:
                print("115200 bps로 다시 시도합니다...\n")
                time.sleep(1.0)
    print("\n업데이트에 실패했습니다.")
    print("- 케이블을 다시 꽂고 한 번 더 실행해 보세요.")
    print("- 장치 관리자에서 포트 드라이버가 WCH(CH340) 드라이버인지 확인하세요.")
    return 2


if __name__ == "__main__":
    # A console that cannot show a character must not stop the update.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    try:
        code = main()
    except KeyboardInterrupt:
        code = 130
    if "--check" not in sys.argv:
        input("\n엔터를 누르면 창이 닫힙니다.")
    sys.exit(code)
