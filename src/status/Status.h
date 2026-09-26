#ifndef STATUS_STATUS_H
#define STATUS_STATUS_H

namespace SlimeVR::Status {
enum Status {
	LOADING = 1 << 0,
	LOW_BATTERY = 1 << 1,
	IMU_ERROR = 1 << 2,
	WIFI_CONNECTING = 1 << 3,
	SERVER_CONNECTING = 1 << 4,
	MAG_CALIBRATING = 1 << 5,
	MAG_FAULT = 1 << 6,
	CALIBRATION_SIGNAL = 1 << 7,  // three quick blinks: a calibration starts / ends
	CALIBRATING = 1 << 8,  // LED steadily on while it runs
	CALIBRATION_CONFIRM = 1 << 9  // one blink a second: keep it so to calibrate
};

const char* statusToString(Status status);
}  // namespace SlimeVR::Status

#endif
