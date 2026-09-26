#include "Status.h"

namespace SlimeVR::Status {
const char* statusToString(Status status) {
	switch (status) {
		case LOADING:
			return "LOADING";
		case LOW_BATTERY:
			return "LOW_BATTERY";
		case IMU_ERROR:
			return "IMU_ERROR";
		case WIFI_CONNECTING:
			return "WIFI_CONNECTING";
		case SERVER_CONNECTING:
			return "SERVER_CONNECTING";
		case MAG_CALIBRATING:
			return "MAG_CALIBRATING";
		case MAG_FAULT:
			return "MAG_FAULT";
		case CALIBRATION_SIGNAL:
			return "CALIBRATION_SIGNAL";
		case CALIBRATING:
			return "CALIBRATING";
		case CALIBRATION_CONFIRM:
			return "CALIBRATION_CONFIRM";
		default:
			return "UNKNOWN";
	}
}
}  // namespace SlimeVR::Status
