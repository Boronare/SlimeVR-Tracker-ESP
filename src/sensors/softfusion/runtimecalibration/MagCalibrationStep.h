/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2026 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#pragma once

#include <array>
#include <cmath>
#include <optional>

#include "../../../GlobalVars.h"
#include "CalibrationStep.h"
#include "logging/Logger.h"
#include "magneto1.4.h"

namespace SlimeVR::Sensors::RuntimeCalibration {

template <typename SensorRawT>
class MagCalibrationStep : public CalibrationStep<SensorRawT> {
	using CalibrationStep<SensorRawT>::sensorConfig;
	using typename CalibrationStep<SensorRawT>::TickResult;

public:
	MagCalibrationStep(
		SlimeVR::Configuration::RuntimeCalibrationSensorConfig& sensorConfig,
		SlimeVR::Logging::Logger& logger
	)
		: CalibrationStep<SensorRawT>{sensorConfig}
		, logger{logger} {}

	void start() override final {
		CalibrationStep<SensorRawT>::start();
		calibrationData = CalibrationData{};
		calibrationData.value().startMillis = millis();
		statusManager.setStatus(SlimeVR::Status::MAG_CALIBRATING, true);

		logger.info(
			"Mag calibration started: rotate the tracker through as many orientations as possible"
		);
	}

	TickResult tick() override final {
		if (!calibrationData.has_value()) {
			return TickResult::CONTINUE;
		}

		const uint64_t elapsedMillis = millis() - calibrationData.value().startMillis;
		if (elapsedMillis > maxCalibrationTimeSeconds * 1e3
			&& calibrationData.value().storedSampleCount < minCalibrationSampleCount) {
			logger.warn(
				"Mag calibration timed out (%u/%u filtered samples)",
				static_cast<unsigned>(calibrationData.value().storedSampleCount),
				static_cast<unsigned>(minCalibrationSampleCount)
			);
			statusManager.setStatus(SlimeVR::Status::MAG_CALIBRATING, false);
			ledManager.pattern(60, 60, 3);
			return TickResult::SKIP;
		}

		if (calibrationData.value().storedSampleCount < minCalibrationSampleCount) {
			return TickResult::CONTINUE;
		}

		float M_BAinv[4][3];
		computeCalibration(M_BAinv);

		if (!isCalibrationMatrixSane(M_BAinv)) {
			if (elapsedMillis > maxCalibrationTimeSeconds * 1e3) {
				logger.warn("Mag calibration matrix sanity check failed");
				statusManager.setStatus(SlimeVR::Status::MAG_CALIBRATING, false);
				ledManager.pattern(60, 60, 3);
				return TickResult::SKIP;
			}
			return TickResult::CONTINUE;
		}

		float meanMagnitude = 0.0f;
		if (!isCalibrationQualityValid(M_BAinv, meanMagnitude)) {
			if (elapsedMillis > maxCalibrationTimeSeconds * 1e3) {
				logger.warn(
					"Mag calibration quality check failed after timeout (%u samples)",
					static_cast<unsigned>(calibrationData.value().storedSampleCount)
				);
				statusManager.setStatus(SlimeVR::Status::MAG_CALIBRATING, false);
				ledManager.pattern(60, 60, 3);
				return TickResult::SKIP;
			}
			return TickResult::CONTINUE;
		}

		for (uint8_t i = 0; i < 3; i++) {
			sensorConfig.M_B[i] = M_BAinv[0][i];
			sensorConfig.M_Ainv[0][i] = M_BAinv[1][i];
			sensorConfig.M_Ainv[1][i] = M_BAinv[2][i];
			sensorConfig.M_Ainv[2][i] = M_BAinv[3][i];
		}
		sensorConfig.M_refNorm = meanMagnitude;

		logger.info(
			"Mag calibration completed with %u filtered samples",
			static_cast<unsigned>(calibrationData.value().storedSampleCount)
		);
		statusManager.setStatus(SlimeVR::Status::MAG_CALIBRATING, false);
		ledManager.pattern(100, 100, 2);

		return TickResult::DONE;
	}

	void cancel() override final {
		calibrationData.reset();
		statusManager.setStatus(SlimeVR::Status::MAG_CALIBRATING, false);
	}
	bool requiresRest() override final { return false; }

	void processMagSample(const SensorRawT magSample[3]) override final {
		if (!calibrationData.has_value()) {
			return;
		}

		if (!shouldAcceptSample(magSample)) {
			return;
		}

		auto& data = calibrationData.value();
		data.samples[data.writeIndex] = {magSample[0], magSample[1], magSample[2]};
		data.writeIndex = (data.writeIndex + 1) % calibrationWindowSampleCount;
		if (data.storedSampleCount < calibrationWindowSampleCount) {
			data.storedSampleCount++;
		}
	}

private:
	static constexpr size_t calibrationWindowSampleCount = 240;
	static constexpr size_t minCalibrationSampleCount = 240;
	static constexpr uint8_t maxCalibrationTimeSeconds = 90;
	static constexpr int16_t minSampleDelta = 40;
	static constexpr int16_t maxRawAbsValue = 1500;
	static constexpr int16_t maxSpikeDelta = 2200;
	static constexpr float maxMagnitudeDeviation = 20.0f;

	bool shouldAcceptSample(const SensorRawT magSample[3]) {
		if (magSample[0] == 0 && magSample[1] == 0 && magSample[2] == 0) {
			return false;
		}
		if (std::abs(static_cast<int32_t>(magSample[0])) > maxRawAbsValue
			|| std::abs(static_cast<int32_t>(magSample[1])) > maxRawAbsValue
			|| std::abs(static_cast<int32_t>(magSample[2])) > maxRawAbsValue) {
			return false;
		}

		auto& data = calibrationData.value();
		if (!data.lastAcceptedSample.has_value()) {
			data.lastAcceptedSample = {magSample[0], magSample[1], magSample[2]};
			return true;
		}

		const auto& prev = data.lastAcceptedSample.value();
		const int32_t dx = std::abs(
			static_cast<int32_t>(magSample[0]) - static_cast<int32_t>(prev[0])
		);
		const int32_t dy = std::abs(
			static_cast<int32_t>(magSample[1]) - static_cast<int32_t>(prev[1])
		);
		const int32_t dz = std::abs(
			static_cast<int32_t>(magSample[2]) - static_cast<int32_t>(prev[2])
		);

		const bool changedEnough
			= dx > minSampleDelta || dy > minSampleDelta || dz > minSampleDelta;
		if (!changedEnough) {
			return false;
		}

		if (dx > maxSpikeDelta || dy > maxSpikeDelta || dz > maxSpikeDelta) {
			return false;
		}

		data.lastAcceptedSample = {magSample[0], magSample[1], magSample[2]};
		return true;
	}

	void computeCalibration(float outM_BAinv[4][3]) {
		auto& data = calibrationData.value();
		MagnetoCalibration magneto;
		for (size_t i = 0; i < data.storedSampleCount; i++) {
			magneto.sample(
				static_cast<double>(data.samples[i][0]),
				static_cast<double>(data.samples[i][1]),
				static_cast<double>(data.samples[i][2])
			);
		}
		magneto.current_calibration(outM_BAinv);
	}

	static float calibratedMagnitude(
		const std::array<SensorRawT, 3>& rawSample,
		const float M_BAinv[4][3]
	) {
		const float tx = static_cast<float>(rawSample[0]) - M_BAinv[0][0];
		const float ty = static_cast<float>(rawSample[1]) - M_BAinv[0][1];
		const float tz = static_cast<float>(rawSample[2]) - M_BAinv[0][2];

		const float x = M_BAinv[1][0] * tx + M_BAinv[1][1] * ty + M_BAinv[1][2] * tz;
		const float y = M_BAinv[2][0] * tx + M_BAinv[2][1] * ty + M_BAinv[2][2] * tz;
		const float z = M_BAinv[3][0] * tx + M_BAinv[3][1] * ty + M_BAinv[3][2] * tz;

		return std::sqrt(x * x + y * y + z * z);
	}

	static bool isCalibrationMatrixSane(const float M_BAinv[4][3]) {
		float diagonalAbs[3]{0.0f, 0.0f, 0.0f};
		for (size_t i = 0; i < 3; i++) {
			if (!std::isfinite(M_BAinv[0][i])) {
				return false;
			}
			for (size_t j = 0; j < 3; j++) {
				const float v = M_BAinv[i + 1][j];
				if (!std::isfinite(v) || std::abs(v) > 20.0f) {
					return false;
				}
			}
			diagonalAbs[i] = std::abs(M_BAinv[i + 1][i]);
		}

		const float meanDiagonal
			= (diagonalAbs[0] + diagonalAbs[1] + diagonalAbs[2]) / 3.0f;
		if (!std::isfinite(meanDiagonal) || meanDiagonal < 1e-4f
			|| meanDiagonal > 10.0f) {
			return false;
		}

		const float diagonalTolerance
			= (meanDiagonal * 0.6f > 0.05f) ? (meanDiagonal * 0.6f) : 0.05f;
		for (size_t i = 0; i < 3; i++) {
			if (std::abs(diagonalAbs[i] - meanDiagonal) > diagonalTolerance) {
				return false;
			}
		}

		return true;
	}

	bool isCalibrationQualityValid(
		const float M_BAinv[4][3],
		float& outMeanMagnitude
	) const {
		const auto& data = calibrationData.value();
		if (data.storedSampleCount < minCalibrationSampleCount) {
			return false;
		}

		float magnitudeSum = 0.0f;
		for (size_t i = 0; i < data.storedSampleCount; i++) {
			magnitudeSum += calibratedMagnitude(data.samples[i], M_BAinv);
		}
		const float meanMagnitude = magnitudeSum / static_cast<float>(data.storedSampleCount);
		if (!std::isfinite(meanMagnitude)) {
			return false;
		}
		outMeanMagnitude = meanMagnitude;

		size_t invalidCount = 0;
		for (size_t i = 0; i < data.storedSampleCount; i++) {
			const float magnitude = calibratedMagnitude(data.samples[i], M_BAinv);
			if (!std::isfinite(magnitude)
				|| std::abs(magnitude - meanMagnitude) > maxMagnitudeDeviation) {
				invalidCount++;
			}
		}

		return invalidCount < (data.storedSampleCount / 6);
	}

	struct CalibrationData {
		uint64_t startMillis = 0;
		size_t storedSampleCount = 0;
		size_t writeIndex = 0;
		std::array<std::array<SensorRawT, 3>, calibrationWindowSampleCount> samples{};
		std::optional<std::array<SensorRawT, 3>> lastAcceptedSample;
	};

	std::optional<CalibrationData> calibrationData;
	SlimeVR::Logging::Logger& logger;
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
