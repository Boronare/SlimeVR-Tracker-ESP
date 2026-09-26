/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

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

#include <vector3.h>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "../../../GlobalVars.h"
#include "../../../configuration/Configuration.h"
#include "AccelBiasCalibrationStep.h"
#include "GyroBiasCalibrationStep.h"
#include "MagCalibrationStep.h"
#include "MotionlessCalibrationStep.h"
#include "NullCalibrationStep.h"
#include "SampleRateCalibrationStep.h"
#include "configuration/SensorConfig.h"
#include "logging/Logger.h"
#include "sensors/SensorFusion.h"
#include "sensors/softfusion/CalibrationBase.h"

namespace SlimeVR::Sensors::RuntimeCalibration {

template <typename IMU>
class RuntimeCalibrator : public Sensors::CalibrationBase<IMU> {
public:
	static constexpr bool HasUpsideDownCalibration = false;

	using Base = Sensors::CalibrationBase<IMU>;
	using Self = RuntimeCalibrator<IMU>;
	using Consts = typename Base::Consts;
	using RawSensorT = typename Consts::RawSensorT;
	using RawVectorT = typename Consts::RawVectorT;

	RuntimeCalibrator(
		SensorFusion& fusion,
		IMU& imu,
		uint8_t sensorId,
		Logging::Logger& logger,
		SensorToggleState& toggles
	)
		: Base{fusion, imu, sensorId, logger, toggles}
		, magCalibrationStep{calibration, logger} {
		calibration.T_Ts = Consts::getDefaultTempTs();
		activeCalibration.T_Ts = Consts::getDefaultTempTs();
	}

	bool calibrationMatches(const Configuration::SensorConfig& sensorCalibration
	) final {
		return sensorCalibration.type
				== SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION
			&& (sensorCalibration.data.sfusion.ImuType == IMU::Type)
			&& (sensorCalibration.data.sfusion.MotionlessDataLen
				== Base::MotionlessCalibDataSize());
	}

	void assignCalibration(const Configuration::SensorConfig& sensorCalibration) final {
		calibration = sensorCalibration.data.runtimeCalibration;
		activeCalibration = sensorCalibration.data.runtimeCalibration;
		if (!toggles.getToggle(SensorToggles::CalibrationEnabled)) {
			activeCalibration.gyroPointsCalibrated = 0;
			for (size_t i = 0; i < 3; i++) {
				activeCalibration.G_off1[i] = 0;
				activeCalibration.G_off2[i] = 0;
			}

			for (size_t i = 0; i < 3; i++) {
				activeCalibration.accelCalibrated[i] = false;
				activeCalibration.A_off[i] = 0;
			}
		} else {
			calculateZROChange();
		}

		{
			if (!isMagCalibrationMissing(calibration)
				&& !isMagCalibrationValid(calibration)) {
				logger.warn("Invalid mag calibration data found, clearing it");
				resetMagCalibration(calibration);
			}

			if (!isMagNormReferenceValid(calibration.M_refNorm)) {
				calibration.M_refNorm = 0.0f;
			}

			syncMagCalibrationToActive();

			const bool isMagCalibrationReadya = isMagCalibrationReady();
			pendingMagCalibrationRequest
				= shouldRequestMagCalibration(isMagCalibrationReadya);
			if (!isMagCalibrationReadya) {
				fusion.setMagFusionEnabled(false);
			}
		}

		currentStep = &nullCalibrationStep;
	}

	void begin() final {
		startupMillis = millis();

		gyroBiasCalibrationStep.swapCalibrationIfNecessary();

		currentStep = &sampleRateCalibrationStep;
		currentStep->start();
		nextCalibrationStep = CalibrationStepEnum::SAMPLING_RATE;

		calculateZROChange();
		syncMagCalibrationToActive();

		printCalibration();
	}

	void tick() final {
		maybeStartMagCalibration();
		if (!isMagCalibrationReady()) {
			fusion.setMagFusionEnabled(false);
		}

		if (skippedAStep && !lastTickRest && fusion.getRestDetected()) {
			computeNextCalibrationStep();
			skippedAStep = false;
		}

		if (millis() - startupMillis < initialStartupDelaySeconds * 1e3) {
			return;
		}

		if (!fusion.getRestDetected() && currentStep->requiresRest()) {
			if (isCalibrating) {
				currentStep->cancel();
				isCalibrating = false;
			}

			lastTickRest = fusion.getRestDetected();
			return;
		}

		if (!isCalibrating) {
			isCalibrating = true;
			currentStep->start();
		}

		if (currentStep->requiresRest() && !currentStep->restDetectionDelayElapsed()) {
			lastTickRest = fusion.getRestDetected();
			return;
		}

		auto result = currentStep->tick();

		switch (result) {
			case CalibrationStep<RawSensorT>::TickResult::DONE:
				if (nextCalibrationStep == CalibrationStepEnum::SAMPLING_RATE) {
					stepCalibrationForward(true, false);
					break;
				}
				stepCalibrationForward();
				break;
			case CalibrationStep<RawSensorT>::TickResult::SKIP:
				stepCalibrationForward(false, false);
				break;
			case CalibrationStep<RawSensorT>::TickResult::CONTINUE:
				break;
		}

		lastTickRest = fusion.getRestDetected();
	}

	void scaleAccelSample(sensor_real_t accelSample[3]) final {
		accelSample[0] = accelSample[0] * Consts::AScale - activeCalibration.A_off[0];
		accelSample[1] = accelSample[1] * Consts::AScale - activeCalibration.A_off[1];
		accelSample[2] = accelSample[2] * Consts::AScale - activeCalibration.A_off[2];
	}

	float getAccelTimestep() final { return activeCalibration.A_Ts; }

	void scaleGyroSample(sensor_real_t gyroSample[3]) final {
		gyroSample[0] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[0] - activeCalibration.G_off1[0])
		);
		gyroSample[1] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[1] - activeCalibration.G_off1[1])
		);
		gyroSample[2] = static_cast<sensor_real_t>(
			Consts::GScale * (gyroSample[2] - activeCalibration.G_off1[2])
		);
	}

	void scaleMagSample(sensor_real_t magSample[3]) final {
		float tmp[3];
		for (uint8_t i = 0; i < 3; i++) {
			tmp[i] = (magSample[i] - activeCalibration.M_B[i]);
		}

		magSample[0]
			= (activeCalibration.M_Ainv[0][0] * tmp[0]
			   + activeCalibration.M_Ainv[0][1] * tmp[1]
			   + activeCalibration.M_Ainv[0][2] * tmp[2]);
		magSample[1]
			= (activeCalibration.M_Ainv[1][0] * tmp[0]
			   + activeCalibration.M_Ainv[1][1] * tmp[1]
			   + activeCalibration.M_Ainv[1][2] * tmp[2]);
		magSample[2]
			= (activeCalibration.M_Ainv[2][0] * tmp[0]
			   + activeCalibration.M_Ainv[2][1] * tmp[1]
			   + activeCalibration.M_Ainv[2][2] * tmp[2]);
	}

	float getGyroTimestep() final { return activeCalibration.G_Ts; }
	float getMagTimestep() final { return activeCalibration.M_Ts; }

	float getTempTimestep() final { return activeCalibration.T_Ts; }

	const uint8_t* getMotionlessCalibrationData() final {
		return activeCalibration.MotionlessData;
	}

	void signalOverwhelmed() final {
		if (isCalibrating) {
			currentStep->signalOverwhelmed();
		}
	}

	void provideAccelSample(const RawSensorT accelSample[3]) final {
		if (isCalibrating) {
			currentStep->processAccelSample(accelSample);
		}
	}

	void provideGyroSample(const RawSensorT gyroSample[3]) final {
		if (isCalibrating) {
			currentStep->processGyroSample(gyroSample);
		}
	}

	void provideMagSample(const RawSensorT magSample[3]) final {
		if (isCalibrating) {
			currentStep->processMagSample(magSample);
		}
	}

	void provideTempSample(float tempSample) final {
		if (isCalibrating) {
			currentStep->processTempSample(tempSample);
		}
	}

	void onMagEnabled() final {
		pendingMagCalibrationRequest = shouldRequestMagCalibration();
	}

	bool shouldUpdateMagFusion() const {
		if (isCalibrating && currentStep == &magCalibrationStep) {
			return false;
		}
		return isMagCalibrationReady();
	}

	bool shouldUseMagSample(const sensor_real_t* magSample) const {
		if (!isMagNormReferenceValid(activeCalibration.M_refNorm)) {
			return true;
		}

		const float magnitude = std::sqrt(
			static_cast<float>(
				magSample[0] * magSample[0] + magSample[1] * magSample[1]
				+ magSample[2] * magSample[2]
			)
		);
		if (!std::isfinite(magnitude)) {
			return false;
		}

		const float allowedDeviation = std::max(
			activeCalibration.M_refNorm * magNormRejectRelativeThreshold,
			magNormRejectAbsoluteThreshold
		);
		return std::abs(magnitude - activeCalibration.M_refNorm) <= allowedDeviation;
	}

	void calculateZROChange() {
		if (activeCalibration.gyroPointsCalibrated < 2) {
			activeZROChange = IMU::TemperatureZROChange;
		}

		float diffX = (activeCalibration.G_off2[0] - activeCalibration.G_off1[0])
					* Consts::GScale;
		float diffY = (activeCalibration.G_off2[1] - activeCalibration.G_off1[1])
					* Consts::GScale;
		float diffZ = (activeCalibration.G_off2[2] - activeCalibration.G_off1[2])
					* Consts::GScale;

		float maxDiff
			= std::max(std::max(std::abs(diffX), std::abs(diffY)), std::abs(diffZ));

		activeZROChange = 0.1f / maxDiff
						/ (activeCalibration.gyroMeasurementTemperature2
						   - activeCalibration.gyroMeasurementTemperature1);
	}

	float getZROChange() final { return activeZROChange; }

	bool clearMagCalibration() final {
		resetMagCalibration(calibration);
		syncMagCalibrationToActive();
		fusion.setMagFusionEnabled(false);

		pendingMagCalibrationRequest = shouldRequestMagCalibration();

		if (nextCalibrationStep == CalibrationStepEnum::MAG_CALIB) {
			currentStep->cancel();
			computeNextCalibrationStep();
			isCalibrating = false;
		}

		saveCalibration();
		logger.info("Mag calibration cleared");
		return true;
	}

private:
	enum class CalibrationStepEnum {
		NONE,
		SAMPLING_RATE,
		MOTIONLESS,
		GYRO_BIAS,
		ACCEL_BIAS,
		MAG_CALIB,
	};

	static constexpr float minValidMagReferenceNorm = 1.0f;
	static constexpr float maxValidMagReferenceNorm = 100000.0f;
	static constexpr float magNormRejectRelativeThreshold = 0.07f;
	static constexpr float magNormRejectAbsoluteThreshold = 20.0f;

	void computeNextCalibrationStep() {
		if (!calibration.motionlessCalibrated && Base::HasMotionlessCalib) {
			nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
			currentStep = &motionlessCalibrationStep;
		} else if (calibration.gyroPointsCalibrated == 0) {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
			// } else if (!accelBiasCalibrationStep.allAxesCalibrated()) {
			// 	nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
			// 	currentStep = &accelBiasCalibrationStep;
		} else {
			nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
			currentStep = &gyroBiasCalibrationStep;
		}
	}

	void stepCalibrationForward(bool print = true, bool save = true) {
		currentStep->cancel();
		switch (nextCalibrationStep) {
			case CalibrationStepEnum::NONE:
				return;
			case CalibrationStepEnum::SAMPLING_RATE:
				nextCalibrationStep = CalibrationStepEnum::MOTIONLESS;
				currentStep = &motionlessCalibrationStep;
				if (print) {
					printCalibration(CalibrationPrintFlags::TIMESTEPS);
				}
				break;
			case CalibrationStepEnum::MOTIONLESS:
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				currentStep = &gyroBiasCalibrationStep;
				if (print) {
					printCalibration(CalibrationPrintFlags::MOTIONLESS);
				}
				break;
			case CalibrationStepEnum::GYRO_BIAS:
				if (calibration.gyroPointsCalibrated == 1) {
					// nextCalibrationStep = CalibrationStepEnum::ACCEL_BIAS;
					// currentStep = &accelBiasCalibrationStep;
					nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
					currentStep = &gyroBiasCalibrationStep;
				}

				if (print) {
					printCalibration(CalibrationPrintFlags::GYRO_BIAS);
				}
				break;
			case CalibrationStepEnum::ACCEL_BIAS:
				nextCalibrationStep = CalibrationStepEnum::GYRO_BIAS;
				currentStep = &gyroBiasCalibrationStep;

				if (print) {
					printCalibration(CalibrationPrintFlags::ACCEL_BIAS);
				}

				if (!accelBiasCalibrationStep.allAxesCalibrated()) {
					skippedAStep = true;
				}
				break;
			case CalibrationStepEnum::MAG_CALIB:
				if (!isMagCalibrationValid(calibration)) {
					logger.warn(
						"Mag calibration output was invalid, keeping mag disabled"
					);
					resetMagCalibration(calibration);
					fusion.setMagFusionEnabled(false);
				}
				syncMagCalibrationToActive();
				pendingMagCalibrationRequest = false;
				computeNextCalibrationStep();

				if (print) {
					printCalibration(CalibrationPrintFlags::MAG_CALIB);
				}
				break;
		}

		isCalibrating = false;

		if (save) {
			saveCalibration();
		}
	}

	void maybeStartMagCalibration() {
		if (!pendingMagCalibrationRequest) {
			return;
		}

		if (magCalibrationAttemptedSinceBoot) {
			pendingMagCalibrationRequest = false;
			return;
		}

		if (!toggles.getToggle(SensorToggles::MagEnabled)) {
			return;
		}

		if (isMagCalibrationReady()) {
			pendingMagCalibrationRequest = false;
			return;
		}

		if (isCalibrating
			|| nextCalibrationStep == CalibrationStepEnum::SAMPLING_RATE) {
			return;
		}

		nextCalibrationStep = CalibrationStepEnum::MAG_CALIB;
		currentStep = &magCalibrationStep;
		pendingMagCalibrationRequest = false;
		magCalibrationAttemptedSinceBoot = true;
	}

	bool shouldRequestMagCalibration(bool magCalibrationReady) const {
		return toggles.getToggle(SensorToggles::MagEnabled) && !magCalibrationReady
			&& !magCalibrationAttemptedSinceBoot;
	}

	bool shouldRequestMagCalibration() const {
		return shouldRequestMagCalibration(isMagCalibrationReady());
	}

	bool isMagCalibrationMissing(
		const Configuration::RuntimeCalibrationSensorConfig& config
	) const {
		constexpr float epsilon = 1e-6f;
		bool offsetsAreZero = true;
		for (uint8_t i = 0; i < 3; i++) {
			if (std::abs(config.M_B[i]) > epsilon) {
				offsetsAreZero = false;
				break;
			}
		}

		bool matrixIsIdentity = true;
		for (uint8_t i = 0; i < 3; i++) {
			for (uint8_t j = 0; j < 3; j++) {
				const float expected = i == j ? 1.0f : 0.0f;
				if (std::abs(config.M_Ainv[i][j] - expected) > epsilon) {
					matrixIsIdentity = false;
					break;
				}
			}
		}

		return offsetsAreZero && matrixIsIdentity;
	}

	bool isMagCalibrationValid(
		const Configuration::RuntimeCalibrationSensorConfig& config
	) const {
		float diagonalAbs[3]{0.0f, 0.0f, 0.0f};
		for (uint8_t i = 0; i < 3; i++) {
			if (!std::isfinite(config.M_B[i])) {
				return false;
			}

			for (uint8_t j = 0; j < 3; j++) {
				const float v = config.M_Ainv[i][j];
				if (!std::isfinite(v) || std::abs(v) > 20.0f) {
					return false;
				}
			}
			diagonalAbs[i] = std::abs(config.M_Ainv[i][i]);
		}

		const float meanDiagonal
			= (diagonalAbs[0] + diagonalAbs[1] + diagonalAbs[2]) / 3.0f;
		if (!std::isfinite(meanDiagonal) || meanDiagonal < 1e-4f
			|| meanDiagonal > 10.0f) {
			return false;
		}

		const float diagonalTolerance = std::max(meanDiagonal * 0.6f, 0.05f);
		for (uint8_t i = 0; i < 3; i++) {
			if (std::abs(diagonalAbs[i] - meanDiagonal) > diagonalTolerance) {
				return false;
			}
		}

		return true;
	}

	static bool isMagNormReferenceValid(float refNorm) {
		return std::isfinite(refNorm) && refNorm >= minValidMagReferenceNorm
			&& refNorm <= maxValidMagReferenceNorm;
	}

	bool isMagCalibrationReady() const {
		return !isMagCalibrationMissing(calibration)
			&& isMagCalibrationValid(calibration);
	}

	void syncMagCalibrationToActive() {
		for (uint8_t i = 0; i < 3; i++) {
			activeCalibration.M_B[i] = calibration.M_B[i];
			for (uint8_t j = 0; j < 3; j++) {
				activeCalibration.M_Ainv[i][j] = calibration.M_Ainv[i][j];
			}
		}
		activeCalibration.M_refNorm = calibration.M_refNorm;
	}

	static void resetMagCalibration(
		Configuration::RuntimeCalibrationSensorConfig& calibration
	) {
		for (uint8_t i = 0; i < 3; i++) {
			calibration.M_B[i] = 0.0f;
			for (uint8_t j = 0; j < 3; j++) {
				calibration.M_Ainv[i][j] = (i == j) ? 1.0f : 0.0f;
			}
		}
		calibration.M_refNorm = 0.0f;
	}

	void saveCalibration() {
		SlimeVR::Configuration::SensorConfig calibration{};
		calibration.type
			= SlimeVR::Configuration::SensorConfigType::RUNTIME_CALIBRATION;
		calibration.data.runtimeCalibration = this->calibration;
		configuration.setSensor(sensorId, calibration);
		configuration.save();
	}

	enum class CalibrationPrintFlags {
		TIMESTEPS = 1,
		MOTIONLESS = 2,
		GYRO_BIAS = 4,
		ACCEL_BIAS = 8,
		MAG_CALIB = 16,
	};

	static constexpr CalibrationPrintFlags PrintAllCalibration
		= CalibrationPrintFlags::TIMESTEPS | CalibrationPrintFlags::MOTIONLESS
		| CalibrationPrintFlags::GYRO_BIAS | CalibrationPrintFlags::ACCEL_BIAS
		| CalibrationPrintFlags::MAG_CALIB;

	void printCalibration(CalibrationPrintFlags toPrint = PrintAllCalibration) {
		if (any(toPrint & CalibrationPrintFlags::TIMESTEPS)) {
			if (activeCalibration.sensorTimestepsCalibrated) {
				logger.info(
					"Calibrated timesteps: Accel %f, Gyro %f, Temperature %f",
					activeCalibration.A_Ts,
					activeCalibration.G_Ts,
					activeCalibration.T_Ts
				);
			} else {
				logger.info("Sensor timesteps not calibrated");
			}
		}

		if (Base::HasMotionlessCalib
			&& any(toPrint & CalibrationPrintFlags::MOTIONLESS)) {
			if (calibration.motionlessCalibrated) {
				logger.info("Motionless calibration done");
			} else {
				logger.info("Motionless calibration not done");
			}
		}

		if (any(toPrint & CalibrationPrintFlags::GYRO_BIAS)) {
			if (calibration.gyroPointsCalibrated != 0) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					calibration.gyroMeasurementTemperature1,
					calibration.G_off1[0],
					calibration.G_off1[1],
					calibration.G_off1[2]
				);
			} else {
				logger.info("Gyro bias not calibrated");
			}

			if (calibration.gyroPointsCalibrated == 2) {
				logger.info(
					"Calibrated gyro bias at %fC: %f %f %f",
					calibration.gyroMeasurementTemperature2,
					calibration.G_off2[0],
					calibration.G_off2[1],
					calibration.G_off2[2]
				);
			}
		}

		if (any(toPrint & CalibrationPrintFlags::ACCEL_BIAS)) {
			if (accelBiasCalibrationStep.allAxesCalibrated()) {
				logger.info(
					"Calibrated accel bias: %f %f %f",
					calibration.A_off[0],
					calibration.A_off[1],
					calibration.A_off[2]
				);
			} else if (accelBiasCalibrationStep.anyAxesCalibrated()) {
				logger.info(
					"Partially calibrated accel bias: %f %f %f",
					calibration.A_off[0],
					calibration.A_off[1],
					calibration.A_off[2]
				);
			} else {
				logger.info("Accel bias not calibrated");
			}
		}

		if (any(toPrint & CalibrationPrintFlags::MAG_CALIB)) {
			if (isMagCalibrationReady()) {
				logger.info(
					"Mag calibration offsets: %f %f %f",
					calibration.M_B[0],
					calibration.M_B[1],
					calibration.M_B[2]
				);
				logger.info("Mag calibration M_Ainv:");
				for (uint8_t i = 0; i < 3; i++) {
					logger.info(
						"  %f %f %f",
						calibration.M_Ainv[i][0],
						calibration.M_Ainv[i][1],
						calibration.M_Ainv[i][2]
					);
				}
				logger.info("Mag calibration ref norm: %f", calibration.M_refNorm);
			} else {
				logger.info("Mag not calibrated");
			}
		}
	}

	CalibrationStepEnum nextCalibrationStep = CalibrationStepEnum::SAMPLING_RATE;

	static constexpr float initialStartupDelaySeconds = 5;
	uint64_t startupMillis = millis();

	SampleRateCalibrationStep<RawSensorT> sampleRateCalibrationStep{activeCalibration};
	MotionlessCalibrationStep<IMU, RawSensorT> motionlessCalibrationStep{
		calibration,
		sensor
	};
	GyroBiasCalibrationStep<RawSensorT> gyroBiasCalibrationStep{calibration};
	AccelBiasCalibrationStep<RawSensorT> accelBiasCalibrationStep{
		calibration,
		static_cast<float>(Consts::AScale)
	};
	MagCalibrationStep<RawSensorT> magCalibrationStep;
	NullCalibrationStep<RawSensorT> nullCalibrationStep{calibration};

	CalibrationStep<RawSensorT>* currentStep = &nullCalibrationStep;

	bool isCalibrating = false;
	bool skippedAStep = false;
	bool lastTickRest = false;
	bool pendingMagCalibrationRequest = false;
	bool magCalibrationAttemptedSinceBoot = false;

	SlimeVR::Configuration::RuntimeCalibrationSensorConfig calibration{
		// let's create here transparent calibration that doesn't affect input data
		.ImuType = {IMU::Type},
		.MotionlessDataLen = {Base::MotionlessCalibDataSize()},

		.sensorTimestepsCalibrated = false,
		.A_Ts = IMU::AccTs,
		.G_Ts = IMU::GyrTs,
		.M_Ts = IMU::MagTs,
		.T_Ts = 0,

		.motionlessCalibrated = false,
		.MotionlessData = {},

		.gyroPointsCalibrated = 0,
		.gyroMeasurementTemperature1 = 0,
		.G_off1 = {0.0, 0.0, 0.0},
		.gyroMeasurementTemperature2 = 0,
		.G_off2 = {0.0, 0.0, 0.0},

		.accelCalibrated = {false, false, false},
		.A_off = {0.0, 0.0, 0.0},

		.M_B = {0.0, 0.0, 0.0},
		.M_Ainv = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}},
		.M_refNorm = 0.0f
	};

	float activeZROChange = 0;

	Configuration::RuntimeCalibrationSensorConfig activeCalibration = calibration;

	using Base::fusion;
	using Base::logger;
	using Base::sensor;
	using Base::sensorId;
	using Base::toggles;
};

}  // namespace SlimeVR::Sensors::RuntimeCalibration
