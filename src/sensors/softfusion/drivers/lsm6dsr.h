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

#include <algorithm>
#include <array>
#include <cstdint>

#include "lsm6ds-common.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 4g
// and gyroscope range at 1000dps
// Gyroscope ODR = 208Hz, accel ODR = 104Hz

struct LSM6DSR : LSM6DSOutputHandler {

	enum class AuxHubOdr : uint8_t {
		k104Hz = 0x00,
		k52Hz = 0x40,
		k26Hz = 0x80,
		k13Hz = 0xC0,
	};

	static constexpr uint8_t Address = 0x6a;
	static constexpr auto Name = "LSM6DSR";
	static constexpr auto Type = SensorTypeID::LSM6DSR;

	static constexpr float GyrFreq = 208;
	static constexpr float AccFreq = 104;
	static constexpr float MagFreq = 26; // m_aux_hub_odr == k26Hz
	static constexpr float TempFreq = 52;

	static constexpr float GyrTs = 1.0 / GyrFreq;
	static constexpr float AccTs = 1.0 / AccFreq;
	static constexpr float MagTs = 1.0 / MagFreq;
	static constexpr float TempTs = 1.0 / TempFreq;

	static constexpr float GyroSensitivity = 1000 / 35.0f;
	static constexpr float AccelSensitivity = 1000 / 0.122f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 256.0f;

	static constexpr float TemperatureZROChange = 20.0f;

	static constexpr VQFParams SensorVQFParams{};

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x0f;
			static constexpr uint8_t value = 0x6b;
		};
		struct Ctrl1XL {
			static constexpr uint8_t reg = 0x10;
			static constexpr uint8_t value = (0b01001000);  // XL at 104 Hz, 4g FS
		};
		struct Ctrl2GY {
			static constexpr uint8_t reg = 0x11;
			static constexpr uint8_t value = (0b01011000);  // GY at 208 Hz, 1000dps FS
		};
		struct Ctrl3C {
			static constexpr uint8_t reg = 0x12;
			static constexpr uint8_t valueSwReset = 1;
			static constexpr uint8_t value = (1 << 6) | (1 << 2);  // BDU = 1, IF_INC =
																   // 1
		};
		struct FifoCtrl3BDR {
			static constexpr uint8_t reg = 0x09;
			static constexpr uint8_t value
				= 0b01010100;  // Gyroscope batched into FIFO at 208Hz, Accel at 104Hz
		};
		struct FifoCtrl4Mode {
			static constexpr uint8_t reg = 0x0a;
			static constexpr uint8_t value = (0b110110);  // continuous mode,
														  // temperature at 52Hz
		};

		static constexpr uint8_t FifoStatus = 0x3a;
		static constexpr uint8_t FifoData = 0x78;
		
	};
	// sensor hub registers bank access
	static constexpr uint8_t RFuncCfgAccess = 0x01;
	static constexpr uint8_t RMasterConfig = 0x14;
	static constexpr uint8_t RSlv0Add = 0x15;
	static constexpr uint8_t RSlv0Subadd = 0x16;
	static constexpr uint8_t RSlv0Config = 0x17;
	static constexpr uint8_t RSlv1Add = 0x18;
	static constexpr uint8_t RSlv1Subadd = 0x19;
	static constexpr uint8_t RSlv1Config = 0x1a;
	static constexpr uint8_t RDatawriteSlv0 = 0x21;
	static constexpr uint8_t RSensorHub1Reg = 0x02;
	static constexpr uint8_t RStatusMaster = 0x22;

	// sensor hub registers values
	static constexpr uint8_t VFuncCfgAccessEnable = 0x40;
	static constexpr uint8_t VFuncCfgAccessDisable = 0x00;
	static constexpr uint8_t VMasterReset = 0x80;
	static constexpr uint8_t VMasterEnable = 0x45;
	static constexpr uint8_t VMasterDisable = 0x00;
	static constexpr uint8_t VMasterEndOpMask = 0x01;  // STATUS_MASTER.sens_hub_endop
	static constexpr uint8_t VMasterNackMask = 0x78;   // STATUS_MASTER.slave[0..3]_nack

	uint8_t m_aux_hub_odr = static_cast<uint8_t>(AuxHubOdr::k26Hz); // 26Hz
	uint8_t m_aux_data_reg = 0;
	uint8_t m_aux_read_len = 0;

	LSM6DSR(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: LSM6DSOutputHandler(registerInterface, logger) {}

	bool initialize() {
		// perform initialization step
		m_RegisterInterface.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::valueSwReset);
		delay(20);
		m_RegisterInterface.writeReg(Regs::Ctrl1XL::reg, Regs::Ctrl1XL::value);
		m_RegisterInterface.writeReg(Regs::Ctrl2GY::reg, Regs::Ctrl2GY::value);
		m_RegisterInterface.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::value);
		m_RegisterInterface.writeReg(
			Regs::FifoCtrl3BDR::reg,
			Regs::FifoCtrl3BDR::value
		);
		m_RegisterInterface.writeReg(
			Regs::FifoCtrl4Mode::reg,
			Regs::FifoCtrl4Mode::value
		);
		return true;
	}

	bool bulkRead(DriverCallbacks<int16_t>&& callbacks) {
		return LSM6DSOutputHandler::template bulkRead<Regs>(
			std::move(callbacks),
			GyrTs,
			AccTs,
			TempTs,
			MagTs
		);
	}

	void setAuxId(uint8_t deviceId) {
		enterEmbeddedAccess();

		m_RegisterInterface.writeReg(RMasterConfig, VMasterReset);
		delay(20);
		m_RegisterInterface.writeReg(RMasterConfig, VMasterEnable);
		delay(5);
		m_RegisterInterface.writeReg(RMasterConfig, VMasterDisable);
		delay(1);

		m_RegisterInterface.writeReg(RSlv0Add, (deviceId << 1)); // bit0 is R/W bit, write
		m_RegisterInterface.writeReg(RSlv0Config, m_aux_hub_odr);
		m_RegisterInterface.writeReg(RSlv1Add, (deviceId << 1) | 0x01); // read
		m_RegisterInterface.writeReg(RSlv1Config, 0x00); // reset
		m_RegisterInterface.writeReg(RMasterConfig, VMasterEnable);
		delay(5);

		exitEmbeddedAccess();
	}

	void writeAux(uint8_t regAddr, uint8_t value) {
		enterEmbeddedAccess();

		constexpr uint8_t maxRetries = 5;
		constexpr uint32_t auxOpTimeoutMs = 120;
		constexpr uint32_t retryDelayMs = 40;
		bool writeSuccess = false;

		for (uint8_t attempt = 0; attempt < maxRetries; ++attempt) {
			m_RegisterInterface.writeReg(RMasterConfig, VMasterDisable);
			m_RegisterInterface.writeReg(RSlv0Subadd, regAddr);
			m_RegisterInterface.writeReg(RDatawriteSlv0, value);
			m_RegisterInterface.writeReg(RMasterConfig, VMasterEnable);
			if (waitAuxOperationDone(auxOpTimeoutMs)) {
				writeSuccess = true;
				break;
			}

			if (attempt + 1 < maxRetries) {
				delay(retryDelayMs);
			}
		}

		if (!writeSuccess) {
			m_Logger.error("LSM6DSR: Failed writing aux register 0x%02X after %u attempts", regAddr, static_cast<unsigned>(maxRetries));
		}

		resetAuxPollingSetting();
		if (m_aux_read_len != 0) {
			setAuxPollingSetting();
			if (!waitAuxOperationDone(auxOpTimeoutMs)) {
				m_Logger.error("LSM6DSR: Timeout restoring aux polling after write register 0x%02X", regAddr);
			}
		}

		exitEmbeddedAccess();
	}

	uint8_t readAux(uint8_t address) {
		enterEmbeddedAccess();
		
		uint8_t out_value = 0;
		constexpr uint8_t maxRetries = 5;
		constexpr uint32_t auxOpTimeoutMs = 120;
		constexpr uint32_t retryDelayMs = 40;
		bool readSuccess = false;

		for (uint8_t attempt = 0; attempt < maxRetries; ++attempt) {
			m_RegisterInterface.writeReg(RMasterConfig, VMasterDisable);
			m_RegisterInterface.writeReg(RSlv1Config, 0x00);
			m_RegisterInterface.writeReg(RSlv1Subadd, address);
			m_RegisterInterface.writeReg(RSlv1Config, 0x09);
			m_RegisterInterface.writeReg(RMasterConfig, VMasterEnable);

			if (waitAuxOperationDone(auxOpTimeoutMs)) {
				out_value = m_RegisterInterface.readReg(RSensorHub1Reg);
				readSuccess = true;
				break;
			}

			if (attempt + 1 < maxRetries) {
				delay(retryDelayMs);
			}
		}

		if (!readSuccess) {
			m_Logger.error("LSM6DSR: Failed reading aux register 0x%02X after %u attempts", address, static_cast<unsigned>(maxRetries));
		}

		resetAuxPollingSetting();

		if (m_aux_read_len != 0) {
			setAuxPollingSetting();
			if (!waitAuxOperationDone(auxOpTimeoutMs)) {
				m_Logger.error("LSM6DSR: Timeout restoring aux polling after read register 0x%02X", address);
			}
		}

		exitEmbeddedAccess();
		return out_value;
	}

	void startAuxPolling(uint8_t dataReg, MagDataWidth dataWidth) {
		enterEmbeddedAccess();
		constexpr uint32_t auxStartTimeoutMs = 120;

		m_aux_data_reg = dataReg;
		m_aux_read_len = (dataWidth == MagDataWidth::SixByte) ? 6 : 9;

		resetAuxPollingSetting();
		setAuxPollingSetting();
		if (!waitAuxOperationDone(auxStartTimeoutMs)) {
			m_Logger.error("LSM6DSR: Timeout starting aux polling");
		}

		exitEmbeddedAccess();
	}

	void stopAuxPolling() {
		enterEmbeddedAccess();

		resetAuxPollingSetting();
		m_aux_read_len = 0;

		exitEmbeddedAccess();
	}

	void setAuxPollingSetting() {
		if (m_aux_read_len == 0) {
			return;
		}

		m_RegisterInterface.writeReg(RSlv1Subadd, m_aux_data_reg);
		m_RegisterInterface.writeReg(RSlv1Config, static_cast<uint8_t>((m_aux_read_len & 0x0F) | 0x08));
		m_RegisterInterface.writeReg(RMasterConfig, VMasterEnable);
	}

	void resetAuxPollingSetting() {
		m_RegisterInterface.writeReg(RMasterConfig, VMasterDisable);
		m_RegisterInterface.writeReg(RSlv1Config, 0x00);
	}

	void setAuxHubOdr(AuxHubOdr odr) {
		m_aux_hub_odr = static_cast<uint8_t>(odr);
		enterEmbeddedAccess();

		m_RegisterInterface.writeReg(RSlv0Config, m_aux_hub_odr);

		exitEmbeddedAccess();
	}

	void enterEmbeddedAccess() {
		m_RegisterInterface.writeReg(RFuncCfgAccess, VFuncCfgAccessEnable);
		delayMicroseconds(50);
	}

	void exitEmbeddedAccess() {
		delayMicroseconds(50);
		m_RegisterInterface.writeReg(RFuncCfgAccess, VFuncCfgAccessDisable);
		delayMicroseconds(50);
	}

	bool waitAuxOperationDone(uint32_t timeoutMs) {
		uint32_t startTime = millis();
		while (millis() - startTime < timeoutMs) {
			uint8_t status = m_RegisterInterface.readReg(RStatusMaster);
			if ((status & VMasterNackMask) != 0) {
				return false;
			}
			if ((status & VMasterEndOpMask) != 0) {
				return true;
			}
			delay(1);
		}
		
		return false;
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
