/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Gorbit99 & SlimeVR Contributors

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

#include "magdriver.h"
#include "../axisremap.h"

namespace SlimeVR::Sensors::SoftFusion {
namespace {
	int16_t clampToInt16(const int32_t value) {
		if (value > 32767) {
			return 32767;
		}
		if (value < -32768) {
			return -32768;
		}
		return static_cast<int16_t>(value);
	}

	#ifndef MAG_AXIS_REMAP
	#define MAG_AXIS_REMAP \
		(AXIS_REMAP_AXIS_X(AXIS_REMAP_USE_X) | AXIS_REMAP_AXIS_Y(AXIS_REMAP_USE_Y) \
		 | AXIS_REMAP_AXIS_Z(AXIS_REMAP_USE_Z))
	#endif

	static constexpr int MagAxisRemap = MAG_AXIS_REMAP;

	// Board-level override can provide MAG_AXIS_REMAP via build flags.
	void applyMagAxisMapping(int16_t sample[3]) {
		int32_t mx = sample[0];
		int32_t my = sample[1];
		int32_t mz = sample[2];

		remapAllAxis<int32_t>(MagAxisRemap, &mx, &my, &mz);

		sample[0] = clampToInt16(mx);
		sample[1] = clampToInt16(my);
		sample[2] = clampToInt16(mz);
	}

	bool decodeUnsignedBigEndianCentered32768(const uint8_t* rawSample, MagDataWidth width, int16_t* outDecodedSample) {
		// guard or performance?
		// if (width != MagDataWidth::SixByte) {
		// 	return false;
		// }

		constexpr int32_t centerOffset = 32768;
		for (uint8_t axis = 0; axis < 3; ++axis) {
			const uint8_t msb = rawSample[axis * 2];
			const uint8_t lsb = rawSample[axis * 2 + 1];
			const uint16_t packed = static_cast<uint16_t>(msb) << 8 | lsb;
			outDecodedSample[axis] = static_cast<int16_t>(static_cast<int32_t>(packed) - centerOffset);
		}
		return true;
	}

	bool decodeSignedLittleEndian(const uint8_t* rawSample,	MagDataWidth width,	int16_t* outDecodedSample) {
		// guard or performance?
		// if (width != MagDataWidth::SixByte) {
		// 	return false;
		// }

		for (uint8_t axis = 0; axis < 3; ++axis) {
			const uint8_t lsb = rawSample[axis * 2];
			const uint8_t msb = rawSample[axis * 2 + 1];
			outDecodedSample[axis] = static_cast<int16_t>(static_cast<uint16_t>(msb) << 8 | lsb);
		}
		return true;
	}
}

std::vector<MagDefinition> MagDriver::supportedMags{
	MagDefinition{
		// There are register names that are inconsistent with the documentation, which can cause confusion...
		//Address = 0x30; -> deviceId
		//ProductIdReg = 0x39; -> whoAmIReg
		//ProductIdValue = 0x10; -> expectedWhoAmI
		//OutXReg = 0x00; -> dataReg
		//Ctrl0Reg = 0x1B; DoSet = 0x08(1000); DoReset = 0x10(10000);
		//Ctrl1Reg = 0x1C; SoftReset = 0x80;
		//Ctrl2Reg = 0x1D; Continuous mode = 0x10;
		//ODR = 0x1A; 26 = 26Hz
		.name = "MMC5603NJ",

		.deviceId = 0x30,

		.whoAmIReg = 0x39,
		.expectedWhoAmI = 0x10,

		.dataWidth = MagDataWidth::SixByte,
		.dataReg = 0x00,

		.setup =
			[](MagInterface& interface) {
				interface.writeByte(0x1C, 0x80);	// Ctrl1Reg: Internal Control 1: SW reset
				delay(20);							// MMC5603NJ needs 20ms delay after soft reset
				interface.writeByte(0x1C, 0x00);	// Ctrl1Reg: Internal Control 1: Normal mode
				delay(5);
				interface.writeByte(0x1A, 26);		// ODR: 26Hz
				interface.writeByte(0x1B, 0xA0);	// Ctrl0Reg: BW=00, AutoSR=1, CM freq enable
				interface.writeByte(0x1D, 0x10);	// Ctrl2Reg: Continuous mode enable
				return true;
			},
		.decodeRawSample = decodeUnsignedBigEndianCentered32768,
	},
	MagDefinition{
		.name = "QMC6309",

		.deviceId = 0x7c,

		.whoAmIReg = 0x00,
		.expectedWhoAmI = 0x90,

		.dataWidth = MagDataWidth::SixByte,
		.dataReg = 0x01,

		.setup =
			[](MagInterface& interface) {
				interface.writeByte(0x0b, 0x80);
				interface.writeByte(0x0b, 0x00);  // Soft reset
				delay(10);
				interface.writeByte(0x0b, 0x48);  // Set/reset on, 8g full range, 200Hz
				interface.writeByte(
					0x0a,
					0x21
				);  // LP filter 2, 8x Oversampling, normal mode
				return true;
			},
		.decodeRawSample = decodeSignedLittleEndian,
	},
	MagDefinition{
		.name = "IST8306",

		.deviceId = 0x19,

		.whoAmIReg = 0x00,
		.expectedWhoAmI = 0x06,

		.dataWidth = MagDataWidth::SixByte,
		.dataReg = 0x11,

		.setup =
			[](MagInterface& interface) {
				interface.writeByte(0x32, 0x01);  // Soft reset
				delay(50);
				interface.writeByte(0x30, 0x20);  // Noise suppression: low
				interface.writeByte(0x41, 0x2d);  // Oversampling: 32X
				interface.writeByte(0x31, 0x02);  // Continuous measurement @ 10Hz
				return true;
			},
		.decodeRawSample = decodeSignedLittleEndian,
	},
};

bool MagDriver::init(MagInterface&& interface, bool supports9ByteMags) {
	constexpr uint8_t whoAmIRetries = 5;
	constexpr uint8_t whoAmIRetryDelayMs = 20;

	for (auto& mag : supportedMags) {
		interface.setDeviceId(mag.deviceId);

		logger.info("Trying mag %s!", mag.name);
		uint8_t whoAmI = 0x00;
		bool whoAmIMatched = false;
		for (uint8_t attempt = 0; attempt < whoAmIRetries; ++attempt) {
			whoAmI = interface.readByte(mag.whoAmIReg);
			// logger.info("Trying mag %s! WhoAmI = 0x%02X", mag.name, whoAmI);
			if (whoAmI == mag.expectedWhoAmI) {
				whoAmIMatched = true;
				break;
			}
			delay(whoAmIRetryDelayMs);
		}
		
		if (!whoAmIMatched) {
			continue;
		}

		if (!supports9ByteMags && mag.dataWidth == MagDataWidth::NineByte) {
			logger.error("The sensor doesn't support this mag!");
			return false;
		}

		logger.info("Found mag %s! Initializing", mag.name);
		
		delay(50);
		if (!mag.setup(interface)) {
			logger.error("Mag %s failed to initialize!", mag.name);
			return false;
		}

		detectedMag = mag;

		break;
	}

	this->interface = interface;
	return detectedMag.has_value();
}

void MagDriver::startPolling() const {
	if (!detectedMag) {
		return;
	}

	interface.startPolling(detectedMag->dataReg, detectedMag->dataWidth);
}

void MagDriver::stopPolling() const {
	if (!detectedMag) {
		return;
	}

	interface.stopPolling();
}

bool MagDriver::decodeRawSample(const uint8_t* rawSample, int16_t* outDecodedSample) const {
	if (!detectedMag || !detectedMag->decodeRawSample) {
		return false;
	}

	const bool decoded
		= detectedMag->decodeRawSample(rawSample, detectedMag->dataWidth, outDecodedSample);
	if (!decoded) {
		return false;
	}

	applyMagAxisMapping(outDecodedSample);
	return true;
}

const char* MagDriver::getAttachedMagName() const {
	if (!detectedMag) {
		return nullptr;
	}

	return detectedMag->name;
}

}  // namespace SlimeVR::Sensors::SoftFusion
