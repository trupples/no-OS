/**
 * \file adrv9009/profiles/tx_bw400_ir491p52_rx_bw200_or245p76_orx_bw400_or491p52_dc245p76/talise_config.c
 * \brief Contains Talise configuration settings for the Talise API
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 *
 * The top level structure taliseDevice_t talDevice uses keyword
 * extern to allow the application layer main() to have visibility
 * to these settings.
 *
 * This file may not be fully complete for the end user application and
 * may need to updated for AGC, GPIO, and DAC full scale settings.
 * To create a full initialisation routine, the user should also refer to the
 * Iron Python initialisation routine generated by the GUI, and also the Talise User Guide.
 *
 */

#include "talise_types.h"
#include "talise_config.h"
#include "talise_error.h"
#include "talise_agc.h"
#ifdef ADI_ZYNQ_PLATFORM
#include "zynq_platform.h"
#endif

int16_t txFirCoefs[20] = {32, -76, 124, -160, 176, -121, -145, 1031, -3015, 20138, -3015, 1031, -145, -121, 176, -160, 124, -76, 32, 0};

int16_t rxFirCoefs[24] = {-194, -715, 777, 907, -1163, -1890, 2240, 3306, -4068, -7024, 9205, 31112, 31112, 9205, -7024, -4068, 3306, 2240, -1890, -1163, 907, 777, -715, -194};

int16_t obsrxFirCoefs[24] = {-44, 22,-18, -1, 32, -75, 83, -81, -15, 354, -1940, 19672, -1940, 354, -15, -81, 83, -75, 32, -1, -18, 22, -44, 0};

#ifdef ADI_ZYNQ_PLATFORM /** < Insert Customer Platform HAL State Container here>*/
/*
 * Platform Layer SPI settings - this structure is specific to ADI's platform layer code.
 * User should replace with their own structure or settings for their hardware
 */
zynqSpiSettings_t spiDev1 = {
	.chipSelectIndex = 1,
	.writeBitPolarity = 0,
	.longInstructionWord = 1,
	.CPHA = 0,
	.CPOL = 0,
	.mode = 0,
	.spiClkFreq_Hz = 25000000
};

/*
 * Platform Layer settings - this structure is specific to ADI's platform layer code.
 * User should replace with their own structure or settings for their hardware
 * The structure is held in taliseDevice_t below as a void pointer, allowing
 * the customer to pass any information for their specific hardware down to the
 * hardware layer code.
 */
zynqAdiDev_t talDevHalInfo = {
	.devIndex = 1,
	.spiSettings = &spiDev1,
	.spiErrCode = 0,
	.timerErrCode = 0,
	.gpioErrCode = 0,
	.logLevel = ADIHAL_LOG_ALL
};
#endif
/**
 *  TalDevice a structure used by the Talise API to hold the platform hardware
 *  structure information, as well as an internal Talise API state container
 *  (devStateInfo) of runtime information used by the API.
 **/
taliseDevice_t talDevice = {
#ifdef ADI_ZYNQ_PLATFORM
	/* Void pointer of users platform HAL settings to pass to HAL layer calls
	 * Talise API does not use the devHalInfo member */
	.devHalInfo = &talDevHalInfo,
#else
	.devHalInfo = NULL,     /* < Insert Customer Platform HAL State Container here>*/
#endif
	/* devStateInfo is maintained internal to the Talise API, just create the memory */
	.devStateInfo = {0}

};

taliseInit_t talInit = {
	/* SPI settings */
	.spiSettings =
	{
		.MSBFirst            = 1,  /* 1 = MSBFirst, 0 = LSBFirst */
		.enSpiStreaming      = 0,  /* Not implemented in ADIs platform layer. SW feature to improve SPI throughput */
		.autoIncAddrUp       = 1,  /* Not implemented in ADIs platform layer. For SPI Streaming, set address increment direction. 1= next addr = addr+1, 0:addr=addr-1 */
		.fourWireMode        = 1,  /* 1: Use 4-wire SPI, 0: 3-wire SPI (SDIO pin is bidirectional). NOTE: ADI's FPGA platform always uses 4-wire mode */
		.cmosPadDrvStrength  = TAL_CMOSPAD_DRV_2X /* Drive strength of CMOS pads when used as outputs (SDIO, SDO, GP_INTERRUPT, GPIO 1, GPIO 0) */
	},

	/* Rx settings */
	.rx =
	{
		.rxProfile =
		{
			.rxFir =
			{
				.gain_dB = -6,                /* filter gain */
				.numFirCoefs = 24,            /* number of coefficients in the FIR filter */
				.coefs = &rxFirCoefs[0]
			},
			.rxFirDecimation = 2,            /* Rx FIR decimation (1,2,4) */
			.rxDec5Decimation = 4,            /* Decimation of Dec5 or Dec4 filter (5,4) */
			.rhb1Decimation = 1,            /* RX Half band 1 decimation (1 or 2) */
			.rxOutputRate_kHz = 245760,            /* Rx IQ data rate in kHz */
			.rfBandwidth_Hz = 200000000,    /* The Rx RF passband bandwidth for the profile */
			.rxBbf3dBCorner_kHz = 200000,    /* Rx BBF 3dB corner in kHz */
			.rxAdcProfile = {185, 141, 172, 90, 1280, 942, 1332, 90, 1368, 46, 1016, 19, 48, 48, 37, 208, 0, 0, 0, 0, 52, 0, 7, 6, 42, 0, 7, 6, 42, 0, 25, 27, 0, 0, 25, 27, 0, 0, 165, 44, 31, 905},            /* pointer to custom ADC profile */
			.rxDdcMode = TAL_RXDDC_BYPASS,   /* Rx DDC mode */
			.rxNcoShifterCfg =
			{
				.bandAInputBandWidth_kHz = 0,
				.bandAInputCenterFreq_kHz = 0,
				.bandANco1Freq_kHz = 0,
				.bandANco2Freq_kHz = 0,
				.bandBInputBandWidth_kHz = 0,
				.bandBInputCenterFreq_kHz = 0,
				.bandBNco1Freq_kHz = 0,
				.bandBNco2Freq_kHz = 0
			}
		},
		.framerSel = TAL_FRAMER_A,            /* Rx JESD204b framer configuration */
		.rxGainCtrl =
		{
			.gainMode = TAL_MGC,            /* taliserxGainMode_t gainMode */
			.rx1GainIndex = 255,            /* uint8_t rx1GainIndex */
			.rx2GainIndex = 255,            /* uint8_t rx2GainIndex */
			.rx1MaxGainIndex = 255,            /* uint8_t rx1MaxGainIndex */
			.rx1MinGainIndex = 195,            /* uint8_t rx1MinGainIndex */
			.rx2MaxGainIndex = 255,            /* uint8_t rx2MaxGainIndex */
			.rx2MinGainIndex = 195            /* uint8_t rx2MinGainIndex */
		},
		.rxChannels = TAL_RX1RX2,                /* The desired Rx Channels to enable during initialization */
	},


	/* Tx settings */
	.tx =
	{
		.txProfile =
		{
			.dacDiv = 1,                        /* The divider used to generate the DAC clock */
			.txFir =
			{
				.gain_dB = 0,                        /* filter gain */
				.numFirCoefs = 20,                    /* number of coefficients in the FIR filter */
				.coefs = &txFirCoefs[0]
			},
			.txFirInterpolation = 1,                    /* The Tx digital FIR filter interpolation (1,2,4) */
			.thb1Interpolation = 2,                    /* Tx Halfband1 filter interpolation (1,2) */
			.thb2Interpolation = 2,                    /* Tx Halfband2 filter interpolation (1,2)*/
			.thb3Interpolation = 1,                    /* Tx Halfband3 filter interpolation (1,2)*/
			.txInt5Interpolation = 1,                    /* Tx Int5 filter interpolation (1,5) */
			.txInputRate_kHz = 491520,                    /* Primary Signal BW */
			.primarySigBandwidth_Hz = 200000000,    /* The Rx RF passband bandwidth for the profile */
			.rfBandwidth_Hz = 450000000,            /* The Tx RF passband bandwidth for the profile */
			.txDac3dBCorner_kHz = 450000,                /* The DAC filter 3dB corner in kHz */
			.txBbf3dBCorner_kHz = 225000,                /* The BBF 3dB corner in kHz */
			.loopBackAdcProfile = {186, 148, 176, 90, 1280, 901, 1479, 225, 1401, 85, 995, 21, 48, 48, 36, 207, 0, 0, 0, 0, 52, 0, 0, 6, 24, 0, 0, 6, 24, 0, 25, 27, 0, 0, 25, 27, 0, 0, 165, 44, 15, 905}
		},
		.deframerSel = TAL_DEFRAMER_A,                    /* Talise JESD204b deframer config for the Tx data path */
		.txChannels = TAL_TX1TX2,                            /* The desired Tx channels to enable during initialization */
		.txAttenStepSize = TAL_TXATTEN_0P05_DB,            /* Tx Attenuation step size */
		.tx1Atten_mdB = 10000,                            /* Initial Tx1 Attenuation */
		.tx2Atten_mdB = 10000,                            /* Initial Tx2 Attenuation */
		.disTxDataIfPllUnlock = TAL_TXDIS_TX_RAMP_DOWN_TO_ZERO    /* Options to disable the transmit data when the RFPLL unlocks. */
	},


	/* ObsRx settings */
	.obsRx =
	{
		.orxProfile =
		{
			.rxFir =
			{
				.gain_dB = 6,                /* filter gain */
				.numFirCoefs = 24,            /* number of coefficients in the FIR filter */
				.coefs = &obsrxFirCoefs[0]
			},
			.rxFirDecimation = 1,            /* Rx FIR decimation (1,2,4) */
			.rxDec5Decimation = 4,            /* Decimation of Dec5 or Dec4 filter (5,4) */
			.rhb1Decimation = 1,            /* RX Half band 1 decimation (1 or 2) */
			.orxOutputRate_kHz = 491520,            /* Rx IQ data rate in kHz */
			.rfBandwidth_Hz = 400000000,    /* The Rx RF passband bandwidth for the profile */
			.rxBbf3dBCorner_kHz = 225000,    /* Rx BBF 3dB corner in kHz */
			.orxLowPassAdcProfile = {113, 171, 181, 90, 1280, 1737, 1574, 839, 1305, 297, 846, 74, 30, 41, 32, 193, 0, 0, 0, 0, 48, 0, 0, 0, 24, 0, 0, 0, 24, 0, 25, 27, 0, 0, 25, 27, 0, 0, 165, 44, 15, 905},
			.orxBandPassAdcProfile = {113, 171, 181, 90, 1280, 1737, 1574, 839, 1305, 297, 846, 74, 30, 41, 32, 193, 0, 0, 0, 0, 48, 0, 0, 0, 24, 0, 0, 0, 24, 0, 25, 27, 0, 0, 25, 27, 0, 0, 165, 44, 15, 905},
			.orxDdcMode = TAL_ORXDDC_DISABLED,   /* ORx DDC mode */
			.orxMergeFilter  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
		},
		.orxGainCtrl =
		{
			.gainMode = TAL_MGC,
			.orx1GainIndex = 255,
			.orx2GainIndex = 255,
			.orx1MaxGainIndex = 255,
			.orx1MinGainIndex = 195,
			.orx2MaxGainIndex = 255,
			.orx2MinGainIndex = 195
		},
		.framerSel = TAL_FRAMER_B,                /* ObsRx JESD204b framer configuration */
		.obsRxChannelsEnable = TAL_ORX1ORX2,        /* The desired ObsRx Channels to enable during initialization */
		.obsRxLoSource = TAL_OBSLO_RF_PLL                /* The ORx mixers can use the TX_PLL */
	},

	/* Digital Clock Settings */
	.clocks =
	{
		.deviceClock_kHz = 245760,            /* CLKPLL and device reference clock frequency in kHz */
		.clkPllVcoFreq_kHz = 9830400,        /* CLKPLL VCO frequency in kHz */
		.clkPllHsDiv = TAL_HSDIV_2P5,            /* CLKPLL high speed clock divider */
		.rfPllUseExternalLo = 0,                /* 1= Use external LO for RF PLL, 0 = use internal LO generation for RF PLL */
		.rfPllPhaseSyncMode = TAL_RFPLLMCS_NOSYNC                /* RFPLL MCS (Phase sync) mode */
	},

	/* JESD204B settings */
	.jesd204Settings =
	{
		/* Framer A settings */
		.framerA =
		{
			.bankId = 1,                    /* JESD204B Configuration Bank ID -extension to Device ID (Valid 0..15) */
			.deviceId = 0,                    /* JESD204B Configuration Device ID - link identification number. (Valid 0..255) */
			.lane0Id = 0,                    /* JESD204B Configuration starting Lane ID.  If more than one lane used, each lane will increment from the Lane0 ID. (Valid 0..31) */
			.M = 4,                            /* number of ADCs (0, 2, or 4) - 2 ADCs per receive chain */
			.K = 32,                        /* number of frames in a multiframe (default=32), F*K must be a multiple of 4. (F=2*M/numberOfLanes) */
			.F = 4,                            /* F (number of bytes per frame) */
			.Np = 16,                            /* Np (converter sample resolution) */
			.scramble = 1,                    /* scrambling off if framerScramble= 0, if framerScramble>0 scramble is enabled. */
			.externalSysref = 1,            /* 0=use internal SYSREF, 1= use external SYSREF */
			.serializerLanesEnabled = 0x03,    /* serializerLanesEnabled - bit per lane, [0] = Lane0 enabled, [1] = Lane1 enabled */
			.serializerLaneCrossbar = 0xE4,    /* serializerLaneCrossbar */
			.lmfcOffset = 31,                /* lmfcOffset - LMFC offset value for deterministic latency setting */
			.newSysrefOnRelink = 0,            /* newSysrefOnRelink */
			.syncbInSelect = 0,                /* syncbInSelect; */
			.overSample = 0,                    /* 1=overSample, 0=bitRepeat */
			.syncbInLvdsMode = 1,
			.syncbInLvdsPnInvert = 0,
			.enableManualLaneXbar = 0 /* 0=auto, 1=manual */
		},
		/* Framer B settings */
		.framerB =
		{
			.bankId = 0,                    /* JESD204B Configuration Bank ID -extension to Device ID (Valid 0..15) */
			.deviceId = 0,                    /* JESD204B Configuration Device ID - link identification number. (Valid 0..255) */
			.lane0Id = 0,                    /* JESD204B Configuration starting Lane ID.  If more than one lane used, each lane will increment from the Lane0 ID. (Valid 0..31) */
			.M = 2,                            /* number of ADCs (0, 2, or 4) - 2 ADCs per receive chain */
			.K = 32,                        /* number of frames in a multiframe (default=32), F*K must be a multiple of 4. (F=2*M/numberOfLanes) */
			.F = 2,                            /* F (number of bytes per frame) */
			.Np = 16,                            /* Np (converter sample resolution) */
			.scramble = 1,                    /* scrambling off if framerScramble= 0, if framerScramble>0 scramble is enabled. */
			.externalSysref = 1,            /* 0=use internal SYSREF, 1= use external SYSREF */
			.serializerLanesEnabled = 0x0C,    /* serializerLanesEnabled - bit per lane, [0] = Lane0 enabled, [1] = Lane1 enabled */
			.serializerLaneCrossbar = 0xE4,    /* serializerLaneCrossbar */
			.lmfcOffset = 31,                /* lmfcOffset - LMFC offset value for deterministic latency setting */
			.newSysrefOnRelink = 0,            /* newSysrefOnRelink */
			.syncbInSelect = 1,                /* syncbInSelect; */
			.overSample = 0,                    /* 1=overSample, 0=bitRepeat */
			.syncbInLvdsMode = 1,
			.syncbInLvdsPnInvert = 0,
			.enableManualLaneXbar = 0 /* 0=auto, 1=manual */
		},
		/* Deframer A settings */
		.deframerA =
		{
			.bankId = 0,                    /* bankId extension to Device ID (Valid 0..15) */
			.deviceId = 0,                    /* deviceId  link identification number. (Valid 0..255) */
			.lane0Id = 0,                    /* lane0Id Lane0 ID. (Valid 0..31) */
			.M = 4,                            /* M  number of DACss (0, 2, or 4) - 2 DACs per transmit chain */
			.K = 32,                        /* K  #frames in a multiframe (default=32), F*K=multiple of 4. (F=2*M/numberOfLanes) */
			.scramble = 1,                    /* scramble  scrambling off if scramble= 0 */
			.externalSysref = 1,            /* externalSysref  0= use internal SYSREF, 1= external SYSREF */
			.deserializerLanesEnabled = 0x0F,    /* deserializerLanesEnabled  bit per lane, [0] = Lane0 enabled */
			.deserializerLaneCrossbar = 0xE4,    /* deserializerLaneCrossbar */
			.lmfcOffset = 17,                /* lmfcOffset	 LMFC offset value to adjust deterministic latency */
			.newSysrefOnRelink = 0,            /* newSysrefOnRelink */
			.syncbOutSelect = 0,                /* SYNCBOUT0/1 select */
			.Np = 16,                /* Np (converter sample resolution) */
			.syncbOutLvdsMode = 1,
			.syncbOutLvdsPnInvert = 0,
			.syncbOutCmosSlewRate = 0,
			.syncbOutCmosDriveLevel = 0,
			.enableManualLaneXbar = 0 /* 0=auto, 1=manual */
		},
		/* Deframer B settings */
		.deframerB =
		{
			.bankId = 0,                    /* bankId extension to Device ID (Valid 0..15) */
			.deviceId = 0,                    /* deviceId  link identification number. (Valid 0..255) */
			.lane0Id = 0,                    /* lane0Id Lane0 ID. (Valid 0..31) */
			.M = 0,                            /* M  number of DACss (0, 2, or 4) - 2 DACs per transmit chain */
			.K = 32,                        /* K  #frames in a multiframe (default=32), F*K=multiple of 4. (F=2*M/numberOfLanes) */
			.scramble = 1,                    /* scramble  scrambling off if scramble= 0 */
			.externalSysref = 1,            /* externalSysref  0= use internal SYSREF, 1= external SYSREF */
			.deserializerLanesEnabled = 0x00,    /* deserializerLanesEnabled  bit per lane, [0] = Lane0 enabled */
			.deserializerLaneCrossbar = 0xE4,    /* deserializerLaneCrossbar */
			.lmfcOffset = 0,                /* lmfcOffset	 LMFC offset value to adjust deterministic latency */
			.newSysrefOnRelink = 0,            /* newSysrefOnRelink */
			.syncbOutSelect = 1,                /* SYNCBOUT0/1 select */
			.Np = 16,                /* Np (converter sample resolution) */
			.syncbOutLvdsMode = 1,
			.syncbOutLvdsPnInvert = 0,
			.syncbOutCmosSlewRate = 0,
			.syncbOutCmosDriveLevel = 0,
			.enableManualLaneXbar = 0 /* 0=auto, 1=manual */
		},
		.serAmplitude = 15,                    /* Serializer amplitude setting. Default = 15. Range is 0..15 */
		.serPreEmphasis = 1,                /* Serializer pre-emphasis setting. Default = 1 Range is 0..4 */
		.serInvertLanePolarity = 0,            /* Serializer Lane PN inversion select. Default = 0. Where, bit[0] = 1 will invert lane [0], bit[1] = 1 will invert lane 1, etc. */
		.desInvertLanePolarity = 0,            /* Deserializer Lane PN inversion select.  bit[0] = 1 Invert PN of Lane 0, bit[1] = Invert PN of Lane 1, etc */
		.desEqSetting = 1,                    /* Deserializer Equalizer setting. Applied to all deserializer lanes. Range is 0..4 */
		.sysrefLvdsMode = 1,                /* Use LVDS inputs on Talise for SYSREF */
		.sysrefLvdsPnInvert = 0              /*0= Do not PN invert SYSREF */
	}
};

//Only needs to be called if user wants to setup AGC parameters
static taliseAgcCfg_t rxAgcCtrl = {
	4,
	255,
	195,
	255,
	195,
	30720,  /* AGC gain update time in us (125us-250us - based on IQ data rate - set for 125us @ 245.76 Mhz) */
	10,
	10,
	16,
	0,
	1,
	0,
	0,
	0,
	1,
	31,
	246,
	4,
	1,          /*!<1- bit field to enable the multiple time constants in AGC loop for fast attack and fast recovery to max gain. */
	/* agcPower */
	{
		1,      /*!<1-bit field, enables the Rx power measurement block. */
		1,      /*!<1-bit field, allows using Rx PFIR for power measurement. */
		0,      /*!<1-bit field, allows to use the output of the second digital offset block in the Rx datapath for power measurement. */
		9,      /*!<AGC power measurement detect lower 0 threshold. Default = -12dBFS == 5, 7-bit register value where max = 0x7F, min = 0x00 */
		2,      /*!<AGC power measurement detect lower 1 threshold. Default = (offset) 4dB == 0, 4-bit register value where  max = 0xF, min = 0x00 */
		4,      /*!<AGC power measurement detect lower 0 recovery gain step. Default = 2dB - based on gain table step  size, 5-bit register value where max = 0x1F, min = 0x00 */
		4,      /*!<AGC power measurement detect lower 1 recovery gain step. Default = 4dB - based on gain table step size, 5-bit register value where max = 0x1F, min = 0x00 */
		5,      /*!< power measurement duration used by the decimated power block. Default = 0x05, 5-bit register value where max = 0x1F, min = 0x00 */
		5,      /*!<Allows power detection of data for a specific slice of the gain update counter. 16-bit register value (currently not used) */
		1,      /*!<Allows power detection of data for a specific slice of the gain update counter. 16-bit register value (currently not used) */
		5,      /*!<Allows power detection of data for a specific slice of the gain update counter. 16-bit register value (currently not used) */
		1,      /*!<Allows power detection of data for a specific slice of the gain update counter. 16-bit register value (currently not used) */
		2,      /*!<Default value should be 2*/
		0,
		0
	},
	/* agcPeak */
	{
		205,        /*!<1st update interval for the multiple time constant in AGC loop mode, Default:205. */
		2,          /*!<sets the 2nd update interval for the multiple time constant in AGC loop mode. Calculated as a multiple of  agcUnderRangeLowInterval  , Default: 4 */
		4,          /*!<sets the 3rd update interval for the multiple time constant in AGC loop mode. Calculated as a multiple of agcUnderRangeMidInterval and agcUnderRangeLowInterval, Default: 4 */
		39,         /*!<AGC APD high threshold. Default=0x1F, 6-bit register value where max=0x3F, min =0x00 */
		49,         /*!<AGC APD peak detect high threshold. default = 0x1F, 6-bit register value where max = 0x3F, min = 0x00.  Set to 3dB below apdHighThresh */
		23,         /*!<AGC APD peak detect low threshold. default = 3dB below high threshold, 6-bit register value where max =0x3F, min = 0x00 */
		19,         /*!<AGC APD peak detect low threshold. default = 3dB below high threshold, 6-bit register value where max = 0x3F, min = 0x00 . Set to 3dB below apdLowThresh  */
		6,          /*!<AGC APD peak detect upper threshold count. Default = 0x06 8-bit register value where max = 0xFF, min = 0x20  */
		3,          /*!<AGC APD peak detect lower threshold count. Default = 0x03, 8-bit register value where max = 0xFF, min = 0x00  */
		4,          /*!<AGC APD peak detect attack gain step. Default = 2dB step - based on gain table step size, 5-bit register  value, where max = 0x1F, min = 0x00  */
		2,          /*!<AGC APD gain index step size. Recommended to be same as hb2GainStepRecovery. Default = 0x00, 5-bit register value where max = 0x1F, min = 0x00  */
		1,          /*!<1-bit field, enables or disables the HB2 overload detector.  */
		1,          /*!<3-bit field. Sets the window of clock cycles (at the HB2 output rate) to meet the overload count. */
		1,          /*!<4-bit field. Sets the number of actual overloads required to trigger the overload signal.  */
		181,        /*!<AGC decimator output high threshold. Default = 0xB5, 8-bit register value where max = 0xFF, min = 0x00 */
		45,         /*!<AGC decimator output low threshold. Default = 0x80, 8-bit register value where max = 0xFF, min = 0x00 */
		90,         /*!<AGC decimator output low threshold. Default = 0x80, 8-bit register value where max = 0xFF, min = 0x00 */
		128,        /*!<AGC decimator output low threshold. Default = 0x80, 8-bit register value where max = 0xFF, min = 0x00 */
		6,          /*!<AGC HB2 output upper threshold count. Default = 0x06, 8-bit register value where max = 0xFF, min =  0x20 */
		3,          /*!<AGC HB2 output lower threshold count. Default = 0x03, 8-bit register value where max = 0xFF, min = 0x00 */
		2,          /*!<AGC decimator gain index step size. Default = 0x00, 5-bit register value where max = 0x1F, min = 0x00 */
		4,          /*!<AGC HB2 gain index step size, when the HB2 Low Overrange interval 0 triggers a programmable number  of times. Default = 0x08, 5-bit register value where max = 0x1F, min = 0x00 */
		8,          /*!<AGC HB2 gain index step size, when the HB2 Low Overrange interval 1 triggers a programmable number of times. Default = 0x04, 5-bit register value where max = 0x1F, min = 0x00 */
		4,          /*!<AGC decimator output attack gain step. Default = 2dB step - based on gain table step size, 5-bit register value, where max = 0x1F, min = 0x00 */
		1,
		0,
		0
	}
};
