/**
 * Copyright 2015 - 2022 Analog Devices Inc.
 * Released under the ADRV904X API license, for more information
 * see the "LICENSE.pdf" file in this zip file.
 */

/**
 * \file adrv904x_radioctrl_types.h
 * \brief Contains ADRV904X RADIOCTRL related private data prototypes for
 *        adrv904x_radioctrl.c
 *
 * ADRV904X API Version: 2.10.0.4
 */

#ifndef _ADRV904X_RADIOCTRL_TYPES_H_
#define _ADRV904X_RADIOCTRL_TYPES_H_

typedef enum adrv904x_StreamGpioFeatureSelection
{
    ADRV904X_STREAM_GPIO_TX_TO_ORX_MAPPING_BIT0    = 0U,
    ADRV904X_STREAM_GPIO_TX_TO_ORX_MAPPING_BIT1    = 1U,
    ADRV904X_STREAM_GPIO_TX_TO_ORX_MAPPING_BIT2    = 2U,
    ADRV904X_STREAM_GPIO_TX_TO_ORX_MAPPING_BIT3    = 3U,
    ADRV904X_STREAM_GPIO_TX_TO_ORX_MAPPING_BIT4    = 4U,
    ADRV904X_STREAM_GPIO_TX_TO_ORX_MAPPING_BIT5    = 5U,
    ADRV904X_STREAM_GPIO_TX_TO_ORX_MAPPING_BIT6    = 6U,
    ADRV904X_STREAM_GPIO_TX_TO_ORX_MAPPING_BIT7    = 7U,
    ADRV904X_STREAM_GPIO_TX_ANTENNA_CAL            = 8U,
    ADRV904X_STREAM_GPIO_RX_ANTENNA_CAL            = 9U,
    ADRV904X_STREAM_GPIO_TX_PAP_FOR_EXT_LO0_UNLOCK = 10U,
    ADRV904X_STREAM_GPIO_TX_PAP_FOR_EXT_LO1_UNLOCK = 11U,

    ADRV904X_STREAM_GPIO_ALARM_INPUT_0                      = 20U,
    ADRV904X_STREAM_GPIO_ALARM_INPUT_1                      = 21U,
    ADRV904X_STREAM_GPIO_ALARM_INPUT_2                      = 22U,
    ADRV904X_STREAM_GPIO_ALARM_INPUT_3                      = 23U,
    ADRV904X_STREAM_GPIO_ALARM_INPUT_4                      = 24U,
    ADRV904X_STREAM_GPIO_ALARM_INPUT_5                      = 25U,
    ADRV904X_STREAM_GPIO_ALARM_INPUT_6                      = 26U,
    ADRV904X_STREAM_GPIO_ALARM_INPUT_7                      = 27U,
    ADRV904X_STREAM_GPIO_ALARM_INPUT_8                      = 28U,
    ADRV904X_STREAM_GPIO_ALARM_INPUT_9                      = 29U,
    ADRV904X_STREAM_GPIO_PA_EN_OUT_0                        = 30U,
    ADRV904X_STREAM_GPIO_PA_EN_OUT_1                        = 31U,
    ADRV904X_STREAM_GPIO_PA_EN_OUT_2                        = 32U,
    ADRV904X_STREAM_GPIO_PA_EN_OUT_3                        = 33U,
    ADRV904X_STREAM_GPIO_PA_EN_OUT_4                        = 34U,
    ADRV904X_STREAM_GPIO_PA_EN_OUT_5                        = 35U,
    ADRV904X_STREAM_GPIO_PA_EN_OUT_6                        = 36U,
    ADRV904X_STREAM_GPIO_PA_EN_OUT_7                        = 37U,
    ADRV904X_STREAM_GPIO_LNA_EN_OUT_0                       = 38U,
    ADRV904X_STREAM_GPIO_LNA_EN_OUT_1                       = 39U,
    ADRV904X_STREAM_GPIO_LNA_EN_OUT_2                       = 40U,
    ADRV904X_STREAM_GPIO_LNA_EN_OUT_3                       = 41U,
    ADRV904X_STREAM_GPIO_LNA_EN_OUT_4                       = 42U,
    ADRV904X_STREAM_GPIO_LNA_EN_OUT_5                       = 43U,
    ADRV904X_STREAM_GPIO_LNA_EN_OUT_6                       = 44U,
    ADRV904X_STREAM_GPIO_LNA_EN_OUT_7                       = 45U,
    ADRV904X_STREAM_GPIO_OC_FUSE_EN_OUT_0                   = 46U,
    ADRV904X_STREAM_GPIO_OC_FUSE_EN_OUT_1                   = 47U,
    ADRV904X_STREAM_GPIO_OC_FUSE_EN_OUT_2                   = 48U,
    ADRV904X_STREAM_GPIO_OC_FUSE_EN_OUT_3                   = 49U,
    ADRV904X_STREAM_GPIO_OC_FUSE_EN_OUT_4                   = 50U,
    ADRV904X_STREAM_GPIO_OC_FUSE_EN_OUT_5                   = 51U,
    ADRV904X_STREAM_GPIO_OC_FUSE_EN_OUT_6                   = 52U,
    ADRV904X_STREAM_GPIO_OC_FUSE_EN_OUT_7                   = 53U,
    ADRV904X_STREAM_GPIO_SBET_LATCH_DPD_MODEL_INDEX         = 59U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX0_MAP_BIT0            = 60U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX0_MAP_BIT1            = 61U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX0_MAP_BIT2            = 62U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX0_MAP_BIT3            = 63U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX1_MAP_BIT0            = 64U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX1_MAP_BIT1            = 65U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX1_MAP_BIT2            = 66U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX1_MAP_BIT3            = 67U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT0         = 68U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT1         = 69U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT2         = 70U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT3         = 71U,
    ADRV904X_STREAM_GPIO_DTX_INPUT_0                        = 72U,
    ADRV904X_STREAM_GPIO_DTX_INPUT_1                        = 73U,
    ADRV904X_STREAM_GPIO_DTX_INPUT_2                        = 74U,
    ADRV904X_STREAM_GPIO_DTX_INPUT_3                        = 75U,
    ADRV904X_STREAM_GPIO_DTX_INPUT_4                        = 76U,
    ADRV904X_STREAM_GPIO_DTX_INPUT_5                        = 77U,
    ADRV904X_STREAM_GPIO_DTX_INPUT_6                        = 78U,
    ADRV904X_STREAM_GPIO_DTX_INPUT_7                        = 79U,
    ADRV904X_STREAM_GPIO_ANTENNA_CAL_OUT_PIN0               = 80U,
    ADRV904X_STREAM_GPIO_ANTENNA_CAL_OUT_PIN1               = 81U,
    ADRV904X_STREAM_GPIO_ANTENNA_CAL_OUT_PIN2               = 82U,
    ADRV904X_STREAM_GPIO_ANTENNA_CAL_OUT_PIN3               = 83U,
    ADRV904X_STREAM_GPIO_ANTENNA_CAL_OUT_PIN4               = 84U,
    ADRV904X_STREAM_GPIO_ANTENNA_CAL_OUT_PIN5               = 85U,
    ADRV904X_STREAM_GPIO_ANTENNA_CAL_OUT_PIN6               = 86U,
    ADRV904X_STREAM_GPIO_ANTENNA_CAL_OUT_PIN7               = 87U,
    ADRV904X_STREAM_GPIO_TDD_SW                             = 88U,
    ADRV904X_STREAM_GPIO_PREDRIVE_EN                        = 89U,
    ADRV904X_STREAM_GPIO_RS_PATTERN_SWITCHING               = 90U,
    ADRV904X_STREAM_GPIO_MODEL_SWITCH_INPUT_TX0             = 91U,
    ADRV904X_STREAM_GPIO_MODEL_SWITCH_INPUT_TX1             = 92U,
    ADRV904X_STREAM_GPIO_MODEL_SWITCH_INPUT_TX2             = 93U,
    ADRV904X_STREAM_GPIO_MODEL_SWITCH_INPUT_TX3             = 94U,
    ADRV904X_STREAM_GPIO_MODEL_SWITCH_INPUT_TX4             = 95U,
    ADRV904X_STREAM_GPIO_MODEL_SWITCH_INPUT_TX5             = 96U,
    ADRV904X_STREAM_GPIO_MODEL_SWITCH_INPUT_TX6             = 97U,
    ADRV904X_STREAM_GPIO_MODEL_SWITCH_INPUT_TX7             = 98U,
    ADRV904X_STREAM_GPIO_RS_SSB_SYNC_PIN                    = 99U,
    ADRV904X_STREAM_GPIO_AUX_GRP0_OUT_0                     = 100U,
    ADRV904X_STREAM_GPIO_AUX_GRP0_OUT_1                     = 101U,
    ADRV904X_STREAM_GPIO_AUX_GRP0_OUT_2                     = 102U,
    ADRV904X_STREAM_GPIO_AUX_GRP0_OUT_3                     = 103U,
    ADRV904X_STREAM_GPIO_AUX_GRP0_OUT_4                     = 104U,
    ADRV904X_STREAM_GPIO_AUX_GRP0_OUT_5                     = 105U,
    ADRV904X_STREAM_GPIO_AUX_GRP0_OUT_6                     = 106U,
    ADRV904X_STREAM_GPIO_AUX_GRP0_OUT_7                     = 107U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX0_MAP_BIT4            = 108U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX0_MAP_BIT5            = 109U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX0_MAP_BIT6            = 110U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX0_MAP_BIT7            = 111U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX1_MAP_BIT4            = 112U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX1_MAP_BIT5            = 113U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX1_MAP_BIT6            = 114U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX1_MAP_BIT7            = 115U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT4         = 116U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT5         = 117U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT6         = 118U,
    ADRV904X_STREAM_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT7         = 119U,
    ADRV904X_STREAM_GPIO_AUX_GRP1_OUT_0                     = 120U,
    ADRV904X_STREAM_GPIO_AUX_GRP1_OUT_1                     = 121U,
    ADRV904X_STREAM_GPIO_AUX_GRP1_OUT_2                     = 122U,
    ADRV904X_STREAM_GPIO_AUX_GRP1_OUT_3                     = 123U,
    ADRV904X_STREAM_GPIO_AUX_GRP1_OUT_4                     = 124U,
    ADRV904X_STREAM_GPIO_AUX_GRP1_OUT_5                     = 125U,
    ADRV904X_STREAM_GPIO_AUX_GRP1_OUT_6                     = 126U,
    ADRV904X_STREAM_GPIO_AUX_GRP1_OUT_7                     = 127U,

    ADRV904X_STREAM_GPIO_VSWR_DIR_ORX0                      = 252U,
    ADRV904X_STREAM_GPIO_VSWR_DIR_ORX1                      = 253U,
    ADRV904X_STREAM_GPIO_VSWR_DIR_ORX_CMN                   = 254U,
    ADRV904X_STREAM_GPIO_UNUSED                             = 255U
} adrv904x_StreamGpioFeatureSelection_e;

typedef enum adrv904x_StreamAnaGpioFeatureSelection
{

    ADRV904X_STREAM_ANA_GPIO_PA_EN_OUT_0                         = 30U,
    ADRV904X_STREAM_ANA_GPIO_PA_EN_OUT_1                         = 31U,
    ADRV904X_STREAM_ANA_GPIO_PA_EN_OUT_2                         = 32U,
    ADRV904X_STREAM_ANA_GPIO_PA_EN_OUT_3                         = 33U,
    ADRV904X_STREAM_ANA_GPIO_PA_EN_OUT_4                         = 34U,
    ADRV904X_STREAM_ANA_GPIO_PA_EN_OUT_5                         = 35U,
    ADRV904X_STREAM_ANA_GPIO_PA_EN_OUT_6                         = 36U,
    ADRV904X_STREAM_ANA_GPIO_PA_EN_OUT_7                         = 37U,
    ADRV904X_STREAM_ANA_GPIO_LNA_EN_OUT_0                        = 38U,
    ADRV904X_STREAM_ANA_GPIO_LNA_EN_OUT_1                        = 39U,
    ADRV904X_STREAM_ANA_GPIO_LNA_EN_OUT_2                        = 40U,
    ADRV904X_STREAM_ANA_GPIO_LNA_EN_OUT_3                        = 41U,
    ADRV904X_STREAM_ANA_GPIO_LNA_EN_OUT_4                        = 42U,
    ADRV904X_STREAM_ANA_GPIO_LNA_EN_OUT_5                        = 43U,
    ADRV904X_STREAM_ANA_GPIO_LNA_EN_OUT_6                        = 44U,
    ADRV904X_STREAM_ANA_GPIO_LNA_EN_OUT_7                        = 45U,
    ADRV904X_STREAM_ANA_GPIO_OC_FUSE_EN_OUT_0                    = 46U,
    ADRV904X_STREAM_ANA_GPIO_OC_FUSE_EN_OUT_1                    = 47U,
    ADRV904X_STREAM_ANA_GPIO_OC_FUSE_EN_OUT_2                    = 48U,
    ADRV904X_STREAM_ANA_GPIO_OC_FUSE_EN_OUT_3                    = 49U,
    ADRV904X_STREAM_ANA_GPIO_OC_FUSE_EN_OUT_4                    = 50U,
    ADRV904X_STREAM_ANA_GPIO_OC_FUSE_EN_OUT_5                    = 51U,
    ADRV904X_STREAM_ANA_GPIO_OC_FUSE_EN_OUT_6                    = 52U,
    ADRV904X_STREAM_ANA_GPIO_OC_FUSE_EN_OUT_7                    = 53U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX0_MAP_BIT0          = 60U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX0_MAP_BIT1          = 61U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX0_MAP_BIT2          = 62U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX0_MAP_BIT3          = 63U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX1_MAP_BIT0          = 64U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX1_MAP_BIT1          = 65U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX1_MAP_BIT2          = 66U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX1_MAP_BIT3          = 67U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT0       = 68U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT1       = 69U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT2       = 70U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT3       = 71U,
    ADRV904X_STREAM_ANALOG_GPIO_ANTENNA_CAL_OUT_PIN0             = 80U,
    ADRV904X_STREAM_ANALOG_GPIO_ANTENNA_CAL_OUT_PIN1             = 81U,
    ADRV904X_STREAM_ANALOG_GPIO_ANTENNA_CAL_OUT_PIN2             = 82U,
    ADRV904X_STREAM_ANALOG_GPIO_ANTENNA_CAL_OUT_PIN3             = 83U,
    ADRV904X_STREAM_ANALOG_GPIO_ANTENNA_CAL_OUT_PIN4             = 84U,
    ADRV904X_STREAM_ANALOG_GPIO_ANTENNA_CAL_OUT_PIN5             = 85U,
    ADRV904X_STREAM_ANALOG_GPIO_ANTENNA_CAL_OUT_PIN6             = 86U,
    ADRV904X_STREAM_ANALOG_GPIO_ANTENNA_CAL_OUT_PIN7             = 87U,
    ADRV904X_STREAM_ANALOG_GPIO_TDD_SW                           = 88U,
    ADRV904X_STREAM_ANALOG_GPIO_PREDRIVE_EN                      = 89U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP0_OUT_0                   = 100U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP0_OUT_1                   = 101U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP0_OUT_2                   = 102U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP0_OUT_3                   = 103U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP0_OUT_4                   = 104U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP0_OUT_5                   = 105U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP0_OUT_6                   = 106U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP0_OUT_7                   = 107U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX0_MAP_BIT4          = 108U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX0_MAP_BIT5          = 109U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX0_MAP_BIT6          = 110U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX0_MAP_BIT7          = 111U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX1_MAP_BIT4          = 112U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX1_MAP_BIT5          = 113U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX1_MAP_BIT6          = 114U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX1_MAP_BIT7          = 115U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT4       = 116U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT5       = 117U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT6       = 118U,
    ADRV904X_STREAM_ANALOG_GPIO_DFE_TX_TO_ORX_CMN_MAP_BIT7       = 119U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP1_OUT_0                   = 120U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP1_OUT_1                   = 121U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP1_OUT_2                   = 122U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP1_OUT_3                   = 123U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP1_OUT_4                   = 124U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP1_OUT_5                   = 125U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP1_OUT_6                   = 126U,
    ADRV904X_STREAM_ANALOG_GPIO_AUX_GRP1_OUT_7                   = 127U,
    ADRV904X_STREAM_ANA_GPIO_UNUSED                              = 255U
} adrv904x_StreamAnaGpioFeatureSelection_e;
#endif /* _ADRV904X_RADIOCTRL_TYPES_H_ */
