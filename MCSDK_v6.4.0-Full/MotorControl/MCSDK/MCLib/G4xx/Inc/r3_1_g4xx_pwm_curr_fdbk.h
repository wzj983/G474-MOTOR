/**
  ******************************************************************************
  * @file    r3_1_g4xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          R3_1_G4XX_pwm_curr_fdbk component of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup R3_1_G4XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef R3_1_G4XX_PWMNCURRFDBK_H
#define R3_1_G4XX_PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

/* Exported defines --------------------------------------------------------*/
#define GPIO_NoRemap_TIM1 ((uint32_t)(0))
#define SHIFTED_TIMs      ((uint8_t) 1)
#define NO_SHIFTED_TIMs   ((uint8_t) 0)

#define NONE    ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup R3_1_pwm_curr_fdbk
  * @{
  */

/* Exported types ------------------------------------------------------- */

#ifndef R3_3_OPAMP
#define R3_3_OPAMP
/**
  * @brief  Current feedback component 3-OPAMP parameters structure definition. Specific to G4XX.
  */
typedef const struct
{
  /* First OPAMP settings ------------------------------------------------------*/
  OPAMP_TypeDef *OPAMPSelect_1 [6] ;  /*!< Define for each sector first conversion which OPAMP is involved - Null otherwise */
  OPAMP_TypeDef *OPAMPSelect_2 [6] ;  /*!< Define for each sector second conversion which OPAMP is involved - Null otherwise */
  uint32_t OPAMPConfig1 [6]; /*!< Define the OPAMP_CSR_OPAMPINTEN and the OPAMP_CSR_VPSEL config for each ADC conversions*/
  uint32_t OPAMPConfig2 [6]; /*!< Define the OPAMP_CSR_OPAMPINTEN and the OPAMP_CSR_VPSEL config for each ADC conversions*/
} R3_3_OPAMPParams_t;
#endif

/*
  * @brief  PWM and current feedback component parameters definition for single ADC configurations.
  */
typedef const struct
{
  /* HW IP involved ------------------------------------------------------------- */
  ADC_TypeDef * ADCx;                       /* ADC peripheral to be used */
  TIM_TypeDef * TIMx;                       /* timer used for PWM generation */
  R3_3_OPAMPParams_t * OPAMPParams;         /*!< Pointer to the OPAMP params struct.
                                             *   It must be #MC_NULL if internal OPAMP are not used. Specific to G4XX */
  COMP_TypeDef * CompOCPASelection;         /* Internal comparator used for Phase A protection */
  COMP_TypeDef * CompOCPBSelection;         /* Internal comparator used for Phase B protection */
  COMP_TypeDef * CompOCPCSelection;         /* Internal comparator used for Phase C protection */
  COMP_TypeDef * CompOVPSelection;          /* Internal comparator used for Over Voltage protection */
  DAC_TypeDef * DAC_OCP_ASelection;         /*!< DAC used for Phase A protection. Specific to G4XX */
  DAC_TypeDef * DAC_OCP_BSelection;         /*!< DAC used for Phase B protection. Specific to G4XX */
  DAC_TypeDef * DAC_OCP_CSelection;         /*!< DAC used for Phase C protection. Specific to G4XX */
  DAC_TypeDef * DAC_OVP_Selection;          /*!< DAC used for Over Voltage protection. Specific to G4XX */
  uint32_t DAC_Channel_OCPA;                /*!< DAC channel used for Phase A current protection. Specific to G4XX */
  uint32_t DAC_Channel_OCPB;                /*!< DAC channel used for Phase B current protection. Specific to G4XX */
  uint32_t DAC_Channel_OCPC;                /*!< DAC channel used for Phase C current protection. Specific to G4XX */
  uint32_t DAC_Channel_OVP;                 /*!< DAC channel used for Over Voltage protection. Specific to G4XX */

  /* Currents sampling parameters ----------------------------------------------- */
  uint16_t Tafter;                          /* Sum of dead time plus max value between rise time and noise time expressed in number of TIM clocks */
  uint16_t Tbefore;                         /* Total time of the sampling sequence expressed in number of TIM clocks */
  uint16_t Tcase2;                          /* Sum of dead time, noise time and sampling time divided by 2 ; expressed in number of TIM clocks */
  uint16_t Tcase3;                          /* Sum of dead time, rise time and sampling time ; expressed in number of TIM clocks */
  uint32_t ADCConfig[6];                    /* Stores ADC sequence for the 6 sectors */

  /* DAC settings --------------------------------------------------------------- */
  uint16_t DAC_OCP_Threshold;               /* Value of analog reference expressed
                                             * as 16bit unsigned integer.
                                             * Ex. 0 = 0V ; 65536 = VDD_DAC.*/
  uint16_t DAC_OVP_Threshold;               /* Value of analog reference expressed
                                             * as 16bit unsigned integer.
                                             * Ex. 0 = 0V ; 65536 = VDD_DAC. */

  /* PWM Driving signals initialization ----------------------------------------- */
  uint8_t  RepetitionCounter;               /* Number of elapsed PWM periods before Compare Registers are updated again.
                                             * In particular : RepetitionCounter = (2 * PWM periods) - 1 */
                                             
  /* Internal COMP settings ----------------------------------------------------- */
  uint8_t       CompOCPAInvInput_MODE;      /* COMPx inverting input mode. It must be either
                                             * equal to EXT_MODE or INT_MODE. */
  uint8_t       CompOCPBInvInput_MODE;      /* COMPx inverting input mode. It must be either
                                             * equal to EXT_MODE or INT_MODE. */
  uint8_t       CompOCPCInvInput_MODE;      /* COMPx inverting input mode. It must be either
                                             * equal to EXT_MODE or INT_MODE. */
  uint8_t       CompOVPInvInput_MODE;       /* COMPx inverting input mode. It must be either
                                             * equal to EXT_MODE or INT_MODE. */

  /* Dual MC parameters --------------------------------------------------------- */
  uint8_t  FreqRatio;                       /* Used in case of dual MC to
                                             * synchronize TIM1 and TIM8. It has
                                             * effect only on the second instanced
                                             * object and must be equal to the
                                             * ratio between the two PWM frequencies
                                             * (higher/lower). Supported values are
                                             * 1, 2 or 3 */
  uint8_t  IsHigherFreqTim;                 /* When FreqRatio is greater than 1
                                             * this param is used to indicate if this
                                             * instance is the one with the highest
                                             * frequency. Allowed value are: HIGHER_FREQ
                                             * or LOWER_FREQ */

} R3_1_Params_t, *pR3_1_Params_t;

/*
  * @brief  PWM and current feedback component for single ADC configurations.
  */
typedef struct
{
  PWMC_Handle_t _Super;                     /* Base component handler */
  uint32_t PhaseAOffset;                    /* Offset of Phase A current sensing network */
  uint32_t PhaseBOffset;                    /* Offset of Phase B current sensing network */
  uint32_t PhaseCOffset;                    /* Offset of Phase C current sensing network */
  uint16_t Half_PWMPeriod;                  /* Half PWM Period in timer clock counts */
  volatile uint32_t ADCTriggerEdge;         /* Polarity of the ADC triggering, can be either on rising or falling edge */
  volatile uint8_t PolarizationCounter;     /* Number of conversions performed during the calibration phase */
  uint8_t PolarizationSector;               /* Sector selected during calibration phase */

  pR3_1_Params_t pParams_str;
} PWMC_R3_1_Handle_t;

/* Exported functions ------------------------------------------------------- */

/*
  * Initializes TIM1, ADC1, GPIO, DMA1 and NVIC for three shunt current
  * reading configuration using STM32G4xx.
  */
void R3_1_Init(PWMC_R3_1_Handle_t *pHandle);

/*
  * Stores into the handler the voltage present on the
  * current feedback analog channel when no current is flowing into the
  * motor.
  */
void R3_1_CurrentReadingPolarization(PWMC_Handle_t *pHdl);

/*
  * Computes and returns latest converted motor phase currents.
  */
void R3_1_GetPhaseCurrents(PWMC_Handle_t *pHdl, ab_t *Iab);

/*
  * Computes and returns latest converted motor phase currents.
  */
void R3_1_GetPhaseCurrents_OVM(PWMC_Handle_t *pHdl, ab_t *Iab);

/*
  * Turns on low side switches.
  */
void R3_1_TurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks );

/*
  * Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R3_1_SwitchOnPWM( PWMC_Handle_t * pHdl );

/*
  * Disables PWM generation on the proper Timer peripheral acting on MOE bit and resets the TIM status.
  */
void R3_1_SwitchOffPWM( PWMC_Handle_t * pHdl );

/*
  * Configures the ADC for the currents sampling.
  */
uint16_t R3_1_SetADCSampPointSectX(PWMC_Handle_t *pHdl);

/*
  * Configures the ADC for the current sampling. Specific to overmodulation.
  */
uint16_t R3_1_SetADCSampPointSectX_OVM(PWMC_Handle_t *pHdl);

/*
  *  Contains the TIMx Update event interrupt.
  */
void *R3_1_TIMx_UP_IRQHandler(PWMC_R3_1_Handle_t *pHandle);

/*
  * Sets the calibrated offset.
  */
void R3_1_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
  * Reads the calibrated offsets.
  */
void R3_1_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
  * @brief  Sets the PWM mode for R/L detection.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
void R3_1_RLDetectionModeEnable(PWMC_Handle_t *pHdl);

/*
  * @brief  Disables the PWM mode for R/L detection.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
void R3_1_RLDetectionModeDisable(PWMC_Handle_t *pHdl);

/*
  * @brief  Sets the PWM dutycycle for R/L detection.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  hDuty: Duty cycle to apply, written in uint16_t.
  * @retval Returns #MC_NO_ERROR if no error occurred or #MC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
uint16_t R3_1_RLDetectionModeSetDuty(PWMC_Handle_t *pHdl, uint16_t hDuty);

/*
 * @brief  Turns on low sides switches and start ADC triggering.
 * 
 * This function is specific for MP phase.
 *
 * @param  pHdl: Handler of the current instance of the PWM component.
 */
void R3_1_RLTurnOnLowSidesAndStart(PWMC_Handle_t *pHdl);


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*R3_1_G4XX_PWMNCURRFDBK_H*/

/************************ (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
