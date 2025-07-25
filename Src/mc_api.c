
/**
  ******************************************************************************
  * @file    mc_api.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements the high level interface of the Motor Control SDK.
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
  * @ingroup MCIAPI
  */

#include "mc_interface.h"
#include "mc_api.h"
#include "mc_config.h"
#include "mcp.h"

/** @addtogroup MCSDK
  * @{
  */

/**
  * @defgroup CAI Application Programming Interface
  * @brief Interface for Motor Control applications using the classic SDK
  *
  * @{
  */

/** @defgroup MCIAPI Motor Control API
  *
  * @brief High level Programming Interface of the Motor Control SDK
  *
  *  This interface allows for performing basic operations on the motor(s) driven by an
  * Motor Control SDK based application. With it, motors can be started and stopped, speed or
  * torque ramps can be programmed and executed and information on the state of the motors can
  * be retrieved, among others.
  *
  *  This interface consists in functions that target a specific motor, indicated in their name.
  * These functions aims at being the main interface used by an Application to control motors.
  *
  *  The current Motor Control API can cope with up to 2 motors.
  * @{
  */

/**
  * @brief  Initiates the start-up procedure for Motor 1
  *
  *  If the state machine of Motor 1 is in #IDLE state, the command is immediately
  * executed. Otherwise the command is discarded. The Application can check the
  * return value to know whether the command was executed or discarded.
  *
  *  One of the following commands must be executed before calling MC_StartMotor1()
  * in order to set a torque or a speed reference:
  *
  * - MC_ProgramSpeedRampMotor1()
  * - MC_ProgramTorqueRampMotor1()
  * - MC_SetCurrentReferenceMotor1()
  *
  * Failing to do so results in an unpredictable behaviour.
  *
  * If the offsets of the current measurement circuitry offsets are not known yet,
  * an offset calibration procedure is executed to measure them prior to acutally
  * starting up the motor.
  *
  * @note The MCI_StartMotor1 command only triggers the execution of the start-up
  * procedure (or eventually the offset calibration procedure) and returns
  * immediately after. It is not blocking the execution of the application until
  * the motor is indeed running in steady state. If the application needs to wait
  * for the motor to be running in steady state, the application has to check the
  * state machine of the motor and verify that the #RUN state has been reached.
  * Note also that if the startup sequence fails the #RUN state may never be reached.
  *
  * @retval returns true if the command is successfully executed, false otherwise.
  */
__weak bool MC_StartMotor1(void)
{
  return (MCI_StartMotor(pMCI[M1]));
}

/**
  * @brief  Initiates the stop procedure for Motor 1.
  *
  *  If the state machine is in any state but the #ICLWAIT, #IDLE, FAULT_NOW and
  * #FAULT_OVER states, the command is immediately executed. Otherwise, it is
  * discarded. The Application can check the return value to know whether the
  * command was executed or discarded.
  *
  * @note The MC_StopMotor1() command only triggers the stop motor procedure
  * and then returns. It is not blocking the application until the motor is indeed
  * stopped. To know if it has stopped, the application can query the motor's state
  * machine and check if the #IDLE state has been reached.
  *
  * @retval returns true if the command is successfully executed, false otherwise.
  */
__weak bool MC_StopMotor1(void)
{
  return (MCI_StopMotor(pMCI[M1]));
}

/**
  * @brief Programs a speed ramp for Motor 1 for later or immediate execution.
  *
  *  A speed ramp is a linear change from the current speed reference to the @p hFinalSpeed
  * target speed in the given @p hDurationms time.
  *
  *  Invoking the MC_ProgramSpeedRampMotor1() function programs a new speed ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 1's state machine is in the #RUN states. Otherwise, the ramp is buffered
  * and will be executed when the state machine reaches any of the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * speed or a torque one - or if another buffered command is programmed before the
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using
  * sensorless motor control techniques. If the sign of the hFinalSpeed parameter
  * differs from that of the current speed, the ramp will not complete and a Speed
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to
  * reach 0 rpm.
  *
  * @param  hFinalSpeed Mechanical rotor speed reference at the end of the ramp.
  *                     Expressed in the unit defined by #SPEED_UNIT.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the speed
  *         value.
  */
__weak void MC_ProgramSpeedRampMotor1(int16_t hFinalSpeed, uint16_t hDurationms)
{
  MCI_ExecSpeedRamp(pMCI[M1], hFinalSpeed, hDurationms);
}

/**
  * @brief Programs a speed ramp for Motor 1 for later or immediate execution.
  *
  *  A speed ramp is a linear change from the current speed reference to the @p FinalSpeed
  * target speed in the given @p hDurationms time.
  *
  *  Invoking the MC_ProgramSpeedRampMotor1() function programs a new speed ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 1's state machine is in the #RUN states. Otherwise, the ramp is buffered
  * and will be executed when the state machine reaches any of the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * speed or a torque one - or if another buffered command is programmed before the
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using
  * sensorless motor control techniques. If the sign of the hFinalSpeed parameter
  * differs from that of the current speed, the ramp will not complete and a Speed
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to
  * reach 0 rpm.
  *
  * @param  FinalSpeed Mechanical rotor speed reference at the end of the ramp.
  *         Expressed in rpm.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the speed
  *         value.
  */
__weak void MC_ProgramSpeedRampMotor1_F(float_t FinalSpeed, uint16_t hDurationms)
{
  MCI_ExecSpeedRamp_F(pMCI[M1], FinalSpeed, hDurationms);
}

/**
  * @brief Programs a torque ramp for Motor 1 for later or immediate execution.
  *
  *  A torque ramp is a linear change from the current torque reference to the @p hFinalTorque
  * target torque reference in the given @p hDurationms time.
  *
  *  Invoking the MC_ProgramTorqueRampMotor1() function programs a new torque ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 1's state machine is in the #RUN states. Otherwise, the ramp is buffered
  * and will be executed when the state machine reaches any of the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * torque or a speed one - or if another buffered command is programmed before the
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using
  * sensorless motor control techniques. If the sign of the hFinalTorque parameter
  * differs from that of the current torque, the ramp will not complete and a Speed
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to
  * reach 0 rpm.
  *
  * @param  hFinalTorque Mechanical motor torque reference at the end of the ramp.
  *         This value represents actually the Iq current expressed in digit.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the torque
  *         value.
  */
__weak void MC_ProgramTorqueRampMotor1(int16_t hFinalTorque, uint16_t hDurationms)
{
  MCI_ExecTorqueRamp(pMCI[M1], hFinalTorque, hDurationms);
}

/**
  * @brief Programs a torque ramp for Motor 1 for later or immediate execution.
  *
  *  A torque ramp is a linear change from the current torque reference to the @p FinalTorque
  * target torque reference in the given @p hDurationms time.
  *
  *  Invoking the MC_ProgramTorqueRampMotor1() function programs a new torque ramp
  * with the provided parameters. The programmed ramp is executed immediately if
  * Motor 1's state machine is in the #RUN states. Otherwise, the ramp is buffered
  * and will be executed when the state machine reaches any of the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another ramp - whether a
  * torque or a speed one - or if another buffered command is programmed before the
  * current one has completed, the latter replaces the former.
  *
  * @note A ramp cannot reverse the rotation direction if the Application is using
  * sensorless motor control techniques. If the sign of the FinalTorque parameter
  * differs from that of the current torque, the ramp will not complete and a Speed
  * Feedback error (#MC_SPEED_FDBK) will occur when the rotation speed is about to
  * reach 0 rpm.
  *
  * @param  FinalTorque Mechanical motor torque reference at the end of the ramp.
  *         This value represents actually the Iq current expressed in Ampere.
  * @param  hDurationms Duration of the ramp expressed in milliseconds. It
  *         is possible to set 0 to perform an instantaneous change in the torque
  *         value.
  */
__weak void MC_ProgramTorqueRampMotor1_F(float_t FinalTorque, uint16_t hDurationms)
{
  MCI_ExecTorqueRamp_F(pMCI[M1], FinalTorque, hDurationms);
}

/**
  * @brief Programs the current reference to Motor 1 for later or immediate execution.
  *
  *  The current reference to consider is made of the $I_d$ and $I_q$ current components.
  *
  *  Invoking the MC_SetCurrentReferenceMotor1() function programs a current reference
  * with the provided parameters. The programmed reference is executed immediately if
  * Motor 1's state machine is in the #RUN states. Otherwise, the command is buffered
  * and will be executed when the state machine reaches any of the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another buffered command is
  * programmed before the current one has completed, the latter replaces the former.
  *
  * @param  Iqdref current reference in the Direct-Quadratic reference frame. Expressed
  *         in the qd_t format.
  */
__weak void MC_SetCurrentReferenceMotor1(qd_t Iqdref)
{
  MCI_SetCurrentReferences(pMCI[M1], Iqdref);
}

/**
  * @brief Programs the current reference to Motor 1 for later or immediate execution.
  *
  *  The current reference to consider is made of the $I_d$ and $I_q$ current components.
  *
  *  Invoking the MC_SetCurrentReferenceMotor1_F() function programs a current reference
  * with the provided parameters. The programmed reference is executed immediately if
  * Motor 1's state machine is in the #RUN states. Otherwise, the command is buffered
  * and will be executed when the state machine reaches any of the aforementioned state.
  *
  *  The Application can check the status of the command with the MC_GetCommandStateMotor1()
  * to know whether the last command was executed immediately or not.
  *
  * Only one command can be buffered at any given time. If another buffered command is
  * programmed before the current one has completed, the latter replaces the former.
  *
  * @param  IqdRef current reference in the Direct-Quadratic reference frame. Expressed
  *         in the qd_f_t format.
  */
__weak void MC_SetCurrentReferenceMotor1_F(qd_f_t IqdRef)
{
  MCI_SetCurrentReferences_F(pMCI[M1], IqdRef);
}

/**
  * @brief  Returns the status of the last buffered command for Motor 1.
  *
  * The status can be one of the following values:
  * - #MCI_BUFFER_EMPTY: no buffered command is currently programmed.
  * - #MCI_COMMAND_NOT_ALREADY_EXECUTED: A command has been buffered but the conditions for its
  *   execution have not occurred yet. The command is still in the buffer, pending execution.
  * - #MCI_COMMAND_EXECUTED_SUCCESSFULLY: the last buffered command has been executed successfully.
  *   In this case calling this function resets the command state to #MCI_BUFFER_EMPTY.
  * - #MCI_COMMAND_EXECUTED_UNSUCCESSFULLY: the buffered command has been executed unsuccessfully.
  *   In this case calling this function resets the command state to #MCI_BUFFER_EMPTY.
  */
__weak MCI_CommandState_t  MC_GetCommandStateMotor1(void)
{
  return (MCI_IsCommandAcknowledged(pMCI[M1]));
}

/**
 * @brief Stops the execution of the on-going speed ramp for Motor 1, if any.
 *
 *  If a speed ramp is currently being executed, it is immediately stopped, the rotation
 * speed of Motor 1 is maintained to its current value and true is returned. If no speed
 * ramp is on-going, nothing is done and false is returned.
 *
 * @deprecated This function is deprecated and should not be used anymore. It will be
 *             removed in a future version of the MCSDK. Use MC_StopRampMotor1() instead.
 */
__weak bool MC_StopSpeedRampMotor1(void)
{
  return (MCI_StopSpeedRamp(pMCI[M1]));
}

/**
 * @brief Stops the execution of the on-going ramp for Motor 1, if any.
 *
 *  If a ramp is currently being executed, it is immediately stopped, the torque or the speed
 *  of Motor 1 is maintained to its current value.
 */
__weak void MC_StopRampMotor1(void)
{
  MCI_StopRamp(pMCI[M1]);
}

/**
 * @brief Returns true if the last ramp submited for Motor 1 has completed, false otherwise
 */
__weak bool MC_HasRampCompletedMotor1(void)
{
  return (MCI_RampCompleted(pMCI[M1]));
}

/**
 *  @brief Returns the current mechanical rotor speed reference set for Motor 1, expressed in the unit defined by #SPEED_UNIT
 */
__weak int16_t MC_GetMecSpeedReferenceMotor1(void)
{
  return (MCI_GetMecSpeedRefUnit(pMCI[M1]));
}

/**
 *  @brief Returns the current mechanical rotor speed reference set for Motor 1, expressed in rpm.
 */
__weak float_t MC_GetMecSpeedReferenceMotor1_F(void)
{
  return (MCI_GetMecSpeedRef_F(pMCI[M1]));
}

/**
 * @brief Returns the last computed average mechanical rotor speed for Motor 1, expressed in the unit defined by #SPEED_UNIT
 */
__weak int16_t MC_GetMecSpeedAverageMotor1(void)
{
  return (MCI_GetAvrgMecSpeedUnit(pMCI[M1]));
}

#define S16ToRAD 10430.37835f            /* 2^16/2Pi */

/**
 * @brief Returns the last computed average mechanical rotor speed from auxiliary sensor for Motor 1, expressed in the unit defined by #SPEED_UNIT
 */
__weak int16_t MC_GetMecAuxiliarySpeedAverageM1(void)
{
  return (SPD_GetAvrgMecSpeedUnit(&STO_PLL_M1._Super));
}

/**
 * @brief Returns the last computed average mechanical rotor speed from auxiliary sensor for Motor 1, expressed in RPM
 */
__weak float_t MC_GetMecAuxiliarySpeedAvgM1_F(void)
{
  int16_t returnValue;
  returnValue = (SPD_GetAvrgMecSpeedUnit(&STO_PLL_M1._Super) * U_RPM) / SPEED_UNIT;
  return ((float_t)returnValue);
}

/**
 * @brief Returns the electrical angle of the rotor from auxiliary sensor of Motor 1, in DDP format
 */
__weak int16_t MC_GetAuxiliaryElAngledppMotor1(void)
{
  return (SPD_GetElAngle(&STO_PLL_M1._Super));
}

/**
 * @brief Returns the electrical angle of the rotor from auxiliary sensor of Motor 1, expressed in radians
 */
__weak float_t MC_GetAuxiliaryElAngleMotor1_F(void)
{
  return (((float_t)SPD_GetElAngle(&STO_PLL_M1._Super)) / S16ToRAD);
}

/**
 * @brief Returns the last computed average mechanical rotor speed for Motor 1, expressed in rpm.
 */
__weak float_t MC_GetAverageMecSpeedMotor1_F(void)
{
  return (MCI_GetAvrgMecSpeed_F(pMCI[M1]));
}

/**
 * @brief Returns the final speed of the last ramp programmed for Motor 1 if this ramp was a speed ramp, 0 otherwise.
 */
__weak int16_t MC_GetLastRampFinalSpeedMotor1(void)
{
  return (MCI_GetLastRampFinalSpeed(pMCI[M1]));
}

/**
 * @brief Returns the final speed of the last ramp programmed for Motor 1 if this ramp was a speed ramp, 0 otherwise.
 */
__weak float_t MC_GetLastRampFinalSpeedM1_F(void)
{
  return (MCI_GetLastRampFinalSpeed_F(pMCI[M1]));
}
/**
 * @brief Returns the final torque reference for Motor 1, expressed in Ampere.
 */
__weak float_t MC_GetFinalTorqueReferenceMotor1_F(void)
{
  return (MCI_GetLastRampFinalTorque_F(pMCI[M1]));
}

/**
 * @brief Returns the final torque reference for Motor 1, expressed in digit.
 */
__weak int16_t MC_GetFinalTorqueReferenceMotor1(void)
{
  return (MCI_GetLastRampFinalTorque(pMCI[M1]));
}
/**
 * @brief Returns the Control Mode used for Motor 1 (either Speed or Torque)
 */
__weak MC_ControlMode_t MC_GetControlModeMotor1(void)
{
  return (MCI_GetControlMode(pMCI[M1]));
}

/**
 * @brief Returns the rotation direction imposed by the last command on Motor 1
 *
 * The last command is either MC_ProgramSpeedRampMotor1(), MC_ProgramTorqueRampMotor1() or
 * MC_SetCurrentReferenceMotor1().
 *
 * The function returns -1 if the sign of the final speed, the final torque or the Iq current
 * reference component of the last command is negative. Otherwise, 1 is returned.
 *
 * @note if no such command has ever been submitted, 1 is returned as well.
 */
__weak int16_t MC_GetImposedDirectionMotor1(void)
{
  return (MCI_GetImposedMotorDirection(pMCI[M1]));
}

/**
 * @brief Returns true if the speed sensor used for Motor 1 is reliable, false otherwise
 */
__weak bool MC_GetSpeedSensorReliabilityMotor1(void)
{
  return (MCI_GetSpdSensorReliability(pMCI[M1]));
}

/**
 * @brief returns the amplitude of the phase current injected in Motor 1
 *
 * The returned amplitude (0-to-peak) is expressed in s16A unit. To convert it to amperes, use the following formula:
 *
 * @f[
 * I_{Amps} = \frac{ I_{s16A} \times V_{dd}}{ 65536 \times R_{shunt} \times A_{op} }
 * @f]
 *
 */
__weak int16_t MC_GetPhaseCurrentAmplitudeMotor1(void)
{
  return (MCI_GetPhaseCurrentAmplitude(pMCI[M1]));
}

/**
 * @brief returns the amplitude of the phase voltage applied to Motor 1
 *
 * The returned amplitude (0-to-peak) is expressed in s16V unit. To convert it to volts, use the following formula:
 *
 * @f[
 * U_{Volts} = \frac{ U_{s16V} \times V_{bus}}{ \sqrt{3} \times 32768  }
 * @f]
 *
 */
__weak int16_t MC_GetPhaseVoltageAmplitudeMotor1(void)
{
  return (MCI_GetPhaseVoltageAmplitude(pMCI[M1]));
}

/**
 * @brief returns Ia and Ib current values for Motor 1 in ab_t format
 */
__weak ab_t MC_GetIabMotor1(void)
{
  return (MCI_GetIab(pMCI[M1]));
}

/**
 * @brief returns Ia and Ib current values for Motor 1 in ab_f_t format
 */
__weak ab_f_t MC_GetIabMotor1_F(void)
{
  return (MCI_GetIab_F(pMCI[M1]));
}

/**
 * @brief returns Ialpha and Ibeta current values for Motor 1 in alphabeta_t format
 */
__weak alphabeta_t MC_GetIalphabetaMotor1(void)
{
  return (MCI_GetIalphabeta(pMCI[M1]));
}

/**
 * @brief returns Iq and Id current values for Motor 1 in qd_t format
 */
__weak qd_t MC_GetIqdMotor1(void)
{
  return (MCI_GetIqd(pMCI[M1]));
}

/**
 * @brief returns Iq and Id current values for Motor 1 in float_t type
 */
__weak qd_f_t MC_GetIqdMotor1_F(void)
{
  return (MCI_GetIqd_F(pMCI[M1]));
}

/**
 * @brief returns Iq and Id reference current values for Motor 1 in qd_t format
 */
__weak qd_t MC_GetIqdrefMotor1(void)
{
  return (MCI_GetIqdref(pMCI[M1]));
}

/**
 * @brief returns Iq and Id reference current values for Motor 1 in float_t type
 */
__weak qd_f_t MC_GetIqdrefMotor1_F(void)
{
  return (MCI_GetIqdref_F(pMCI[M1]));
}

/**
 * @brief returns Vq and Vd voltage values for Motor 1 in qd_t format
 */
__weak qd_t MC_GetVqdMotor1(void)
{
  return (MCI_GetVqd(pMCI[M1]));
}

/**
 * @brief returns Valpha and Vbeta voltage values for Motor 1 in alphabeta_t format
 */
__weak alphabeta_t MC_GetValphabetaMotor1(void)
{
  return (MCI_GetValphabeta(pMCI[M1]));
}

/**
 * @brief returns the electrical angle of the rotor of Motor 1, in DDP format
 */
__weak int16_t MC_GetElAngledppMotor1(void)
{
  return (MCI_GetElAngledpp(pMCI[M1]));
}

/**
 * @brief returns the electrical torque reference for Motor 1
 */
__weak int16_t MC_GetTerefMotor1(void)
{
  return (MCI_GetTeref(pMCI[M1]));
}

/**
 * @brief returns the electrical torque reference for Motor 1
 */
__weak float_t MC_GetTerefMotor1_F(void)
{
  return (MCI_GetTeref_F(pMCI[M1]));
}

/**
 * @brief re-initializes Iq and Id references to their default values for Motor 1
 *
 * The default values for the Iq and Id references are coming from the Speed
 * or the Torque controller depending on the control mode.
 *
 * @see   SpeednTorqCtrl for more details.
 */
__weak void MC_Clear_IqdrefMotor1(void)
{
  MCI_Clear_Iqdref(pMCI[M1]);
}

/**
 * @brief Acknowledge a Motor Control fault that occured on Motor 1
 *
 *  This function informs Motor 1's state machine that the Application has taken
 * the error condition that occured into account. If no error condition exists when
 * the function is called, nothing is done and false is returned. Otherwise, true is
 * returned.
 */
__weak bool MC_AcknowledgeFaultMotor1(void)
{
  return (MCI_FaultAcknowledged(pMCI[M1]));
}

/**
 * @brief Returns a bit-field showing non acknowledged faults that occurred on Motor 1.
 *
 * This function returns a 16 bit fields containing the Motor Control faults
 * that have occurred on Motor 1 since its state machine moved to the #FAULT_NOW state.
 *
 * See @ref fault_codes "Motor Control Faults" for a list of
 * of all possible faults codes.
 */
__weak uint16_t MC_GetOccurredFaultsMotor1(void)
{
  return (MCI_GetOccurredFaults(pMCI[M1]));
}

/**
 * @brief returns a bitfield showing all current faults on Motor 1
 *
 * This function returns a 16 bit fields containing the Motor Control faults
 * that are currently active.
 *
 * See @ref fault_codes "Motor Control Faults" for a list of
 * of all possible faults codes.
 */
__weak uint16_t MC_GetCurrentFaultsMotor1(void)
{
  return (MCI_GetCurrentFaults(pMCI[M1]));
}

/**
 * @brief returns the current state of Motor 1 state machine
 */
__weak MCI_State_t MC_GetSTMStateMotor1(void)
{
  return (MCI_GetSTMState(pMCI[M1]));
}

/**
  * @brief Sets the polarization offset values to use for Motor 1
  *
  * The Motor Control algorithm relies on a number of current and voltage measures. The hardware
  * parts that make these measurements need to be characterized at least once in the course of
  * product life, prior to its first activation. This characterization consists in measuring the
  * voltage presented to the ADC channels when either no current flows into the phases of the motor
  * or no voltage is applied to them. This characterization is named polarization offsets measurement
  * and its results are the polarization offsets.
  *
  * The Motor Control Firmware can performs this polarization offsets measurement procedure which
  * results in a number of offset values that the application can store in a non volatile memory and
  * then set into the Motor Control subsystem at power-on or after a reset.
  *
  * The application uses this function to set the polarization offset values that the Motor Control
  * subsystem is to use in the current session. This function can only be used when the state machine
  * of the motor is in the #IDLE state in which case it returns #MC_SUCCESS. Otherwise, it does nothing
  * and returns the #MC_WRONG_STATE_ERROR error code.
  *
  *  The Motor Control subsystem needs to know the polarization offsets before the motor can be controlled.
  * The MC_SetPolarizationOffsetsMotor1() function provides a way to set these offsets. Alternatively, the
  * application can either:
  *
  *  * Execute the polarization offsets measurement procedure with a call to
  *    MC_StartPolarizationOffsetsMeasurementMotor1() after a reset or a power on;
  *  * Start the motor control with the MC_StartWithPolarizationMotor1() that will execute the procedure
  *    before actually starting the motor, on the first time it is called after a reset or a power on.
  *
  * When this function completes successfully, the state of the polarization offsets measurement procedure
  * is set to #COMPLETED. See MC_GetPolarizationState().
  *
  * @param PolarizationOffsets pointer on the structure containing the offset values
  */
bool MC_SetPolarizationOffsetsMotor1(PolarizationOffsets_t * PolarizationOffsets)
{
  return (MCI_SetCalibratedOffsetsMotor(pMCI[M1], PolarizationOffsets));
}

/**
  * @brief Returns the polarization offset values measured or set for Motor 1
  *
  *  See MC_SetPolarizationOffsetsMotor1() for more details.
  *
  *  If the Motor Control Firmware knows the polarization offset values, they are copied into the
  * @p PolarizationOffsets structure and #MC_SUCCESS is returned. Otherwise, nothing is done and
  * #MC_NO_POLARIZATION_OFFSETS_ERROR is returned.
  *
  * @param PolarizationOffsets pointer on the structure into which the polarization offsets will be
  *        copied
  * @return #MC_SUCCESS if calibration data were present and could be copied into @p PolarizationOffsets,
  *         #MC_NO_POLARIZATION_OFFSETS_ERROR otherwise.
  */
bool MC_GetPolarizationOffsetsMotor1(PolarizationOffsets_t * PolarizationOffsets)
{
   return (MCI_GetCalibratedOffsetsMotor(pMCI[M1], PolarizationOffsets));
}

/**
  * @brief Starts the polarization offsets measurement procedure.
  *
  * See MC_SetPolarizationOffsetsMotor1() for more details.
  *
  * If the Motor Control Firmware is in the #IDLE state, the procedure is started, the state machine
  * of the motor switches to #OFFSET_CALIB and #MC_SUCCESS is returned. Otherwise, nothing is done
  * and the #MC_WRONG_STATE_ERROR error code is returned.
  *
  * The polarization offsets measurement procedure is only triggered by this function and it is has not
  * completed when this function returns. The application can use the MC_GetPolarizationState()
  * function to query the state of the procedure.
  *
  * @see MC_GetPolarizationState()
  */
bool MC_StartPolarizationOffsetsMeasurementMotor1(void)
{
  return (MCI_StartOffsetMeasurments(pMCI[M1]));
}

/**
 * @brief This method is used to get the average measured motor power
 *        expressed in watt for Motor 1.

 * @retval float_t The average measured motor power expressed in watt.
 */
__weak float_t MC_GetAveragePowerMotor1_F(void)
{
  return (PQD_GetAvrgElMotorPowerW(pMPM[M1]));
}

/**
 * @brief This method is used to switch between the primary and the auxiliary sensor
 *        for Motor 1.
 * @param  deltaAngleMargin maximum delta between primary and auxiliary sensor to allow switching.
 * @retval returns true if the command is successfully executed, false otherwise.
 *
 */
__weak bool MC_SensorSwitchMotor1(uint16_t deltaAngleMargin)
{
   return (MCI_SensorSwitch(pMCI[M1], deltaAngleMargin));
}

/**
 * @brief This method is used to set the primary sensor used by FOC algo for Motor 1.
 * @param  deltaAngleMargin maximum delta between primary and auxiliary sensor to allow switching.
 * @param  newSensor new primary sensor used by FOC algo
 * @retval returns true if the command is successfully executed, false otherwise.
 *
 * example is provided in [Auxiliary speed sensor usage](rotor_aux_speed_pos_feedback.md)
 */
__weak bool MC_SensorSetMotor1(uint16_t deltaAngleMargin, SpeednPosFdbk_Handle_t * newSensor)
{
   return (MCI_SensorSet(pMCI[M1], deltaAngleMargin, newSensor));
}

/**
 * @brief Not implemented MC_Profiler function.
 *  */ //cstat !MISRAC2012-Rule-2.7 !RED-unused-param  !MISRAC2012-Rule-2.7  !MISRAC2012-Rule-8.13
__weak uint8_t MC_ProfilerCommand(uint16_t rxLength, uint8_t *rxBuffer, int16_t txSyncFreeSpace, uint16_t *txLength, uint8_t *txBuffer)
{
  return (MCP_CMD_UNKNOWN);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/

