/**
  ******************************************************************************
  * @file    blower_osc.h
  * @brief   Sinusoidal pressure-setpoint perturbation for forced-oscillation
  *          based airway-blockage detection (FOT - Forced Oscillation
  *          Technique).
  *
  *  Concept
  *  -------
  *  When enabled, this module returns a small additive offset
  *      offset(t) = A * sin(2*pi*f*t)
  *  that the pressure controller adds to the patient's slewed therapy
  *  setpoint. The PID then drives the blower so the *measured* pressure
  *  contains an AC component at frequency f.
  *
  *  Once a flow sensor is fitted to the airway, the AC component of flow
  *  divided by the AC component of pressure yields the airway impedance
  *  Z = P_ac / Q_ac. A blocked airway shows as a large jump in |Z|. That
  *  detection logic is NOT implemented here - this file only generates
  *  the perturbation. Hook the impedance computation into the flow-sensor
  *  task when that hardware lands.
  *
  *  Design notes
  *  ------------
  *  - We use a phase accumulator in [0, 1) so phase wraps cleanly with
  *    no float drift even after hours of operation.
  *  - On the rising edge of "enable" we reset phase to 0 so the sine
  *    starts from a zero crossing - this avoids a sudden setpoint step
  *    when the oscillator turns on.
  *  - On the falling edge of "enable" we instantly drop the offset to 0;
  *    the PID's own output filter handles the transient.
  *  - Amplitude and frequency are clamped to therapy-safe ranges so a
  *    typo in the watch window cannot stress the patient.
  *
  *  Typical numbers (adult CPAP FOT, after Lorino/Brochard/etc.)
  *  ------------------------------------------------------------
  *      Frequency  : 4 - 8 Hz       (default 4 Hz)
  *      Amplitude  : 0.5 - 2 cmH2O  (default 1.0 cmH2O peak)
  *      Update rate: matches PID    (200 Hz here)
  ******************************************************************************
  */

#ifndef BLOWER_OSC_H
#define BLOWER_OSC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Therapy-safe clamps applied inside SetAmplitude / SetFrequency. */
#define BLOWER_OSC_AMPL_MIN_CMH2O   0.0f
#define BLOWER_OSC_AMPL_MAX_CMH2O   3.0f
#define BLOWER_OSC_FREQ_MIN_HZ      0.5f
#define BLOWER_OSC_FREQ_MAX_HZ      20.0f

/* Reset internal state and load defaults (disabled, 1.0 cmH2O @ 4 Hz). */
void  BlowerOsc_Init(void);

/* Master enable. Rising edge resets phase to 0 (zero-crossing start). */
void  BlowerOsc_SetEnable(bool en);
bool  BlowerOsc_IsEnabled(void);

/* Tunables. Both clamp internally; pass any value, get a safe value. */
void  BlowerOsc_SetAmplitude(float cmh2o_peak);
void  BlowerOsc_SetFrequency(float hz);
float BlowerOsc_GetAmplitude(void);
float BlowerOsc_GetFrequency(void);

/* Advance the phase accumulator by dt_s and return the current sample
 * (= amplitude * sin(2*pi*phase)). When disabled, returns 0.0f and does
 * NOT advance the phase. Designed to be called once per PID tick. */
float BlowerOsc_StepAndGetSample(float dt_s);

/* Read-only accessor for the most recent sample (no side effect). */
float BlowerOsc_GetLastSample(void);

#ifdef __cplusplus
}
#endif

#endif /* BLOWER_OSC_H */
