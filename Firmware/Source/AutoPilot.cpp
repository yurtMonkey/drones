//============================================================================+
//
// $RCSfile: AutoPilot.cpp,v $ (SOURCE FILE)
// $Revision: 1.6 $
// $Date: 2010/12/30 09:46:03 $
// $Author: Lorenz $
//
//  LANGUAGE C
/// \brief   Aileron control
///
/// \file
/// Il controllo degli alettoni ha lo scopo primario di stabilizzare il rollio
/// dell'aereo. Puo' essere utilizzato anche per la navigazione, in alternativa
/// al timone.
///
/// 1. Stabilizzazione rollio, MatrixPilot:
/// - il controllo e' abilitato se ROLL_STABILIZATION e pitch_feedback sono TRUE
/// - la parte proporzionale e' (rmat[6] * rollkp), dove rmat[6] e' l'angolo di
///   beccheggio
/// - la parte derivativa e' (rollkd * omegaAccum[1]), dove omegaAccum[1] e'
///   la velocita' di rollio corretta con il vettore gravita'
///
/// 2. Navigazione, MatrixPilot:
/// - la navigazione e' abilitata se AILERON_NAVIGATION e GPS_steering sono TRUE
/// - calcola seno e coseno dell'angolo tra la direzione voluta e quella reale
/// - dal segno del coseno capisce se l'angolo e' compreso tra -90 e 90 deg
/// - se l'angolo e' tra -90 e 90 deg la correzione e' yawkpail * seno angolo
/// - se l'angolo non e' tra -90 e 90 deg la correzione e' + / - (yawkpail / 4)
///
/// 3. Stabilizzazione rollio, IMUCortex:
/// - algoritmo preso da modello matlab di Bizard / Premerlani.
/// - controllo PID della deflessione degli alettoni
/// - alettoni = ((Roll_Kp * errore di banco) - velocita' di rollio) * Roll_Kd
///   NB: prima sottrae la velocita' di rollio e poi moltiplica per il guadagno
///   derivativo. E' come se il guadagno proporzionale fosse Roll_Kp * Roll_Kd.
/// - l'errore di banco è angolo di banco richiesto - angolo di rollio attuale
/// - l'angolo di rollio è DCM_Matrix[2][1]
/// - la velocita' angolare di rollio e' Omega_Vector[0] cioe' la velocità
///   di rollio corretta con il vettore gravita'.
/// - il risultato è saturato tra -20 e 20
/// - Roll_Kp = 2, Roll_Kd = 0.1
///
/// 4. Navigazione, IMUCortex:
/// - algoritmo preso da modello matlab di Bizard / Premerlani.
/// - controllo PID dell'angolo di banco
/// - banco = Dir_Kp * errore di direzione * norma del vettore COG
/// - errore di direzione =
///     voluta - attuale + 360° se voluta - attuale < -180°
///     voluta - attuale - 360° se voluta - attuale > 180°
///     voluta - attuale        se voluta - attuale >= -180° e <= 180°
/// - direzione attuale = atan2(Vxe, Vye) * 180 / 3.1415
/// - il risultato è saturato tra -45° e 45°
/// - Dir_Kp = 0.03
///
//  CHANGES
//
//============================================================================*/

#include "stdafx.h"

#include "inc/hw_types.h"
#include "math.h"
#include "adcdriver.h"
#include "DCM.h"
#include "ppmdriver.h"
#include "nav.h"
#include "config.h"
#include "telemetry.h"
#include "autopilot.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef    VAR_STATIC
#  undef VAR_STATIC
#endif
#define   VAR_STATIC static
#ifdef    VAR_GLOBAL
#  undef VAR_GLOBAL
#endif
#define   VAR_GLOBAL

//
// PID saturation limits
//
#define AILERON_MIN (-0.1f)
#define AILERON_MAX (0.1f)
#define PHI_MIN     (-30.0f)
#define PHI_MAX     (30.0f)

/*----------------------------------- Macros ---------------------------------*/

/*--------------------------------- Enumeration ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

VAR_GLOBAL float Dir_Kp     = DIR_KP;      // Direction error proportional gain
VAR_GLOBAL float Dir_Ki     = DIR_KI;      // Direction error integral gain
VAR_GLOBAL float Dir_Kd     = DIR_KD;      // Direction error derivative gain
VAR_GLOBAL float Roll_Kp    = ROLL_KP;     // Roll error proportional gain
VAR_GLOBAL float Roll_Kd    = ROLL_KD;     // Roll error derivative gain

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC float da_c = 0.0f ;      // Commanded aileron deflection
VAR_STATIC float phi_c = 0.0f ;     // Commanded bank

//----------------------------------------------------------------------------
//
/// \brief   Control of aircraft heading.
/// \remarks Steers aircraft to required direction
///
//----------------------------------------------------------------------------
void
Heading_Control(void) {

    float khi, khi_c, khi_err;

    // Get commanded heading
    khi_c = (float)Nav_Bearing();

    // Compute actual heading
    khi = acosf(DCM_Matrix[0][0]);

    // Convert to degrees
    khi = (khi * 180.0f) / PI;
    if (DCM_Matrix[0][1] > 0.0f) {
       khi = 360 - khi;
    }

    // Compute heading error
    khi_err = khi - khi_c;
    if (khi_err < -180.0f) {
        khi_err += 360.0f;
    } else if (khi_err > 180.0f) {
        khi_err -= 360.0f;
    }

    // Multiply by speed
#ifndef _WINDOWS
    khi_err *= ((float)GPSSpeed())
#elif (SIMULATOR == SIM_NONE)
    khi_err *= Sim_Speed();
#else
    khi_err *= speed_3d;
#endif

    // Proportional term
    phi_c = khi_err * Dir_Kp;

    // Saturate result
    if (phi_c < PHI_MIN) {
      phi_c = PHI_MIN;
    } else if (phi_c > PHI_MAX) {
      phi_c = PHI_MAX;
    }
}

//----------------------------------------------------------------------------
//
/// \brief   Control of aircraft bank.
/// \remarks Stabilizes aircraft to required bank angle
///
//----------------------------------------------------------------------------
void
Bank_Control(void) {

   float temp;

    // Compute commanded angle between aircraft Y axis and earth Z axis
    temp = ((phi_c + 90.0f) * PI) / 180.0f;

    // Compute error
    temp = temp - acosf(DCM_Matrix[2][1]);

    // Compute commanded aileron deflection
    da_c = temp * Roll_Kp +            // Proportional term
           Omega_Vector[0] * Roll_Kd;  // Derivative term

    // Saturate result
    if (da_c < AILERON_MIN) {
      da_c = AILERON_MIN;
    } else if (da_c > AILERON_MAX) {
      da_c = AILERON_MAX;
    }
}

//----------------------------------------------------------------------------
//
/// \brief   aileron control interface.
/// \return  aileron deflection.
/// \remarks
///
//----------------------------------------------------------------------------
float
Ailerons ( void ) {
    if (PPMGetChannel(4) < 1200) {
      return ((float)PPMGetChannel(1) - 1500.0f) / 500.0f;
    } else {
      return da_c;
    }
}
