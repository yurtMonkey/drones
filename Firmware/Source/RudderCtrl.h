//============================================================================
//
// $RCSfile: RudderCtrl.h,v $ (HEADER FILE)
// $Revision: 1.1 $
// $Date: 2009/12/20 16:20:51 $
// $Author: Lorenz $
//
//  LANGUAGE    C
//  DESCRIPTION
/// \file
///             
//  CHANGES     
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

VAR_GLOBAL float Dir_Kp ;
VAR_GLOBAL float Dir_Ki ;
VAR_GLOBAL float Dir_Kd ;

/*---------------------------------- Interface -------------------------------*/

void Rudder_Control( void );
float Rudder( void );
