//============================================================================
//
// $RCSfile: cordic.h,v $ (HEADER FILE)
// $Revision: 1.2 $
// $Date: 2009/12/16 18:27:59 $
// $Author: Lorenz $
//
//  LANGUAGE    C
//  DESCRIPTION
/// \file
///             
//  CHANGES     argument and return value of cordic_atan( ) of type 'long'
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

/*--------------------------------- Constants --------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*--------------------------------- Interface --------------------------------*/

long cordic_atan(long I, long Q);
void cordic_rotate( int *px, int *py, int theta );
