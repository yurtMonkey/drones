//============================================================================+
//
// $RCSfile: log.c,v $ (SOURCE FILE)
// $Revision: 1.10 $
// $Date: 2011/01/19 18:31:28 $
// $Author: Lorenz $
//
/// \brief    Log manager
///
/// \file
///
//  CHANGES funzione Log_DCM(): resa non sospensiva, semplificato il formato
//          per l'invio della matrice DCM
//
//============================================================================*/

#include "math.h"

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "adcdriver.h"
#include "tff.h"
#include "DCM.h"
#include "tick.h"
#include "uartdriver.h"
#include "ppmdriver.h"
#include "log.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef    VAR_STATIC
#   undef VAR_STATIC
#endif
#define   VAR_STATIC static
#ifdef    VAR_GLOBAL
#   undef VAR_GLOBAL
#endif
#define   VAR_GLOBAL

#define FILE_BUFFER_LENGTH  128
#define SENSOR_LOG_LENGTH   32
#define DCM_LOG_LENGTH      47
#define LOG_STRING_SIZE     48

#define LOG_TO_SDCARD

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC unsigned char szString[LOG_STRING_SIZE];
VAR_STATIC const char szFileName[16] = "log.txt";   // File name
VAR_STATIC FATFS stFat;                             // FAT
VAR_STATIC FIL stFile;                              // File object
VAR_STATIC char pcBuffer[FILE_BUFFER_LENGTH];       // File data buffer
VAR_STATIC WORD wWriteIndex = 0;                    // File buffer write index
VAR_STATIC tBoolean bFileOk = false;                // File status

/*--------------------------------- Prototypes -------------------------------*/

static void Int2Hex(long lNumber, char * pcDest);

//----------------------------------------------------------------------------
//
/// \brief   Initialize log manager
///
/// \remarks opens log file for writing
///
//----------------------------------------------------------------------------
void
Log_Init( void ) {

    if (FR_OK == f_mount(0, &stFat)) {
        if (FR_OK == f_open(&stFile, szFileName, FA_WRITE)) {
            bFileOk = true;                     // File succesfully open
        } else {                                // Error opening file
            bFileOk = false;                    // Halt file logging
        }
    } else {                                    // Error mounting FS
        bFileOk = false;                        // Halt file logging
    }
}

//----------------------------------------------------------------------------
//
/// \brief   Put characters to log file
///
/// \remarks Characters are saved in a buffer and written to file when EOL is
///          received
///
//----------------------------------------------------------------------------
void
Log_PutChar( char c ) {

    WORD wWritten;

    if (wWriteIndex < FILE_BUFFER_LENGTH) { // Provided buffer is not full
        pcBuffer[wWriteIndex++] = c;        // Save character in buffer
    }
    if ((c == '\n') && bFileOk) {                           // End of line
        f_write(&stFile, pcBuffer, wWriteIndex, &wWritten); // Write
        if ((wWriteIndex != wWritten) ||                    // No file space
            (HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS))) {   // button pressed
            bFileOk = false;                                // Halt file logging
            f_close(&stFile);                               // close file
            HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS) = 0;   // clear button flag
        } else {                                            // Write successfull
            wWriteIndex = 0;                                // Empty buffer
        }
    }
}


///----------------------------------------------------------------------------
///
/// \brief   Log DCM matrix
///
/// \return  -
/// \remarks DCM to Euler angles conversion :
///
///             pitch = -asin(DCM_Matrix[2][0])
///             roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2])
///             yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0])
///
///----------------------------------------------------------------------------
void
Log_DCM(void)
{
    char sString[5];
    int iRow, iCol, j;                      // Indexes of DCM entries
    long lEntry;                            // DCM entry

    Log_PutChar('*');                       // Header for DCM data
    for (iCol = 0; iCol < 3; iCol++) {      // Convert DCM matrix to hex
      for (iRow = 0; iRow < 3; iRow++) {
          Log_PutChar(' ');                 // Space before entries
          lEntry = (long)ceil(DCM_Matrix[iCol][iRow] * 32767.0f);
          Int2Hex(lEntry, sString);         // Convert DCM entry to hex
          for (j = 0; j < 4; j++) {         // Log DCM entry
             Log_PutChar(sString[j]);
          }
      }
    }
    Log_PutChar('\n');                      // Terminate log string
}
/*
#if (0)
    while ((j < DCM_LOG_LENGTH) && UARTSpaceAvail(UART1_BASE)) {
      UARTCharPutNonBlocking(UART1_BASE, szString[j++]);
    }
#endif
*/

///----------------------------------------------------------------------------
///
/// \brief   Log sensor data
///
/// \return  -
/// \remarks sensor data order:
///
///             0   accel x
///             1   accel y
///             2   accel z
///             3   omega x
///             4   omega y
///             5   omega z
///
///----------------------------------------------------------------------------
void
Log_Sensors(void)
{
    char sString[5];
    int iIndex, j;                          // Index of sensor
    long lSensor;                           // Sensor value

    Log_PutChar('^');                       // Header for sensor data
    for (iIndex = 0; iIndex < 6; iIndex++) {
      Log_PutChar(' ');                     // Space before sensors
      lSensor = (long)ceil(ADCGetData(iIndex));
      Int2Hex(lSensor, sString);            // Convert sensor to hex
      for (j = 0; j < 4; j++) {             // Log sensor data
         Log_PutChar(sString[j]);
      }
    }
    Log_PutChar('\n');                      // Terminate log string
}


///----------------------------------------------------------------------------
///
/// \brief   Log PPM values
///
/// \return  -
/// \remarks
///
///----------------------------------------------------------------------------
void
Log_PPM(void)
{
    unsigned long ulTemp;
    int j = 0, chan = 0;

    for (chan = 0; chan < RC_CHANNELS; chan++) {
        ulTemp = PPMGetChannel(chan);
        szString[j++] = ' ';
        szString[j++] = '0' + ((ulTemp / 10000) % 10);
        szString[j++] = '0' + ((ulTemp / 1000) % 10);
        szString[j++] = '0' + ((ulTemp / 100) % 10);
        szString[j++] = '0' + ((ulTemp / 10) % 10);
        szString[j++] = '0' +  (ulTemp % 10);
    }
    szString[j] = '\n';
    UART1Send((const unsigned char *)szString, RC_CHANNELS * 6 + 1);
}

///----------------------------------------------------------------------------
///
/// \brief   Converts an integer to an hexadecimal string (ASCII)
///
/// \param   [in] lNumber (long) integer to be converted
/// \param   [out] pcDest pointer to destination buffer
/// \return  -
/// \remarks Result not predictable if lNumber > FFFF
///
///----------------------------------------------------------------------------
static void
Int2Hex(long lNumber, char * pcDest)
{
    unsigned char ucDigit;

    ucDigit = ((lNumber >> 12) & 0x0000000F);
    *pcDest++ = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
    ucDigit = ((lNumber >> 8) & 0x0000000F);
    *pcDest++ = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
    ucDigit = ((lNumber >> 4) & 0x0000000F);
    *pcDest++ = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
    ucDigit = (lNumber & 0x0000000F);
    *pcDest   = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
}
