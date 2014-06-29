cls
@echo off
	@ECHO *********************************************************
	@ECHO  Batch file for calibration of Atmel AVR
	@ECHO  oscillator through the ISP interface with AVRISP mkII
	@ECHO  - The internal RC is calibrated to value and accuracy 
	@ECHO    given in RC_Calibration.asm (fuses set for 1.2 MHz)
	@ECHO  - Programming FLASH and Fuses is performed initially.
	@ECHO  - stk500.exe -h / AVR Studio help for more options
	@ECHO  
	@ECHO  $Name$
	@ECHO  $Revision: 3900 $                            
	@ECHO  $RCSfile$
	@ECHO  $Date: 2008-04-30 14:28:26 +0200 (on, 30 apr 2008) $
	@ECHO  $Date: 2011-02-25 Popravek za EAvto - Tadej$
	@ECHO *********************************************************
	
	@ECHO ---------------------------------------------------------
	@ECHO.
	@ECHO   ** S T A R T   P R O G R A M M I N G **
	@ECHO.		
@ECHO ---------------------------------------------------------


@REM Fill in your cpu type and full path to the programming tool
@SET CPU=ATtiny13A
@SET TOOL="C:\Program Files\Atmel\AVR Tools\STK500\stk500.exe"
@SET CAL_FUSES=ff4a
@REM @SET LOCK_FUSES=ff
@SET LOCK_FUSES=fc
@SET CUSTOMERCODE="D:\Razvoj\EAvto\Software\Atmel\BMS2\default\BMS.hex"
@SET CALIBRATECODE="D:\Razvoj\EAvto\Software\Atmel\RC_Callibrate\rc_calib01.hex"
@REM @SET CUSTOMERCODE="test.hex"


%TOOL% -cUSB -I125000 -d%CPU% -s -f%CAL_FUSES% -e -pf -if%CALIBRATECODE%
@REM %TOOL% -cUSB -I125000 -d%CPU% -s -e -pf -if%CALIBRATECODE%


@IF ERRORLEVEL ==1 GOTO prog_Calib_code_error
		
	@ECHO.
	@ECHO  ** Start Calibration

@ECHO ---------------------------------------------------------
%TOOL% -cUSB -d%CPU% -Y

@IF ERRORLEVEL ==1 GOTO Calibration_error

@ECHO ---------------------------------------------------------
	@ECHO.
	@ECHO Verify that OSCCAL value is different from 0xFF.
	@ECHO Note, this test is intended to fail. If it does
	@ECHO not fail, OSCCAL equals 0xFF which is considered 
	@ECHO an error.

%TOOL% -cUSB -I125000 -d%CPU% -ae0,0 -ve -ie0xFF_byte.hex


@IF ERRORLEVEL ==1 GOTO continue

@GOTO EEPROM_OSCCAL_value_error

:continue
		
@ECHO           ^^ Ignore Error above ^^
@ECHO.
@ECHO ---------------------------------------------------------
	@ECHO Read out new OSCCAL value from EEPROM and erase the device.
	@ECHO Program in customers code to FLASH "CUSTOMERCODE".
	@ECHO Verify programming of customers code.
	@ECHO Program in new OSCCAL value in flash at byte addr 0x00.
	@ECHO Verify programming of new OSCCAL value in EEPROM
%TOOL% -cUSB -I125000 -d%CPU% -e -pf -vf -if%CUSTOMERCODE% -Z0 -Se00

@IF ERRORLEVEL ==1 GOTO prog_customer_code_error

@ECHO.
@ECHO ---------------------------------------------------------
	@ECHO Program lock byte.
%TOOL% -cUSB -I125000 -d%CPU% -l%LOCK_FUSES%

@IF ERRORLEVEL ==1 GOTO prog_lock_code_error

	@ECHO.
	@ECHO *********************************************************
	@ECHO 		P R O G R A M M I N G   O K
	@ECHO *********************************************************
	@PAUSE

@GOTO END

:prog_Calib_code_error
	@ECHO.
@ECHO ---------------------------------------------------------
	@ECHO.
	@ECHO 		E R R O R		
	@ECHO Programming calibration program to AVR failed.
	@ECHO Programming aborted.
	@PAUSE
	@GOTO END
@ECHO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


:Calibration_error
	@ECHO.
@ECHO ---------------------------------------------------------
	@ECHO.
	@ECHO 		E R R O R
	@ECHO Calibration failed.
	@ECHO Programming aborted.
	@PAUSE
	@GOTO END
@ECHO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

:EEPROM_OSCCAL_value_error
	@ECHO.
@ECHO ---------------------------------------------------------
	@ECHO.
	@ECHO 		E R R O R
	@ECHO EEPROM OSCCAL location contain an invalid value: 0xFF
	@ECHO Programming aborted.
	@PAUSE
	@GOTO END
@ECHO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

:prog_customer_code_error
	@ECHO.
@ECHO ---------------------------------------------------------
	@ECHO.
	@ECHO 		E R R O R
	@ECHO Programming main application program or OSCCAL to AVR failed.
	@ECHO Programming aborted.
	@PAUSE
	@GOTO END
@ECHO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

:prog_lock_code_error
	@ECHO.
@ECHO ---------------------------------------------------------
	@ECHO.
	@ECHO 		E R R O R
	@ECHO Programming lock byte to AVR failed.
	@ECHO Programming aborted.
	@PAUSE
	@GOTO END
@ECHO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

:END				
