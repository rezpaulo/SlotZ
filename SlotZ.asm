$NOMOD51
;**** **** **** **** ****
;
; BLHeli program for controlling brushless motors in multirotors
;
; Copyright 2011, 2012 Steffen Skaug
; This program is distributed under the terms of the GNU General Public License
;
; This file is part of BLHeli.
;
; BLHeli is free software: you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation, either version 3 of the License, or
; (at your option) any later version.
;
; BLHeli is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with BLHeli.  If not, see <http://www.gnu.org/licenses/>.
;
;**** **** **** **** ****
;
; This software was initially designed for use with Eflite mCP X, but is now adapted to copters/planes in general
;
; The software was inspired by and started from from Bernard Konze's BLMC: http://home.versanet.de/~bkonze/blc_6a/blc_6a.htm
; And also Simon Kirby's TGY: https://github.com/sim-/tgy
;
; This file is best viewed with tab width set to 5
;
; The code is designed for multirotor applications, running damped light mode
;
; The input signal can be Normal (1-2ms), OneShot125 (125-250us), OneShot42 (41.7-83.3us) or Multishot (5-25us) at rates as high as allowed by the format.
; Three Dshot signal rates are also supported, Dshot150, Dshot300 and Dshot600. A 48MHz MCU is required for Dshot600.
; The code autodetects normal, OneShot125, Oneshot42, Multishot or Dshot.
;
; The first lines of the software must be modified according to the chosen environment:
; ESCNO EQU "ESC"
; MCU_48MHZ EQU "N"
; FETON_DELAY EQU "N"
; 
;**** **** **** **** ****
; Revision history:
; - Rev16.0 Started. Built upon rev 14.5 of base code
;           Using hardware pwm for very smooth throttle response, silent running and support of very high rpms
;           Implemented reverse bidirectional mode
;           Implemented separate throttle gains fwd and rev in bidirectional mode
;           Implemented support for Oneshot42 and Multishot
; - Rev16.1 Made low rpm power limiting programmable through the startup power parameter
; - Rev16.2 Fixed bug that prevented temperature protection
;           Improved robustness to very high input signal rates
;           Beeps can be turned off by programming beep strength to 1
;           Throttle cal difference is checked to be above required minimum before storing. Throttle cal max is not stored until successful min throttle cal
; - Rev16.3 Implemented programmable temperature protection
;           Improved protection of bootloader and generally reduced risk of flash corruption
;           Some small changes for improved sync hold
; - Rev16.4 Fixed bug where bootloader operation could be blocked by a defective "eeprom" signature
; - Rev16.5 Added support for DShot150, DShot300 and DShot600
; - Rev16.6 Fixed signal detection issue of multishot at 32kHz
;           Improved bidirectional mode for high input signal rates
; - Rev16.7 Addition of Dshot commands for beeps and temporary reverse direction (largely by brycedjohnson)
;
;
;**** **** **** **** ****
; Minimum 8K Bytes of In-System Self-Programmable Flash
; Minimum 512 Bytes Internal SRAM
;
;**** **** **** **** ****
; Master clock is internal 24MHz oscillator (or 48MHz, for which the times below are halved)
; Although 24/48 are used in the code, the exact clock frequencies are 24.5MHz or 49.0 MHz
; Timer 0 (41.67ns counts) always counts up and is used for
; - RC pulse measurement
; Timer 1 (41.67ns counts) always counts up and is used for
; - DShot frame sync detection
; Timer 2 (500ns counts) always counts up and is used for
; - RC pulse timeout counts and commutation times
; Timer 3 (500ns counts) always counts up and is used for
; - Commutation timeouts
; PCA0 (41.67ns counts) always counts up and is used for
; - Hardware PWM generation
;
;**** **** **** **** ****
; Interrupt handling
; The C8051 does not disable interrupts when entering an interrupt routine.
; Also some interrupt flags need to be cleared by software
; The code disables interrupts in some interrupt routines
; - Interrupts are disabled during beeps, to avoid audible interference from interrupts
;
;**** **** **** **** ****
; Motor control:
; - Brushless motor control with 6 states for each electrical 360 degrees
; - An advance timing of 0deg has zero cross 30deg after one commutation and 30deg before the next
; - Timing advance in this implementation is set to 15deg nominally
; - Motor pwm is always damped light (aka complementary pwm, regenerative braking)
; Motor sequence starting from zero crossing:
; - Timer wait: Wt_Comm			15deg	; Time to wait from zero cross to actual commutation
; - Timer wait: Wt_Advance		15deg	; Time to wait for timing advance. Nominal commutation point is after this
; - Timer wait: Wt_Zc_Scan		7.5deg	; Time to wait before looking for zero cross
; - Scan for zero cross			22.5deg	; Nominal, with some motor variations
;
; Motor startup:
; There is a startup phase and an initial run phase, before normal bemf commutation run begins.
;
;**** **** **** **** ****
; List of enumerated supported ESCs

;**** **** **** **** ****
MCU_48MHZ EQU	1
FETON_DELAY EQU 15	; 20.4ns per step
;ESCNO EQU 1

;**** **** **** **** ****
; ESC selection statements
$include (SlotZ.inc)	; Select pinout A

;**** **** **** **** ****
; Programming defaults
;
DEFAULT_PGM_STARTUP_PWR 				EQU 9 	; 1=0.031 2=0.047 3=0.063 4=0.094 5=0.125 6=0.188	7=0.25  8=0.38  9=0.50  10=0.75 11=1.00 12=1.25 13=1.50
DEFAULT_PGM_COMM_TIMING				EQU 3 	; 1=Low 		2=MediumLow 	3=Medium 		4=MediumHigh 	5=High
DEFAULT_PGM_DEMAG_COMP 				EQU 2 	; 1=Disabled	2=Low		3=High
DEFAULT_PGM_DIRECTION				EQU 1 	; 1=Normal 	2=Reversed	3=Bidir		4=Bidir rev
DEFAULT_PGM_BEEP_STRENGTH			EQU 40	; Beep strength
DEFAULT_PGM_BEACON_STRENGTH			EQU 80	; Beacon strength
DEFAULT_PGM_BEACON_DELAY				EQU 4 	; 1=1m		2=2m			3=5m			4=10m		5=Infinite

; COMMON
DEFAULT_PGM_ENABLE_TX_PROGRAM 		EQU 1 	; 1=Enabled 	0=Disabled
DEFAULT_PGM_MIN_THROTTLE				EQU 37	; 4*37+1000=1148
DEFAULT_PGM_MAX_THROTTLE				EQU 208	; 4*208+1000=1832
DEFAULT_PGM_CENTER_THROTTLE			EQU 122	; 4*122+1000=1488 (used in bidirectional mode)
DEFAULT_PGM_ENABLE_TEMP_PROT	 		EQU 7 	; 0=Disabled	1=80C	2=90C	3=100C	4=110C	5=120C	6=130C	7=140C
DEFAULT_PGM_ENABLE_POWER_PROT 		EQU 1 	; 1=Enabled 	0=Disabled
DEFAULT_PGM_BRAKE_ON_STOP	 		EQU 0 	; 1=Enabled 	0=Disabled
DEFAULT_PGM_LED_CONTROL	 			EQU 0 	; Byte for LED control. 2bits per LED, 0=Off, 1=On

;**** **** **** **** ****
; Temporary register definitions
Temp1		EQU	R0
Temp2		EQU	R1
Temp3		EQU	R2
Temp4		EQU	R3
Temp5		EQU	R4
Temp6		EQU	R5
Temp7		EQU	R6
Temp8		EQU	R7

;**** **** **** **** ****
; Register definitions
DSEG AT 20h					; Variables segment 

Bit_Access:				DS	1		; MUST BE AT THIS ADDRESS. Variable at bit accessible address (for non interrupt routines)
Bit_Access_Int:			DS	1		; Variable at bit accessible address (for interrupts)

Rcp_Outside_Range_Cnt:		DS	1		; RC pulse outside range counter (incrementing) 
Rcp_Timeout_Cntd:			DS	1		; RC pulse timeout counter (decrementing) 


Flags0:					DS	1    	; State flags. Reset upon init_start
T3_PENDING				EQU 	0		; Timer 3 pending flag
DEMAG_DETECTED				EQU 	1		; Set when excessive demag time is detected
COMP_TIMED_OUT				EQU 	2		; Set when comparator reading timed out
;						EQU 	3
;						EQU 	4
;						EQU 	5	
;						EQU 	6	
;						EQU 	7	


Flags1:					DS	1    	; State flags. Reset upon init_start 
STARTUP_PHASE				EQU 	0		; Set when in startup phase
INITIAL_RUN_PHASE			EQU	1		; Set when in initial run phase, before synchronized run is achieved
MOTOR_STARTED				EQU 	2		; Set when motor is started
DIR_CHANGE_BRAKE			EQU 	3		; Set when braking before direction change
HIGH_RPM					EQU 	4		; Set when motor rpm is high (Comm_Period4x_H less than 2)
;						EQU 	5
;						EQU 	6	
;						EQU 	7	

Flags2:					DS	1		; State flags. NOT reset upon init_start
RCP_UPDATED				EQU 	0		; New RC pulse length value available
RCP_ONESHOT125				EQU 	1		; RC pulse input is OneShot125 (125-250us)
RCP_ONESHOT42				EQU 	2		; RC pulse input is OneShot42 (41.67-83us)
RCP_MULTISHOT				EQU 	3		; RC pulse input is Multishot (5-25us)
RCP_DSHOT					EQU 	4		; RC pulse input is digital shot
RCP_DIR_REV				EQU 	5		; RC pulse direction in bidirectional mode
RCP_FULL_RANGE				EQU 	6		; When set full input signal range is used (1000-2000us) and stored calibration values are ignored
;						EQU 	7	

Flags3:					DS	1		; State flags. NOT reset upon init_start
PGM_DIR_REV				EQU 	0		; Programmed direction. 0=normal, 1=reversed
PGM_BIDIR_REV				EQU 	1		; Programmed bidirectional direction. 0=normal, 1=reversed
PGM_BIDIR					EQU 	2		; Programmed bidirectional operation. 0=normal, 1=bidirectional
;						EQU 	3
;						EQU 	4	
;						EQU 	5	
;						EQU 	6	
;						EQU 	7	


;**** **** **** **** ****
; RAM definitions
DSEG AT 30h						; Ram data segment, direct addressing
Initial_Arm:				DS	1		; Variable that is set during the first arm sequence after power on

Min_Throttle_L:			DS	1		; Minimum throttle scaled (lo byte)
Min_Throttle_H:			DS	1		; Minimum throttle scaled (hi byte)
Center_Throttle_L:			DS	1		; Center throttle scaled (lo byte)
Center_Throttle_H:			DS	1		; Center throttle scaled (hi byte)
Max_Throttle_L:			DS	1		; Maximum throttle scaled (lo byte)
Max_Throttle_H:			DS	1		; Maximum throttle scaled (hi byte)

Power_On_Wait_Cnt_L: 		DS	1		; Power on wait counter (lo byte)
Power_On_Wait_Cnt_H: 		DS	1		; Power on wait counter (hi byte)

Startup_Cnt:				DS	1		; Startup phase commutations counter (incrementing)
Startup_Zc_Timeout_Cntd:		DS	1		; Startup zero cross timeout counter (decrementing)
Initial_Run_Rot_Cntd:		DS	1		; Initial run rotations counter (decrementing)
Stall_Cnt:				DS	1		; Counts start/run attempts that resulted in stall. Reset upon a proper stop
Demag_Detected_Metric:		DS	1		; Metric used to gauge demag event frequency
Demag_Pwr_Off_Thresh:		DS	1		; Metric threshold above which power is cut
Low_Rpm_Pwr_Slope:			DS	1		; Sets the slope of power increase for low rpms

Timer0_X:					DS	1		; Timer 0 extended byte
Timer2_X:					DS	1		; Timer 2 extended byte
Prev_Comm_L:				DS	1		; Previous commutation timer 3 timestamp (lo byte)
Prev_Comm_H:				DS	1		; Previous commutation timer 3 timestamp (hi byte)
Prev_Comm_X:				DS	1		; Previous commutation timer 3 timestamp (ext byte)
Prev_Prev_Comm_L:			DS	1		; Pre-previous commutation timer 3 timestamp (lo byte)
Prev_Prev_Comm_H:			DS	1		; Pre-previous commutation timer 3 timestamp (hi byte)
Comm_Period4x_L:			DS	1		; Timer 3 counts between the last 4 commutations (lo byte)
Comm_Period4x_H:			DS	1		; Timer 3 counts between the last 4 commutations (hi byte)
Comparator_Read_Cnt: 		DS	1		; Number of comparator reads done

Wt_Adv_Start_L:			DS	1		; Timer 3 start point for commutation advance timing (lo byte)
Wt_Adv_Start_H:			DS	1		; Timer 3 start point for commutation advance timing (hi byte)
Wt_Zc_Scan_Start_L:			DS	1		; Timer 3 start point from commutation to zero cross scan (lo byte)
Wt_Zc_Scan_Start_H:			DS	1		; Timer 3 start point from commutation to zero cross scan (hi byte)
Wt_Zc_Tout_Start_L:			DS	1		; Timer 3 start point for zero cross scan timeout (lo byte)
Wt_Zc_Tout_Start_H:			DS	1		; Timer 3 start point for zero cross scan timeout (hi byte)
Wt_Comm_Start_L:			DS	1		; Timer 3 start point from zero cross to commutation (lo byte)
Wt_Comm_Start_H:			DS	1		; Timer 3 start point from zero cross to commutation (hi byte)

Dshot_Cmd:				DS	1		; Dshot command
Dshot_Cmd_Cnt:				DS  	1		; Dshot command count

New_Rcp:					DS	1		; New RC pulse value in pca counts
Rcp_Stop_Cnt:				DS	1		; Counter for RC pulses below stop value

Power_Pwm_Reg_L:			DS	1		; Power pwm register setting (lo byte)
Power_Pwm_Reg_H:			DS	1		; Power pwm register setting (hi byte). 0x3F is minimum power
Damp_Pwm_Reg_L:			DS	1		; Damping pwm register setting (lo byte)
Damp_Pwm_Reg_H:			DS	1		; Damping pwm register setting (hi byte)
Current_Power_Pwm_Reg_H:		DS	1		; Current power pwm register setting that is loaded in the PCA register (hi byte)

Pwm_Limit:				DS	1		; Maximum allowed pwm 
Pwm_Limit_By_Rpm:			DS	1		; Maximum allowed pwm for low or high rpms
Pwm_Limit_Beg:				DS	1		; Initial pwm limit

Adc_Conversion_Cnt:			DS	1		; Adc conversion counter

Current_Average_Temp:		DS	1		; Current average temperature (lo byte ADC reading, assuming hi byte is 1)

Throttle_Gain:				DS	1		; Gain to be applied to RCP value
Throttle_Gain_M:			DS	1		; Gain to be applied to RCP value (multiplier 0=1x, 1=2x, 2=4x etc))
Throttle_Gain_BD_Rev:		DS	1		; Gain to be applied to RCP value for reverse direction in bidirectional mode
Throttle_Gain_BD_Rev_M:		DS	1		; Gain to be applied to RCP value for reverse direction in bidirectional mode (multiplier 0=1x, 1=2x, 2=4x etc)
Beep_Strength:				DS	1		; Strength of beeps

Skip_T2_Int:				DS	1		; Set for 48MHz MCUs when timer 2 interrupt shall be ignored
Clock_Set_At_48MHz:			DS	1		; Variable set if 48MHz MCUs run at 48MHz

Flash_Key_1:				DS	1		; Flash key one
Flash_Key_2:				DS	1		; Flash key two

Temp_Prot_Limit:			DS	1		; Temperature protection limit

DShot_Pwm_Thr:				DS	1		; DShot pulse width threshold value
DShot_Timer_Preset:			DS	1		; DShot timer preset for frame sync detection
DShot_Frame_Start_L:		DS	1		; DShot frame start timestamp (lo byte)
DShot_Frame_Start_H:		DS	1		; DShot frame start timestamp (hi byte)
DShot_Frame_Length_Thr:		DS	1		; DShot frame length criteria (in units of 4 timer 2 ticks)

; Indirect addressing data segment. The variables below must be in this sequence
ISEG AT 080h					
_Pgm_Gov_P_Gain:			DS	1		; Programmed governor P gain
_Pgm_Gov_I_Gain:			DS	1		; Programmed governor I gain
_Pgm_Gov_Mode:				DS	1		; Programmed governor mode
_Pgm_Low_Voltage_Lim:		DS	1		; Programmed low voltage limit
_Pgm_Motor_Gain:			DS	1		; Programmed motor gain
_Pgm_Motor_Idle:			DS	1		; Programmed motor idle speed
Pgm_Startup_Pwr:			DS	1		; Programmed startup power
_Pgm_Pwm_Freq:				DS	1		; Programmed pwm frequency
Pgm_Direction:				DS	1		; Programmed rotation direction
Pgm_Input_Pol:				DS	1		; Programmed input pwm polarity
Initialized_L_Dummy:		DS	1		; Place holder
Initialized_H_Dummy:		DS	1		; Place holder
Pgm_Enable_TX_Program:		DS 	1		; Programmed enable/disable value for TX programming
_Pgm_Main_Rearm_Start:		DS 	1		; Programmed enable/disable re-arming main every start 
_Pgm_Gov_Setup_Target:		DS 	1		; Programmed main governor setup target
_Pgm_Startup_Rpm:			DS	1		; Programmed startup rpm (unused - place holder)
_Pgm_Startup_Accel:			DS	1		; Programmed startup acceleration (unused - place holder)
_Pgm_Volt_Comp:			DS	1		; Place holder
Pgm_Comm_Timing:			DS	1		; Programmed commutation timing
_Pgm_Damping_Force:			DS	1		; Programmed damping force (unused - place holder)
_Pgm_Gov_Range:			DS	1		; Programmed governor range
_Pgm_Startup_Method:		DS	1		; Programmed startup method (unused - place holder)
Pgm_Min_Throttle:			DS	1		; Programmed throttle minimum
Pgm_Max_Throttle:			DS	1		; Programmed throttle maximum
Pgm_Beep_Strength:			DS	1		; Programmed beep strength
Pgm_Beacon_Strength:		DS	1		; Programmed beacon strength
Pgm_Beacon_Delay:			DS	1		; Programmed beacon delay
_Pgm_Throttle_Rate:			DS	1		; Programmed throttle rate (unused - place holder)
Pgm_Demag_Comp:			DS	1		; Programmed demag compensation
_Pgm_BEC_Voltage_High:		DS	1		; Programmed BEC voltage
Pgm_Center_Throttle:		DS	1		; Programmed throttle center (in bidirectional mode)
_Pgm_Main_Spoolup_Time:		DS	1		; Programmed main spoolup time
Pgm_Enable_Temp_Prot:		DS	1		; Programmed temperature protection enable
Pgm_Enable_Power_Prot:		DS	1		; Programmed low rpm power protection enable
_Pgm_Enable_Pwm_Input:		DS	1		; Programmed PWM input signal enable
_Pgm_Pwm_Dither:			DS	1		; Programmed output PWM dither
Pgm_Brake_On_Stop:			DS	1		; Programmed braking when throttle is zero
Pgm_LED_Control:			DS	1		; Programmed LED control

; The sequence of the variables below is no longer of importance
Pgm_Startup_Pwr_Decoded:		DS	1		; Programmed startup power decoded


; Indirect addressing data segment
ISEG AT 0D0h					
Temp_Storage:				DS	48		; Temporary storage

;**** **** **** **** ****
IF MCU_48MHZ == 2
CSEG AT 3000h            ; "Eeprom" segment
ELSE
CSEG AT 1A00h            ; "Eeprom" segment
ENDIF
EEPROM_FW_MAIN_REVISION		EQU	16		; Main revision of the firmware
EEPROM_FW_SUB_REVISION		EQU	7		; Sub revision of the firmware
EEPROM_LAYOUT_REVISION		EQU	33		; Revision of the EEPROM layout

Eep_FW_Main_Revision:		DB	EEPROM_FW_MAIN_REVISION			; EEPROM firmware main revision number
Eep_FW_Sub_Revision:		DB	EEPROM_FW_SUB_REVISION			; EEPROM firmware sub revision number
Eep_Layout_Revision:		DB	EEPROM_LAYOUT_REVISION			; EEPROM layout revision number

_Eep_Pgm_Gov_P_Gain:		DB	0FFh	
_Eep_Pgm_Gov_I_Gain:		DB	0FFh	
_Eep_Pgm_Gov_Mode:			DB	0FFh	
_Eep_Pgm_Low_Voltage_Lim:	DB	0FFh							
_Eep_Pgm_Motor_Gain:		DB	0FFh	
_Eep_Pgm_Motor_Idle:		DB	0FFh						
Eep_Pgm_Startup_Pwr:		DB	DEFAULT_PGM_STARTUP_PWR			; EEPROM copy of programmed startup power
_Eep_Pgm_Pwm_Freq:			DB	0FFh	
Eep_Pgm_Direction:			DB	DEFAULT_PGM_DIRECTION			; EEPROM copy of programmed rotation direction
_Eep_Pgm_Input_Pol:			DB	0FFh
Eep_Initialized_L:			DB	055h							; EEPROM initialized signature low byte
Eep_Initialized_H:			DB	0AAh							; EEPROM initialized signature high byte
Eep_Enable_TX_Program:		DB	DEFAULT_PGM_ENABLE_TX_PROGRAM		; EEPROM TX programming enable
_Eep_Main_Rearm_Start:		DB	0FFh							
_Eep_Pgm_Gov_Setup_Target:	DB	0FFh							
_Eep_Pgm_Startup_Rpm:		DB	0FFh
_Eep_Pgm_Startup_Accel:		DB	0FFh
_Eep_Pgm_Volt_Comp:			DB	0FFh	
Eep_Pgm_Comm_Timing:		DB	DEFAULT_PGM_COMM_TIMING			; EEPROM copy of programmed commutation timing
_Eep_Pgm_Damping_Force:		DB	0FFh
_Eep_Pgm_Gov_Range:			DB	0FFh	
_Eep_Pgm_Startup_Method:		DB	0FFh
Eep_Pgm_Min_Throttle:		DB	DEFAULT_PGM_MIN_THROTTLE			; EEPROM copy of programmed minimum throttle
Eep_Pgm_Max_Throttle:		DB	DEFAULT_PGM_MAX_THROTTLE			; EEPROM copy of programmed minimum throttle
Eep_Pgm_Beep_Strength:		DB	DEFAULT_PGM_BEEP_STRENGTH		; EEPROM copy of programmed beep strength
Eep_Pgm_Beacon_Strength:		DB	DEFAULT_PGM_BEACON_STRENGTH		; EEPROM copy of programmed beacon strength
Eep_Pgm_Beacon_Delay:		DB	DEFAULT_PGM_BEACON_DELAY			; EEPROM copy of programmed beacon delay
_Eep_Pgm_Throttle_Rate:		DB	0FFh
Eep_Pgm_Demag_Comp:			DB	DEFAULT_PGM_DEMAG_COMP			; EEPROM copy of programmed demag compensation
_Eep_Pgm_BEC_Voltage_High:	DB	0FFh	
Eep_Pgm_Center_Throttle:		DB	DEFAULT_PGM_CENTER_THROTTLE		; EEPROM copy of programmed center throttle
_Eep_Pgm_Main_Spoolup_Time:	DB	0FFh
Eep_Pgm_Temp_Prot_Enable:	DB	DEFAULT_PGM_ENABLE_TEMP_PROT		; EEPROM copy of programmed temperature protection enable
Eep_Pgm_Enable_Power_Prot:	DB	DEFAULT_PGM_ENABLE_POWER_PROT		; EEPROM copy of programmed low rpm power protection enable
_Eep_Pgm_Enable_Pwm_Input:	DB	0FFh	
_Eep_Pgm_Pwm_Dither:		DB	0FFh	
Eep_Pgm_Brake_On_Stop:		DB	DEFAULT_PGM_BRAKE_ON_STOP		; EEPROM copy of programmed braking when throttle is zero
Eep_Pgm_LED_Control:		DB	DEFAULT_PGM_LED_CONTROL			; EEPROM copy of programmed LED control

Eep_Dummy:				DB	0FFh							; EEPROM address for safety reason

CSEG AT 1A60h
Eep_Name:					DB	"SlotZ           "				; Name tag (16 Bytes)

;**** **** **** **** ****
Interrupt_Table_Definition		; SiLabs interrupts
CSEG AT 80h			; Code segment after interrupt vectors 

;**** **** **** **** ****

; Table definitions
STARTUP_POWER_TABLE:  	DB 	04h, 06h, 08h, 0Ch, 10h, 18h, 20h, 30h, 40h, 60h, 80h, 0A0h, 0C0h


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Timer 0 interrupt routine
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
t0_int:
	inc	Timer0_X
	reti

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Timer 2 interrupt routine
;
; No assumptions
; Requirements: Temp variables can NOT be used since PSW.x is not set
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
t2_int:	; Happens every 32ms
	push	PSW			; Preserve registers through interrupt
	push	ACC
	clr	TMR2CN0_TF2H				; Clear interrupt flag
	inc	Timer2_X
	mov	A, Clock_Set_At_48MHz
	jz 	t2_int_start

	; Check skip variable
	mov	A, Skip_T2_Int
	jz	t2_int_start				; Execute this interrupt

	mov	Skip_T2_Int, #0
	ajmp	t2_int_exit

t2_int_start:
	mov	Skip_T2_Int, #1			; Skip next interrupt
	; Update RC pulse timeout counter 
	mov	A, Rcp_Timeout_Cntd			; RC pulse timeout count zero?
	jz	($+4)					; Yes - do not decrement

	dec	Rcp_Timeout_Cntd			; No decrement

	; Check RC pulse against stop value
	clr	C
	;mov	A, New_Rcp				; Load new pulse value
	;jz	t2_int_rcp_stop			; Check if pulse is below stop value

	; RC pulse higher than stop value, reset stop counter
	mov	Rcp_Stop_Cnt, #0			; Reset rcp stop counter
	ajmp	t2_int_exit

t2_int_rcp_stop:
	; RC pulse less than stop value
	mov	A, Rcp_Stop_Cnt			; Increment stop counter
	add	A, #1
	mov	Rcp_Stop_Cnt, A
	jnc	($+5)					; Branch if counter has not wrapped

	mov	Rcp_Stop_Cnt, #0FFh			; Set stop counter to max

t2_int_exit:
	pop	ACC			; Restore preserved registers
	pop	PSW
	reti

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Timer 3 interrupt routine
;
; No assumptions
; Requirements: Temp variables can NOT be used since PSW.x is not set
;               ACC can not be used, as it is not pushed to stack
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
t3_int:	; Used for commutation timing
	clr 	IE_EA			; Disable all interrupts
	anl	EIE1, #7Fh		; Disable timer 3 interrupts
	mov	TMR3RLL, #0FAh		; Set a short delay before next interrupt
	mov	TMR3RLH, #0FFh
	clr	Flags0.T3_PENDING 	; Flag that timer has wrapped
	anl	TMR3CN0, #07Fh		; Timer 3 interrupt flag cleared
	setb	IE_EA			; Enable all interrupts
	reti

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Wait xms ~(x*4*250)  (Different entry points)	
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
wait1ms:	
	mov	Temp2, #1
	jmp	waitxms_o

wait3ms:	
	mov	Temp2, #3
	jmp	waitxms_o

wait10ms:	
	mov	Temp2, #10
	jmp	waitxms_o

waitxms_o:	; Outer loop
	mov	Temp1, #23
waitxms_m:	; Middle loop
	clr	A
 	djnz	ACC, $	; Inner loop (42.7us - 1024 cycles)
	djnz	Temp1, waitxms_m
	djnz	Temp2, waitxms_o
	ret

	
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Start ADC conversion
;
; No assumptions
;
; Start conversion used for measuring power supply voltage
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
start_adc_conversion:
	; Start adc
	Start_Adc 
	ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Initialize timing routine
;
; No assumptions
;
; Part of initialization before motor start
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
initialize_timing: 
	mov	Comm_Period4x_L, #00h				; Set commutation period registers
	mov	Comm_Period4x_H, #0F0h
	ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Calculate next commutation timing routine
;
; No assumptions
;
; Called immediately after each commutation
; Also sets up timer 3 to wait advance timing
; Two entry points are used
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
calc_next_comm_timing:		; Entry point for run phase
	; Read commutation time
	clr	IE_EA
	clr	TMR2CN0_TR2		; Timer 2 disabled
	mov	Temp1, TMR2L		; Load timer value
	mov	Temp2, TMR2H	
	mov	Temp3, Timer2_X
	jnb	TMR2CN0_TF2H, ($+4)	; Check if interrupt is pending
	inc	Temp3			; If it is pending, then timer has already wrapped
	setb	TMR2CN0_TR2		; Timer 2 enabled
	setb	IE_EA
IF MCU_48MHZ >= 1
	clr	C
	mov	A, Temp3
	rrc	A
	mov	Temp3, A
	mov	A, Temp2
	rrc	A
	mov	Temp2, A
	mov	A, Temp1
	rrc	A
	mov	Temp1, A
ENDIF
	; Calculate this commutation time
	mov	Temp4, Prev_Comm_L
	mov	Temp5, Prev_Comm_H
	mov	Prev_Comm_L, Temp1		; Store timestamp as previous commutation
	mov	Prev_Comm_H, Temp2
	clr	C
	mov	A, Temp1
	subb	A, Temp4				; Calculate the new commutation time
	mov	Temp1, A
	mov	A, Temp2
	subb	A, Temp5
	jb	Flags1.STARTUP_PHASE, calc_next_comm_startup

IF MCU_48MHZ >= 1
	anl	A, #7Fh
ENDIF
	mov	Temp2, A
	jnb	Flags1.HIGH_RPM, ($+5)	; Branch if high rpm
	ajmp	calc_next_comm_timing_fast

	ajmp	calc_next_comm_normal

calc_next_comm_startup:
	mov	Temp6, Prev_Comm_X
	mov	Prev_Comm_X, Temp3		; Store extended timestamp as previous commutation
	mov	Temp2, A
	mov	A, Temp3
	subb	A, Temp6				; Calculate the new extended commutation time
IF MCU_48MHZ >= 1
	anl	A, #7Fh
ENDIF
	mov	Temp3, A
	jz	calc_next_comm_startup_no_X

	mov	Temp1, #0FFh
	mov	Temp2, #0FFh
	ajmp	calc_next_comm_startup_average

calc_next_comm_startup_no_X:
	mov	Temp7, Prev_Prev_Comm_L
	mov	Temp8, Prev_Prev_Comm_H
	mov	Prev_Prev_Comm_L, Temp4
	mov	Prev_Prev_Comm_H, Temp5
	mov	Temp1, Prev_Comm_L		; Reload this commutation time	
	mov	Temp2, Prev_Comm_H
	clr	C
	mov	A, Temp1
	subb	A, Temp7				; Calculate the new commutation time based upon the two last commutations (to reduce sensitivity to offset)
	mov	Temp1, A
	mov	A, Temp2
	subb	A, Temp8
	mov	Temp2, A

calc_next_comm_startup_average:
	clr	C
	mov	A, Comm_Period4x_H		; Average with previous and save
	rrc	A
	mov	Temp4, A
	mov	A, Comm_Period4x_L
	rrc	A
	mov	Temp3, A
	mov	A, Temp1			
	add	A, Temp3
	mov	Comm_Period4x_L, A
	mov	A, Temp2
	addc	A, Temp4
	mov	Comm_Period4x_H, A
	jnc	($+8)

	mov	Comm_Period4x_L, #0FFh
	mov	Comm_Period4x_H, #0FFh

	ajmp	calc_new_wait_times_setup

calc_next_comm_normal:
	; Calculate new commutation time 
	mov	Temp3, Comm_Period4x_L	; Comm_Period4x(-l-h) holds the time of 4 commutations
	mov	Temp4, Comm_Period4x_H
	mov	Temp5, Comm_Period4x_L	; Copy variables
	mov	Temp6, Comm_Period4x_H
	mov	Temp7, #4				; Divide Comm_Period4x 4 times as default
	mov	Temp8, #2				; Divide new commutation time 2 times as default
	clr	C
	mov	A, Temp4
	subb	A, #04h
	jc	calc_next_comm_avg_period_div

	dec	Temp7				; Reduce averaging time constant for low speeds
	dec	Temp8

	clr	C
	mov	A, Temp4
	subb	A, #08h
	jc	calc_next_comm_avg_period_div

	jb	Flags1.INITIAL_RUN_PHASE, calc_next_comm_avg_period_div	; Do not average very fast during initial run

	dec	Temp7				; Reduce averaging time constant more for even lower speeds
	dec	Temp8

calc_next_comm_avg_period_div:
	clr	C
	mov	A, Temp6					
	rrc	A					; Divide by 2
	mov	Temp6, A	
	mov	A, Temp5				
	rrc	A
	mov	Temp5, A
	djnz	Temp7, calc_next_comm_avg_period_div

	clr	C
	mov	A, Temp3
	subb	A, Temp5				; Subtract a fraction
	mov	Temp3, A
	mov	A, Temp4
	subb	A, Temp6
	mov	Temp4, A
	mov	A, Temp8				; Divide new time
	jz	calc_next_comm_new_period_div_done

calc_next_comm_new_period_div:
	clr	C
	mov	A, Temp2					
	rrc	A					; Divide by 2
	mov	Temp2, A	
	mov	A, Temp1				
	rrc	A
	mov	Temp1, A
	djnz	Temp8, calc_next_comm_new_period_div

calc_next_comm_new_period_div_done:
	mov	A, Temp3
	add	A, Temp1				; Add the divided new time
	mov	Temp3, A
	mov	A, Temp4
	addc	A, Temp2
	mov	Temp4, A
	mov	Comm_Period4x_L, Temp3	; Store Comm_Period4x_X
	mov	Comm_Period4x_H, Temp4
	jnc	calc_new_wait_times_setup; If period larger than 0xffff - go to slow case

	mov	Temp4, #0FFh
	mov	Comm_Period4x_L, Temp4	; Set commutation period registers to very slow timing (0xffff)
	mov	Comm_Period4x_H, Temp4

calc_new_wait_times_setup:	
	; Set high rpm bit (if above 156k erpm)
	clr	C
	mov	A, Temp4
	subb	A, #2
	jnc	($+4)

	setb	Flags1.HIGH_RPM 		; Set high rpm bit
	
	; Load programmed commutation timing
	jnb	Flags1.STARTUP_PHASE, calc_new_wait_per_startup_done	; Set dedicated timing during startup

	mov	Temp8, #3
	ajmp	calc_new_wait_per_demag_done

calc_new_wait_per_startup_done:
	mov	Temp1, #Pgm_Comm_Timing	; Load timing setting
	mov	A, @Temp1				
	mov	Temp8, A				; Store in Temp8
	clr	C
	mov	A, Demag_Detected_Metric	; Check demag metric
	subb	A, #130
	jc	calc_new_wait_per_demag_done

	inc	Temp8				; Increase timing

	clr	C
	mov	A, Demag_Detected_Metric
	subb	A, #160
	jc	($+3)

	inc	Temp8				; Increase timing again

	clr	C
	mov	A, Temp8				; Limit timing to max
	subb	A, #6
	jc	($+4)

	mov	Temp8, #5				; Set timing to max

calc_new_wait_per_demag_done:
	; Set timing reduction
	mov	Temp7, #2
	; Load current commutation timing
	mov	A, Comm_Period4x_H		; Divide 4 times
	swap	A
	anl	A, #00Fh
	mov	Temp2, A
	mov	A, Comm_Period4x_H
	swap	A
	anl	A, #0F0h
	mov	Temp1, A
	mov	A, Comm_Period4x_L
	swap	A
	anl	A, #00Fh
	add	A, Temp1
	mov	Temp1, A

	clr	C
	mov	A, Temp1
	subb	A, Temp7
	mov	Temp3, A
	mov	A, Temp2				
	subb	A, #0
	mov	Temp4, A
	jc	load_min_time			; Check that result is still positive

	clr	C
	mov	A, Temp3
	subb	A, #1
	mov	A, Temp4			
	subb	A, #0
	jnc	calc_new_wait_times_exit	; Check that result is still above minumum

load_min_time:
	mov	Temp3, #1
	clr	A
	mov	Temp4, A 

calc_new_wait_times_exit:	
	ljmp	wait_advance_timing


; Fast calculation (Comm_Period4x_H less than 2)
calc_next_comm_timing_fast:			
	; Calculate new commutation time
	mov	Temp3, Comm_Period4x_L	; Comm_Period4x(-l-h) holds the time of 4 commutations
	mov	Temp4, Comm_Period4x_H
	mov	A, Temp4				; Divide by 2 4 times
	swap	A
	mov	Temp7, A
	mov	A, Temp3
	swap A
	anl	A, #0Fh
	orl	A, Temp7
	mov	Temp5, A
	clr	C
	mov	A, Temp3				; Subtract a fraction
	subb	A, Temp5
	mov	Temp3, A
	mov	A, Temp4				
	subb	A, #0
	mov	Temp4, A
	clr	C
	mov	A, Temp1
	rrc	A					; Divide by 2 2 times
	clr	C
	rrc	A
	mov	Temp1, A
	mov	A, Temp3				; Add the divided new time
	add	A, Temp1
	mov	Temp3, A
	mov	A, Temp4
	addc	A, #0
	mov	Temp4, A
	mov	Comm_Period4x_L, Temp3	; Store Comm_Period4x_X
	mov	Comm_Period4x_H, Temp4
	clr	C
	mov	A, Temp4				; If erpm below 156k - go to normal case
	subb	A, #2
	jc	($+4)

	clr	Flags1.HIGH_RPM 		; Clear high rpm bit

	; Set timing reduction
	mov	Temp1, #2
	mov	A, Temp4				; Divide by 2 4 times
	swap	A
	mov	Temp7, A
	mov	Temp4, #0
	mov	A, Temp3
	swap A
	anl	A, #0Fh
	orl	A, Temp7
	mov	Temp3, A
	clr	C
	mov	A, Temp3
	subb	A, Temp1
	mov	Temp3, A
	jc	load_min_time_fast		; Check that result is still positive

	clr	C
	subb	A, #1
	jnc	calc_new_wait_times_fast_done	; Check that result is still above minumum

load_min_time_fast:
	mov	Temp3, #1

calc_new_wait_times_fast_done:	
	mov	Temp1, #Pgm_Comm_Timing	; Load timing setting
	mov	A, @Temp1				
	mov	Temp8, A				; Store in Temp8


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Wait advance timing routine
;
; No assumptions
; NOTE: Be VERY careful if using temp registers. They are passed over this routine
;
; Waits for the advance timing to elapse and sets up the next zero cross wait
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
wait_advance_timing:	
	jnb	Flags0.T3_PENDING, ($+5)
	ajmp	wait_advance_timing

	; Setup next wait time
	mov	TMR3RLL, Wt_ZC_Tout_Start_L
	mov	TMR3RLH, Wt_ZC_Tout_Start_H
	setb	Flags0.T3_PENDING
	orl	EIE1, #80h	; Enable timer 3 interrupts


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Calculate new wait times routine
;
; No assumptions
;
; Calculates new wait times
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
calc_new_wait_times:	
	clr	C
	clr	A
	subb	A, Temp3				; Negate
	mov	Temp1, A	
	clr	A
	subb	A, Temp4				
	mov	Temp2, A	
IF MCU_48MHZ >= 1
	clr	C
	mov	A, Temp1				; Multiply by 2
	rlc	A
	mov	Temp1, A
	mov	A, Temp2
	rlc	A
	mov	Temp2, A
ENDIF
	jnb	Flags1.HIGH_RPM, ($+6)	; Branch if high rpm
	ljmp	calc_new_wait_times_fast

	mov	A, Temp1				; Copy values
	mov	Temp3, A
	mov	A, Temp2
	mov	Temp4, A
	setb	C					; Negative numbers - set carry
	mov	A, Temp2				
	rrc	A					; Divide by 2
	mov	Temp6, A
	mov	A, Temp1
	rrc	A
	mov	Temp5, A
	mov	Wt_Zc_Tout_Start_L, Temp1; Set 15deg time for zero cross scan timeout
	mov	Wt_Zc_Tout_Start_H, Temp2
	clr	C
	mov	A, Temp8				; (Temp8 has Pgm_Comm_Timing)
	subb	A, #3				; Is timing normal?
	jz	store_times_decrease	; Yes - branch

	mov	A, Temp8				
	jb	ACC.0, adjust_timing_two_steps	; If an odd number - branch

	mov	A, Temp1				; Add 7.5deg and store in Temp1/2
	add	A, Temp5
	mov	Temp1, A
	mov	A, Temp2
	addc	A, Temp6
	mov	Temp2, A
	mov	A, Temp5				; Store 7.5deg in Temp3/4
	mov	Temp3, A
	mov	A, Temp6			
	mov	Temp4, A
	jmp	store_times_up_or_down

adjust_timing_two_steps:
	mov	A, Temp1				; Add 15deg and store in Temp1/2
	add	A, Temp1
	mov	Temp1, A
	mov	A, Temp2
	addc	A, Temp2
	mov	Temp2, A
	clr	C
	mov	A, Temp1
	add	A, #1
	mov	Temp1, A
	mov	A, Temp2
	addc	A, #0
	mov	Temp2, A
	mov	Temp3, #-1				; Store minimum time in Temp3/4
	mov	Temp4, #0FFh

store_times_up_or_down:
	clr	C
	mov	A, Temp8				
	subb	A, #3					; Is timing higher than normal?
	jc	store_times_decrease		; No - branch

store_times_increase:
	mov	Wt_Comm_Start_L, Temp3		; Now commutation time (~60deg) divided by 4 (~15deg nominal)
	mov	Wt_Comm_Start_H, Temp4
	mov	Wt_Adv_Start_L, Temp1		; New commutation advance time (~15deg nominal)
	mov	Wt_Adv_Start_H, Temp2
	mov	Wt_Zc_Scan_Start_L, Temp5	; Use this value for zero cross scan delay (7.5deg)
	mov	Wt_Zc_Scan_Start_H, Temp6
	ljmp	wait_before_zc_scan

store_times_decrease:
	mov	Wt_Comm_Start_L, Temp1		; Now commutation time (~60deg) divided by 4 (~15deg nominal)
	mov	Wt_Comm_Start_H, Temp2
	mov	Wt_Adv_Start_L, Temp3		; New commutation advance time (~15deg nominal)
	mov	Wt_Adv_Start_H, Temp4
	mov	Wt_Zc_Scan_Start_L, Temp5	; Use this value for zero cross scan delay (7.5deg)
	mov	Wt_Zc_Scan_Start_H, Temp6
	jnb	Flags1.STARTUP_PHASE, store_times_exit

	mov	Wt_Comm_Start_L, #0F0h		; Set very short delays for all but advance time during startup, in order to widen zero cross capture range
	mov	Wt_Comm_Start_H, #0FFh
	mov	Wt_Zc_Scan_Start_L, #0F0h
	mov	Wt_Zc_Scan_Start_H, #0FFh
	mov	Wt_Zc_Tout_Start_L, #0F0h
	mov	Wt_Zc_Tout_Start_H, #0FFh

store_times_exit:
	ljmp	wait_before_zc_scan


calc_new_wait_times_fast:	
	mov	A, Temp1				; Copy values
	mov	Temp3, A
	setb	C					; Negative numbers - set carry
	mov	A, Temp1				; Divide by 2
	rrc	A
	mov	Temp5, A
	mov	Wt_Zc_Tout_Start_L, Temp1; Set 15deg time for zero cross scan timeout
	clr	C
	mov	A, Temp8				; (Temp8 has Pgm_Comm_Timing)
	subb	A, #3				; Is timing normal?
	jz	store_times_decrease_fast; Yes - branch

	mov	A, Temp8				
	jb	ACC.0, adjust_timing_two_steps_fast	; If an odd number - branch

	mov	A, Temp1				; Add 7.5deg and store in Temp1
	add	A, Temp5
	mov	Temp1, A
	mov	A, Temp5				; Store 7.5deg in Temp3
	mov	Temp3, A
	ajmp	store_times_up_or_down_fast

adjust_timing_two_steps_fast:
	mov	A, Temp1				; Add 15deg and store in Temp1
	add	A, Temp1
	add	A, #1
	mov	Temp1, A
	mov	Temp3, #-1			; Store minimum time in Temp3

store_times_up_or_down_fast:
	clr	C
	mov	A, Temp8				
	subb	A, #3				; Is timing higher than normal?
	jc	store_times_decrease_fast; No - branch

store_times_increase_fast:
	mov	Wt_Comm_Start_L, Temp3		; Now commutation time (~60deg) divided by 4 (~15deg nominal)
	mov	Wt_Adv_Start_L, Temp1		; New commutation advance time (~15deg nominal)
	mov	Wt_Zc_Scan_Start_L, Temp5	; Use this value for zero cross scan delay (7.5deg)
	ljmp	wait_before_zc_scan

store_times_decrease_fast:
	mov	Wt_Comm_Start_L, Temp1		; Now commutation time (~60deg) divided by 4 (~15deg nominal)
	mov	Wt_Adv_Start_L, Temp3		; New commutation advance time (~15deg nominal)
	mov	Wt_Zc_Scan_Start_L, Temp5	; Use this value for zero cross scan delay (7.5deg)


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Wait before zero cross scan routine
;
; No assumptions
;
; Waits for the zero cross scan wait time to elapse
; Also sets up timer 3 for the zero cross scan timeout time
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
wait_before_zc_scan:	
	jnb	Flags0.T3_PENDING, ($+5)
	ajmp	wait_before_zc_scan

	mov	Startup_Zc_Timeout_Cntd, #2
setup_zc_scan_timeout:
	setb	Flags0.T3_PENDING
	orl	EIE1, #80h			; Enable timer 3 interrupts
	mov	A, Flags1
	anl	A, #((1 SHL STARTUP_PHASE)+(1 SHL INITIAL_RUN_PHASE))
	jz	wait_before_zc_scan_exit		

	mov	Temp1, Comm_Period4x_L	; Set long timeout when starting
	mov	Temp2, Comm_Period4x_H
	clr	C
	mov	A, Temp2
	rrc	A
	mov	Temp2, A
	mov	A, Temp1
	rrc	A
	mov	Temp1, A
IF MCU_48MHZ == 0
	clr	C
	mov	A, Temp2
	rrc	A
	mov	Temp2, A
	mov	A, Temp1
	rrc	A
	mov	Temp1, A
ENDIF
	jnb	Flags1.STARTUP_PHASE, setup_zc_scan_timeout_startup_done

	mov	A, Temp2
	add	A, #40h				; Increase timeout somewhat to avoid false wind up
	mov	Temp2, A

setup_zc_scan_timeout_startup_done:
	clr	IE_EA
	anl	EIE1, #7Fh			; Disable timer 3 interrupts
	mov	TMR3CN0, #00h			; Timer 3 disabled and interrupt flag cleared
	clr	C
	clr	A
	subb	A, Temp1				; Set timeout
	mov	TMR3L, A
	clr	A
	subb	A, Temp2		
	mov	TMR3H, A
	mov	TMR3CN0, #04h			; Timer 3 enabled and interrupt flag cleared
	setb	Flags0.T3_PENDING
	orl	EIE1, #80h			; Enable timer 3 interrupts
	setb	IE_EA

wait_before_zc_scan_exit:          
	ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Wait for comparator to go low/high routines
;
; No assumptions
;
; Waits for the zero cross scan wait time to elapse
; Then scans for comparator going low/high
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
wait_for_comp_out_low:
	setb	Flags0.DEMAG_DETECTED		; Set demag detected flag as default
	mov	Comparator_Read_Cnt, #0		; Reset number of comparator reads
	mov	Bit_Access, #00h			; Desired comparator output
	jnb	Flags1.DIR_CHANGE_BRAKE, ($+6)
	mov	Bit_Access, #40h		
	jmp	wait_for_comp_out_start

wait_for_comp_out_high:
	setb	Flags0.DEMAG_DETECTED		; Set demag detected flag as default
	mov	Comparator_Read_Cnt, #0		; Reset number of comparator reads
	mov	Bit_Access, #40h			; Desired comparator output
	jnb	Flags1.DIR_CHANGE_BRAKE, ($+6)
	mov	Bit_Access, #00h		

wait_for_comp_out_start:
	; Set number of comparator readings
	mov	Temp1, #1					; Number of OK readings required
	mov	Temp2, #1					; Max number of readings required
	jb	Flags1.HIGH_RPM, comp_scale_samples	; Branch if high rpm

	mov	A, Flags1					; Clear demag detected flag if start phases
	anl	A, #((1 SHL STARTUP_PHASE)+(1 SHL INITIAL_RUN_PHASE))
	jz	($+4)
		
	clr	Flags0.DEMAG_DETECTED

	mov	Temp2, #20 				; Too low value (~<15) causes rough running at pwm harmonics. Too high a value (~>35) causes the RCT4215 630 to run rough on full throttle
	mov 	A, Comm_Period4x_H			; Set number of readings higher for lower speeds
	clr	C
	rrc	A
	jnz	($+3)
	inc	A
	mov	Temp1, A
	clr	C						
	subb	A, #20
	jc	($+4)

	mov	Temp1, #20
	
	jnb	Flags1.STARTUP_PHASE, comp_scale_samples

	mov	Temp1, #27				; Set many samples during startup, approximately one pwm period
	mov	Temp2, #27

comp_scale_samples:
IF MCU_48MHZ >= 1
	clr	C
	mov	A, Temp1
	rlc	A
	mov	Temp1, A
	clr	C
	mov	A, Temp2
	rlc	A
	mov	Temp2, A
ENDIF
comp_check_timeout:
	jb	Flags0.T3_PENDING, comp_check_timeout_not_timed_out		; Has zero cross scan timeout elapsed?

	mov	A, Comparator_Read_Cnt			; Check that comparator has been read
	jz	comp_check_timeout_not_timed_out	; If not read - branch

	jnb	Flags1.STARTUP_PHASE, comp_check_timeout_timeout_extended	; Extend timeout during startup

	djnz	Startup_Zc_Timeout_Cntd, comp_check_timeout_extend_timeout

comp_check_timeout_timeout_extended:
	setb	Flags0.COMP_TIMED_OUT
	ajmp	setup_comm_wait

comp_check_timeout_extend_timeout:
	call	setup_zc_scan_timeout
comp_check_timeout_not_timed_out:
	inc	Comparator_Read_Cnt			; Increment comparator read count
	Read_Comp_Out					; Read comparator output
	anl	A, #40h
	cjne	A, Bit_Access, comp_read_wrong
	ajmp	comp_read_ok
	
comp_read_wrong:
	jnb	Flags1.STARTUP_PHASE, comp_read_wrong_not_startup

	inc	Temp1					; Increment number of OK readings required
	clr	C
	mov	A, Temp1
	subb	A, Temp2					; If above initial requirement - do not increment further
	jc	($+3)
	dec	Temp1

	ajmp	comp_check_timeout			; Continue to look for good ones

comp_read_wrong_not_startup:
	jb	Flags0.DEMAG_DETECTED, comp_read_wrong_extend_timeout

	inc	Temp1					; Increment number of OK readings required
	clr	C
	mov	A, Temp1
	subb	A, Temp2					
	jc	($+4)
	ajmp	wait_for_comp_out_start		; If above initial requirement - go back and restart

	ajmp	comp_check_timeout			; Otherwise - take another reading

comp_read_wrong_extend_timeout:
	clr	Flags0.DEMAG_DETECTED		; Clear demag detected flag
	anl	EIE1, #7Fh				; Disable timer 3 interrupts
	mov	TMR3CN0, #00h				; Timer 3 disabled and interrupt flag cleared
	jnb	Flags1.HIGH_RPM, comp_read_wrong_low_rpm	; Branch if not high rpm

	mov	TMR3L, #00h				; Set timeout to ~1ms
IF MCU_48MHZ >= 1
	mov	TMR3H, #0F0h
ELSE
	mov	TMR3H, #0F8h
ENDIF
comp_read_wrong_timeout_set:
	mov	TMR3CN0, #04h				; Timer 3 enabled and interrupt flag cleared
	setb	Flags0.T3_PENDING
	orl	EIE1, #80h				; Enable timer 3 interrupts
	ljmp	wait_for_comp_out_start		; If comparator output is not correct - go back and restart

comp_read_wrong_low_rpm:
	mov	A, Comm_Period4x_H			; Set timeout to ~4x comm period 4x value
	mov	Temp7, #0FFh				; Default to long
IF MCU_48MHZ >= 1
	clr	C
	rlc	A
	jc	comp_read_wrong_load_timeout

ENDIF
	clr	C
	rlc	A
	jc	comp_read_wrong_load_timeout

	clr	C
	rlc	A
	jc	comp_read_wrong_load_timeout

	mov	Temp7, A

comp_read_wrong_load_timeout:
	clr	C
	clr	A
	subb	A, Temp7
	mov	TMR3L, #0
	mov	TMR3H, A
	ajmp	comp_read_wrong_timeout_set

comp_read_ok:
	clr	C
	mov	A, Startup_Cnt				; Force a timeout for the first commutation
	subb	A, #1
	jnc	($+4)
	ajmp	wait_for_comp_out_start

	jnb	Flags0.DEMAG_DETECTED, ($+5)	; Do not accept correct comparator output if it is demag
	ajmp	wait_for_comp_out_start

	djnz	Temp1, comp_read_ok_jmp		; Decrement readings counter - repeat comparator reading if not zero
	ajmp	($+4)

comp_read_ok_jmp:
	ajmp	comp_check_timeout

	clr	Flags0.COMP_TIMED_OUT


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Setup commutation timing routine
;
; No assumptions
;
; Sets up and starts wait from commutation to zero cross
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
setup_comm_wait:
	clr	IE_EA
	anl	EIE1, #7Fh		; Disable timer 3 interrupts
	mov	TMR3CN0, #00h		; Timer 3 disabled and interrupt flag cleared
	mov	TMR3L, Wt_Comm_Start_L
	mov	TMR3H, Wt_Comm_Start_H
	mov	TMR3CN0, #04h		; Timer 3 enabled and interrupt flag cleared
	; Setup next wait time
	mov	TMR3RLL, Wt_Adv_Start_L
	mov	TMR3RLH, Wt_Adv_Start_H
	setb	Flags0.T3_PENDING
	orl	EIE1, #80h		; Enable timer 3 interrupts
	setb	IE_EA			; Enable interrupts again


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Evaluate comparator integrity
;
; No assumptions
;
; Checks comparator signal behaviour versus expected behaviour
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
evaluate_comparator_integrity:
	mov	A, Flags1
	anl	A, #((1 SHL STARTUP_PHASE)+(1 SHL INITIAL_RUN_PHASE))
	jz	eval_comp_check_timeout

	jb	Flags1.INITIAL_RUN_PHASE, ($+5)	; Do not increment beyond startup phase
	inc	Startup_Cnt					; Increment counter
	jmp	eval_comp_exit

eval_comp_check_timeout:
	jnb	Flags0.COMP_TIMED_OUT, eval_comp_exit	; Has timeout elapsed?
	jb	Flags1.DIR_CHANGE_BRAKE, eval_comp_exit	; Do not exit run mode if it is braking
	jb	Flags0.DEMAG_DETECTED, eval_comp_exit	; Do not exit run mode if it is a demag situation
	dec	SP								; Routine exit without "ret" command
	dec	SP

eval_comp_exit:
	ret

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Wait for commutation routine
;
; No assumptions
;
; Waits from zero cross to commutation 
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
wait_for_comm: 
        ret
	; Update demag metric
	mov	Temp1, #0
	jnb	Flags0.DEMAG_DETECTED, ($+5)

	mov	Temp1, #1

	mov	A, Demag_Detected_Metric	; Sliding average of 8, 256 when demag and 0 when not. Limited to minimum 120
	mov	B, #7
	mul	AB					; Multiply by 7
	mov	Temp2, A
	mov	A, B					; Add new value for current demag status
	add	A, Temp1				
	mov	B, A
	mov	A, Temp2
	mov	C, B.0				; Divide by 8
	rrc	A					
	mov	C, B.1
	rrc	A
	mov	C, B.2
	rrc	A
	mov	Demag_Detected_Metric, A
	clr	C
	subb	A, #120				; Limit to minimum 120
	jnc	($+5)

	mov	Demag_Detected_Metric, #120

	clr	C
	mov	A, Demag_Detected_Metric	; Check demag metric
	subb	A, Demag_Pwr_Off_Thresh
	;jc	wait_for_comm_wait		; Cut power if many consecutive demags. This will help retain sync during hard accelerations

	;All_pwmFETs_off
	;Set_Pwms_Off

wait_for_comm_wait:
	jnb Flags0.T3_PENDING, ($+5)			
	;ajmp	wait_for_comm_wait

	; Setup next wait time
	mov	TMR3RLL, Wt_Zc_Scan_Start_L
	mov	TMR3RLH, Wt_Zc_Scan_Start_H
	setb	Flags0.T3_PENDING
	orl	EIE1, #80h			; Enable timer 3 interrupts
	ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Commutation routines
;
; No assumptions
;
; Performs commutation switching 
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
; Comm phase 1 to comm phase 2
comm1comm2:	
	Set_RPM_Out

	clr 	IE_EA				; Disable all interrupts
	BcomFET_off 				; Turn off comfet
	AcomFET_on				; Turn on comfet
	Set_Pwm_C					; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_B 			; Set comparator phase
	jmp	comm_exit

comm12_rev:	
	clr 	IE_EA				; Disable all interrupts
	BcomFET_off 				; Turn off comfet
	CcomFET_on				; Turn on comfet (reverse)
	Set_Pwm_A					; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_B 			; Set comparator phase
	jmp	comm_exit


; Comm phase 2 to comm phase 3
comm2comm3:	
	Clear_RPM_Out

	clr 	IE_EA				; Disable all interrupts
	CpwmFET_off				; Turn off pwmfet
	Set_Pwm_B					; To reapply power after a demag cut
	AcomFET_on
	setb	IE_EA
	Set_Comp_Phase_C 			; Set comparator phase
	ajmp	comm_exit

comm23_rev:
	clr 	IE_EA				; Disable all interrupts
	ApwmFET_off				; Turn off pwmfet (reverse)
	Set_Pwm_B					; To reapply power after a demag cut
	CcomFET_on
	setb	IE_EA
	Set_Comp_Phase_A 			; Set comparator phase (reverse)
	ajmp	comm_exit


; Comm phase 3 to comm phase 4
comm3comm4:	
	Set_RPM_Out

	clr 	IE_EA				; Disable all interrupts
	AcomFET_off 				; Turn off comfet
	CcomFET_on				; Turn on comfet
	Set_Pwm_B					; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_A 			; Set comparator phase
	jmp	comm_exit

comm34_rev:	
	clr 	IE_EA				; Disable all interrupts
	CcomFET_off 				; Turn off comfet (reverse)
	AcomFET_on				; Turn on comfet (reverse)
	Set_Pwm_B					; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_C 			; Set comparator phase (reverse)
	jmp	comm_exit


; Comm phase 4 to comm phase 5
comm4comm5:	
	Clear_RPM_Out

	clr 	IE_EA				; Disable all interrupts
	BpwmFET_off				; Turn off pwmfet
	Set_Pwm_A					; To reapply power after a demag cut
	CcomFET_on
	setb	IE_EA
	Set_Comp_Phase_B 			; Set comparator phase
	jmp	comm_exit

comm45_rev:
	clr 	IE_EA				; Disable all interrupts
	BpwmFET_off				; Turn off pwmfet
	Set_Pwm_C
	AcomFET_on				; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_B 			; Set comparator phase
	jmp	comm_exit


; Comm phase 5 to comm phase 6
comm5comm6:	
	Set_RPM_Out

	clr 	IE_EA				; Disable all interrupts
	CcomFET_off 				; Turn off comfet
	BcomFET_on				; Turn on comfet
	Set_Pwm_A					; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_C 			; Set comparator phase
	jmp	comm_exit

comm56_rev:
	clr 	IE_EA				; Disable all interrupts
	AcomFET_off 				; Turn off comfet (reverse)
	BcomFET_on				; Turn on comfet
	Set_Pwm_C					; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_A 			; Set comparator phase (reverse)
	jmp	comm_exit


; Comm phase 6 to comm phase 1
comm6comm1:	
	Clear_RPM_Out

	clr 	IE_EA				; Disable all interrupts
	ApwmFET_off				; Turn off pwmfet
	Set_Pwm_C
	BcomFET_on				; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_A 			; Set comparator phase
	jmp	comm_exit

comm61_rev:
	clr 	IE_EA				; Disable all interrupts
	CpwmFET_off				; Turn off pwmfet (reverse)
	Set_Pwm_A
	BcomFET_on				; To reapply power after a demag cut
	setb	IE_EA
	Set_Comp_Phase_C 			; Set comparator phase (reverse)

comm_exit:
	ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Set default parameters
;
; No assumptions
;
; Sets default programming parameters
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
set_default_parameters:
	mov	Temp1, #_Pgm_Gov_P_Gain
	mov	@Temp1, #0FFh	; Governor P gain
	inc	Temp1
	mov	@Temp1, #0FFh	; Governor I gain
	inc	Temp1
	mov	@Temp1, #0FFh	; Governor mode
	inc	Temp1
	mov	@Temp1, #0FFh	; Low voltage limit
	inc	Temp1
	mov	@Temp1, #0FFh	; Multi gain
	inc	Temp1
	mov	@Temp1, #0FFh	
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_STARTUP_PWR
	inc	Temp1
	mov	@Temp1, #0FFh	; Pwm freq
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_DIRECTION

	mov	Temp1, #Pgm_Enable_TX_Program
	mov	@Temp1, #DEFAULT_PGM_ENABLE_TX_PROGRAM
	inc	Temp1
	mov	@Temp1, #0FFh	; Main rearm start
	inc	Temp1
	mov	@Temp1, #0FFh	; Governor setup target
	inc	Temp1
	mov	@Temp1, #0FFh	; Startup rpm	
	inc	Temp1
	mov	@Temp1, #0FFh	; Startup accel
	inc	Temp1
	mov	@Temp1, #0FFh	; Voltage comp
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_COMM_TIMING
	inc	Temp1
	mov	@Temp1, #0FFh	; Damping force	
	inc	Temp1
	mov	@Temp1, #0FFh	; Governor range
	inc	Temp1
	mov	@Temp1, #0FFh	; Startup method	
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_MIN_THROTTLE
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_MAX_THROTTLE
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_BEEP_STRENGTH
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_BEACON_STRENGTH
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_BEACON_DELAY
	inc	Temp1
	mov	@Temp1, #0FFh	; Throttle rate	
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_DEMAG_COMP
	inc	Temp1
	mov	@Temp1, #0FFh	; Bec voltage high
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_CENTER_THROTTLE
	inc	Temp1
	mov	@Temp1, #0FFh	
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_ENABLE_TEMP_PROT
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_ENABLE_POWER_PROT
	inc	Temp1
	mov	@Temp1, #0FFh	; Enable pwm input
	inc	Temp1
	mov	@Temp1, #0FFh	; Pwm dither
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_BRAKE_ON_STOP
	inc	Temp1
	mov	@Temp1, #DEFAULT_PGM_LED_CONTROL
	ret




;**** **** **** **** **** **** **** **** **** **** **** **** ****
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Main program start
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;**** **** **** **** **** **** **** **** **** **** **** **** ****

pgm_start:
	; Initialize flash keys to invalid values
	mov	Flash_Key_1, #0
	mov	Flash_Key_2, #0
	; Disable the WDT.
	mov	WDTCN, #0DEh		; Disable watchdog
	mov	WDTCN, #0ADh		
	; Initialize stack
	mov	SP, #0c0h			; Stack = 64 upper bytes of RAM
	; Initialize VDD monitor
IF MCU_48MHZ < 2
	orl	VDM0CN, #080h    	; Enable the VDD monitor
ENDIF
	mov 	RSTSRC, #06h   	; Set missing clock and VDD monitor as a reset source if not 1S capable
	; Set clock frequency
	mov	CLKSEL, #00h		; Set clock divider to 1
	; Ports initialization
	mov	P0, #P0_INIT
	mov	P0MDIN, #P0_DIGITAL
	mov	P0MDOUT, #P0_PUSHPULL
	mov	P0, #P0_INIT
	mov	P0SKIP, #P0_SKIP				
	mov	P1, #P1_INIT
	mov	P1MDIN, #P1_DIGITAL
	mov	P1MDOUT, #P1_PUSHPULL
	mov	P1, #P1_INIT
	mov	P1SKIP, #P1_SKIP				
	mov	P2MDOUT, #P2_PUSHPULL				
	; Initialize the XBAR and related functionality
	Initialize_Xbar
	; Clear RAM
	clr	A				; Clear accumulator
	mov	Temp1, A			; Clear Temp1
	clear_ram:	
	mov	@Temp1, A			; Clear RAM
	djnz Temp1, clear_ram	; Is A not zero? - jump
	; Set default programmed parameters
	call	set_default_parameters
	; Read all programmed parameters
	call read_all_eeprom_parameters

	; Set initial arm variable
	mov	Initial_Arm, #1


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; No signal entry point
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
init_no_signal:
	; Disable interrupts explicitly
	clr	IE_EA
	; Initialize flash keys to invalid values
	mov	Flash_Key_1, #0
	mov	Flash_Key_2, #0
	; Check if input signal is high for more than 15ms
	mov	Temp1, #250
input_high_check_1:
	mov	Temp2, #250
input_high_check_2:
	jb	RTX_PORT.RTX_PIN, bootloader_done	; Look for low
	djnz	Temp2, input_high_check_2
	djnz	Temp1, input_high_check_1

IF MCU_48MHZ == 2
	ljmp	0F000h			; Jump to bootloader
ELSE
	ljmp	1C00h			; Jump to bootloader
ENDIF

bootloader_done:

	; Set clock frequency
IF MCU_48MHZ >= 1
	Set_MCU_Clk_24MHz
ENDIF
	; Setup timers for pwm input
	mov	IT01CF, #RTX_PIN	; Route RCP input to INT0
	mov	TCON, #11h		; Timer 0 run and INT0 edge triggered
	mov	CKCON0, #04h		; Timer 0 clock is system clock
	mov	TMOD, #09h		; Timer 0 set to 16bits and gated by INT0
	mov	TMR2CN0, #04h		; Timer 2 enabled
	mov	TMR3CN0, #04h		; Timer 3 enabled
	Initialize_PCA			; Initialize PCA
	Set_Pwm_Polarity		; Set pwm polarity
	Enable_Power_Pwm_Module	; Enable power pwm module
	Enable_Damp_Pwm_Module	; Enable damping pwm module
	; Enable interrupts
IF MCU_48MHZ == 0
	mov	IE, #21h			; Enable timer 2 interrupts and INT0 interrupts
ELSE
	mov	IE, #23h			; Enable timer 0, timer 2 interrupts and INT0 interrupts
ENDIF
	mov	EIE1, #90h		; Enable timer 3 and PCA0 interrupts
	mov	IP, #01h			; High priority to INT0 interrupts
	; Initialize comparator
	Initialize_Comparator	; Initialize comparator
	; Initialize ADC
	Initialize_Adc			; Initialize ADC operation
	call	wait1ms
	setb	IE_EA			; Enable all interrupts
	; Reset stall count
	mov	Stall_Cnt, #0
	; Initialize RC pulse
	clr	Flags2.RCP_UPDATED		 	; Clear updated flag
	; Clear all shot flags
	mov 	Dshot_Cmd, #0					; Clear Dshot command
	mov 	Dshot_Cmd_Cnt, #0				; Clear Dshot command count
	; Test whether signal is regular pwm
	mov	Rcp_Outside_Range_Cnt, #0		; Reset out of range counter
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check how many pulses were outside normal range ("900-2235us")
	subb	A, #10						
	jnc	($+4)
	ajmp	validate_rcp_start

	; Test whether signal is OneShot125
	;setb	Flags2.RCP_ONESHOT125			; Set OneShot125 flag
	mov	Rcp_Outside_Range_Cnt, #0		; Reset out of range counter
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check how many pulses were outside normal range ("900-2235us")
	subb	A, #10
	jnc	($+4)
	ajmp	validate_rcp_start

	; Test whether signal is OneShot42
	mov	Rcp_Outside_Range_Cnt, #0		; Reset out of range counter

	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check how many pulses were outside normal range ("900-2235us")
	subb	A, #10
	jnc	($+4)
	ajmp	validate_rcp_start

	; Setup timers for DShot
	mov	IT01CF, #(80h+(RTX_PIN SHL 4)+(RTX_PIN))	; Route RCP input to INT0/1, with INT1 inverted
	mov	TCON, #51h		; Timer 0/1 run and INT0 edge triggered
	mov	CKCON0, #01h		; Timer 0/1 clock is system clock divided by 4 (for DShot150)
	mov	TMOD, #0AAh		; Timer 0/1 set to 8bits auto reload and gated by INT0
	mov	TH0, #0			; Auto reload value zero
	mov	TH1, #0
	; Setup interrupts for DShot
	clr	IE_ET0			; Disable timer 0 interrupts
	setb	IE_ET1			; Enable timer 1 interrupts
	setb	IE_EX1			; Enable int1 interrupts
	; Setup variables for DSshot150
IF MCU_48MHZ >= 1
	mov	DShot_Timer_Preset, #128			; Load DShot sync timer preset (for DShot150)
ELSE
	mov	DShot_Timer_Preset, #192
ENDIF
	mov	DShot_Pwm_Thr, #20				; Load DShot qualification pwm threshold (for DShot150)
	mov	DShot_Frame_Length_Thr, #80		; Load DShot frame length criteria
	; Test whether signal is DShot150
	mov	Rcp_Outside_Range_Cnt, #10		; Set out of range counter
	mov	DShot_Pwm_Thr, #16				; Load DShot regular pwm threshold
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check if pulses were accepted
	subb	A, #10
	mov 	Dshot_Cmd, #0
	mov 	Dshot_Cmd_Cnt, #0
	jc	validate_rcp_start

	; Setup variables for DShot300
	mov	CKCON0, #0Ch					; Timer 0/1 clock is system clock (for DShot300)
IF MCU_48MHZ >= 1
	mov	DShot_Timer_Preset, #0			; Load DShot sync timer preset (for DShot300)
ELSE
	mov	DShot_Timer_Preset, #128
ENDIF
	mov	DShot_Pwm_Thr, #40				; Load DShot qualification pwm threshold (for DShot300)
	mov	DShot_Frame_Length_Thr, #40		; Load DShot frame length criteria
	; Test whether signal is DShot300
	mov	Rcp_Outside_Range_Cnt, #10		; Set out of range counter
	mov	DShot_Pwm_Thr, #32				; Load DShot regular pwm threshold
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check if pulses were accepted
	subb	A, #10
	mov 	Dshot_Cmd, #0
	mov 	Dshot_Cmd_Cnt, #0
	jc	validate_rcp_start

	; Setup variables for DShot600
	mov	CKCON0, #0Ch					; Timer 0/1 clock is system clock (for DShot600)
IF MCU_48MHZ >= 1
	mov	DShot_Timer_Preset, #128			; Load DShot sync timer preset (for DShot600)
ELSE
	mov	DShot_Timer_Preset, #192
ENDIF
	mov	DShot_Pwm_Thr, #20				; Load DShot qualification pwm threshold (for DShot600)
	mov	DShot_Frame_Length_Thr, #20		; Load DShot frame length criteria
	; Test whether signal is DShot600
	mov	Rcp_Outside_Range_Cnt, #10		; Set out of range counter
	mov	DShot_Pwm_Thr, #16				; Load DShot regular pwm threshold
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check if pulses were accepted
	subb	A, #10
	mov 	Dshot_Cmd, #0
	mov 	Dshot_Cmd_Cnt, #0
	jc	validate_rcp_start

	; Setup timers for Multishot
	mov	IT01CF, #RTX_PIN	; Route RCP input to INT0
	mov	TCON, #11h		; Timer 0 run and INT0 edge triggered
	mov	CKCON0, #04h		; Timer 0 clock is system clock
	mov	TMOD, #09h		; Timer 0 set to 16bits and gated by INT0
	; Setup interrupts for Multishot
	setb	IE_ET0			; Enable timer 0 interrupts
	clr	IE_ET1			; Disable timer 1 interrupts
	clr	IE_EX1			; Disable int1 interrupts
	; Test whether signal is Multishot
	;clr	Flags2.RCP_DSHOT
	;setb	Flags2.RCP_MULTISHOT			; Set Multishot flag
	mov	Rcp_Outside_Range_Cnt, #0		; Reset out of range counter
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check how many pulses were outside normal range ("900-2235us")
	subb	A, #10
	jc	validate_rcp_start


validate_rcp_start:
	; Validate RC pulse
	call wait3ms						; Wait for new RC pulse
	;jb	Flags2.RCP_UPDATED, ($+6)		; Is there an updated RC pulse available - proceed

	; Arming sequence start
arming_start:
	;jb	Flags2.RCP_DSHOT, ($+6)	; Disable tx programming for DShot
	;jnb	Flags3.PGM_BIDIR, ($+6)

	;lmp	program_by_tx_checked	; Disable tx programming if bidirectional operation

	call wait3ms
	mov	Temp1, #Pgm_Enable_TX_Program; Start programming mode entry if enabled
	mov	A, @Temp1				
	clr	C
	subb	A, #1				; Is TX programming enabled?

arming_check:
	; Initialize flash keys to valid values
	mov	Flash_Key_1, #0A5h
	mov	Flash_Key_2, #0F1h
	mov	Temp8, #2				; Set 1 seconds wait time

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Start entry point
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
init_start:
	clr	IE_EA
	clr	A
	setb	IE_EA
	clr	A
	mov	Adc_Conversion_Cnt, A
	mov	Flags0, A				; Clear flags0
	mov	Flags1, A				; Clear flags1
	mov	Demag_Detected_Metric, A	; Clear demag metric
	;**** **** **** **** ****
	; Motor start beginning
	;**** **** **** **** **** 
	mov	Adc_Conversion_Cnt, #8				; Make sure a temp reading is done
	call wait1ms
	call start_adc_conversion
read_initial_temp:
	jnb	ADC0CN0_ADINT, read_initial_temp
	Read_Adc_Result						; Read initial temperature
	mov	A, Temp2
	jnz	($+3)							; Is reading below 256?

	mov	Temp1, A							; Yes - set average temperature value to zero

	mov	Current_Average_Temp, Temp1			; Set initial average temperature
	mov	Adc_Conversion_Cnt, #8				; Make sure a temp reading is done next time
	; Set up start operating conditions
	clr	IE_EA				; Disable interrupts
	mov	Pwm_Limit, #255
	mov	Pwm_Limit_By_Rpm, #255
	setb	IE_EA
	; Begin startup sequence
	Set_MCU_Clk_48MHz
	setb	Flags1.STARTUP_PHASE		; Set startup phase flag
	mov	Startup_Cnt, #0			; Reset counter
	call comm5comm6				; Initialize commutation
	call comm6comm1
	call initialize_timing			; Initialize timing
	call	calc_next_comm_timing		; Set virtual commutation point
	call initialize_timing			; Initialize timing
	call	calc_next_comm_timing
	call	initialize_timing			; Initialize timing

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Run entry point
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****

; Run 1 = B(p-on) + C(n-pwm) - comparator A evaluated
; Out_cA changes from low to high
run1:

	call wait_for_comp_out_high	; Wait for high
;		setup_comm_wait		; Setup wait time from zero cross to commutation
;		evaluate_comparator_integrity	; Check whether comparator reading has been normal
	call wait_for_comm			; Wait from zero cross to commutation
	call comm1comm2			; Commutate
	call calc_next_comm_timing	; Calculate next timing and wait advance timing wait
;		wait_advance_timing		; Wait advance timing and start zero cross wait
;		calc_new_wait_times
;		wait_before_zc_scan		; Wait zero cross wait and start zero cross timeout

; Run 2 = A(p-on) + C(n-pwm) - comparator B evaluated
; Out_cB changes from high to low
run2:
	call wait_for_comp_out_low
;		setup_comm_wait
;		evaluate_comparator_integrity
	mov	Pwm_Limit_By_Rpm, #255
	call wait_for_comm
	call comm2comm3
	call calc_next_comm_timing
;		wait_advance_timing
;		calc_new_wait_times
;		wait_before_zc_scan

; Run 3 = A(p-on) + B(n-pwm) - comparator C evaluated
; Out_cC changes from low to high
run3:
	call wait_for_comp_out_high
;		setup_comm_wait
;		evaluate_comparator_integrity
	call wait_for_comm
	call comm3comm4
	call calc_next_comm_timing
;		wait_advance_timing
;		calc_new_wait_times
;		wait_before_zc_scan

; Run 4 = C(p-on) + B(n-pwm) - comparator A evaluated
; Out_cA changes from high to low
run4:
	call wait_for_comp_out_low
;		setup_comm_wait
;		evaluate_comparator_integrity
	call wait_for_comm
	call comm4comm5
	call calc_next_comm_timing
;		wait_advance_timing
;		calc_new_wait_times
;		wait_before_zc_scan

; Run 5 = C(p-on) + A(n-pwm) - comparator B evaluated
; Out_cB changes from low to high
run5:
	call wait_for_comp_out_high
;		setup_comm_wait
;		evaluate_comparator_integrity
	call wait_for_comm
	call comm5comm6
	call calc_next_comm_timing
;		wait_advance_timing
;		calc_new_wait_times
;		wait_before_zc_scan

; Run 6 = B(p-on) + A(n-pwm) - comparator C evaluated
; Out_cC changes from high to low
run6:
	call start_adc_conversion
	call wait_for_comp_out_low
;		setup_comm_wait
;		evaluate_comparator_integrity
	call wait_for_comm
	call comm6comm1
	call calc_next_comm_timing
;		wait_advance_timing
;		calc_new_wait_times
;		wait_before_zc_scan

	; Check if it is direct startup

	; Set spoolup power variables
	mov	Pwm_Limit, #255		; Set initial max power
	; Check startup counter
	mov	Temp2, #24				; Set nominal startup parameters
	mov	Temp3, #12
	clr	C
	mov	A, Startup_Cnt				; Load counter
	subb	A, Temp2					; Is counter above requirement?
	
	clr	Flags1.STARTUP_PHASE		; Clear startup phase flag
	setb	Flags1.INITIAL_RUN_PHASE		; Set initial run phase flag
	mov	Initial_Run_Rot_Cntd, Temp3	; Set initial run rotation count
	mov	Pwm_Limit, #255
	mov	Pwm_Limit_By_Rpm, #255
	
	ljmp	run1						; Continue to run

;**** **** **** **** **** **** **** **** **** **** **** **** ****

$include (SlotZPgm.inc)				; Include source code for programming the ESC
$include (SlotZBootLoad.inc)			; Include source code for bootloader

;**** **** **** **** **** **** **** **** **** **** **** **** ****

CSEG AT 19FDh
reset:
ljmp	pgm_start

END
