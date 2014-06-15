EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:ESS_Technology
LIBS:Panasonic
LIBS:fujisoku
LIBS:amphenol
LIBS:analog_devices
LIBS:marushin-musen
LIBS:my_opamp
LIBS:linear_technology
LIBS:my_regulator
LIBS:my_capacitor
LIBS:my_resistor
LIBS:jrc
LIBS:rohm
LIBS:alps
LIBS:on_semiconductor
LIBS:AnalogDelay-cache
EELAYER 27 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 8
Title ""
Date "8 jun 2014"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 3500 4950 1350 950 
U 5266EDDE
F0 "Input/Anti Alias Filter Block" 50
F1 "Input_Block.sch" 50
F2 "ON_OFF" I L 3500 5050 60 
F3 "BYPOUT2" O R 4850 5150 60 
F4 "BYPOUT1" O R 4850 5050 60 
F5 "AAILIASOUT1" O R 4850 5300 60 
F6 "AAILIASOUT2" O R 4850 5400 60 
F7 "A+5V" I L 3500 5600 60 
F8 "AGND" I L 3500 5700 60 
F9 "A-5V" I L 3500 5800 60 
F10 "IN1DET" O R 4850 5600 60 
F11 "IN2DET" O R 4850 5700 60 
F12 "FBIN1_1" I L 3500 5200 60 
F13 "FBIN2_2" I L 3500 5500 60 
F14 "FBIN2_1" I L 3500 5400 60 
F15 "FBIN1_2" I L 3500 5300 60 
$EndSheet
$Sheet
S 7150 5000 1200 1200
U 526DDC0A
F0 "BBD Block" 50
F1 "BBD_Block.sch" 50
F2 "BBDIN1" I L 7150 5100 60 
F3 "CLK_P" I L 7150 5350 60 
F4 "CLK_N" I L 7150 5450 60 
F5 "A+5V" I L 7150 5900 60 
F6 "AGND" I L 7150 6000 60 
F7 "BBDIN2" I L 7150 5200 60 
F8 "BBDOUT2" O R 8350 5200 60 
F9 "BBDOUT1" O R 8350 5100 60 
F10 "A-5V" I L 7150 6100 60 
F11 "ROUTESEL1" I L 7150 5600 60 
F12 "ROUTESEL2" I L 7150 5700 60 
$EndSheet
$Sheet
S 1150 5500 850  650 
U 5271D51F
F0 "Supply Block" 50
F1 "Supply_Block.sch" 50
F2 "A+5V" O R 2000 5600 60 
F3 "AGND" O R 2000 5700 60 
F4 "A-5V" O R 2000 5800 60 
F5 "D+5V" O R 2000 5950 60 
F6 "DGND" O R 2000 6050 60 
$EndSheet
Entry Wire Line
	6500 5450 6600 5350
Entry Wire Line
	6500 5550 6600 5450
Text Label 6600 5350 0    60   ~ 0
CLK_P
Text Label 6600 5450 0    60   ~ 0
CLK_N
Entry Wire Line
	2600 5150 2700 5050
Text Label 2700 5050 0    60   ~ 0
ON_OFF
Entry Wire Line
	8600 5700 8700 5600
Entry Wire Line
	8600 5800 8700 5700
Text Label 8700 5600 0    60   ~ 0
MIXSEL1
Text Label 8700 5700 0    60   ~ 0
MIXSEL2
Entry Wire Line
	14250 4900 14150 4800
Text Label 14150 4800 2    60   ~ 0
CLK_P
Text Label 14150 4900 2    60   ~ 0
CLK_N
Text Label 14150 5800 2    60   ~ 0
SCL
Text Label 14150 5900 2    60   ~ 0
SDA
Entry Wire Line
	14150 5800 14250 5900
Entry Wire Line
	14150 5900 14250 6000
Text Label 8700 5800 0    60   ~ 0
SCL
Entry Wire Line
	8600 5900 8700 5800
Entry Wire Line
	8600 6000 8700 5900
Text Label 8700 5900 0    60   ~ 0
SDA
$Sheet
S 9350 5000 1150 1750
U 52718D84
F0 "Clock Cancel Filter/Output Block" 50
F1 "Output_Block.sch" 50
F2 "A+5V" I L 9350 6450 60 
F3 "AGND" I L 9350 6550 60 
F4 "A-5V" I L 9350 6650 60 
F5 "CCANCELIN1" I L 9350 5100 60 
F6 "CCANCELIN2" I L 9350 5200 60 
F7 "BYPIN2" I L 9350 5450 60 
F8 "BYP1IN" I L 9350 5350 60 
F9 "MIXSEL1" I L 9350 5600 60 
F10 "MIXSEL2" I L 9350 5700 60 
F11 "SCL" I L 9350 5800 60 
F12 "SDA" I L 9350 5900 60 
F13 "OUTDET1" O R 10500 5650 60 
F14 "OUTDET2" O R 10500 5750 60 
F15 "FBOUT1_1" O R 10500 6150 60 
F16 "FBOUT2_1" O R 10500 6350 60 
F17 "D+5V" I L 9350 6350 60 
F18 "FBOUT1_2" O R 10500 6250 60 
F19 "FBOUT2_2" O R 10500 6450 60 
F20 "O1" O R 10500 5200 60 
F21 "O2" O R 10500 5300 60 
F22 "DGND" I L 9350 6250 60 
$EndSheet
$Sheet
S 12250 4700 1150 2050
U 5279B794
F0 "Connect Block" 50
F1 "Connect_Block.sch" 50
F2 "CLK_P" O R 13400 4800 60 
F3 "CLK_N" O R 13400 4900 60 
F4 "SCL" O R 13400 5800 60 
F5 "SDA" B R 13400 5900 60 
F6 "D+5V" I L 12250 4800 60 
F7 "DGND" I L 12250 4900 60 
F8 "OUTDET1" I L 12250 6100 60 
F9 "OUTDET2" I L 12250 6200 60 
F10 "INDET1" I L 12250 5900 60 
F11 "INDET2" I L 12250 6000 60 
F12 "ON_OFF" O R 13400 6350 60 
F13 "AGND" I L 12250 5100 60 
$EndSheet
Entry Wire Line
	14150 6350 14250 6450
Entry Wire Line
	11450 5200 11550 5300
Entry Wire Line
	11450 5300 11550 5400
Text Label 14150 6350 2    60   ~ 0
ON_OFF
Text Label 11450 5200 2    60   ~ 0
MIXSEL1
Text Label 11450 5300 2    60   ~ 0
MIXSEL2
Wire Wire Line
	13400 4900 14150 4900
Wire Wire Line
	13400 4800 14150 4800
Wire Wire Line
	14150 5900 13400 5900
Wire Wire Line
	14150 5800 13400 5800
Wire Bus Line
	14250 4800 14250 7800
Wire Wire Line
	8700 5700 9350 5700
Wire Wire Line
	8700 5600 9350 5600
Wire Bus Line
	8600 5600 8600 7800
Wire Wire Line
	2700 5050 3500 5050
Wire Bus Line
	2600 5050 2600 7800
Connection ~ 3400 5800
Connection ~ 3300 5700
Connection ~ 3200 5600
Wire Wire Line
	6600 5450 7150 5450
Wire Wire Line
	6600 5350 7150 5350
Wire Bus Line
	6500 5350 6500 7800
Connection ~ 6850 7000
Wire Wire Line
	6850 5900 6850 7000
Wire Wire Line
	7150 5900 6850 5900
Connection ~ 6950 7100
Wire Wire Line
	6950 6000 6950 7100
Wire Wire Line
	7150 6000 6950 6000
Connection ~ 7050 7200
Wire Wire Line
	7050 6100 7050 7200
Wire Wire Line
	7150 6100 7050 6100
Wire Wire Line
	9050 6450 9350 6450
Wire Wire Line
	9050 7000 9050 6450
Wire Wire Line
	3200 5600 3200 7000
Wire Wire Line
	2000 5600 3500 5600
Wire Wire Line
	2000 5800 3500 5800
Wire Wire Line
	3400 7200 3400 5800
Wire Wire Line
	9250 7200 9250 6650
Wire Wire Line
	9250 6650 9350 6650
Wire Wire Line
	9150 6550 9350 6550
Wire Wire Line
	9150 7100 9150 6550
Wire Wire Line
	3300 5700 3300 7100
Wire Wire Line
	2000 5700 3500 5700
Wire Wire Line
	3200 5400 3500 5400
Wire Wire Line
	3300 5300 3500 5300
Wire Wire Line
	8350 5200 9350 5200
Wire Wire Line
	8350 5100 9350 5100
Wire Wire Line
	8850 5450 9350 5450
Wire Wire Line
	8850 4800 8850 5450
Wire Wire Line
	5300 4800 8850 4800
Wire Wire Line
	5300 5150 5300 4800
Wire Wire Line
	4850 5150 5300 5150
Wire Wire Line
	8950 5350 9350 5350
Wire Wire Line
	8950 4700 8950 5350
Wire Wire Line
	5200 4700 8950 4700
Wire Wire Line
	5200 5050 5200 4700
Wire Wire Line
	4850 5050 5200 5050
Wire Wire Line
	5600 5200 7150 5200
Wire Wire Line
	5600 5400 5600 5200
Wire Wire Line
	4850 5400 5600 5400
Wire Wire Line
	5500 5100 7150 5100
Wire Wire Line
	5500 5300 5500 5100
Wire Wire Line
	4850 5300 5500 5300
Wire Bus Line
	2600 7800 14250 7800
Wire Wire Line
	2000 5950 2200 5950
Wire Wire Line
	2200 5950 2200 8000
Wire Wire Line
	2200 8000 11650 8000
Wire Wire Line
	11650 8000 11650 4800
Wire Wire Line
	2000 6050 2300 6050
Wire Wire Line
	2300 6050 2300 8100
Wire Wire Line
	2300 8100 11750 8100
Wire Wire Line
	11750 8100 11750 4900
Wire Wire Line
	11750 4900 12250 4900
Wire Wire Line
	11650 4800 12250 4800
Wire Wire Line
	4850 5600 7150 5600
Wire Wire Line
	4850 5700 7150 5700
Wire Wire Line
	10500 5650 11900 5650
Wire Wire Line
	10500 5750 12000 5750
Wire Wire Line
	5600 5600 5600 7600
Wire Wire Line
	5600 7600 11200 7600
Wire Wire Line
	11200 7600 11200 5900
Wire Wire Line
	11200 5900 12250 5900
Connection ~ 5600 5600
Wire Wire Line
	5700 5700 5700 7700
Wire Wire Line
	5700 7700 11100 7700
Wire Wire Line
	11100 7700 11100 6000
Wire Wire Line
	11100 6000 12250 6000
Connection ~ 5700 5700
Wire Wire Line
	11900 5650 11900 6100
Wire Wire Line
	11900 6100 12250 6100
Wire Wire Line
	12250 6200 12000 6200
Wire Wire Line
	12000 6200 12000 5750
Wire Wire Line
	8700 5800 9350 5800
Wire Wire Line
	8700 5900 9350 5900
Wire Wire Line
	10600 6150 10500 6150
Wire Wire Line
	10600 4600 10600 6150
Wire Wire Line
	3400 4600 10600 4600
Wire Wire Line
	3500 5200 3400 5200
Wire Wire Line
	3400 5200 3400 4600
Wire Wire Line
	3300 5300 3300 4500
Wire Wire Line
	3300 4500 10700 4500
Wire Wire Line
	10700 4500 10700 6250
Wire Wire Line
	10700 6250 10500 6250
Wire Wire Line
	10500 6350 10800 6350
Wire Wire Line
	10800 6350 10800 4400
Wire Wire Line
	10800 4400 3200 4400
Wire Wire Line
	3200 4400 3200 5400
Wire Wire Line
	3500 5500 3100 5500
Wire Wire Line
	3100 5500 3100 4300
Wire Wire Line
	3100 4300 10900 4300
Wire Wire Line
	10900 4300 10900 6450
Wire Wire Line
	10900 6450 10500 6450
Wire Wire Line
	9350 6350 8950 6350
Wire Wire Line
	8950 6350 8950 8000
Connection ~ 8950 8000
Wire Wire Line
	3300 7100 11850 7100
Wire Wire Line
	3400 7200 9250 7200
Wire Wire Line
	3200 7000 9050 7000
Wire Bus Line
	11550 7800 11550 5100
Wire Wire Line
	11450 5200 10500 5200
Wire Wire Line
	10500 5300 11450 5300
Wire Wire Line
	9350 6250 8850 6250
Wire Wire Line
	8850 6250 8850 8100
Connection ~ 8850 8100
Wire Wire Line
	13400 6350 14150 6350
Wire Wire Line
	12250 5100 11850 5100
Wire Wire Line
	11850 5100 11850 7100
Connection ~ 9150 7100
$EndSCHEMATC
