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
Sheet 8 8
Title ""
Date "8 jun 2014"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 11300 6250 2    60   Output ~ 0
CLK_P
Text HLabel 11300 6150 2    60   Output ~ 0
CLK_N
Text HLabel 3000 2400 0    60   Input ~ 0
D+5V
Text HLabel 3000 2500 0    60   Input ~ 0
DGND
Text Label 5000 3850 0    60   ~ 0
SW_ON_OFF
Text Label 5000 4950 0    60   ~ 0
SW_TAP
Text HLabel 11300 5450 2    60   Output ~ 0
SCL
Text HLabel 11300 5750 2    60   Input ~ 0
OUTDET1
Text HLabel 11300 5650 2    60   Input ~ 0
OUTDET2
Text HLabel 11300 5350 2    60   Input ~ 0
INDET1
Text HLabel 11300 5550 2    60   Input ~ 0
INDET2
NoConn ~ 4700 4050
NoConn ~ 4700 5150
$Comp
L 8Y1021 SW501
U 1 1 52883799
P 4200 3950
F 0 "SW501" H 4000 4100 50  0000 C CNN
F 1 "8Y1021" H 4050 3800 50  0000 C CNN
F 2 "8Y10" H 4200 3950 60  0001 C CNN
F 3 "~" H 4200 3950 60  0000 C CNN
	1    4200 3950
	1    0    0    -1  
$EndComp
$Comp
L 8Y1021 SW502
U 1 1 528837A6
P 4200 5050
F 0 "SW502" H 4000 5200 50  0000 C CNN
F 1 "8Y1021" H 4050 4900 50  0000 C CNN
F 2 "8Y10" H 4200 5050 60  0001 C CNN
F 3 "~" H 4200 5050 60  0000 C CNN
	1    4200 5050
	1    0    0    -1  
$EndComp
Text HLabel 11300 5250 2    60   BiDi ~ 0
SDA
$Comp
L ATTINY861A-S IC601
U 1 1 52970584
P 7300 6050
F 0 "IC601" H 6500 7000 60  0000 C CNN
F 1 "ATTINY861A-S" H 7900 5100 60  0000 C CNN
F 2 "SO20" H 6550 5100 60  0001 C CNN
F 3 "" H 7300 6050 60  0000 C CNN
	1    7300 6050
	1    0    0    -1  
$EndComp
$Comp
L CONN_4X2 P601
U 1 1 5297CCCE
P 7700 2850
F 0 "P601" H 7700 3100 50  0000 C CNN
F 1 "CONN_4X2" V 7700 2850 40  0000 C CNN
F 2 "" H 7700 2850 60  0000 C CNN
F 3 "" H 7700 2850 60  0000 C CNN
	1    7700 2850
	1    0    0    -1  
$EndComp
$Comp
L C_1005 C644
U 1 1 5297CF9E
P 5750 5250
F 0 "C644" H 5750 5425 40  0000 C CNN
F 1 "0.1u" H 5750 5375 40  0000 C CNN
F 2 "c_1005" H 5750 5125 30  0000 C CNN
F 3 "~" V 5750 5250 60  0000 C CNN
	1    5750 5250
	-1   0    0    1   
$EndComp
$Comp
L C_1608 C646
U 1 1 5297D0DE
P 5750 5650
F 0 "C646" H 5750 5825 40  0000 C CNN
F 1 "2.2u" H 5750 5775 40  0000 C CNN
F 2 "c_1608" H 5750 5525 30  0000 C CNN
F 3 "~" V 5750 5650 60  0000 C CNN
	1    5750 5650
	1    0    0    -1  
$EndComp
$Comp
L R_1005 R634
U 1 1 5297D0F2
P 10800 2300
F 0 "R634" V 10880 2300 40  0000 C CNN
F 1 "10k" V 10807 2301 40  0000 C CNN
F 2 "r_1005" V 10730 2300 30  0000 C CNN
F 3 "~" H 10800 2300 30  0000 C CNN
	1    10800 2300
	1    0    0    -1  
$EndComp
$Comp
L R_1005 R633
U 1 1 5297D0FF
P 10600 2300
F 0 "R633" V 10680 2300 40  0000 C CNN
F 1 "10k" V 10607 2301 40  0000 C CNN
F 2 "r_1005" V 10530 2300 30  0000 C CNN
F 3 "~" H 10600 2300 30  0000 C CNN
	1    10600 2300
	1    0    0    -1  
$EndComp
$Comp
L R_1005 R635
U 1 1 52981123
P 9375 6150
F 0 "R635" V 9455 6150 40  0000 C CNN
F 1 "100" V 9382 6151 40  0000 C CNN
F 2 "r_1005" V 9305 6150 30  0000 C CNN
F 3 "~" H 9375 6150 30  0000 C CNN
	1    9375 6150
	0    -1   -1   0   
$EndComp
$Comp
L R_1005 R636
U 1 1 52981135
P 9375 6250
F 0 "R636" V 9455 6250 40  0000 C CNN
F 1 "100" V 9382 6251 40  0000 C CNN
F 2 "r_1005" V 9305 6250 30  0000 C CNN
F 3 "~" H 9375 6250 30  0000 C CNN
	1    9375 6250
	0    -1   -1   0   
$EndComp
$Comp
L R_1005 R637
U 1 1 5298113B
P 9050 5250
F 0 "R637" V 9130 5250 40  0000 C CNN
F 1 "100" V 9057 5251 40  0000 C CNN
F 2 "r_1005" V 8980 5250 30  0000 C CNN
F 3 "~" H 9050 5250 30  0000 C CNN
	1    9050 5250
	0    -1   -1   0   
$EndComp
$Comp
L R_1005 R638
U 1 1 52981141
P 9050 5450
F 0 "R638" V 9130 5450 40  0000 C CNN
F 1 "100" V 9057 5451 40  0000 C CNN
F 2 "r_1005" V 8980 5450 30  0000 C CNN
F 3 "~" H 9050 5450 30  0000 C CNN
	1    9050 5450
	0    -1   -1   0   
$EndComp
$Comp
L R_1005 R639
U 1 1 52981446
P 7200 7250
F 0 "R639" V 7280 7250 40  0000 C CNN
F 1 "10k" V 7207 7251 40  0000 C CNN
F 2 "r_1005" V 7130 7250 30  0000 C CNN
F 3 "~" H 7200 7250 30  0000 C CNN
	1    7200 7250
	0    -1   -1   0   
$EndComp
$Comp
L C_1005 C647
U 1 1 52981537
P 7700 7550
F 0 "C647" H 7700 7725 40  0000 C CNN
F 1 "1000p" H 7700 7675 40  0000 C CNN
F 2 "c_1005" H 7700 7425 30  0000 C CNN
F 3 "~" V 7700 7550 60  0000 C CNN
	1    7700 7550
	0    -1   -1   0   
$EndComp
Text Label 9700 6350 0    60   ~ 0
ENC_A
Text Label 9700 6450 0    60   ~ 0
ENC_B
$Comp
L C_1608 C643
U 1 1 52981849
P 5000 4300
F 0 "C643" H 5000 4475 40  0000 C CNN
F 1 "1u" H 5000 4425 40  0000 C CNN
F 2 "c_1608" H 5000 4175 30  0000 C CNN
F 3 "~" V 5000 4300 60  0000 C CNN
	1    5000 4300
	0    -1   -1   0   
$EndComp
$Comp
L C_1608 C645
U 1 1 5298190D
P 5000 5350
F 0 "C645" H 5000 5525 40  0000 C CNN
F 1 "1u" H 5000 5475 40  0000 C CNN
F 2 "c_1608" H 5000 5225 30  0000 C CNN
F 3 "~" V 5000 5350 60  0000 C CNN
	1    5000 5350
	0    -1   -1   0   
$EndComp
$Comp
L AVR-ISP-6 CON601
U 1 1 52992219
P 10550 8250
F 0 "CON601" H 10470 8490 50  0000 C CNN
F 1 "AVR-ISP-6" H 10310 8020 50  0000 L BNN
F 2 "AVR-ISP-6" V 10030 8290 50  0001 C CNN
F 3 "" H 10550 8250 60  0000 C CNN
	1    10550 8250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 3850 5000 3850
Wire Wire Line
	5000 3850 8600 3850
Wire Wire Line
	4700 4950 5000 4950
Wire Wire Line
	5000 4950 8700 4950
Wire Wire Line
	3000 3950 3600 3950
Wire Wire Line
	3600 3950 3700 3950
Wire Wire Line
	3600 5050 3700 5050
Connection ~ 3600 3950
Wire Wire Line
	8400 6350 8900 6350
Wire Wire Line
	8900 6350 10200 6350
Wire Wire Line
	9300 5450 10800 5450
Wire Wire Line
	10800 5450 11300 5450
Wire Wire Line
	9300 5250 10600 5250
Wire Wire Line
	10600 5250 11300 5250
Wire Wire Line
	5400 6850 6200 6850
Wire Wire Line
	5400 2500 5400 5250
Wire Wire Line
	5400 5250 5400 5650
Wire Wire Line
	5400 5650 5400 6450
Wire Wire Line
	5400 6450 5400 6850
Wire Wire Line
	5400 6850 5400 7850
Wire Wire Line
	5400 6450 6200 6450
Wire Wire Line
	5950 5650 6100 5650
Wire Wire Line
	6100 5650 6200 5650
Wire Wire Line
	6100 2400 6100 5250
Wire Wire Line
	6100 5250 6100 5650
Wire Wire Line
	6100 5650 6100 7250
Wire Wire Line
	6100 7250 6100 8800
Wire Wire Line
	5950 5250 6100 5250
Wire Wire Line
	6100 5250 6200 5250
Wire Wire Line
	8700 4950 8700 5950
Wire Wire Line
	8700 5950 8400 5950
Wire Wire Line
	8600 3850 8600 5850
Wire Wire Line
	8600 5850 8400 5850
Connection ~ 6100 2400
Connection ~ 6100 5250
Connection ~ 5400 6450
Connection ~ 5400 2500
Wire Wire Line
	11300 5350 8400 5350
Wire Wire Line
	8400 5550 11300 5550
Wire Wire Line
	8400 5650 11300 5650
Wire Wire Line
	8400 5750 11300 5750
Wire Wire Line
	10400 6650 10400 3000
Wire Wire Line
	8400 6450 10300 6450
Wire Wire Line
	6850 2700 7300 2700
Wire Wire Line
	6850 1950 6850 2400
Wire Wire Line
	6850 2400 6850 2700
Wire Wire Line
	8550 2500 8550 2800
Wire Wire Line
	3000 2500 5400 2500
Wire Wire Line
	5400 2500 8550 2500
Wire Wire Line
	3000 2400 6100 2400
Wire Wire Line
	6100 2400 6850 2400
Wire Wire Line
	10800 2550 10800 2900
Wire Wire Line
	10800 2900 10800 5450
Wire Wire Line
	10800 2900 8100 2900
Wire Wire Line
	8550 2800 8100 2800
Wire Wire Line
	8100 2700 10600 2700
Wire Wire Line
	10600 2550 10600 2700
Wire Wire Line
	10600 2700 10600 5250
Wire Wire Line
	10400 3000 8100 3000
Wire Wire Line
	10200 6350 10200 3300
Wire Wire Line
	10200 3300 6450 3300
Wire Wire Line
	6450 3300 6450 2900
Wire Wire Line
	6450 2900 7300 2900
Wire Wire Line
	10300 6450 10300 3200
Wire Wire Line
	10300 3200 6350 3200
Wire Wire Line
	6350 3200 6350 3000
Wire Wire Line
	6350 3000 7300 3000
Connection ~ 6100 5650
Wire Wire Line
	5550 5250 5400 5250
Connection ~ 5400 5250
Connection ~ 5400 5650
Connection ~ 10800 2900
Connection ~ 10600 2700
Wire Wire Line
	10600 2050 10600 1950
Wire Wire Line
	6850 1950 10600 1950
Wire Wire Line
	10600 1950 10800 1950
Wire Wire Line
	10800 1950 10800 2050
Connection ~ 6850 2400
Connection ~ 10600 1950
Wire Wire Line
	8400 6150 9050 6150
Wire Wire Line
	9050 6150 9125 6150
Wire Wire Line
	8400 6250 8975 6250
Wire Wire Line
	8975 6250 9125 6250
Wire Wire Line
	8400 5250 8800 5250
Wire Wire Line
	8400 5450 8800 5450
Wire Wire Line
	9625 6150 11300 6150
Wire Wire Line
	9625 6250 11300 6250
Wire Wire Line
	8400 6850 8600 6850
Wire Wire Line
	8600 6850 8600 7250
Wire Wire Line
	8600 7250 8600 8350
Wire Wire Line
	7450 7250 7700 7250
Wire Wire Line
	7700 7250 8600 7250
Wire Wire Line
	6100 7250 6950 7250
Wire Wire Line
	7700 7350 7700 7250
Connection ~ 7700 7250
Wire Wire Line
	7700 7750 7700 7850
Wire Wire Line
	7700 7850 7700 8875
Wire Wire Line
	5400 7850 7700 7850
Connection ~ 5400 6850
Wire Wire Line
	5000 3850 5000 4100
Connection ~ 5000 3850
Wire Wire Line
	5000 4500 5000 4600
Wire Wire Line
	5000 5150 5000 4950
Connection ~ 5000 4950
Wire Wire Line
	5000 5550 5000 5650
Wire Wire Line
	10100 6550 10100 3400
Wire Wire Line
	10100 3400 6550 3400
Wire Wire Line
	6550 3400 6550 2800
Wire Wire Line
	6550 2800 7300 2800
Connection ~ 10600 5250
Connection ~ 10800 5450
Wire Wire Line
	8400 6550 9300 6550
Wire Wire Line
	9300 6550 10100 6550
Wire Wire Line
	8400 6650 10100 6650
Wire Wire Line
	10100 6650 10400 6650
Wire Wire Line
	8975 8150 10425 8150
Wire Wire Line
	10675 8150 11400 8150
Wire Wire Line
	11400 8150 11400 8800
Wire Wire Line
	11400 8800 6100 8800
Connection ~ 6100 7250
Wire Wire Line
	8900 8250 10425 8250
Wire Wire Line
	10675 8250 11150 8250
Wire Wire Line
	11150 8250 11150 7950
Wire Wire Line
	11150 7950 9050 7950
Wire Wire Line
	8600 8350 10425 8350
Connection ~ 8600 7250
Wire Wire Line
	10675 8350 11300 8350
Wire Wire Line
	11300 8350 11300 8875
Wire Wire Line
	11300 8875 7700 8875
Connection ~ 7700 7850
Wire Wire Line
	9050 7950 9050 6150
Connection ~ 9050 6150
Wire Wire Line
	8975 8150 8975 6250
Connection ~ 8975 6250
Wire Wire Line
	8900 8250 8900 6350
Connection ~ 8900 6350
Wire Wire Line
	8400 6750 10550 6750
$Comp
L R_1005 R640
U 1 1 529994E4
P 10800 6750
F 0 "R640" V 10880 6750 40  0000 C CNN
F 1 "100" V 10807 6751 40  0000 C CNN
F 2 "r_1005" V 10730 6750 30  0000 C CNN
F 3 "~" H 10800 6750 30  0000 C CNN
	1    10800 6750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	11050 6750 11300 6750
Text HLabel 11300 6750 2    60   Output ~ 0
ON_OFF
Wire Wire Line
	3600 3950 3600 4600
Wire Wire Line
	3600 4600 3600 5050
Wire Wire Line
	3600 5050 3600 5650
Text HLabel 3000 3950 0    60   Input ~ 0
AGND
Wire Wire Line
	5400 5650 5550 5650
Wire Wire Line
	3600 5650 5000 5650
Connection ~ 3600 5050
Wire Wire Line
	5000 4600 3600 4600
Connection ~ 3600 4600
Text Label 7200 2900 2    60   ~ 0
ENC_A
Text Label 7200 3000 2    60   ~ 0
ENC_B
Text Label 7200 2700 2    60   ~ 0
D+5V
Text Label 8150 2800 0    60   ~ 0
DGND
Text Label 8150 2700 0    60   ~ 0
SDA
Text Label 8150 2900 0    60   ~ 0
SCL
Text Label 8150 3000 0    60   ~ 0
FB
Text Label 7200 2800 2    60   ~ 0
VOL
$EndSCHEMATC
