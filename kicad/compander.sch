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
Sheet 7 8
Title "compander.sch"
Date "8 jun 2014"
Rev ""
Comp "A.S Electroworks"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L R_1608 R109
U 1 1 53335914
P 5650 1250
AR Path="/5266EDDE/539408DE/53335914" Ref="R109"  Part="1" 
AR Path="/52718D84/53946925/53335914" Ref="R109"  Part="1" 
F 0 "R109" V 5730 1250 40  0000 C CNN
F 1 "OPEN" V 5657 1251 40  0000 C CNN
F 2 "r_1608" V 5580 1250 30  0000 C CNN
F 3 "~" H 5650 1250 30  0000 C CNN
	1    5650 1250
	-1   0    0    -1  
$EndComp
$Comp
L POT RV101
U 1 1 53335900
P 5650 1850
AR Path="/5266EDDE/539408DE/53335900" Ref="RV101"  Part="1" 
AR Path="/52718D84/53946925/53335900" Ref="RV101"  Part="1" 
F 0 "RV101" H 5650 1750 50  0000 C CNN
F 1 "OPEN" H 5650 1850 50  0000 C CNN
F 2 "~" H 5650 1850 60  0000 C CNN
F 3 "~" H 5650 1850 60  0000 C CNN
	1    5650 1850
	0    -1   -1   0   
$EndComp
$Comp
L C_1608 C113
U 1 1 533358BA
P 5300 2200
AR Path="/5266EDDE/539408DE/533358BA" Ref="C113"  Part="1" 
AR Path="/52718D84/53946925/533358BA" Ref="C113"  Part="1" 
F 0 "C113" H 5300 2375 40  0000 C CNN
F 1 "180p" H 5300 2325 40  0000 C CNN
F 2 "c_1608" H 5300 2075 30  0000 C CNN
F 3 "~" V 5300 2200 60  0000 C CNN
	1    5300 2200
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR01
U 1 1 533358AD
P 5650 900
AR Path="/5266EDDE/539408DE/533358AD" Ref="#PWR01"  Part="1" 
AR Path="/52718D84/53946925/533358AD" Ref="#PWR01"  Part="1" 
F 0 "#PWR01" H 5650 1000 30  0001 C CNN
F 1 "VCC" H 5650 1000 30  0000 C CNN
F 2 "" H 5650 900 60  0000 C CNN
F 3 "" H 5650 900 60  0000 C CNN
	1    5650 900 
	1    0    0    -1  
$EndComp
$Comp
L C_1608 C119
U 1 1 533356CB
P 7450 3400
AR Path="/5266EDDE/539408DE/533356CB" Ref="C119"  Part="1" 
AR Path="/52718D84/53946925/533356CB" Ref="C119"  Part="1" 
F 0 "C119" H 7450 3575 40  0000 C CNN
F 1 "2.2u" H 7450 3525 40  0000 C CNN
F 2 "c_1608" H 7450 3275 30  0000 C CNN
F 3 "~" V 7450 3400 60  0000 C CNN
	1    7450 3400
	1    0    0    -1  
$EndComp
$Comp
L C_1608 C101
U 1 1 53324F1C
P 3050 1950
AR Path="/5266EDDE/539408DE/53324F1C" Ref="C101"  Part="1" 
AR Path="/52718D84/53946925/53324F1C" Ref="C101"  Part="1" 
F 0 "C101" H 3050 2125 40  0000 C CNN
F 1 "2.2u" H 3050 2075 40  0000 C CNN
F 2 "c_1608" H 3050 1825 30  0000 C CNN
F 3 "~" V 3050 1950 60  0000 C CNN
	1    3050 1950
	1    0    0    -1  
$EndComp
$Comp
L C_1608 C102
U 1 1 53324F16
P 3050 2300
AR Path="/5266EDDE/539408DE/53324F16" Ref="C102"  Part="1" 
AR Path="/52718D84/53946925/53324F16" Ref="C102"  Part="1" 
F 0 "C102" H 3050 2475 40  0000 C CNN
F 1 "2.2u" H 3050 2425 40  0000 C CNN
F 2 "c_1608" H 3050 2175 30  0000 C CNN
F 3 "~" V 3050 2300 60  0000 C CNN
	1    3050 2300
	1    0    0    -1  
$EndComp
$Comp
L R_1608 R115
U 1 1 53324E12
P 8100 3400
AR Path="/5266EDDE/539408DE/53324E12" Ref="R115"  Part="1" 
AR Path="/52718D84/53946925/53324E12" Ref="R115"  Part="1" 
F 0 "R115" V 8180 3400 40  0000 C CNN
F 1 "5.6k" V 8107 3401 40  0000 C CNN
F 2 "r_1608" V 8030 3400 30  0000 C CNN
F 3 "~" H 8100 3400 30  0000 C CNN
	1    8100 3400
	0    1    -1   0   
$EndComp
$Comp
L R_1608 R104
U 1 1 53324DBA
P 3650 2300
AR Path="/5266EDDE/539408DE/53324DBA" Ref="R104"  Part="1" 
AR Path="/52718D84/53946925/53324DBA" Ref="R104"  Part="1" 
F 0 "R104" V 3730 2300 40  0000 C CNN
F 1 "22k" V 3657 2301 40  0000 C CNN
F 2 "r_1608" V 3580 2300 30  0000 C CNN
F 3 "~" H 3650 2300 30  0000 C CNN
	1    3650 2300
	0    1    -1   0   
$EndComp
$Comp
L R_1608 R103
U 1 1 53324DB4
P 3650 1950
AR Path="/5266EDDE/539408DE/53324DB4" Ref="R103"  Part="1" 
AR Path="/52718D84/53946925/53324DB4" Ref="R103"  Part="1" 
F 0 "R103" V 3730 1950 40  0000 C CNN
F 1 "22k" V 3657 1951 40  0000 C CNN
F 2 "r_1608" V 3580 1950 30  0000 C CNN
F 3 "~" H 3650 1950 30  0000 C CNN
	1    3650 1950
	0    1    -1   0   
$EndComp
$Comp
L R_1608 R113
U 1 1 53324C1D
P 6450 2850
AR Path="/5266EDDE/539408DE/53324C1D" Ref="R113"  Part="1" 
AR Path="/52718D84/53946925/53324C1D" Ref="R113"  Part="1" 
F 0 "R113" V 6530 2850 40  0000 C CNN
F 1 "10k" V 6457 2851 40  0000 C CNN
F 2 "r_1608" V 6380 2850 30  0000 C CNN
F 3 "~" H 6450 2850 30  0000 C CNN
	1    6450 2850
	0    1    -1   0   
$EndComp
$Comp
L R_1608 R111
U 1 1 53324C17
P 5750 2850
AR Path="/5266EDDE/539408DE/53324C17" Ref="R111"  Part="1" 
AR Path="/52718D84/53946925/53324C17" Ref="R111"  Part="1" 
F 0 "R111" V 5830 2850 40  0000 C CNN
F 1 "10k" V 5757 2851 40  0000 C CNN
F 2 "r_1608" V 5680 2850 30  0000 C CNN
F 3 "~" H 5750 2850 30  0000 C CNN
	1    5750 2850
	0    1    -1   0   
$EndComp
$Comp
L C_1608 C117
U 1 1 53324BA8
P 6850 4000
AR Path="/5266EDDE/539408DE/53324BA8" Ref="C117"  Part="1" 
AR Path="/52718D84/53946925/53324BA8" Ref="C117"  Part="1" 
F 0 "C117" H 6850 4175 40  0000 C CNN
F 1 "2.2u" H 6850 4125 40  0000 C CNN
F 2 "c_1608" H 6850 3875 30  0000 C CNN
F 3 "~" V 6850 4000 60  0000 C CNN
	1    6850 4000
	0    -1   -1   0   
$EndComp
$Comp
L R_1608 R107
U 1 1 53324B6E
P 5500 2650
AR Path="/5266EDDE/539408DE/53324B6E" Ref="R107"  Part="1" 
AR Path="/52718D84/53946925/53324B6E" Ref="R107"  Part="1" 
F 0 "R107" V 5580 2650 40  0000 C CNN
F 1 "0" V 5507 2651 40  0000 C CNN
F 2 "r_1608" V 5430 2650 30  0000 C CNN
F 3 "~" H 5500 2650 30  0000 C CNN
	1    5500 2650
	0    1    -1   0   
$EndComp
$Comp
L C_1608 C103
U 1 1 53324B05
P 3050 2650
AR Path="/5266EDDE/539408DE/53324B05" Ref="C103"  Part="1" 
AR Path="/52718D84/53946925/53324B05" Ref="C103"  Part="1" 
F 0 "C103" H 3050 2825 40  0000 C CNN
F 1 "2.2u" H 3050 2775 40  0000 C CNN
F 2 "c_1608" H 3050 2525 30  0000 C CNN
F 3 "~" V 3050 2650 60  0000 C CNN
	1    3050 2650
	1    0    0    -1  
$EndComp
$Comp
L AGND #PWR02
U 1 1 53324AD8
P 4350 4500
AR Path="/5266EDDE/539408DE/53324AD8" Ref="#PWR02"  Part="1" 
AR Path="/52718D84/53946925/53324AD8" Ref="#PWR02"  Part="1" 
F 0 "#PWR02" H 4350 4500 40  0001 C CNN
F 1 "AGND" H 4350 4430 50  0000 C CNN
F 2 "" H 4350 4500 60  0000 C CNN
F 3 "" H 4350 4500 60  0000 C CNN
	1    4350 4500
	1    0    0    -1  
$EndComp
$Comp
L C_1608 C111
U 1 1 53324A8B
P 4350 4200
AR Path="/5266EDDE/539408DE/53324A8B" Ref="C111"  Part="1" 
AR Path="/52718D84/53946925/53324A8B" Ref="C111"  Part="1" 
F 0 "C111" H 4350 4375 40  0000 C CNN
F 1 "0.22u" H 4350 4325 40  0000 C CNN
F 2 "c_1608" H 4350 4075 30  0000 C CNN
F 3 "~" V 4350 4200 60  0000 C CNN
	1    4350 4200
	0    -1   -1   0   
$EndComp
$Comp
L R_1608 R101
U 1 1 533249FC
P 3500 3450
AR Path="/5266EDDE/539408DE/533249FC" Ref="R101"  Part="1" 
AR Path="/52718D84/53946925/533249FC" Ref="R101"  Part="1" 
F 0 "R101" V 3580 3450 40  0000 C CNN
F 1 "0" V 3507 3451 40  0000 C CNN
F 2 "r_1608" V 3430 3450 30  0000 C CNN
F 3 "~" H 3500 3450 30  0000 C CNN
	1    3500 3450
	1    0    0    1   
$EndComp
$Comp
L C_1608 C105
U 1 1 533249E6
P 3050 3800
AR Path="/5266EDDE/539408DE/533249E6" Ref="C105"  Part="1" 
AR Path="/52718D84/53946925/533249E6" Ref="C105"  Part="1" 
F 0 "C105" H 3050 3975 40  0000 C CNN
F 1 "2.2u" H 3050 3925 40  0000 C CNN
F 2 "c_1608" H 3050 3675 30  0000 C CNN
F 3 "~" V 3050 3800 60  0000 C CNN
	1    3050 3800
	1    0    0    -1  
$EndComp
$Comp
L C_1608 C104
U 1 1 533249D9
P 3050 3100
AR Path="/5266EDDE/539408DE/533249D9" Ref="C104"  Part="1" 
AR Path="/52718D84/53946925/533249D9" Ref="C104"  Part="1" 
F 0 "C104" H 3050 3275 40  0000 C CNN
F 1 "2.2u" H 3050 3225 40  0000 C CNN
F 2 "c_1608" H 3050 2975 30  0000 C CNN
F 3 "~" V 3050 3100 60  0000 C CNN
	1    3050 3100
	1    0    0    -1  
$EndComp
$Comp
L SA571 U101
U 1 1 5332139D
P 4600 3400
AR Path="/5266EDDE/539408DE/5332139D" Ref="U101"  Part="1" 
AR Path="/52718D84/53946925/5332139D" Ref="U101"  Part="1" 
F 0 "U101" H 5500 3700 70  0000 C CNN
F 1 "SA571" H 5500 3600 70  0000 C CNN
F 2 "SOIC-16" H 5350 3400 60  0000 C CNN
F 3 "http://www.onsemi.jp/pub/Collateral/SA571-D.PDF" H 5350 3400 60  0001 C CNN
	1    4600 3400
	1    0    0    -1  
$EndComp
$Comp
L R_1608 R110
U 1 1 53335D8D
P 5650 5650
AR Path="/5266EDDE/539408DE/53335D8D" Ref="R110"  Part="1" 
AR Path="/52718D84/53946925/53335D8D" Ref="R110"  Part="1" 
F 0 "R110" V 5730 5650 40  0000 C CNN
F 1 "OPEN" V 5657 5651 40  0000 C CNN
F 2 "r_1608" V 5580 5650 30  0000 C CNN
F 3 "~" H 5650 5650 30  0000 C CNN
	1    5650 5650
	-1   0    0    -1  
$EndComp
$Comp
L POT RV102
U 1 1 53335D93
P 5650 6250
AR Path="/5266EDDE/539408DE/53335D93" Ref="RV102"  Part="1" 
AR Path="/52718D84/53946925/53335D93" Ref="RV102"  Part="1" 
F 0 "RV102" H 5650 6150 50  0000 C CNN
F 1 "OPEN" H 5650 6250 50  0000 C CNN
F 2 "~" H 5650 6250 60  0000 C CNN
F 3 "~" H 5650 6250 60  0000 C CNN
	1    5650 6250
	0    -1   -1   0   
$EndComp
$Comp
L C_1608 C114
U 1 1 53335D99
P 5300 6600
AR Path="/5266EDDE/539408DE/53335D99" Ref="C114"  Part="1" 
AR Path="/52718D84/53946925/53335D99" Ref="C114"  Part="1" 
F 0 "C114" H 5300 6775 40  0000 C CNN
F 1 "180p" H 5300 6725 40  0000 C CNN
F 2 "c_1608" H 5300 6475 30  0000 C CNN
F 3 "~" V 5300 6600 60  0000 C CNN
	1    5300 6600
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR03
U 1 1 53335D9F
P 5650 5300
AR Path="/5266EDDE/539408DE/53335D9F" Ref="#PWR03"  Part="1" 
AR Path="/52718D84/53946925/53335D9F" Ref="#PWR03"  Part="1" 
F 0 "#PWR03" H 5650 5400 30  0001 C CNN
F 1 "VCC" H 5650 5400 30  0000 C CNN
F 2 "" H 5650 5300 60  0000 C CNN
F 3 "" H 5650 5300 60  0000 C CNN
	1    5650 5300
	1    0    0    -1  
$EndComp
$Comp
L C_1608 C120
U 1 1 53335DA8
P 7450 7800
AR Path="/5266EDDE/539408DE/53335DA8" Ref="C120"  Part="1" 
AR Path="/52718D84/53946925/53335DA8" Ref="C120"  Part="1" 
F 0 "C120" H 7450 7975 40  0000 C CNN
F 1 "2.2u" H 7450 7925 40  0000 C CNN
F 2 "c_1608" H 7450 7675 30  0000 C CNN
F 3 "~" V 7450 7800 60  0000 C CNN
	1    7450 7800
	1    0    0    -1  
$EndComp
$Comp
L C_1608 C106
U 1 1 53335DB4
P 3050 6350
AR Path="/5266EDDE/539408DE/53335DB4" Ref="C106"  Part="1" 
AR Path="/52718D84/53946925/53335DB4" Ref="C106"  Part="1" 
F 0 "C106" H 3050 6525 40  0000 C CNN
F 1 "2.2u" H 3050 6475 40  0000 C CNN
F 2 "c_1608" H 3050 6225 30  0000 C CNN
F 3 "~" V 3050 6350 60  0000 C CNN
	1    3050 6350
	1    0    0    -1  
$EndComp
$Comp
L C_1608 C107
U 1 1 53335DBA
P 3050 6700
AR Path="/5266EDDE/539408DE/53335DBA" Ref="C107"  Part="1" 
AR Path="/52718D84/53946925/53335DBA" Ref="C107"  Part="1" 
F 0 "C107" H 3050 6875 40  0000 C CNN
F 1 "2.2u" H 3050 6825 40  0000 C CNN
F 2 "c_1608" H 3050 6575 30  0000 C CNN
F 3 "~" V 3050 6700 60  0000 C CNN
	1    3050 6700
	1    0    0    -1  
$EndComp
$Comp
L R_1608 R116
U 1 1 53335DC0
P 8100 7800
AR Path="/5266EDDE/539408DE/53335DC0" Ref="R116"  Part="1" 
AR Path="/52718D84/53946925/53335DC0" Ref="R116"  Part="1" 
F 0 "R116" V 8180 7800 40  0000 C CNN
F 1 "5.6k" V 8107 7801 40  0000 C CNN
F 2 "r_1608" V 8030 7800 30  0000 C CNN
F 3 "~" H 8100 7800 30  0000 C CNN
	1    8100 7800
	0    1    -1   0   
$EndComp
$Comp
L R_1608 R106
U 1 1 53335DCA
P 3650 6700
AR Path="/5266EDDE/539408DE/53335DCA" Ref="R106"  Part="1" 
AR Path="/52718D84/53946925/53335DCA" Ref="R106"  Part="1" 
F 0 "R106" V 3730 6700 40  0000 C CNN
F 1 "22k" V 3657 6701 40  0000 C CNN
F 2 "r_1608" V 3580 6700 30  0000 C CNN
F 3 "~" H 3650 6700 30  0000 C CNN
	1    3650 6700
	0    1    -1   0   
$EndComp
$Comp
L R_1608 R105
U 1 1 53335DD0
P 3650 6350
AR Path="/5266EDDE/539408DE/53335DD0" Ref="R105"  Part="1" 
AR Path="/52718D84/53946925/53335DD0" Ref="R105"  Part="1" 
F 0 "R105" V 3730 6350 40  0000 C CNN
F 1 "22k" V 3657 6351 40  0000 C CNN
F 2 "r_1608" V 3580 6350 30  0000 C CNN
F 3 "~" H 3650 6350 30  0000 C CNN
	1    3650 6350
	0    1    -1   0   
$EndComp
$Comp
L R_1608 R114
U 1 1 53335DE5
P 6450 7250
AR Path="/5266EDDE/539408DE/53335DE5" Ref="R114"  Part="1" 
AR Path="/52718D84/53946925/53335DE5" Ref="R114"  Part="1" 
F 0 "R114" V 6530 7250 40  0000 C CNN
F 1 "10k" V 6457 7251 40  0000 C CNN
F 2 "r_1608" V 6380 7250 30  0000 C CNN
F 3 "~" H 6450 7250 30  0000 C CNN
	1    6450 7250
	0    1    -1   0   
$EndComp
$Comp
L R_1608 R112
U 1 1 53335DEB
P 5750 7250
AR Path="/5266EDDE/539408DE/53335DEB" Ref="R112"  Part="1" 
AR Path="/52718D84/53946925/53335DEB" Ref="R112"  Part="1" 
F 0 "R112" V 5830 7250 40  0000 C CNN
F 1 "10k" V 5757 7251 40  0000 C CNN
F 2 "r_1608" V 5680 7250 30  0000 C CNN
F 3 "~" H 5750 7250 30  0000 C CNN
	1    5750 7250
	0    1    -1   0   
$EndComp
$Comp
L C_1608 C118
U 1 1 53335DFA
P 6850 8400
AR Path="/5266EDDE/539408DE/53335DFA" Ref="C118"  Part="1" 
AR Path="/52718D84/53946925/53335DFA" Ref="C118"  Part="1" 
F 0 "C118" H 6850 8575 40  0000 C CNN
F 1 "2.2u" H 6850 8525 40  0000 C CNN
F 2 "c_1608" H 6850 8275 30  0000 C CNN
F 3 "~" V 6850 8400 60  0000 C CNN
	1    6850 8400
	0    -1   -1   0   
$EndComp
$Comp
L R_1608 R108
U 1 1 53335E01
P 5500 7050
AR Path="/5266EDDE/539408DE/53335E01" Ref="R108"  Part="1" 
AR Path="/52718D84/53946925/53335E01" Ref="R108"  Part="1" 
F 0 "R108" V 5580 7050 40  0000 C CNN
F 1 "0" V 5507 7051 40  0000 C CNN
F 2 "r_1608" V 5430 7050 30  0000 C CNN
F 3 "~" H 5500 7050 30  0000 C CNN
	1    5500 7050
	0    1    -1   0   
$EndComp
$Comp
L C_1608 C108
U 1 1 53335E0C
P 3050 7050
AR Path="/5266EDDE/539408DE/53335E0C" Ref="C108"  Part="1" 
AR Path="/52718D84/53946925/53335E0C" Ref="C108"  Part="1" 
F 0 "C108" H 3050 7225 40  0000 C CNN
F 1 "2.2u" H 3050 7175 40  0000 C CNN
F 2 "c_1608" H 3050 6925 30  0000 C CNN
F 3 "~" V 3050 7050 60  0000 C CNN
	1    3050 7050
	1    0    0    -1  
$EndComp
$Comp
L AGND #PWR04
U 1 1 53335E14
P 4350 8900
AR Path="/5266EDDE/539408DE/53335E14" Ref="#PWR04"  Part="1" 
AR Path="/52718D84/53946925/53335E14" Ref="#PWR04"  Part="1" 
F 0 "#PWR04" H 4350 8900 40  0001 C CNN
F 1 "AGND" H 4350 8830 50  0000 C CNN
F 2 "" H 4350 8900 60  0000 C CNN
F 3 "" H 4350 8900 60  0000 C CNN
	1    4350 8900
	1    0    0    -1  
$EndComp
$Comp
L C_1608 C112
U 1 1 53335E1F
P 4350 8600
AR Path="/5266EDDE/539408DE/53335E1F" Ref="C112"  Part="1" 
AR Path="/52718D84/53946925/53335E1F" Ref="C112"  Part="1" 
F 0 "C112" H 4350 8775 40  0000 C CNN
F 1 "0.22u" H 4350 8725 40  0000 C CNN
F 2 "c_1608" H 4350 8475 30  0000 C CNN
F 3 "~" V 4350 8600 60  0000 C CNN
	1    4350 8600
	0    -1   -1   0   
$EndComp
$Comp
L R_1608 R102
U 1 1 53335E38
P 3500 7850
AR Path="/5266EDDE/539408DE/53335E38" Ref="R102"  Part="1" 
AR Path="/52718D84/53946925/53335E38" Ref="R102"  Part="1" 
F 0 "R102" V 3580 7850 40  0000 C CNN
F 1 "0" V 3507 7851 40  0000 C CNN
F 2 "r_1608" V 3430 7850 30  0000 C CNN
F 3 "~" H 3500 7850 30  0000 C CNN
	1    3500 7850
	1    0    0    1   
$EndComp
$Comp
L C_1608 C110
U 1 1 53335E3E
P 3050 8200
AR Path="/5266EDDE/539408DE/53335E3E" Ref="C110"  Part="1" 
AR Path="/52718D84/53946925/53335E3E" Ref="C110"  Part="1" 
F 0 "C110" H 3050 8375 40  0000 C CNN
F 1 "2.2u" H 3050 8325 40  0000 C CNN
F 2 "c_1608" H 3050 8075 30  0000 C CNN
F 3 "~" V 3050 8200 60  0000 C CNN
	1    3050 8200
	1    0    0    -1  
$EndComp
$Comp
L C_1608 C109
U 1 1 53335E44
P 3050 7500
AR Path="/5266EDDE/539408DE/53335E44" Ref="C109"  Part="1" 
AR Path="/52718D84/53946925/53335E44" Ref="C109"  Part="1" 
F 0 "C109" H 3050 7675 40  0000 C CNN
F 1 "2.2u" H 3050 7625 40  0000 C CNN
F 2 "c_1608" H 3050 7375 30  0000 C CNN
F 3 "~" V 3050 7500 60  0000 C CNN
	1    3050 7500
	1    0    0    -1  
$EndComp
$Comp
L SA571 U101
U 2 1 53335E4A
P 4600 7800
AR Path="/5266EDDE/539408DE/53335E4A" Ref="U101"  Part="2" 
AR Path="/52718D84/53946925/53335E4A" Ref="U101"  Part="2" 
F 0 "U101" H 5500 8100 70  0000 C CNN
F 1 "SA571" H 5500 8000 70  0000 C CNN
F 2 "SOIC-16" H 5350 7800 60  0000 C CNN
F 3 "http://www.onsemi.jp/pub/Collateral/SA571-D.PDF" H 5350 7800 60  0001 C CNN
	2    4600 7800
	1    0    0    -1  
$EndComp
$Comp
L SA571 U101
U 3 1 53335E55
P 8900 1900
AR Path="/5266EDDE/539408DE/53335E55" Ref="U101"  Part="3" 
AR Path="/52718D84/53946925/53335E55" Ref="U101"  Part="3" 
F 0 "U101" H 9200 1850 70  0000 C CNN
F 1 "SA571" H 9200 1700 70  0000 C CNN
F 2 "SOIC-16" H 9150 1550 60  0000 C CNN
F 3 "http://www.onsemi.jp/pub/Collateral/SA571-D.PDF" H 9650 1900 60  0001 C CNN
	3    8900 1900
	1    0    0    -1  
$EndComp
$Comp
L R_1608 R117
U 1 1 5334A0F8
P 9400 2800
AR Path="/5266EDDE/539408DE/5334A0F8" Ref="R117"  Part="1" 
AR Path="/52718D84/53946925/5334A0F8" Ref="R117"  Part="1" 
F 0 "R117" V 9480 2800 40  0000 C CNN
F 1 "0" V 9407 2801 40  0000 C CNN
F 2 "r_1608" V 9330 2800 30  0000 C CNN
F 3 "~" H 9400 2800 30  0000 C CNN
	1    9400 2800
	1    0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG05
U 1 1 5334AD2E
P 5300 4350
AR Path="/5266EDDE/539408DE/5334AD2E" Ref="#FLG05"  Part="1" 
AR Path="/52718D84/53946925/5334AD2E" Ref="#FLG05"  Part="1" 
F 0 "#FLG05" H 5300 4445 30  0001 C CNN
F 1 "PWR_FLAG" H 5300 4530 30  0000 C CNN
F 2 "" H 5300 4350 60  0000 C CNN
F 3 "" H 5300 4350 60  0000 C CNN
	1    5300 4350
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG06
U 1 1 5334ADE6
P 6950 850
AR Path="/5266EDDE/539408DE/5334ADE6" Ref="#FLG06"  Part="1" 
AR Path="/52718D84/53946925/5334ADE6" Ref="#FLG06"  Part="1" 
F 0 "#FLG06" H 6950 945 30  0001 C CNN
F 1 "PWR_FLAG" H 6950 1030 30  0000 C CNN
F 2 "" H 6950 850 60  0000 C CNN
F 3 "" H 6950 850 60  0000 C CNN
	1    6950 850 
	1    0    0    -1  
$EndComp
$Comp
L C_2012 C122
U 1 1 5334EDD7
P 8450 1950
AR Path="/5266EDDE/539408DE/5334EDD7" Ref="C122"  Part="1" 
AR Path="/52718D84/53946925/5334EDD7" Ref="C122"  Part="1" 
F 0 "C122" H 8450 2125 40  0000 C CNN
F 1 "10/16" H 8450 2075 40  0000 C CNN
F 2 "c_2012" H 8450 1825 30  0000 C CNN
F 3 "~" V 8450 1950 60  0000 C CNN
	1    8450 1950
	0    -1   -1   0   
$EndComp
$Comp
L 1SS355VMTE-17 D101
U 1 1 533757A9
P 9750 1350
AR Path="/5266EDDE/539408DE/533757A9" Ref="D101"  Part="1" 
AR Path="/52718D84/53946925/533757A9" Ref="D101"  Part="1" 
F 0 "D101" H 9750 1500 40  0000 C CNN
F 1 "1SS355VMTE-17" H 9750 1450 40  0000 C CNN
F 2 "UMD2" H 9750 1275 30  0000 C CNN
F 3 "~" H 9750 1350 60  0000 C CNN
	1    9750 1350
	0    1    1    0   
$EndComp
$Comp
L 1SS355VMTE-17 D102
U 1 1 533759A1
P 9750 1950
AR Path="/5266EDDE/539408DE/533759A1" Ref="D102"  Part="1" 
AR Path="/52718D84/53946925/533759A1" Ref="D102"  Part="1" 
F 0 "D102" H 9750 2100 40  0000 C CNN
F 1 "1SS355VMTE-17" H 9750 2050 40  0000 C CNN
F 2 "UMD2" H 9750 1875 30  0000 C CNN
F 3 "~" H 9750 1950 60  0000 C CNN
	1    9750 1950
	0    1    1    0   
$EndComp
$Comp
L 1SS355VMTE-17 D103
U 1 1 533759A7
P 9750 2550
AR Path="/5266EDDE/539408DE/533759A7" Ref="D103"  Part="1" 
AR Path="/52718D84/53946925/533759A7" Ref="D103"  Part="1" 
F 0 "D103" H 9750 2700 40  0000 C CNN
F 1 "1SS355VMTE-17" H 9750 2650 40  0000 C CNN
F 2 "UMD2" H 9750 2475 30  0000 C CNN
F 3 "~" H 9750 2550 60  0000 C CNN
	1    9750 2550
	0    1    1    0   
$EndComp
$Comp
L 1SS355VMTE-17 D104
U 1 1 533759AD
P 9750 3150
AR Path="/5266EDDE/539408DE/533759AD" Ref="D104"  Part="1" 
AR Path="/52718D84/53946925/533759AD" Ref="D104"  Part="1" 
F 0 "D104" H 9750 3300 40  0000 C CNN
F 1 "1SS355VMTE-17" H 9750 3250 40  0000 C CNN
F 2 "UMD2" H 9750 3075 30  0000 C CNN
F 3 "~" H 9750 3150 60  0000 C CNN
	1    9750 3150
	0    1    1    0   
$EndComp
$Comp
L 1SS355VMTE-17 D105
U 1 1 533759B3
P 9750 3750
AR Path="/5266EDDE/539408DE/533759B3" Ref="D105"  Part="1" 
AR Path="/52718D84/53946925/533759B3" Ref="D105"  Part="1" 
F 0 "D105" H 9750 3900 40  0000 C CNN
F 1 "1SS355VMTE-17" H 9750 3850 40  0000 C CNN
F 2 "UMD2" H 9750 3675 30  0000 C CNN
F 3 "~" H 9750 3750 60  0000 C CNN
	1    9750 3750
	0    1    1    0   
$EndComp
Text Label 1750 6350 0    60   ~ 0
VIN
Text Label 1750 2250 0    60   ~ 0
IN1_3
Text Label 1750 2450 0    60   ~ 0
IN1_1
Text Label 1750 2350 0    60   ~ 0
IN1_2
Text Label 1750 2150 0    60   ~ 0
GND
Text Label 1750 2050 0    60   ~ 0
OUT1
Text Label 1750 1950 0    60   ~ 0
VOUT
Text Label 1750 6650 0    60   ~ 0
IN2_3
Text Label 1750 6750 0    60   ~ 0
IN2_2
Text Label 1750 6850 0    60   ~ 0
IN2_1
Text Label 1750 6550 0    60   ~ 0
GND
Text Label 1750 6450 0    60   ~ 0
OUT2
$Comp
L C_1608 C121
U 1 1 53375EA6
P 8050 1950
AR Path="/5266EDDE/539408DE/53375EA6" Ref="C121"  Part="1" 
AR Path="/52718D84/53946925/53375EA6" Ref="C121"  Part="1" 
F 0 "C121" H 8050 2125 40  0000 C CNN
F 1 "0.1u" H 8050 2075 40  0000 C CNN
F 2 "c_1608" H 8050 1825 30  0000 C CNN
F 3 "~" V 8050 1950 60  0000 C CNN
	1    8050 1950
	0    -1   -1   0   
$EndComp
Text Notes 7850 3600 0    40   ~ 0
Comp: 5.6k\nExpnd: 0
Text Notes 9100 3100 1    40   ~ 0
VIN=12V: OPEN\nVIN= 9V: 0
Text Notes 7850 8000 0    40   ~ 0
Comp: 5.6k\nExpnd: 0
$Comp
L C_2012 C115
U 1 1 5337652E
P 6100 4100
AR Path="/5266EDDE/539408DE/5337652E" Ref="C115"  Part="1" 
AR Path="/52718D84/53946925/5337652E" Ref="C115"  Part="1" 
F 0 "C115" H 6100 4275 40  0000 C CNN
F 1 "10/16" H 6100 4225 40  0000 C CNN
F 2 "c_2012" H 6100 3975 30  0000 C CNN
F 3 "~" V 6100 4100 60  0000 C CNN
	1    6100 4100
	0    -1   -1   0   
$EndComp
$Comp
L C_2012 C116
U 1 1 533766F1
P 6100 8500
AR Path="/5266EDDE/539408DE/533766F1" Ref="C116"  Part="1" 
AR Path="/52718D84/53946925/533766F1" Ref="C116"  Part="1" 
F 0 "C116" H 6100 8675 40  0000 C CNN
F 1 "10/16" H 6100 8625 40  0000 C CNN
F 2 "c_2012" H 6100 8375 30  0000 C CNN
F 3 "~" V 6100 8500 60  0000 C CNN
	1    6100 8500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8500 3400 8350 3400
Wire Wire Line
	8500 4900 8500 3400
Wire Wire Line
	1850 4900 8500 4900
Wire Wire Line
	1850 2050 1850 4900
Wire Wire Line
	2700 2300 2850 2300
Wire Wire Line
	2600 1950 2850 1950
Wire Wire Line
	2000 2150 2000 8850
Connection ~ 5650 950 
Wire Wire Line
	1700 1950 2150 1950
Connection ~ 5650 2500
Connection ~ 6100 4450
Wire Wire Line
	7100 2500 7100 4450
Wire Wire Line
	5650 900  5650 1000
Wire Wire Line
	5650 1600 5650 1500
Connection ~ 5300 1850
Wire Wire Line
	4350 1850 4350 2950
Wire Wire Line
	5300 1850 5300 2000
Wire Wire Line
	4350 1850 5500 1850
Wire Wire Line
	5650 2500 5650 2100
Wire Wire Line
	5300 2500 7100 2500
Wire Wire Line
	5300 2500 5300 2400
Wire Wire Line
	7850 3400 7650 3400
Wire Wire Line
	5850 3400 7250 3400
Connection ~ 2550 2650
Wire Wire Line
	3250 1950 3400 1950
Wire Wire Line
	3400 2300 3250 2300
Connection ~ 5000 2300
Wire Wire Line
	3900 1950 5000 1950
Connection ~ 5000 2850
Wire Wire Line
	3900 2300 5000 2300
Connection ~ 6850 2850
Wire Wire Line
	6700 2850 6850 2850
Connection ~ 4350 4450
Wire Wire Line
	6100 4300 6100 4450
Wire Wire Line
	8900 4450 2000 4450
Wire Wire Line
	5000 1950 5000 2950
Wire Wire Line
	5000 2850 5500 2850
Connection ~ 6100 2850
Wire Wire Line
	6100 2850 6100 3900
Wire Wire Line
	6000 2850 6200 2850
Wire Wire Line
	3500 4700 6850 4700
Wire Wire Line
	6850 4700 6850 4200
Connection ~ 6850 3400
Wire Wire Line
	5750 2650 6850 2650
Wire Wire Line
	3250 2650 5250 2650
Wire Wire Line
	2550 2650 2850 2650
Connection ~ 4800 2650
Wire Wire Line
	4350 4400 4350 4500
Wire Wire Line
	4800 2650 4800 2950
Wire Wire Line
	6850 2650 6850 3800
Wire Wire Line
	4350 4000 4350 3850
Connection ~ 2750 3450
Wire Wire Line
	2550 3450 2750 3450
Wire Wire Line
	2750 3800 2850 3800
Wire Wire Line
	2750 3100 2750 3800
Wire Wire Line
	2850 3100 2750 3100
Connection ~ 3500 3800
Connection ~ 3500 3100
Wire Wire Line
	3250 3100 3650 3100
Wire Wire Line
	3650 3100 3650 3300
Wire Wire Line
	3650 3300 3750 3300
Wire Wire Line
	3500 3700 3500 4700
Wire Wire Line
	3250 3800 3650 3800
Wire Wire Line
	3650 3800 3650 3650
Wire Wire Line
	3650 3650 3750 3650
Wire Wire Line
	3500 3200 3500 3100
Wire Wire Line
	8500 7800 8350 7800
Wire Wire Line
	8500 9300 8500 7800
Wire Wire Line
	1850 9300 8500 9300
Wire Wire Line
	1850 6450 1850 9300
Wire Wire Line
	2550 6850 2550 7850
Wire Wire Line
	2700 6700 2850 6700
Wire Wire Line
	2600 6350 2850 6350
Connection ~ 5650 5350
Connection ~ 5650 6900
Connection ~ 6100 8850
Wire Wire Line
	7100 8850 7100 6900
Wire Wire Line
	5650 5300 5650 5400
Wire Wire Line
	5650 6000 5650 5900
Connection ~ 5300 6250
Wire Wire Line
	4350 6250 4350 7350
Wire Wire Line
	5300 6250 5300 6400
Wire Wire Line
	4350 6250 5500 6250
Wire Wire Line
	5650 6900 5650 6500
Wire Wire Line
	7100 6900 5300 6900
Wire Wire Line
	5300 6900 5300 6800
Wire Wire Line
	7850 7800 7650 7800
Wire Wire Line
	5850 7800 7250 7800
Connection ~ 2550 7050
Wire Wire Line
	3250 6350 3400 6350
Wire Wire Line
	3400 6700 3250 6700
Connection ~ 5000 6700
Wire Wire Line
	3900 6350 5000 6350
Connection ~ 5000 7250
Wire Wire Line
	3900 6700 5000 6700
Connection ~ 6850 7250
Wire Wire Line
	6700 7250 6850 7250
Connection ~ 4350 8850
Wire Wire Line
	6100 8850 6100 8700
Wire Wire Line
	2000 8850 7100 8850
Wire Wire Line
	5000 6350 5000 7350
Wire Wire Line
	5000 7250 5500 7250
Connection ~ 6100 7250
Wire Wire Line
	6100 7250 6100 8300
Wire Wire Line
	6000 7250 6200 7250
Wire Wire Line
	3500 9100 6850 9100
Wire Wire Line
	6850 9100 6850 8600
Connection ~ 6850 7800
Wire Wire Line
	5750 7050 6850 7050
Wire Wire Line
	3250 7050 5250 7050
Wire Wire Line
	2550 7050 2850 7050
Connection ~ 4800 7050
Wire Wire Line
	4350 8800 4350 8900
Wire Wire Line
	4800 7050 4800 7350
Wire Wire Line
	6850 7050 6850 8200
Wire Wire Line
	4350 8400 4350 8250
Connection ~ 2750 7850
Wire Wire Line
	2550 7850 2750 7850
Wire Wire Line
	2750 8200 2850 8200
Wire Wire Line
	2750 7500 2750 8200
Wire Wire Line
	2850 7500 2750 7500
Connection ~ 3500 8200
Connection ~ 3500 7500
Wire Wire Line
	3250 7500 3650 7500
Wire Wire Line
	3650 7500 3650 7700
Wire Wire Line
	3650 7700 3750 7700
Wire Wire Line
	3500 8100 3500 9100
Wire Wire Line
	3250 8200 3650 8200
Wire Wire Line
	3650 8200 3650 8050
Wire Wire Line
	3650 8050 3750 8050
Wire Wire Line
	3500 7600 3500 7500
Wire Wire Line
	8900 950  8900 1350
Wire Wire Line
	8900 2450 8900 4450
Connection ~ 7100 4450
Connection ~ 2000 4450
Connection ~ 8900 950 
Wire Wire Line
	9400 950  9400 2550
Connection ~ 9400 950 
Wire Wire Line
	9400 3050 9400 5100
Wire Wire Line
	9400 4450 9750 4450
Wire Wire Line
	9400 5100 2150 5100
Connection ~ 9400 4450
Wire Wire Line
	5300 4350 5300 4450
Connection ~ 5300 4450
Wire Wire Line
	6950 950  6950 850 
Connection ~ 6950 950 
Wire Wire Line
	8450 1750 8450 1250
Wire Wire Line
	8050 1250 8900 1250
Connection ~ 8900 1250
Wire Wire Line
	8450 2150 8450 2550
Wire Wire Line
	8050 2550 8900 2550
Connection ~ 8900 2550
Wire Wire Line
	9750 950  9750 1150
Wire Wire Line
	9750 1750 9750 1550
Wire Wire Line
	9750 2350 9750 2150
Wire Wire Line
	9750 2950 9750 2750
Wire Wire Line
	9750 3550 9750 3350
Wire Wire Line
	9750 4450 9750 3950
Wire Wire Line
	8050 1750 8050 1250
Connection ~ 8450 1250
Wire Wire Line
	8050 2150 8050 2550
Connection ~ 8450 2550
Wire Notes Line
	9200 2450 9200 3150
Wire Notes Line
	9200 3150 9600 3150
Wire Notes Line
	9600 3150 9600 2450
Wire Notes Line
	9600 2450 9200 2450
Wire Notes Line
	7800 3250 7800 3500
Wire Notes Line
	7800 3500 8400 3500
Wire Notes Line
	8400 3500 8400 3250
Wire Notes Line
	8400 3250 7800 3250
Wire Notes Line
	7800 7650 7800 7900
Wire Notes Line
	7800 7900 8400 7900
Wire Notes Line
	7800 7650 8400 7650
Wire Notes Line
	8400 7650 8400 7900
Wire Wire Line
	2150 5100 2150 1950
Wire Wire Line
	2300 6350 1700 6350
Wire Wire Line
	2300 950  2300 6350
Connection ~ 2300 5350
Wire Wire Line
	2300 5350 5650 5350
Wire Wire Line
	2300 950  9750 950 
Wire Wire Line
	1850 2050 1700 2050
Wire Wire Line
	2550 2450 2550 3450
Wire Wire Line
	2000 2150 1700 2150
Wire Wire Line
	2550 2450 1700 2450
Wire Wire Line
	2700 2300 2700 2350
Wire Wire Line
	2700 2350 1700 2350
Wire Wire Line
	2600 1950 2600 2250
Wire Wire Line
	2600 2250 1700 2250
Wire Wire Line
	1700 6550 2000 6550
Connection ~ 2000 6550
Wire Wire Line
	1700 6450 1850 6450
Wire Wire Line
	1700 6850 2550 6850
Wire Wire Line
	2700 6700 2700 6750
Wire Wire Line
	2700 6750 1700 6750
Wire Wire Line
	1700 6650 2600 6650
Wire Wire Line
	2600 6650 2600 6350
Text Notes 2700 4050 0    40   ~ 0
Comp: C104,C105=OPEN / R101=0ohm\nExpand: C104,C105=2.2u / R101=OPEN
Text Notes 2700 8500 0    40   ~ 0
Comp: C109,C110=OPEN / R102=0ohm\nExpand: C109,C110=2.2u / R102=OPEN
Text Notes 3150 6050 0    40   ~ 0
Comp: C106,C107,C108=2.2u / R105,R106=22k\nExpand: C106,C107,C108=OPEN / R105,R106=OPEN
Text Notes 2950 1600 0    40   ~ 0
Comp: C101,C102,C103=2.2u / R103,R104=22k\nExpand: C101,C102,C103=OPEN / R103,R104=OPEN
Text Notes 5900 2350 0    40   ~ 0
Comp: R107=OPEN / R111,R113=10k\nExpand: R107=0ohm / R111,R113=OPEN
Text Notes 5850 6800 0    40   ~ 0
Comp: R108=OPEN / R112,R114=10k\nExpand: R108=0ohm / R112,R114=OPEN
Text Notes 4950 8700 0    40   ~ 0
Comp: C116=10u/16V / C118=2.2u\nExpand: C116=OPEN / C118=OPEN
Text Notes 4900 3950 0    40   ~ 0
Comp: C115=10u/16V / C117=2.2u\nExpand: C115=OPEN / C117=OPEN
Text HLabel 1700 1950 0    60   Output ~ 0
VOUT
Text HLabel 1700 2050 0    60   Output ~ 0
OUT1
Text HLabel 1700 2150 0    60   UnSpc ~ 0
GND
Text HLabel 1700 2250 0    60   Input ~ 0
IN1_3
Text HLabel 1700 2350 0    60   Input ~ 0
IN1_2
Text HLabel 1700 2450 0    60   Input ~ 0
IN1_1
Text HLabel 1700 6350 0    60   Input ~ 0
VIN
Text HLabel 1700 6450 0    60   Output ~ 0
OUT2
Text HLabel 1700 6550 0    60   UnSpc ~ 0
GND
Text HLabel 1700 6650 0    60   Input ~ 0
IN2_3
Text HLabel 1700 6750 0    60   Input ~ 0
IN2_2
Text HLabel 1700 6850 0    60   Input ~ 0
IN2_1
$EndSCHEMATC
