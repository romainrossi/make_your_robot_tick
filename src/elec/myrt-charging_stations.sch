EESchema Schematic File Version 2
LIBS:myrt-rescue
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
LIBS:myrt-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_2 P7
U 1 1 591C7A4B
P 4800 3600
F 0 "P7" V 4750 3600 40  0000 C CNN
F 1 "CONN_2" V 4850 3600 40  0000 C CNN
F 2 "Connect:bornier2" H 4800 3600 60  0001 C CNN
F 3 "" H 4800 3600 60  0001 C CNN
	1    4800 3600
	-1   0    0    -1  
$EndComp
$Comp
L LED-RESCUE-myrt D4
U 1 1 591C7B22
P 5850 3250
F 0 "D4" H 5850 3350 50  0000 C CNN
F 1 "Yellow" H 5850 3150 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 5850 3250 50  0001 C CNN
F 3 "" H 5850 3250 50  0001 C CNN
	1    5850 3250
	-1   0    0    1   
$EndComp
$Comp
L CONN_2 P10
U 1 1 591C7D17
P 7000 3600
F 0 "P10" V 6950 3600 40  0000 C CNN
F 1 "CONN_2" V 7050 3600 40  0000 C CNN
F 2 "Connect:bornier2" H 7000 3600 60  0001 C CNN
F 3 "" H 7000 3600 60  0001 C CNN
	1    7000 3600
	1    0    0    -1  
$EndComp
Text Notes 3300 2500 0    60   ~ 0
Power input\nFrom energy storage station
Wire Wire Line
	5150 3700 6650 3700
Wire Wire Line
	5150 3500 6200 3500
Text Label 5500 3700 0    60   ~ 0
POWER_GND
Text Label 5500 3500 0    60   ~ 0
POWER_5V
$Comp
L R R7
U 1 1 591C8302
P 6350 3500
F 0 "R7" V 6430 3500 50  0000 C CNN
F 1 "100Ω 0.5W" V 6250 3500 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM30mm" V 6280 3500 50  0001 C CNN
F 3 "" H 6350 3500 50  0001 C CNN
	1    6350 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	6650 3500 6500 3500
$Comp
L R R4
U 1 1 591C834C
P 5500 3250
F 0 "R4" V 5580 3250 50  0000 C CNN
F 1 "470" V 5500 3250 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 5430 3250 50  0001 C CNN
F 3 "" H 5500 3250 50  0001 C CNN
	1    5500 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	6000 3250 6100 3250
Wire Wire Line
	6100 3250 6100 3700
Connection ~ 6100 3700
Wire Wire Line
	5350 3250 5300 3250
Wire Wire Line
	5300 3250 5300 3500
Connection ~ 5300 3500
Wire Wire Line
	5650 3250 5700 3250
Text Notes 5300 1900 0    60   ~ 0
Status display led
$Comp
L CONN_2 P6
U 1 1 591C929A
P 4800 2450
F 0 "P6" V 4750 2450 40  0000 C CNN
F 1 "CONN_2" V 4850 2450 40  0000 C CNN
F 2 "Connect:bornier2" H 4800 2450 60  0001 C CNN
F 3 "" H 4800 2450 60  0001 C CNN
	1    4800 2450
	-1   0    0    -1  
$EndComp
$Comp
L LED-RESCUE-myrt D3
U 1 1 591C92A0
P 5850 2100
F 0 "D3" H 5850 2200 50  0000 C CNN
F 1 "Green" H 5850 2000 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 5850 2100 50  0001 C CNN
F 3 "" H 5850 2100 50  0001 C CNN
	1    5850 2100
	-1   0    0    1   
$EndComp
$Comp
L CONN_2 P9
U 1 1 591C92A6
P 7000 2450
F 0 "P9" V 6950 2450 40  0000 C CNN
F 1 "CONN_2" V 7050 2450 40  0000 C CNN
F 2 "Connect:bornier2" H 7000 2450 60  0001 C CNN
F 3 "" H 7000 2450 60  0001 C CNN
	1    7000 2450
	1    0    0    -1  
$EndComp
Text Notes 7150 2550 0    60   ~ 0
Power output\nTo copper contacts
Wire Wire Line
	5150 2550 6650 2550
Wire Wire Line
	5150 2350 6200 2350
Text Label 5500 2550 0    60   ~ 0
POWER_GND
Text Label 5500 2350 0    60   ~ 0
POWER_5V
$Comp
L R R6
U 1 1 591C92B3
P 6350 2350
F 0 "R6" V 6430 2350 50  0000 C CNN
F 1 "500Ω 0.1W" V 6250 2350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 6280 2350 50  0001 C CNN
F 3 "" H 6350 2350 50  0001 C CNN
	1    6350 2350
	0    1    1    0   
$EndComp
Wire Wire Line
	6650 2350 6500 2350
$Comp
L R R3
U 1 1 591C92BA
P 5500 2100
F 0 "R3" V 5580 2100 50  0000 C CNN
F 1 "470" V 5500 2100 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 5430 2100 50  0001 C CNN
F 3 "" H 5500 2100 50  0001 C CNN
	1    5500 2100
	0    1    1    0   
$EndComp
Wire Wire Line
	6000 2100 6100 2100
Wire Wire Line
	6100 2100 6100 2550
Connection ~ 6100 2550
Wire Wire Line
	5350 2100 5300 2100
Wire Wire Line
	5300 2100 5300 2350
Connection ~ 5300 2350
Wire Wire Line
	5650 2100 5700 2100
Text Notes 4900 2750 0    60   ~ 0
Slow charging station (10mA, Two stations)
Text Notes 5150 3850 0    60   ~ 0
Medium charging station (50mA)
$Comp
L CONN_2 P8
U 1 1 591C9885
P 4800 4700
F 0 "P8" V 4750 4700 40  0000 C CNN
F 1 "CONN_2" V 4850 4700 40  0000 C CNN
F 2 "Connect:bornier2" H 4800 4700 60  0001 C CNN
F 3 "" H 4800 4700 60  0001 C CNN
	1    4800 4700
	-1   0    0    -1  
$EndComp
$Comp
L LED-RESCUE-myrt D5
U 1 1 591C988B
P 5850 4350
F 0 "D5" H 5850 4450 50  0000 C CNN
F 1 "Red" H 5850 4250 50  0000 C CNN
F 2 "LEDs:LED-5MM" H 5850 4350 50  0001 C CNN
F 3 "" H 5850 4350 50  0001 C CNN
	1    5850 4350
	-1   0    0    1   
$EndComp
$Comp
L CONN_2 P11
U 1 1 591C9891
P 7000 4700
F 0 "P11" V 6950 4700 40  0000 C CNN
F 1 "CONN_2" V 7050 4700 40  0000 C CNN
F 2 "Connect:bornier2" H 7000 4700 60  0001 C CNN
F 3 "" H 7000 4700 60  0001 C CNN
	1    7000 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 4800 6650 4800
Wire Wire Line
	5150 4600 6200 4600
Text Label 5500 4800 0    60   ~ 0
POWER_GND
Text Label 5500 4600 0    60   ~ 0
POWER_5V
$Comp
L R R8
U 1 1 591C989D
P 6350 4600
F 0 "R8" V 6430 4600 50  0000 C CNN
F 1 "33Ω 1.5W" V 6250 4600 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM30mm" V 6280 4600 50  0001 C CNN
F 3 "" H 6350 4600 50  0001 C CNN
	1    6350 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	6650 4600 6500 4600
$Comp
L R R5
U 1 1 591C98A4
P 5500 4350
F 0 "R5" V 5580 4350 50  0000 C CNN
F 1 "470" V 5500 4350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 5430 4350 50  0001 C CNN
F 3 "" H 5500 4350 50  0001 C CNN
	1    5500 4350
	0    1    1    0   
$EndComp
Wire Wire Line
	6000 4350 6100 4350
Wire Wire Line
	6100 4350 6100 4800
Connection ~ 6100 4800
Wire Wire Line
	5350 4350 5300 4350
Wire Wire Line
	5300 4350 5300 4600
Connection ~ 5300 4600
Wire Wire Line
	5650 4350 5700 4350
Text Notes 5200 4950 0    60   ~ 0
Fast charging station (150mA)
$EndSCHEMATC
