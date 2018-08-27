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
Sheet 1 2
Title "MakeYourRobotTick - Energy storage & Counter"
Date "2017-05-15"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L arduino-nano-ATmega328 U2
U 1 1 5919B504
P 5650 3500
F 0 "U2" H 5650 3500 60  0000 C CNN
F 1 "arduino-nano-ATmega328" H 5650 2500 60  0000 C CNN
F 2 "Housings_DIP:DIP-32_W15.24mm_LongPads" H 5650 3500 60  0001 C CNN
F 3 "" H 5650 3500 60  0001 C CNN
	1    5650 3500
	1    0    0    -1  
$EndComp
$Comp
L ULN2803A U1
U 1 1 5919B618
P 3400 3550
F 0 "U1" H 3400 4075 50  0000 C CNN
F 1 "ULN2803A" H 3400 4000 50  0000 C CNN
F 2 "Housings_DIP:DIP-18_W7.62mm_LongPads" H 3450 2900 50  0001 L CNN
F 3 "" H 3500 3450 50  0001 C CNN
	1    3400 3550
	1    0    0    -1  
$EndComp
$Comp
L USB_OTG-RESCUE-myrt J6
U 1 1 5919B705
P 5400 1400
F 0 "J6" H 5200 1850 50  0000 L CNN
F 1 "USB_OTG" H 5200 1750 50  0000 L CNN
F 2 "Connect:USB_Micro-B" H 5550 1350 50  0001 C CNN
F 3 "" H 5550 1350 50  0001 C CNN
	1    5400 1400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 5919B7CE
P 6050 2000
F 0 "#PWR01" H 6050 1750 50  0001 C CNN
F 1 "GND" H 6050 1850 50  0000 C CNN
F 2 "" H 6050 2000 50  0001 C CNN
F 3 "" H 6050 2000 50  0001 C CNN
	1    6050 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 1800 5300 1900
Wire Wire Line
	5400 1900 5400 1800
Connection ~ 5400 1900
$Comp
L CP C2
U 1 1 5919B920
P 6050 1500
F 0 "C2" H 6075 1600 50  0000 L CNN
F 1 "CP" H 6075 1400 50  0000 L CNN
F 2 "Capacitors_ThroughHole:C_Radial_D10_L25_P5" H 6088 1350 50  0001 C CNN
F 3 "" H 6050 1500 50  0001 C CNN
	1    6050 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 1650 6050 2000
Connection ~ 6050 1900
Connection ~ 6050 1200
Wire Wire Line
	6350 3150 7250 3150
NoConn ~ 6350 2850
NoConn ~ 6350 4050
NoConn ~ 6350 4150
$Comp
L GND #PWR02
U 1 1 5919BEA1
P 4600 3050
F 0 "#PWR02" H 4600 2800 50  0001 C CNN
F 1 "GND" H 4600 2900 50  0000 C CNN
F 2 "" H 4600 3050 50  0001 C CNN
F 3 "" H 4600 3050 50  0001 C CNN
	1    4600 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 1100 6050 1350
Wire Wire Line
	5300 1900 6050 1900
$Comp
L +5V #PWR03
U 1 1 5919C28E
P 6050 1100
F 0 "#PWR03" H 6050 950 50  0001 C CNN
F 1 "+5V" H 6050 1240 50  0000 C CNN
F 2 "" H 6050 1100 50  0001 C CNN
F 3 "" H 6050 1100 50  0001 C CNN
	1    6050 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 1200 5700 1200
$Comp
L +5V #PWR04
U 1 1 5919C40B
P 7250 3150
F 0 "#PWR04" H 7250 3000 50  0001 C CNN
F 1 "+5V" H 7250 3290 50  0000 C CNN
F 2 "" H 7250 3150 50  0001 C CNN
F 3 "" H 7250 3150 50  0001 C CNN
	1    7250 3150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 5919C427
P 6800 2750
F 0 "#PWR05" H 6800 2500 50  0001 C CNN
F 1 "GND" H 6800 2600 50  0000 C CNN
F 2 "" H 6800 2750 50  0001 C CNN
F 3 "" H 6800 2750 50  0001 C CNN
	1    6800 2750
	1    0    0    -1  
$EndComp
Text Notes 5050 850  0    60   ~ 0
Main power supply\nSmartphone charger\n5V/2A output on micro-usb
$Comp
L USB_A-RESCUE-myrt J1
U 1 1 5919C9BE
P 1400 2250
F 0 "J1" H 1200 2700 50  0000 L CNN
F 1 "USB_A" H 1200 2600 50  0000 L CNN
F 2 "Connect:USB_A" H 1550 2200 50  0001 C CNN
F 3 "" H 1550 2200 50  0001 C CNN
	1    1400 2250
	1    0    0    -1  
$EndComp
$Comp
L USB_A-RESCUE-myrt J2
U 1 1 5919CA73
P 1400 3350
F 0 "J2" H 1200 3800 50  0000 L CNN
F 1 "USB_A" H 1200 3700 50  0000 L CNN
F 2 "Connect:USB_A" H 1550 3300 50  0001 C CNN
F 3 "" H 1550 3300 50  0001 C CNN
	1    1400 3350
	1    0    0    -1  
$EndComp
$Comp
L USB_A-RESCUE-myrt J3
U 1 1 5919CAC7
P 1400 4400
F 0 "J3" H 1200 4850 50  0000 L CNN
F 1 "USB_A" H 1200 4750 50  0000 L CNN
F 2 "Connect:USB_A" H 1550 4350 50  0001 C CNN
F 3 "" H 1550 4350 50  0001 C CNN
	1    1400 4400
	1    0    0    -1  
$EndComp
$Comp
L USB_A-RESCUE-myrt J4
U 1 1 5919CB14
P 1400 5500
F 0 "J4" H 1200 5950 50  0000 L CNN
F 1 "USB_A" H 1200 5850 50  0000 L CNN
F 2 "Connect:USB_A" H 1550 5450 50  0001 C CNN
F 3 "" H 1550 5450 50  0001 C CNN
	1    1400 5500
	1    0    0    -1  
$EndComp
Text Notes 600  1550 0    60   ~ 0
Connectors for charging stations
Text Notes 800  2250 0    60   ~ 0
Slow\n10mA
Text Notes 800  3350 0    60   ~ 0
Slow\n10mA
Text Notes 800  4400 0    60   ~ 0
Medium\n50mA
Text Notes 800  5500 0    60   ~ 0
Fast\n150mA
Wire Wire Line
	1300 2650 1300 2700
Wire Wire Line
	1300 2700 2750 2700
Wire Wire Line
	1400 2700 1400 2650
Wire Wire Line
	2750 2700 2750 3250
Wire Wire Line
	2750 3250 3000 3250
Connection ~ 1400 2700
Wire Wire Line
	1300 3750 1300 3800
Wire Wire Line
	1300 3800 2400 3800
Wire Wire Line
	1400 3800 1400 3750
Wire Wire Line
	2400 3800 2400 3350
Wire Wire Line
	2400 3350 3000 3350
Connection ~ 1400 3800
Wire Wire Line
	1300 4800 1300 4850
Wire Wire Line
	1300 4850 2500 4850
Wire Wire Line
	1400 4850 1400 4800
Wire Wire Line
	2500 4850 2500 3450
Wire Wire Line
	2500 3450 3000 3450
Connection ~ 1400 4850
Wire Wire Line
	1300 5900 1300 5950
Wire Wire Line
	1300 5950 2600 5950
Wire Wire Line
	1400 5950 1400 5900
Wire Wire Line
	2600 5950 2600 3550
Wire Wire Line
	2600 3550 3000 3550
Connection ~ 1400 5950
$Comp
L +5V #PWR06
U 1 1 5919D8B8
P 1850 2000
F 0 "#PWR06" H 1850 1850 50  0001 C CNN
F 1 "+5V" H 1850 2140 50  0000 C CNN
F 2 "" H 1850 2000 50  0001 C CNN
F 3 "" H 1850 2000 50  0001 C CNN
	1    1850 2000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR07
U 1 1 5919D8FB
P 1850 3100
F 0 "#PWR07" H 1850 2950 50  0001 C CNN
F 1 "+5V" H 1850 3240 50  0000 C CNN
F 2 "" H 1850 3100 50  0001 C CNN
F 3 "" H 1850 3100 50  0001 C CNN
	1    1850 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 2050 1850 2050
Wire Wire Line
	1850 2050 1850 2000
Wire Wire Line
	1700 3150 1850 3150
Wire Wire Line
	1850 3150 1850 3100
$Comp
L +5V #PWR08
U 1 1 5919DAC0
P 1850 4150
F 0 "#PWR08" H 1850 4000 50  0001 C CNN
F 1 "+5V" H 1850 4290 50  0000 C CNN
F 2 "" H 1850 4150 50  0001 C CNN
F 3 "" H 1850 4150 50  0001 C CNN
	1    1850 4150
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR09
U 1 1 5919DAE3
P 1850 5250
F 0 "#PWR09" H 1850 5100 50  0001 C CNN
F 1 "+5V" H 1850 5390 50  0000 C CNN
F 2 "" H 1850 5250 50  0001 C CNN
F 3 "" H 1850 5250 50  0001 C CNN
	1    1850 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 5300 1850 5300
Wire Wire Line
	1850 5300 1850 5250
Wire Wire Line
	1700 4200 1850 4200
Wire Wire Line
	1850 4200 1850 4150
$Comp
L +5V #PWR010
U 1 1 5919DC86
P 3950 4050
F 0 "#PWR010" H 3950 3900 50  0001 C CNN
F 1 "+5V" H 3950 4190 50  0000 C CNN
F 2 "" H 3950 4050 50  0001 C CNN
F 3 "" H 3950 4050 50  0001 C CNN
	1    3950 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 4050 3950 4050
$Comp
L GND #PWR011
U 1 1 5919DCF0
P 3400 4300
F 0 "#PWR011" H 3400 4050 50  0001 C CNN
F 1 "GND" H 3400 4150 50  0000 C CNN
F 2 "" H 3400 4300 50  0001 C CNN
F 3 "" H 3400 4300 50  0001 C CNN
	1    3400 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 4300 3400 4250
Wire Wire Line
	3800 3250 4950 3250
Wire Wire Line
	4950 3350 3800 3350
Wire Wire Line
	3800 3450 4950 3450
Wire Wire Line
	4950 3550 3800 3550
Wire Notes Line
	550  1550 550  7250
Wire Notes Line
	550  7250 2300 7250
Wire Notes Line
	2300 7250 2300 1550
Wire Notes Line
	2300 1550 550  1550
$Comp
L CONN_2 P3
U 1 1 5919E287
P 9250 1200
F 0 "P3" V 9200 1200 40  0000 C CNN
F 1 "RESET_BUT_CON" V 9400 1200 40  0000 C CNN
F 2 "Wire_Connections_Bridges:WireConnection_0.80mmDrill" H 9250 1200 60  0001 C CNN
F 3 "" H 9250 1200 60  0001 C CNN
	1    9250 1200
	0    -1   -1   0   
$EndComp
$Comp
L CONN_2 P5
U 1 1 5919E43D
P 10500 1200
F 0 "P5" V 10450 1200 40  0000 C CNN
F 1 "START/STOP_BUT_CONN" V 10650 1200 40  0000 C CNN
F 2 "Connect:bornier2" H 10500 1200 60  0001 C CNN
F 3 "" H 10500 1200 60  0001 C CNN
	1    10500 1200
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR012
U 1 1 5919E487
P 9350 1600
F 0 "#PWR012" H 9350 1350 50  0001 C CNN
F 1 "GND" H 9350 1450 50  0000 C CNN
F 2 "" H 9350 1600 50  0001 C CNN
F 3 "" H 9350 1600 50  0001 C CNN
	1    9350 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 5919E4B0
P 10600 1600
F 0 "#PWR013" H 10600 1350 50  0001 C CNN
F 1 "GND" H 10600 1450 50  0000 C CNN
F 2 "" H 10600 1600 50  0001 C CNN
F 3 "" H 10600 1600 50  0001 C CNN
	1    10600 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10600 1600 10600 1550
Wire Wire Line
	9350 1600 9350 1550
Wire Wire Line
	6350 3050 6600 3050
Text GLabel 6600 3050 2    60   Input ~ 0
RESET_BUT
Text GLabel 9100 1650 0    60   Input ~ 0
RESET_BUT
Wire Wire Line
	9100 1650 9150 1650
Wire Wire Line
	9150 1650 9150 1550
Text Notes 9600 900  0    60   ~ 0
HMI
Wire Notes Line
	10950 4200 8450 4200
Wire Notes Line
	8450 4200 8450 950 
Wire Notes Line
	8450 950  10950 950 
Wire Notes Line
	10950 950  10950 4200
Text GLabel 10350 1650 0    60   Input ~ 0
START/STOP_BUT
Wire Wire Line
	10350 1650 10400 1650
Wire Wire Line
	10400 1650 10400 1550
NoConn ~ 4950 2850
NoConn ~ 4950 2950
Wire Wire Line
	4950 3150 4750 3150
Wire Wire Line
	4750 3150 4750 3000
Wire Wire Line
	4750 3000 4600 3000
Wire Wire Line
	4600 3000 4600 3050
Wire Notes Line
	6350 2200 5050 2200
Wire Notes Line
	5050 2200 5050 900 
Wire Notes Line
	5050 900  6350 900 
Wire Notes Line
	6350 900  6350 2200
$Comp
L CONN_4 P4
U 1 1 591C4386
P 9650 2400
F 0 "P4" V 9600 2400 50  0000 C CNN
F 1 "HMI_Led_Strip" V 9800 2400 50  0000 C CNN
F 2 "Sockets_MOLEX_KK-System:Socket_MOLEX-KK-RM2-54mm_Lock_4pin_straight" H 9650 2400 60  0001 C CNN
F 3 "" H 9650 2400 60  0001 C CNN
	1    9650 2400
	1    0    0    -1  
$EndComp
Text Notes 8750 1950 0    60   ~ 0
HMI LED strip (WS2812b or APA102)
$Comp
L +5V #PWR014
U 1 1 591C4743
P 9200 2150
F 0 "#PWR014" H 9200 2000 50  0001 C CNN
F 1 "+5V" H 9200 2290 50  0000 C CNN
F 2 "" H 9200 2150 50  0001 C CNN
F 3 "" H 9200 2150 50  0001 C CNN
	1    9200 2150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 591C476F
P 9200 2650
F 0 "#PWR015" H 9200 2400 50  0001 C CNN
F 1 "GND" H 9200 2500 50  0000 C CNN
F 2 "" H 9200 2650 50  0001 C CNN
F 3 "" H 9200 2650 50  0001 C CNN
	1    9200 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 2650 9200 2550
Wire Wire Line
	9200 2550 9300 2550
Wire Wire Line
	9300 2250 9200 2250
Wire Wire Line
	9200 2250 9200 2150
Text GLabel 9100 2350 0    60   Input ~ 0
LEDS_MOSI
Text GLabel 9100 2500 0    60   Input ~ 0
LEDS_SCK
Wire Wire Line
	9100 2350 9300 2350
Wire Wire Line
	9100 2500 9200 2500
Wire Wire Line
	9200 2500 9200 2450
Wire Wire Line
	9200 2450 9300 2450
Text GLabel 4750 4150 0    60   Input ~ 0
LEDS_MOSI
Text GLabel 6600 4250 2    60   Input ~ 0
LEDS_SCK
Wire Wire Line
	4750 4150 4950 4150
Wire Wire Line
	6600 4250 6350 4250
Text Notes 5300 2500 0    60   ~ 0
Main processor
Wire Notes Line
	4800 2550 6550 2550
Wire Notes Line
	6550 2550 6550 4650
Wire Notes Line
	6550 4650 4800 4650
Wire Notes Line
	4800 4650 4800 2550
Wire Wire Line
	6350 2950 6600 2950
Wire Wire Line
	6600 2950 6600 2700
Wire Wire Line
	6600 2700 6800 2700
Wire Wire Line
	6800 2700 6800 2750
Text Notes 3100 2850 0    60   ~ 0
Power driver
Wire Notes Line
	2850 2900 4050 2900
Wire Notes Line
	4050 2900 4050 4500
Wire Notes Line
	4050 4500 2850 4500
Wire Notes Line
	2850 4500 2850 2900
Wire Wire Line
	4950 3050 4850 3050
Wire Wire Line
	4850 3050 4850 2600
Wire Wire Line
	4850 2600 6450 2600
Wire Wire Line
	6450 2600 6450 3050
Connection ~ 6450 3050
$Comp
L +5V #PWR016
U 1 1 591C6C13
P 1950 6350
F 0 "#PWR016" H 1950 6200 50  0001 C CNN
F 1 "+5V" H 1950 6490 50  0000 C CNN
F 2 "" H 1950 6350 50  0001 C CNN
F 3 "" H 1950 6350 50  0001 C CNN
	1    1950 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 6450 1950 6450
Wire Wire Line
	1950 6450 1950 6350
$Comp
L USB_A-RESCUE-myrt J5
U 1 1 591C711E
P 1400 6650
F 0 "J5" H 1200 7100 50  0000 L CNN
F 1 "USB_A" H 1200 7000 50  0000 L CNN
F 2 "Connect:USB_A" H 1550 6600 50  0001 C CNN
F 3 "" H 1550 6600 50  0001 C CNN
	1    1400 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 7050 1300 7100
Wire Wire Line
	1300 7100 2700 7100
Wire Wire Line
	1400 7100 1400 7050
Connection ~ 1400 7100
Text Notes 650  6650 0    60   ~ 0
Fast source\nLed strip
$Sheet
S 550  7400 1550 250 
U 591C79D6
F0 "Charging stations" 60
F1 "myrt-charging_stations.sch" 60
$EndSheet
$Comp
L CONN_2 P2
U 1 1 5B45EDBF
P 4250 5650
F 0 "P2" V 4200 5650 40  0000 C CNN
F 1 "CONN_2" V 4300 5650 40  0000 C CNN
F 2 "Wire_Connections_Bridges:WireConnection_0.80mmDrill" H 4250 5650 60  0001 C CNN
F 3 "" H 4250 5650 60  0001 C CNN
	1    4250 5650
	-1   0    0    1   
$EndComp
Text Notes 3600 6100 0    60   ~ 0
To station contacts pads\nfor Energy harvesting measurement
$Comp
L GND #PWR017
U 1 1 5B45F06F
P 5500 5950
F 0 "#PWR017" H 5500 5700 50  0001 C CNN
F 1 "GND" H 5500 5800 50  0000 C CNN
F 2 "" H 5500 5950 50  0001 C CNN
F 3 "" H 5500 5950 50  0001 C CNN
	1    5500 5950
	1    0    0    -1  
$EndComp
$Comp
L SIL10 J7
U 1 1 5B45F375
P 9400 3600
F 0 "J7" H 9400 4200 50  0000 C CNN
F 1 "SIL10" V 9420 3600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x10" H 9400 3600 50  0001 C CNN
F 3 "" H 9400 3600 50  0000 C CNN
	1    9400 3600
	1    0    0    -1  
$EndComp
Text Notes 9700 3600 0    60   ~ 0
Sparkfun\nSerial 7-Segment Display
Wire Wire Line
	4950 4050 4150 4050
Wire Wire Line
	4150 4050 4150 4800
Wire Wire Line
	4150 4800 8100 4800
Wire Wire Line
	8100 4800 8100 3150
Wire Wire Line
	8100 3150 9050 3150
Text Notes 4150 4800 0    60   ~ 0
SW UART on D10
$Comp
L GND #PWR018
U 1 1 5B45FA3C
P 8900 3900
F 0 "#PWR018" H 8900 3650 50  0001 C CNN
F 1 "GND" H 8900 3750 50  0000 C CNN
F 2 "" H 8900 3900 50  0001 C CNN
F 3 "" H 8900 3900 50  0001 C CNN
	1    8900 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 3850 8900 3850
Wire Wire Line
	8900 3850 8900 3900
$Comp
L +5V #PWR019
U 1 1 5B45FB1C
P 8900 3700
F 0 "#PWR019" H 8900 3550 50  0001 C CNN
F 1 "+5V" H 8900 3840 50  0000 C CNN
F 2 "" H 8900 3700 50  0001 C CNN
F 3 "" H 8900 3700 50  0001 C CNN
	1    8900 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 3750 8900 3750
Wire Wire Line
	8900 3750 8900 3700
Text Label 8600 3150 0    60   ~ 0
DisplayRx
$Comp
L CONN_2 P1
U 1 1 5B46066B
P 3500 4850
F 0 "P1" V 3450 4850 40  0000 C CNN
F 1 "Path_Led_Strip" V 3650 4850 40  0000 C CNN
F 2 "Wire_Connections_Bridges:WireConnection_0.80mmDrill" H 3500 4850 60  0001 C CNN
F 3 "" H 3500 4850 60  0001 C CNN
	1    3500 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 7100 2700 3650
Wire Wire Line
	2700 3650 3000 3650
Wire Wire Line
	3800 3650 4950 3650
Wire Wire Line
	4950 3750 4100 3750
Wire Wire Line
	4100 3750 4100 3850
Wire Wire Line
	4100 3850 3800 3850
Wire Wire Line
	3000 3850 2750 3850
Wire Wire Line
	2750 3850 2750 4950
Wire Wire Line
	2750 4950 3150 4950
$Comp
L +5V #PWR020
U 1 1 5B460EE9
P 3050 4700
F 0 "#PWR020" H 3050 4550 50  0001 C CNN
F 1 "+5V" H 3050 4840 50  0000 C CNN
F 2 "" H 3050 4700 50  0001 C CNN
F 3 "" H 3050 4700 50  0001 C CNN
	1    3050 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 4750 3050 4750
Wire Wire Line
	3050 4750 3050 4700
Text Notes 2750 5200 0    60   ~ 0
White led strip\n(path to Fast Source)
$Comp
L R R1
U 1 1 5B461538
P 5250 5550
F 0 "R1" V 5330 5550 50  0000 C CNN
F 1 "90" V 5250 5550 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM30mm" V 5180 5550 50  0001 C CNN
F 3 "" H 5250 5550 50  0000 C CNN
	1    5250 5550
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 5B461599
P 5500 5750
F 0 "R2" V 5580 5750 50  0000 C CNN
F 1 "10" V 5500 5750 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM30mm" V 5430 5750 50  0001 C CNN
F 3 "" H 5500 5750 50  0000 C CNN
	1    5500 5750
	-1   0    0    1   
$EndComp
Wire Wire Line
	4600 5550 5100 5550
Wire Wire Line
	5400 5550 7200 5550
Wire Wire Line
	5500 5550 5500 5600
Wire Wire Line
	5500 5900 5500 5950
Wire Wire Line
	5200 5900 6100 5900
Wire Wire Line
	5200 5900 5200 5750
Wire Wire Line
	5200 5750 4600 5750
Connection ~ 5500 5550
$Comp
L C C1
U 1 1 5B461EE5
P 5750 5700
F 0 "C1" H 5775 5800 50  0000 L CNN
F 1 "10uF" H 5775 5600 50  0000 L CNN
F 2 "Capacitors_SMD:c_elec_8x10" H 5788 5550 50  0000 C CNN
F 3 "" H 5750 5700 50  0000 C CNN
F 4 "No polarity" H 5750 5700 60  0001 C CNN "Note"
	1    5750 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 5900 5750 5850
Connection ~ 5500 5900
Wire Wire Line
	7200 5550 7200 3950
Wire Wire Line
	7200 3950 6350 3950
Connection ~ 5750 5550
$Comp
L D D1
U 1 1 5B462415
P 6100 5350
F 0 "D1" H 6100 5450 50  0000 C CNN
F 1 "1N4004" H 6100 5250 50  0000 C CNN
F 2 "Diodes_ThroughHole:Diode_DO-41_SOD81_Horizontal_RM10" H 6100 5350 50  0001 C CNN
F 3 "" H 6100 5350 50  0000 C CNN
	1    6100 5350
	0    1    1    0   
$EndComp
$Comp
L D D2
U 1 1 5B46246C
P 6100 5750
F 0 "D2" H 6100 5850 50  0000 C CNN
F 1 "1N4004" H 6100 5650 50  0000 C CNN
F 2 "Diodes_ThroughHole:Diode_DO-41_SOD81_Horizontal_RM10" H 6100 5750 50  0001 C CNN
F 3 "" H 6100 5750 50  0000 C CNN
	1    6100 5750
	0    1    1    0   
$EndComp
Wire Wire Line
	6100 5500 6100 5600
Connection ~ 6100 5550
Connection ~ 5750 5900
$Comp
L +5V #PWR021
U 1 1 5B4629E1
P 6100 5150
F 0 "#PWR021" H 6100 5000 50  0001 C CNN
F 1 "+5V" H 6100 5290 50  0000 C CNN
F 2 "" H 6100 5150 50  0001 C CNN
F 3 "" H 6100 5150 50  0001 C CNN
	1    6100 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 5150 6100 5200
Text Notes 4500 5350 0    60   ~ 0
Vharvest Max = 48V
Text Label 4600 5550 0    60   ~ 0
Vharvest
Text Label 6350 5550 0    60   ~ 0
Vh10
Text Notes 6350 5700 0    60   ~ 0
Vh10 Max = 4.8V
$EndSCHEMATC
