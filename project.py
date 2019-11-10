#!/usr/bin/python3

from time import sleep
import RPi.GPIO as GPIO
import sys
import time
from datetime import datetime
import numpy as np
import smbus
import BlynkLib
import signal


GPIO.setmode(GPIO.BCM)

blynk = BlynkLib.Blynk('qZ71OHkDUDfvkI27OTqL1YPxoa8AgQMc')
Alarm = 1
rTime = "10:17:15"
sysTime = "00:00:00"
sysHour = 0
sysMin = 0
sysSec = 0
hour = 0
min = 0
sec = 0
temp = 25
hum = 0.5
light = 595
dac = 1.5
freq = 1
play = 1
 
Alarm_min = -100
Alarm_sec = -100

bus = smbus.SMBus(1)
RTC_ADDR = 0x6f

ADDR_SEC = 0x00
ADDR_MIN = 0x01
ADDR_HOUR = 0x02

var_list = ["Real Time", "System Time", "Humidity", "Temperature", "Light", "DAC Out"]
data = np.array([rTime, sysTime, hum, temp, light, dac])

# DAC Variables
V_Ref = 3300 # 3V3 in mV
Resolution = 2**10 # 10 bits for the MCP 4911

outputlevel = 1021
CS = 12
SCK = 13
SDI = 16

data_DAC = 0

# Button Variables
btn_stopStart = 21
btn_alarm = 19
btn_resetTime = 26
btn_interval = 20

led_alarm = 4
GPIO.setup(led_alarm, GPIO.OUT)
pwm_alarm = GPIO.PWM(led_alarm, 5)
y = True
interval_value = 1          # intervals are 1s,2s and 5s
stopStart_check = True   #True = monitoring, False = stop monitoring

# ADC Variables
# Pin numbers
CLK = 23
MISO = 27
MOSI = 22
CS_ADC = 17


# Sensor values
temp_sen01 = 0
temp_sen02 = 0
pot_sen01 = 0
pot_sen02 = 0
ldr_sen01 = 0
ldr_sen02 = 0

Vout = 0
#Setup blink


def write():
    blynk.virtual_write(0,rTime)   #RTC
    blynk.virtual_write(1,sysTime)   #sys time
    blynk.virtual_write(2,str(round((pot_sen01*3.3/2048),2)) + "V")   #humidity
    blynk.virtual_write(3,(ldr_sen01//2))   #light
    blynk.virtual_write(4,temp_sen02)   #temp
    blynk.virtual_write(5,str(Vout) +"V")   #dac out

# Register Virtual Pins
@blynk.VIRTUAL_WRITE(6)
def my_write_handler(value):
    pwm_alarm.stop()
    """ pwm_alarm.stop()"""
    print("\nAlarm button has been dismissed")
    print("-----------------------------------------")
    print("\n")
    #alarm

@blynk.VIRTUAL_WRITE(7)
def my_write_handler(value):
    global sysTime, sysHour, sysMin, sysSec, hour, min, sec
    sysTime = "00:00:00"
    sysHour = hour
    sysMin = min
    sysSec = sec
@blynk.VIRTUAL_WRITE(8)
def my_write_handler(value):
    global freq
    if(freq==1):
        freq = 2
    elif(freq == 2):
        freq = 5
    elif(freq == 5):
        freq = 1 
#RTC
def setTime():
    global sysHour, sysMin, sysSec, hour, min, sec
    tHours = datetime.now().hour
    if(tHours>11):
        tHours -= 12
    tMin = datetime.now().minute
    tSec = datetime.now().second
    sysHour = tHours
    sysMin = tMin
    sysSec = tSec
    hour = tHours
    min = tMin
    sec = tSec
    bus.write_byte_data(RTC_ADDR, ADDR_HOUR, tHours)
    bus.write_byte_data(RTC_ADDR, ADDR_MIN, tMin)
    bus.write_byte_data(RTC_ADDR, ADDR_SEC, tSec + 0b10000000)
def getTime():
    global rTime,hour, min, sec, sysTime, sysHour, sysMin, sysSec
    hour = bcd2bin(bus.read_byte_data(RTC_ADDR, ADDR_HOUR) &0x3f)
    min =  bcd2bin(bus.read_byte_data(RTC_ADDR, ADDR_MIN) & 0x7f)
    sec =  bcd2bin(bus.read_byte_data(RTC_ADDR, ADDR_SEC) & 0x7f)
    totalSec = hour*60*60 + min*60 + sec -(sysHour*60*60 + sysMin*60 + sysSec)
    if(totalSec<0):
        totalSec = 0
        sysHour = hour
        sysMin = min
        sysSec = sec
    tSec = totalSec%60
    totalMin = totalSec/60
    tHour = totalMin/60
    tMin = totalMin%60
    sysTime =  "%02d:%02d:%02d" % (tHour, tMin, tSec)
    rTime =  "%02d:%02d:%02d" % (hour, min, sec)

def bcd2bin(x):
  return (((x) & 0x0f) + ((x) >> 4) * 10)

def setupPins():
    # Setup pins for DAC
    GPIO.setup(CS, GPIO.OUT)
    GPIO.setup(SCK, GPIO.OUT)
    GPIO.setup(SDI, GPIO.OUT)

    # Setup ports for buttons inputs
    GPIO.setup(btn_stopStart, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set GPIO 21 to be an input pin and set initial value to be pulled high
    GPIO.setup(btn_alarm, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set GPIO 19 to be an input pin and set initial value to be pulled high
    GPIO.setup(btn_resetTime, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set GPIO 26 to be an input pin and set initial value to be pulled high
    GPIO.setup(btn_interval, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set GPIO 20 to be an input pin and set initial value to be pulled high

    # Setup ports for ADC
    ''' Set all pins as an output except MISO (Master Input, Slave Output)'''
    GPIO.setup(CLK, GPIO.OUT)
    GPIO.setup(MISO, GPIO.IN)
    GPIO.setup(MOSI, GPIO.OUT)
    GPIO.setup(CS_ADC, GPIO.OUT)

def printOut():
    data = np.array([rTime, sysTime, round((pot_sen01*3.3/2048),2), temp_sen02, ldr_sen01//2, Vout])
    row_format ="{:>15}" * (len(var_list) + 1)
    sys.stdout.write(row_format.format("\r               " , *data))
    sys.stdout.flush()

def decimalToBinary(n):
    number = n
    x = 0
    binary_value = 0b0

    while(number!=0):
        rem = number%2
        number = number//2
        if 0b1 & rem:
            binary_value = binary_value | 0b1<<x
        else:
           binary_value = binary_value | 0b0<<x
        
        x = x+1
    
    for i in range(x,10):
        binary_value = binary_value | 0b0<<x
        x = x+1
    
    return binary_value

# DAC functions
def setOutput(val):
    """global CS
    global SCK
    global SDI
    """
    read = val*((2**10)/3.3)
    read =  int(read)
    data_DAC = decimalToBinary(read)
    control_bits = 0b0 << 3 | 0b0 << 2 | 0b1 << 1 | 0b1 << 0
#    print("data conveted: {0:10b} (10 bit)".format(data_DAC))


    GPIO.output(CS, GPIO.HIGH)
    GPIO.output(CS, GPIO.LOW)
    GPIO.output(SCK, GPIO.LOW)



    for bit in range(4):
        # Set RPi's output bit high or low depending on highest bit of data field
        if control_bits & 0b1000:
            GPIO.output(SDI, GPIO.HIGH)
        else:
            GPIO.output(SDI, GPIO.LOW)

        # Advance data to the next bit
        control_bits <<= 1

        # Pulse the clock pin HIGH then immediately low
        GPIO.output(SCK, GPIO.HIGH)
        GPIO.output(SCK, GPIO.LOW)

    for bit in range(10):
        # Set RPi's output bit high or low depending on highest bit of data field
        if data_DAC & 0b1000000000:
            GPIO.output(SDI, GPIO.HIGH)
        else:
            GPIO.output(SDI, GPIO.LOW)

        # Advance data to the next bit
        data_DAC <<= 1

        # Pulse the clock pin HIGH then immediately low
        GPIO.output(SCK, GPIO.HIGH)
        GPIO.output(SCK, GPIO.LOW)   
    for i in range(2):
        GPIO.output(SDI, GPIO.LOW)

        GPIO.output(SCK, GPIO.HIGH)
        GPIO.output(SCK, GPIO.LOW)

    GPIO.output(CS, GPIO.HIGH)
# End of DAC functions

# Buttons functions
def stopStart(x):
    global stopStart_check
    if stopStart_check == True:
        stopStart_check = False
    else:
        stopStart_check = True
    #print("Stop/Start button has been pressed")
    #print("-----------------------------------------")
    #print("\n")

def alarm(x):
    pwm_alarm.stop()
    """ pwm_alarm.stop()"""
    blynk.virtual_write(6,0)
    print("\nAlarm button has been dismissed")
    print("-----------------------------------------")
    print("\n")


def resetTime(x):
    global sysTime, sysHour, sysMin, sysSec, hour, min, sec
    sysTime = "00:00:00"
    sysHour = hour
    sysMin = min
    sysSec = sec
    #print("Reset System Time button has been preesed")
   # print("-----------------------------------------")
   # print("\n")


def interval(x):
    global freq
    if(freq==1):
        freq = 2
    elif(freq == 2):
        freq = 5
    elif(freq == 5):
        freq = 1 
   # print("Change invertal button has been pressed")
   # print("-----------------------------------------")
   # print("\n")
# End of Button Functions

# ADC functions
def readAdc(channel):
    if (channel < 0) or (channel > 7):
        print ("Invalid ADC Channel number, must be between [0,7]")
        return -1

    # Datasheet says chip select must be pulled high between conversions
    GPIO.output(CS_ADC, GPIO.HIGH)

    # Start the read with both clock and chip select low
    GPIO.output(CS_ADC, GPIO.LOW)
    GPIO.output(CLK, GPIO.HIGH)


    read_command = 0x18
    read_command |= channel

    sendBits(read_command, 5)

    adcValue = recvBits(12)

    # Set chip select high to end the read
    GPIO.output(CS_ADC, GPIO.HIGH)

    return adcValue

def sendBits(data, numBits):
    ''' Sends 1 Byte or less of data'''
    data <<= (8 - numBits)
    for bit in range(numBits):
        # Set RPi's output bit high or low depending on highest bit of data field
        if data & 0x80:
            GPIO.output(MOSI, GPIO.HIGH)
        else:
            GPIO.output(MOSI, GPIO.LOW)
  
         

        # Advance data to the next bit
        data <<= 1

        # Pulse the clock pin HIGH then immediately low
        GPIO.output(CLK, GPIO.HIGH)
        GPIO.output(CLK, GPIO.LOW)

def recvBits(numBits):
    '''Receives arbitrary number of bits'''
    retVal = 0

    for bit in range(numBits):
        # Pulse clock pin
        GPIO.output(CLK, GPIO.HIGH)
        GPIO.output(CLK, GPIO.LOW)

        # Read 1 data bit in
        if GPIO.input(MISO):
            retVal |= 0x1

        # Advance input to next bit
        retVal <<= 1

    # Divide by two to drop the NULL bit
    return (retVal/2)

# End of ADC functions
try:
    setupPins()

    # Detect button press using interuppts
    GPIO.add_event_detect(btn_stopStart,GPIO.FALLING,callback=stopStart, bouncetime=300) # Setup event on GPIO 21 falling edge
    GPIO.add_event_detect(btn_alarm,GPIO.FALLING,callback=alarm, bouncetime=300) # Setup event on GPIO 19 falling edge
    GPIO.add_event_detect(btn_resetTime,GPIO.FALLING,callback=resetTime, bouncetime=300) # Setup event on GPIO 26 falling edge
    GPIO.add_event_detect(btn_interval,GPIO.FALLING,callback=interval, bouncetime=300) # Setup event on GPIO 20 falling edge
    
    setTime()
    row_format ="{:>15}" * (len(var_list) + 1)
    print(row_format.format("", *var_list))
    while(True):
        blynk.run()
        getTime()
        pot_sen01 = readAdc(4)
        pot_sen02 = readAdc(3)
        temp_sen02 = readAdc(7)
        temp_sen02 = ((temp_sen02*1000*3.3)/2048 - 500)/(10) - 6
        temp_sen02 = round(temp_sen02,2)
        ldr_sen01 = readAdc(0)
        ldr_sen02= readAdc(1)
        Vout = ((pot_sen01/2048)*3.3) * (ldr_sen01/2048)
        Vout = round(Vout, 2)
        setOutput(Vout)
        if(stopStart_check):
            printOut()
            write()
 
        if((min*60 +sec - (Alarm_min*60+Alarm_sec))>180):
            if Vout<0.65:
                pwm_alarm.start(20)
                pwm_alarm.ChangeFrequency(5)
                Alarm_sec = sec
                Alarm_min = min
                blynk.virtual_write(6,1)
            if Vout>2.65:
                pwm_alarm.start(20)
                pwm_alarm.ChangeFrequency(5)
                Alarm_sec = sec
                Alarm_min = min
                blynk.virtual_write(6,1)
        time.sleep(freq)

        
except (KeyboardInterrupt, Exception) as e:
    print(e)
    print ("Shutting down.......")
    GPIO.cleanup()

def main():
    pass

if __name__ == '__main__':
    main()





