import smbus
import RPi.GPIO as GPIO
import time
import serial
import pyrebase
import datetime
from time import sleep

GPIO.setmode(GPIO.BCM) # GPIO Numbers instead of board numbers
config = {
 "apiKey": "*************************************",
 "authDomain": "*************************************",
 "databaseURL": "*************************************",
 "storageBucket": "*************************************"
}
firebase = pyrebase.initialize_app(config)
db = firebase.database()

print("Send Data to Firebase Using Raspberry Pi")
print("—————————————")
print()
#realy
Relay1_GPIO = 17#(pin11)#cooling tower
Relay2_GPIO = 22#(pin15)#condenser pump
Relay3_GPIO = 10#(pin19)#chiller

GPIO.setup(Relay1_GPIO, GPIO.OUT) # GPIO Assign mode
GPIO.setup(Relay2_GPIO, GPIO.OUT) # GPIO Assign mode
GPIO.setup(Relay3_GPIO, GPIO.OUT) # GPIO Assign mode

GPIO.output(Relay1_GPIO, GPIO.HIGH)
GPIO.output(Relay2_GPIO, GPIO.HIGH)
GPIO.output(Relay3_GPIO, GPIO.HIGH)
flag=1
#relay
ser = serial.Serial ("/dev/ttyACM0",9600)
#####
#input  on pin 13,pin6 GND,5V to RPI4 pin 2
import RPi.GPIO as GPIO         #import GPIO library
import time, sys                #import time, sys library
#GPIO.setmode(GPIO.BOARD)        #GPIO pin numbering board
inpt = 27                      #set input pin 13/gpio 27
GPIO.setup(inpt,GPIO.IN)        #set input as inpt pin
rate_cnt = 0                    #Revolution counts (r/min)
tot_cnt = 0                     #Total counts
time_zero = 0.0                 #system start up time
time_start = 0.0                #Keep measurement begin time
time_end = 0.0                  #Keep measurement end time
gpio_last = 0                   #Was  last state 0 or 1 or other?
pulses = 0                      #0-5 pulses from YF-S201
constant = 1.79                 #Water meter calibration factor


print('water Flow -Approximate')
print('Control C to exit')

time_zero= time.time()
##########
# Define some device parameters
I2C_ADDR  = 0x27 # I2C device address
LCD_WIDTH = 16   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line

LCD_BACKLIGHT  = 0x08  # On
#LCD_BACKLIGHT = 0x00  # Off

ENABLE = 0b00000100 # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#splitting
temp1=27
temp2=27
global iii
iii=0
#splitting

#Open I2C interface
#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = the data
  # mode = 1 for data
  #        0 for command

  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
  bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

  # High bits
  bus.write_byte(I2C_ADDR, bits_high)
  lcd_toggle_enable(bits_high)

  # Low bits
  bus.write_byte(I2C_ADDR, bits_low)
  lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
  # Toggle enable
  time.sleep(E_DELAY)
  bus.write_byte(I2C_ADDR, (bits | ENABLE))
  time.sleep(E_PULSE)
  bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display

  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

def main():
  # Main program block

  # Initialise display
  lcd_init()
  lcd_string("   FLOW ",LCD_LINE_1)# first line
  rate_cnt = 0                    #Revolution counts (r/min)
  tot_cnt = 0                     #Total counts
  time_zero = 0.0                 #system start up time
  time_start = 0.0                #Keep measurement begin time
  time_end = 0.0                  #Keep measurement end time
  gpio_last = 0                   #Was  last state 0 or 1 or other?
  pulses = 0                      #0-5 pulses from YF-S201
  constant = 1.79
  while True:
########
           #uno to rpi4
           received_data = ser.read()              #read serial port
           sleep(0.03)
           data_left = ser.inWaiting()             #check for remaining byte
           received_data += ser.read(data_left)
           #print (received_data)
           ##uno to rpi4

           #spliting
           txt=str(received_data)
           x11=txt.split("*") #spliting from * and save as LIST
           current=float(x11[4])
           voltage=float(x11[3])
           power=current*voltage
           add=int(x11[1])

           #splitting
           global iii
           if(iii==0):

            temp1=27
            temp2=27
            iii=2
           #splitting

           if(add==645):
            temp1=int(x11[2])*(27/313)*(28/38) #add multiplication factor
           if(add==812):
            temp2=int(x11[2])*(27/198)*(28/65)*(28/60)#add multiplication factor
           #spliting

           #printing spliting
           print(temp1)
           print(temp2)
           #print(current,voltage,power,temp1,temp2)
           #printing spliting




           rate_cnt = 0                             #Reset rate counter
           pulses = 0                               #0-5 pulses from YF-S201
           time_start = time.time()                 #Keep start time
           while pulses <=5:                        #6 pulses per revolution
                gpio_cur = GPIO.input(inpt)         #Poll input
                if gpio_cur !=0 and gpio_cur != gpio_last:    #Input changed
                  pulses +=1
                gpio_last = gpio_cur                #Keep last input state
                try:
                  #print(GPIO.input(inpt), end='')  #Status bit.LOWER ACCURACY
                  None
                except KeyboardInterrupt:           #Look for exit command
                  print('\nCTRL C -exiting nicely')
                  GPIO.cleanup()                    #Clean up GPIO
                  print('Done')                     #Print  'Done'
                  sys.exit()                        #Exit nicely

           rate_cnt +=1                            #Revoulutions/time
           tot_cnt +=1                             #Total revs since start
           time_end = time.time()                  #End of measurement time

           print('\nLiters /min',
              round((rate_cnt* constant)/(time_end-time_start),2),
               'approximate')
           print('total liters', round(tot_cnt * constant,1))
           print('Time (min&clock)',
                  round((time.time()-time_zero)/60,2), '\t',
                  time.asctime(time.localtime(time.time())),'\n')

           flow1=round((rate_cnt* constant)/(time_end-time_start),2)
           lcd_string("   FLOW ",LCD_LINE_1)# first line only string
           lcd_string(str(flow1),LCD_LINE_2)#LCD PRINT ON SECOND Line only string




           #control1
           if(temp2<25):
             sleep(15)
             #relay
             GPIO.output(Relay1_GPIO, GPIO.LOW)
             sleep(1)
             GPIO.output(Relay2_GPIO, GPIO.LOW)
             sleep(1)
             GPIO.output(Relay3_GPIO, GPIO.LOW)
             flag=0

           #realy
           #control1

           #control2
           if(temp2>26):
             sleep(1)
             #relay
             GPIO.output(Relay1_GPIO, GPIO.HIGH)
             sleep(1)
             GPIO.output(Relay2_GPIO, GPIO.HIGH)
             sleep(1)
             GPIO.output(Relay3_GPIO, GPIO.HIGH)
             flag=1

           #realy
           #control2

           #firebase
           tstamp=datetime.datetime.now().strftime('%m-%d-%Y_%H.%M.%S')
          #kwtr=(power*3024)/(flow1*4.187*(temp2-temp1))
          #Loading= 100/(kwtr*power*capacity)
           kwtr=.81
           data = {
                 "Temperature 1": temp1,
                 "Temperature 2": temp2,
                 "Flow": flow1,
                 "Time": tstamp,
                 "State":flag,
                 "Power": power  ,
                 "Loading":60   ,
                 "KW per TR":kwtr,
                    }

           db.child("users").push(data)
           #firebase
           lcd_string("TEMPERATURE 2 ",LCD_LINE_1)# first line only string
           lcd_string(str(temp2),LCD_LINE_2)#LCD PRINT ON SECOND Line only string
           sleep(1)
           lcd_string("POWER ",LCD_LINE_1)# first line only string
           lcd_string(str(power),LCD_LINE_2)#LCD PRINT ON SECOND Line only string
           sleep(1)
           lcd_string("          ",LCD_LINE_2)#LCD PRINT ON SECOND Line only string
           if(flag==0):
            lcd_string("CHILLER  OFF",LCD_LINE_1)# first line only string
           if(flag==1):
            lcd_string("CHILLER  ON",LCD_LINE_1)# first line only string
           sleep(1)


  ##########
if __name__ == '__main__':

  try:
    main()
  except KeyboardInterrupt:
    pass
  finally:
    lcd_byte(0x01, LCD_CMD)