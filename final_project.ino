/*
CPE 301 Final Project
Names: Daniel Matthies, Ethan Partain, Ibrahim Khondoker
Date: 12/15/2023
*/




#include <Stepper.h>
#include "DHT.h"
#define DHTPIN 6
#define DHTTYPE DHT11
#include <LiquidCrystal.h>
#include <RTClib.h>

DHT dht(DHTPIN, DHTTYPE);
const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
const int rolePerMinute = 15;
unsigned char X_pin = 1; // analog pin connected to X output
const int rs = 12, en = 11, d4 = 10, d5 = 9, d6 = 8, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define RDA 0x80
#define TBE 0x20  

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

unsigned char levelchannel = 2;
unsigned int levelint;

// ports for input and output
volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a  = (unsigned char*) 0x21; 
volatile unsigned char* pin_a  = (unsigned char*) 0x20;

volatile unsigned char* port_k = (unsigned char*) 0x108; 
volatile unsigned char* ddr_k  = (unsigned char*) 0x107; 
volatile unsigned char* pin_k  = (unsigned char*) 0x106; 

// for checking every 30 seconds
const long interval = 30000; // in milliseconds
unsigned long previousMilli = 0; // stores last time an update happened

// for button functionality
bool isDisabled = false;

// for date and time
RTC_DS1307 rtc;
DateTime currentTime;

Stepper myStepper(stepsPerRevolution, 2, 4, 3, 5);

void setup() {
  // setup the UART
  U0init(9600);
  // setup the ADC
  adc_init();

  // setup Stepper Motor
  myStepper.setSpeed(rolePerMinute);
  
  // setup DHT and LCD
  dht.begin(); // initialize the sensor
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // setup pins and ports
  // set input PK2 and enable pull-down resistor
  *ddr_k &= 0xFB;
  *port_k &= 0xFB;

  // set outputs on PB0, PB1, PB2, and PB3
  *ddr_a |= 0x0F;

  // for date and time 
  //rtc.begin();
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void loop() {
  static bool ButtonState = HIGH;
  bool newButtonState = ((*pin_k & 0x04) == 0);
  
  //currentTime = rtc.now();

  if (newButtonState != ButtonState) {
    ButtonState = newButtonState;


    if (ButtonState == LOW) {
      isDisabled = !isDisabled;

      if(isDisabled) {
        //serialPrintTime();

        // Turning on the yellow light
        *port_a |= 0b00000010;
        *port_a &= 0b11110010;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Disabled");

        stepperMotor();
      }
      else {

      }
    }
  }

  if (!isDisabled) {
    // Water Level Integer
    //levelint = 100; // for testing purposes
    levelint = adc_read(levelchannel) - 200; // the reason why I'm subtracting 200 is because my water level sensor seemed damage

    // state red light
    if (levelint < 150) {
      //serialPrintTime();

      // Turning red light on
      *port_a |= 0b00000100;
      *port_a &= 0b11110100;

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Water Level");
      lcd.setCursor(0,1);
      lcd.print("Too Low");
    }

    // state blue light
    else if (dht.readTemperature(true) > 70) {
      //serialPrintTime();
      // Turning blue light on 
      *port_a |= 0b00001000;
      *port_a &= 0b11111000;

      // DHT related function
      getTemp();

      // stepper motor function
      stepperMotor();
    }
    
    // state green light
    else {
      //serialPrintTime();

      // Turning Green Light on
      *port_a |= 0b00000001;
      *port_a &= 0b11110001; // Turn off Red, Blue, and Yellow Lights

      // DHT related function
      getTemp();

      // stepper motor function
      stepperMotor();
    }
    
  }
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char U0getchar()
{
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

// DHT Function
void getTemp(){
  // wait 30 seconds between measurements.
  unsigned long currentMilli = millis();
  if((currentMilli - previousMilli >= interval) || (currentMilli > 0)) {
    previousMilli = currentMilli;
    
    // read humidity
    float humi  = dht.readHumidity();
    // read temperature as Celsius
    float tempC = dht.readTemperature();
    // read temperature as Fahrenheit
    float tempF = dht.readTemperature(true);
  
  
    // check if any reads failed
    if (isnan(humi) || isnan(tempC) || isnan(tempF)) {
      lcd.print("Sensor Read Fail!");
    } 
    else {
      lcd.setCursor(0,0);
      lcd.print("Temp: ");
      //lcd.print(tempC);
      //lcd.print("C ~ ");
      lcd.print(tempF);
      lcd.print("F");

      lcd.setCursor(0,1);
      lcd.print("Humidity: ");
      lcd.print(humi);
      lcd.print("%");
    }
  }
}

void stepperMotor() {
  // for stepper
  int a = adc_read(X_pin);
  if (a > 400 && a < 520)
  {
    for (int i = 2; i < 6; i++)
    {
      digitalWrite(i, LOW);
    }

  }
  else if (a < 400)
  {
    myStepper.setSpeed(rolePerMinute);
    myStepper.step(-30);
  }
  else if (a > 530)
  {
    myStepper.setSpeed(rolePerMinute);
    myStepper.step(30);
  }
}

void serialPrintTime() {
  String datetime = String(currentTime.month());
  for(int i = 0; i < datetime.length(); i++) {
    U0putchar(datetime[i]);
  }
  U0putchar('/');

  datetime = String(currentTime.day());
  for(int i = 0; i < datetime.length(); i++) {
    U0putchar(datetime[i]);
  }
  U0putchar('/');

  datetime = String(currentTime.year());
  for(int i = 0; i < datetime.length(); i++) {
    U0putchar(datetime[i]);
  }
  U0putchar(' ');

  datetime = String(currentTime.hour());
  for(int i = 0; i < datetime.length(); i++) {
    U0putchar(datetime[i]);
  }
  U0putchar(':');

  datetime = String(currentTime.minute());
  for(int i = 0; i < datetime.length(); i++) {
    U0putchar(datetime[i]);
  }

  U0putchar('\n');

}