//***This project is about collecting strain data from 3 different straingauges and HX711 and printing them ve RTC on sd card in 3 different columns.***
//
// Hx711 scale(DOUT, SCK)
float a;float b; float c;
#include "hx711.h"
#include <SPI.h>
#include <SD.h>
const int chipSelect = 53; //MISO 50, SCK 52, MOSI 51, CS 53
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
Hx711 scale(A0, A1);  //No 3
Hx711 scale1(A2, A3); //No 2
Hx711 scale2(A4, A5); //No 1 

const float circumference = 1677; // Write Wheel Circumference *mm -2*pi*r-
uint8_t hall_Thresh = 10; // "10 value +25rpm" set number of hall trips for RPM reading (higher improves accuracy)
uint8_t hall_Count; // Counter for each spin

void setup() {
  pinMode(8, INPUT);  //TCH pin
  
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(200);
//  Serial.println("DS1307RTC Read Test");
  Serial.println("-------------------");
  
//  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
//  Serial.println("card initialized.");
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("-------------------");
  dataFile.close();
  delay(50);
  }
}

void loop() {
  tmElements_t tm;

  a=scale.getGram()*7.875/10000;  //eski factor(0,72/742 = 9,7*E-4)
  b=scale1.getGram()*7.875/10000;
  c=scale2.getGram()*7.875/10000;  

  //Serial.println(millis());
  //delay(1000);

  if (RTC.read(tm)) {
    //Serial.print("Ok, Time = ");
    Serial.print(tm.Day);
    Serial.write('/');
    Serial.print(tm.Month);
    Serial.write('/');
    Serial.print(tmYearToCalendar(tm.Year));
    Serial.print(" ");
    print2digits(tm.Hour);
    Serial.write(':');
    print2digits(tm.Minute);
    Serial.write(':');
    print2digits(tm.Second);
    Serial.print(" ");
  } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped.  Please run the SetTime");
      Serial.println("example to initialize the time and begin running.");
      Serial.println();
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();
    }
 //   delay(9000);
  }
 // delay(1000);

  // Initialize speed values
  hall_Count = 1;
  float start = millis();
  bool on_state = false;

  // Calculation Spin Time
  while (1) {
    if (digitalRead(8) == 0)      // Ya da //if (analogRead(A8) < 100)
    {
      if (on_state == false) 
      {
        on_state = true;
        hall_Count++;                   // Increase counter in each spin 
      }
    }
    else 
    {
      on_state = false;
    }

    clear_Serial_RpmSpeed(start);          // Control whether movement

    if (hall_Count >= hall_Thresh)      // Control counter overflow as threshold 
    {
      break;
    }
  } 

  // Calculation Passed Time 
  float end_Time = millis();
  float time_Passed = (end_Time - start) / 1000;

  calc_Rpm(time_Passed);
  calc_Speed(time_Passed);

  delay(1);// delay in between reads for stability 

   Serial.print(c);
   Serial.print(" ");
   Serial.print(b);
   Serial.print(" ");
   Serial.print(a);
   Serial.println(" ");
  

  // make a string for assembling the data to log:
  String dataString = "";String dataString1 = "";String dataString2 = "";
  dataString += String(c);dataString1 += String(b);dataString2 += String(a);

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(tm.Day);
    dataFile.write('/');
    dataFile.print(tm.Month);
    dataFile.write('/');
    dataFile.print(tmYearToCalendar(tm.Year));
    dataFile.print(" ");
    dataFile.print(tm.Hour);
    dataFile.write(':');
    dataFile.print(tm.Minute);
    dataFile.write(':');
    dataFile.print(tm.Second);
    dataFile.print(" ");
    
    dataFile.print(dataString);dataFile.print(" ");
    dataFile.print(dataString1);dataFile.print(" ");
    dataFile.print(dataString2);dataFile.println("");
    dataFile.close();

  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
 
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

// Calculation Speed
void calc_Speed(float passed_Time)
{
  float m_Speed = (circumference / 1000000) * hall_Count / (passed_Time * 12 / 3600);
  Serial.print((int)m_Speed);
  Serial.println(" km/h ");
}

// Calculation RPM
void calc_Rpm(float passed_Time)
{
  float rpm_Val =(hall_Count/(passed_Time * 12)) * 60;
  Serial.print((int)rpm_Val);
  Serial.print(" rpm ");
}

void clear_Serial_RpmSpeed(float start_Time)
{
    // set 0 values on lcd screen     
    float e_Time = millis();
    if((e_Time - start_Time) / 1000 > 5) //12saniye aynı kalıp sonra sıfırlıyor. !Bu bence daha düşük olabilir!
    {
      Serial.print("0 rpm ");
      Serial.println("0 km/h");  
    }
}
