//***This project is about calculating cycle time and speed via hall effect sensor pin on brushless DC motor.***

const float circumference = 1677; // Write Wheel Circumference *mm -2*pi*r-

uint8_t hall_Thresh = 10; // "10 value +25rpm" set number of hall trips for RPM reading (higher improves accuracy)

uint8_t hall_Count; // Counter for each spin

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

void setup() {
  pinMode(8, INPUT); // TCH pini 5V altı kaldığı için girişi HIGH yapamayabilir!!! Bunu kontrol etmek gerekir. 
                     // Bunun yerine analogRead kullanılarak 0-0.86 volt seviyesi için 0-1 logic atanabilir.
  Serial.begin(9600);
  Serial.print("0 rpm ");
  Serial.println("0 km/h ");
}

void loop() {
  
  //if (analogRead(A8) > 100) { int a = 1 ; }  // 0-1 logic yapma 
  //  else { int a = 0 ; }
  
  // Initialize values
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

  delay(1);// delay in between reads for stability /////////// Burada da bir millis hesabı yapmak gerekebilir.
}
