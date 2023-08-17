// DENGE ROBOTU 2020 ANA PROGRAMI
// Robot Melodi çalması eklendi. 10.4.2020
// ROBOT BLUETOTH UZERINDEN hareket ettirildi. 17.4.2020
// 2x16 LCD eklendi. 24.4.2020
// MPU6050 6 Eksen İvme ve Gyro Sensörü eklendi. 31.05.2020

// LIBRARIES
#include "pitches.h"
#include "LiquidCrystal.h"
#include "TimerOne.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

// DEFINEMENTS
#define SPEED_SLOW   150   // robotun yavas hız değer tanımı (PWM)
#define SPEED_NORMAL 200   // robotun normal hız değer tanımı (PWM)
#define SPEED_FAST   255   // robotun hızlı hız değer tanımı  (PWM)

#define Kp  40             // oransal katsayı
#define Kd  0.05           // türev katsayısı
#define Ki  40             // integral katsayısı
#define sampleTime  0.005  // örnekleme zamanı (dt)
#define targetAngle -2.0   // hata dongusu için duzenli rejim acı değeri

MPU6050 mpu;

// GLOBAL VARIABLES
int i;                  // genel dongu degiskeni
int notasure;           // nota sure hesap değişkeni
int serioku;            // seri kanaldan komut almak icin
boolean inPin1 = LOW;   // motor kontrol
boolean inPin2 = HIGH;  // motor kontrol

int melodi[] = { C4,G3,G3,A3,G3, 0,B3,C4 };
int mesure[] = { 4, 8, 8, 4, 4,  4, 4, 4 }; // 4=ceyrek, 8=1/8nota
int melodi1[] = {E7, E7, 0, E7, 0, C7, E7, 0,  G7, 0, 0,  0,  G6, 0, 0, 0};
int mesure1[] = {12, 12, 12, 12,12, 12, 12, 12,12, 12, 12, 12,12, 12, 12, 12};
float temp,hum,pres;

int16_t accY, accZ, gyroX;          // ivmelendirme sensörü ve jiroskop açı değişkenleri
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;

// PORTS
int SPK  = 52;       // 8 ohm hoparlor
int STBY = 10;       // standby
int PWMA = 7;        // Motor A Speed control   L298 ENA
int AIN1 = 9;        // Motor A Direction       L298 IN1 00:stop, 01:left 10:right 11:brake 
int AIN2 = 8;        // Motor A Direction       L298 IN2
int PWMB = 13;       // Motor B Speed control   L298 ENB
int BIN1 = 11;       // Motor B Direction       L298 IN3
int BIN2 = 12;       // Motor B Direction       L298 IN4
int LCDRW = 31;      // LCD RW

// LIBRARY DEFINITIONS
LiquidCrystal lcd(32,33,34,35,36,37); //rs, en, d4, d5, d6, d7

void init_PID() {             // PID Timer1 tanımlanıyor.
  // Timer1'i baslatma
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B
  // set compare match register to set sample time 5ms
  OCR1A = 9999;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
pinMode(LCDRW,OUTPUT); // LCDRW pinini çıkış olarak belirliyor.
pinMode(SPK,  OUTPUT); // SPK UCU digital çıkış portu 53 te
pinMode(STBY, OUTPUT);
pinMode(PWMA, OUTPUT);
pinMode(AIN1, OUTPUT);
pinMode(AIN2, OUTPUT);
pinMode(PWMB, OUTPUT);
pinMode(BIN1, OUTPUT);
pinMode(BIN2, OUTPUT);

temp=0; hum=0; pres=0;

digitalWrite(LCDRW,LOW); // lcdrw hep LOW yani yazma modunda
delay(10);
lcd.begin(16, 2);  delay(400);
  
Timer1.initialize(1000000); // 1 sn
Timer1.attachInterrupt(tim);
  
Serial.begin(9600); // 9600 baud seri kanal 0  USB BAGLANTI
delay(100);         // seri kanal a baslaması icin bir süre veriyoruz.
Serial1.begin(9600); // 9600 baud seri kanal 1 BLUETOOTH
delay(100);         // seri kanal a baslaması icin bir süre veriyoruz.
Serial.println("ROBOT HAZIRLANIYOR");


  for (int i=0; i < 3; i++) {
  lcd.setCursor(0, 0); lcd.print("Mekatronik  2020"); //KOLON,SATIR
  delay(500);
  lcd.setCursor(0, 0); lcd.print("                "); //KOLON,SATIR
  delay(500); 
   }
   

int boyut = sizeof(melodi1) / sizeof(int);
  for (i=0;i<boyut;i++) { Serial.print(".");
    notasure=1000/mesure1[i];
    tone(SPK,melodi1[i],notasure);
    delay(notasure*1.5);
    noTone(SPK);
  }
  Serial.println("");
  Serial.println("ROBOT HAZIRLANDI");
  lcd.setCursor(0, 0); lcd.print("ROBOT IS READY"); 

  
  
  mpu.initialize();            // MPU6050'yi başlatılıyor 
  mpu.setYAccelOffset(1561);   // ofset değerlerini giriliyor
  mpu.setZAccelOffset(717);
  mpu.setXGyroOffset(483);
  
  init_PID();           // PID örnekleme döngüsü başlatılıyor
}


void tim()
{
  Serial1.print(temp); Serial1.print(',');
  Serial1.print(hum); Serial1.print(',');
  Serial1.println(pres); 
  temp=temp+1; hum=hum+1; pres=pres+1;
}


void loop() {
  
 while (Serial.available() > 0) {
     serioku=Serial.read();
     if (serioku=='B') { geri(SPEED_NORMAL); Serial.println("GERI");  }   // backward yani geri
     if (serioku=='F') { ileri(SPEED_NORMAL); Serial.println("ILERI"); }  // forward yani ileri
     if (serioku=='S') { dur(); Serial.println("DUR");  }                 // stop yani dur
     if (serioku=='L') { sol(SPEED_NORMAL); Serial.println("SOL"); }      // left yani sola dön
     if (serioku=='R') { sag(SPEED_NORMAL); Serial.println("SAG"); }      // right yani saga dön
     if (serioku=='H') { fren(SPEED_NORMAL); Serial.println("FREN"); }    // hold yani fren yap
     while (serioku=='D') { }   //motorları çalıştır    
 }
  while (Serial1.available() > 0) {
     serioku=Serial1.read();
     if (serioku=='B') { geri(SPEED_NORMAL); Serial.println("GERI");  }   // backward yani geri
     if (serioku=='F') { ileri(SPEED_NORMAL); Serial.println("ILERI"); }  // forward yani ileri
     if (serioku=='S') { dur(); Serial.println("DUR");  }                 // stop yani dur
     if (serioku=='L') { sol(SPEED_NORMAL); Serial.println("SOL"); }      // left yani sola dön
     if (serioku=='R') { sag(SPEED_NORMAL); Serial.println("SAG"); }      // right yani saga dön
     if (serioku=='H') { fren(SPEED_NORMAL); Serial.println("FREN"); }    // hold yani fren yap
     while (serioku=='D') { accY = mpu.getAccelerationY();                // y eksenindeki dogrusal ivme  // ivmelendirme ve jiroskop degerlerinin okunması
                         accZ = mpu.getAccelerationZ();                   // z eksenindeki dogrusal ivme 
                         gyroX = mpu.getRotationX();                      // x eksenindeki acısal ivme  
   
                         motorPower = constrain(motorPower, -255, 255);   // motor gucunu sınırlandırma
                         setMotors(motorPower, motorPower) ; Serial.println("DENGELE");}    //motorları çalıştır
 }
                         accY = mpu.getAccelerationY();                   // y eksenindeki dogrusal ivme 
                         accZ = mpu.getAccelerationZ();                   // z eksenindeki dogrusal ivme 
                         gyroX = mpu.getRotationX();                      // x eksenindeki acısal ivme  
  
                         motorPower = constrain(motorPower, -255, 255);   // motor gucunu sınırlandırma
                         setMotors(motorPower, motorPower) ; Serial.println("DENGELE");
}
  
  ISR(TIMER1_COMPA_vect) {                                                // 5 ms kesme rutini
  
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;                              // ivmeolcerde egim acısının radyandan dereceye dönüştürülmesi
  gyroRate = map(gyroX, -32768, 32767, -250, 250);                        // jiroskop dğerinin okunması
  gyroAngle = (float)gyroRate * sampleTime;                               // saniyede dereceye çeviriliyor
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);  // tamamlayıcı filtre uygulanıyor
  
  error = currentAngle - targetAngle;                                     // hata degerinin hesaplanması
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  
  motorPower = Kp * (error) + Ki * (errorSum) * sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;   // PID kontrolcusu uygulanıyor
  prevAngle = currentAngle;                                                 
 }

// SUBROUTINES

void move(int motor, int hiz, int yon)
{
digitalWrite(STBY, HIGH);                  // standby dan cık, yani motorlar donebilir
if(yon==0){ inPin1 = LOW;  inPin2 = HIGH; } // SAG (00:dönme! 01:sağ 10:sol 11:fren)
if(yon==1){ inPin1 = HIGH; inPin2 = LOW;  } // SOL (00:dönme! 01:sağ 10:sol 11:fren)
if(yon==5){ inPin1 = HIGH; inPin2 = HIGH; } // FREN (00:dönme! 01:sağ 10:sol 11:fren)
// if(0) { ... } // 0:false , parantez icindeki komutlar yapılmaz, if in altından devam
// if(1000) { ... } // 1:true, parantez içi yapılır, bitince if altından devam edilir.
                    // 0 harici tüm rakamlar true dur. (-1,1,10,1000000,-233,...)
if(motor == 1){
digitalWrite(AIN1, inPin1);
digitalWrite(AIN2, inPin2);
analogWrite(PWMA, hiz);
}
if(motor == 2){
digitalWrite(BIN1, inPin1);
digitalWrite(BIN2, inPin2);
analogWrite(PWMB, hiz);
}
}

void ileri(int robothiz){
  move(1, robothiz, 1); //motor 1, full speed, left
  move(2, robothiz, 1); //motor 2, full speed, left
  lcd.setCursor(0, 0); lcd.print("                ");
  lcd.setCursor(0, 0); lcd.print("FORWARD"); 
}

void geri(int robothiz){
  move(1, robothiz, 0); //motor 1, full speed, right
  move(2, robothiz, 0); //motor 2, full speed, right
  lcd.setCursor(0, 0); lcd.print("                ");
  lcd.setCursor(0, 0); lcd.print("BACKWARD"); 
}

void sol(int robothiz){
  move(1, robothiz, 1); //motor 1, full speed, left
  move(2, robothiz, 0); //motor 2, full speed, right
  lcd.setCursor(0, 0); lcd.print("                ");
  lcd.setCursor(0, 0); lcd.print("LEFT"); 
}

void sag(int robothiz){
  move(1, robothiz, 0); //motor 1, full speed, left
  move(2, robothiz, 1); //motor 2, full speed, right
  lcd.setCursor(0, 0); lcd.print("                ");
  lcd.setCursor(0, 0); lcd.print("RIGHT"); 
}

void dur(){
  //move(1, 0, 1); //motor 1, full speed, left
  //move(2, 0, 1); //motor 2, full speed, right
  digitalWrite(STBY, LOW);  // MOTORLARIN İKİSİ DE DURSUN
  lcd.setCursor(0, 0); lcd.print("                ");
  lcd.setCursor(0, 0); lcd.print("STOP"); 
}

void fren(int robothiz){
  move(1, robothiz, 5); //motor 1 fren
  move(2, robothiz, 5); //motor 2 fren
  lcd.setCursor(0, 0); lcd.print("                ");
  lcd.setCursor(0, 0); lcd.print("BRAKE"); 
}

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {     // motor hareketi 
  if (leftMotorSpeed >= 0) {
    move(2, leftMotorSpeed, 0); //motor 1, full speed, left

  }
  else {
    move(2, leftMotorSpeed + 255 , 1); //motor 1, full speed, left
   
  }
  if (rightMotorSpeed >= 0) {
    move(1, rightMotorSpeed, 0); //motor 1, full speed, left
 
  }
  else {
    move(1, rightMotorSpeed + 255, 1); //motor 1, full speed, left
    
  }
}
