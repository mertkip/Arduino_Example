///**This project is about a line follower, obstacle avoiding and bluetooth fully controlled robot.**

// Motor Kontrolü ve Bluetooth eklendi. 
// Engelden Kaçma Modu eklendi. 
// Çizgi izleme modu eklendi.
// Modların ayrılması ve seri kanal düzenlemesi. 

//LIBRARIES
#include <QTRSensors.h>

//DEFINEMENTS
int tamhiz = 250;  //Motorun tam hızı
int yarimhiz = 100; //Motorun dönme hızı
int lastError = 0;
#define Kp 0.2 // Trial&Error >> Küçük bir değer ile başlayıp, büyütebilirsiniz en kararlı Kp değerini bulana kadar.. 0.4
#define Kd 2 // Bunu da küçük bir değerden başlatın ve deneyimleyerek büyütün. ( Not: Kp < Kd) 2.0
#define rightMaxSpeed 114
#define leftMaxSpeed 115
#define rightBaseSpeed 85 // robot için kp ve kd değerlerini tutturduysanız şayet motorların dönmesi gereken hız budur.
#define leftBaseSpeed 86 // yukarıdaki değer ile aynıdır

//PORTS
int sagon = 4;    //IN1 //L298N Motor surucu portları
int sagarka = 5;  //IN2
int saghiz = 9;   //ENA  
int solon = 7;    //IN3
int solarka = 6;  //IN4
int solhiz = 10;  //ENB

int mz80 = 3;     //80cm Menzilli Kizilotesi Sensor1
int mz81 = 2;     //80cm Menzilli Kizilotesi Sensor2

//GLOBAL VARIABLES
int gelenVeri;     //seri kanal degiskeni  
int m1,m2;         //Kizilötesi sensor degiskenleri

void setup() {  
  
  pinMode(saghiz, OUTPUT);  // Port's Configurations
  pinMode(solhiz, OUTPUT);
  pinMode(sagon, OUTPUT);
  pinMode(sagarka, OUTPUT);
  pinMode(solon, OUTPUT);
  pinMode(solarka, OUTPUT);
  pinMode(mz80, INPUT);
  pinMode(mz81, INPUT);
  qtr.setTypeAnalog();              // configure the sensors
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);

  Serial.begin(9600); // 9600 baud seri kanal USB+BT
  delay(100);         // Seri kanal a baslaması icin bir süre veriyoruz.

}

void loop() {
  
  while (Serial.available() > 0) {  //Seri kanaldan bilgi geldiği sürece
  gelenVeri = Serial.read();        //Seri kanaldan veri alma

  while (gelenVeri == 'C') {              //Tam Kontrol (Bluetooth) Modu
   int gelenVeri = Serial.read();
  
  if (gelenVeri == 'F') { ileri(); Serial.println("ileri");}      //ileri
  if (gelenVeri == 'B') { geri(); }       //geri
  if (gelenVeri == 'R') { sag();  }       //sag
  if (gelenVeri == 'L') { sol();  }       //sol
  if (gelenVeri == 'S') { dur();  }       //dur
  if (gelenVeri == 'I') { ileri_sag(); }  //ileri sag
  if (gelenVeri == 'G') { ileri_sol(); }  //ileri sol
  if (gelenVeri == 'J') { geri_sag(); }   //geri sag
  if (gelenVeri == 'H') { geri_sol(); }   //geri sol
    if ( gelenVeri == 'A') { break; }
  }
  
  while (gelenVeri == 'A') {              //Engelden Kaçma Modu
    m1 = digitalRead(mz80);               //mz80'den veri alma(sol)
    m2 = digitalRead(mz81);               //mz80'den veri alma(sag)
    engel();                              //engel() fonksiyonunu devreye al  
    
    int gelenVeri = Serial.read();            //Seri kanaldan veri alma
    delay(500);
    if ( !gelenVeri == 'A') { break; }
  }
    
      while (gelenVeri == 'T') {              //Çizgi izleme Modu
    linetrack()
    
    int gelenVeri = Serial.read();            //Seri kanaldan veri alma
    delay(500);
    if ( !gelenVeri == 'T') { break; }
  }
 }
}

// SUBROUTINES
void ileri() {
  digitalWrite(sagon, HIGH);
  digitalWrite(sagarka, LOW);
  analogWrite(saghiz, tamhiz);

  digitalWrite(solon, HIGH);
  digitalWrite(solarka, LOW);
  analogWrite(solhiz, tamhiz);
}

void geri() {
  digitalWrite(sagon, LOW);
  digitalWrite(sagarka, HIGH);
  analogWrite(saghiz, tamhiz);

  digitalWrite(solon, LOW);
  digitalWrite(solarka, HIGH);
  analogWrite(solhiz, tamhiz);
}

void sag() {
  digitalWrite(sagon, LOW);
  digitalWrite(sagarka, LOW);
  analogWrite(saghiz, 0);

  digitalWrite(solon, HIGH);
  digitalWrite(solarka, LOW);
  analogWrite(solhiz, tamhiz);
}

void sol() {
  digitalWrite(sagon, HIGH);
  digitalWrite(sagarka, LOW);
  analogWrite(saghiz, tamhiz);

  digitalWrite(solon, LOW);
  digitalWrite(solarka, LOW);
  analogWrite(solhiz, 0);
}

void dur() {
  digitalWrite(sagon, LOW);
  digitalWrite(sagarka, LOW);
  analogWrite(saghiz, 0);

  digitalWrite(solon, LOW);
  digitalWrite(solarka, LOW);
  analogWrite(solhiz, 0);
}

void ileri_sag() {
  digitalWrite(sagon, HIGH);
  digitalWrite(sagarka, LOW);
  analogWrite(saghiz, yarimhiz);

  digitalWrite(solon, HIGH);
  digitalWrite(solarka, LOW);
  analogWrite(solhiz, tamhiz);
}

void ileri_sol() {
  digitalWrite(sagon, HIGH);
  digitalWrite(sagarka, LOW);
  analogWrite(saghiz, tamhiz);

  digitalWrite(solon, HIGH);
  digitalWrite(solarka, LOW);
  analogWrite(solhiz, yarimhiz);
}

void geri_sag() {
  digitalWrite(sagon, LOW);
  digitalWrite(sagarka, HIGH);
  analogWrite(saghiz, yarimhiz);

  digitalWrite(solon, LOW);
  digitalWrite(solarka, HIGH);
  analogWrite(solhiz, tamhiz);
}

void geri_sol() {
  digitalWrite(sagon, LOW);
  digitalWrite(sagarka, HIGH);
  analogWrite(saghiz, tamhiz);

  digitalWrite(solon, LOW);
  digitalWrite(solarka, HIGH);
  analogWrite(solhiz, yarimhiz);
}

void engel() {        //Engelden Kaçma Fonksiyonu
    if(m1 == 1 && m2 == 1){
        ileri(); // Engel yoksa ileri gider
        Serial.println("Engel Yok");
      }
      
    else if(m1 == 0){ // engel var ise sol 
        sol();
        Serial.println("Sağda Engel Var");
      }
      
    else if(m2 == 0){ // engel varsa sağ
        sag();
        Serial.println("Solda Engel Var");
      }
}

void line_track() {   // Çizgi İzleyen Fonksiyonu
 
  unsigned int position = qtr.readLineBlack(sensorValues);         //SENSOR KOYU RENKLERİ OKUYOR. KONUM BİLGİSİNE ATANIYOR.

  //*  CLOSED LOOP *//
  int error = position - 3000;                                     // Kapalı döngü hatası girdi olarak alınıyor.
  int motorSpeed = Kp * error + Kd * (error - lastError);          // PD Kontrolcü Fonksiyonu
  lastError = error;                                               // Son Hata
  int rightMotorSpeed = rightBaseSpeed + motorSpeed;          
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  
    digitalWrite(solon, HIGH);  
    digitalWrite(sagarka, LOW);
    analogWrite(solhiz, leftMotorSpeed);

    digitalWrite(sagon, HIGH);
    digitalWrite(sagarka, LOW);
    analogWrite(saghiz, rightMotorSpeed);
  
  m1 = digitalRead(mz80);               //mz80'den veri alma(sol)///////////////
  m2 = digitalRead(mz81);               //mz80'den veri alma(sag)
  if (m1 == 0 || m2 == 0) { dur(); }
}

