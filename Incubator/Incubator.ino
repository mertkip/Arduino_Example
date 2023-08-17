///**This project is about a DIY incubator.**

#include <DHT.h>
#include <LiquidCrystal.h>
#define DHTPIN 8
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
int rolePin = 9;

void setup() {
  pinMode(rolePin,OUTPUT);
  
  Serial.begin(9600);
  lcd.begin(16,2);
  dht.begin();
  
  for (int i=0; i < 3; i++) {
  lcd.setCursor(0, 0); lcd.print("Kulucka Makinesi"); lcd.setCursor(0, 1); lcd.print(" Mert KIP "); //KOLON,SATIR
  delay(1000);
  lcd.setCursor(0, 0); lcd.print("                "); lcd.setCursor(0, 1); lcd.print("                "); //KOLON,SATIR
  delay(500); }  
}

void loop() {
  int temp = dht.readTemperature();
  int hum = dht.readHumidity();

  int esikdegeri = analogRead(A0);
  int esik = map(esikdegeri, 0, 1023, 0, 100);
  Serial.println(esik);
  delay(500);
  
  lcd.setCursor(0,0);
  lcd.print("Sicak:");
  lcd.print(temp);
  lcd.print("'C");
  lcd.print(" (KIP)");

  lcd.setCursor(0,1);
  lcd.print("Nem:%");
  lcd.print(hum);
  lcd.print(" Ayar:%");
  lcd.print(esik);
  lcd.print(" "); 
  delay(2000);

  if(hum <= esik){digitalWrite(rolePin, LOW);}
  else{digitalWrite(rolePin, HIGH);}
}
