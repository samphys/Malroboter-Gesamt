//-------------------------------------------------
//SA 1904/05 Malroboter
//Programm für Malroboter
//Erstellt: 02.12.2019
//Ersteller: Samuel Jenny, Daniel Schnider
//Für Arduino UNO
//-------------------------------------------------
/*
 Diese Software wurde im Rahmen einer Semesterarbeit von Maschinenbau Studenten an der HSR am Institut ILT entwickelt. 
 Sie bedient follgende Funktionen:
 -Komunikation mit der Steuereinheit
 -Ansteuerung der Antriebsmotoren
 -Das Wechselnd der aktiven Farbdüse
 -Anheben der Düsen
 -Farbförderung
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Inizialisierung der i2c verbindung mit den Shields
Adafruit_MotorShield AFMS0(0x60); // No jumper closed
Adafruit_MotorShield AFMS1(0x61); // Rightmost jumper closed
Adafruit_MotorShield AFMS2(0x62); // Second to Rightmost jumper closed

//Zuweisung der Motorplätze der Shields an Variablen
Adafruit_DCMotor *P1 = AFMS0.getMotor(1);  // Pumpe 1
Adafruit_DCMotor *P2 = AFMS0.getMotor(2);  // Pumpe 2
Adafruit_DCMotor *P3 = AFMS0.getMotor(3);  // Pumpe 3
Adafruit_DCMotor *P4 = AFMS0.getMotor(4);  // Pumpe 4
Adafruit_DCMotor *P5 = AFMS1.getMotor(1);  // Pumpe 5
Adafruit_DCMotor *P6 = AFMS1.getMotor(2);  // Pumpe 6
Adafruit_DCMotor *WMot = AFMS1.getMotor(3);  // Motor des wechslers
Adafruit_DCMotor *MotorL = AFMS2.getMotor(1);  // Linker Antrieb
Adafruit_DCMotor *MotorR = AFMS2.getMotor(2);  // Rechter Antrieb

Servo servo1;   // erstellt servo1 als Servoobjekt

//Globale Variablen
int duese = 1;   // nummer der gewünschten Düse
int servoheben = 60;  // winkel des eingefahrenen servos
int servosenken = 120;  // winkel des ausgefahrenen servos
int Spos = 120;  // position des Servos
int Ewert[] = {80, 100, 200, 300, 400, 500, 600}; // encoder werte zu den düsenpositionen. das erst ist 0 damit düsenposition mit array nummer übereinstimmte da arrays mit 0 starten.
volatile int encoder;     // zählvariabel der encoderschritte
volatile boolean drehrichtung;    // drehrichtung des Wechslers. 0 = forwärts, 1 = rückwärts
boolean endtaster = 0;   // endtaster um die Position des Düsenwechslers zu definieren
int duesenstand = 0;  // zum vergleich gewählter düse mit aktiver
boolean farbON = 0;  // position des schalters zur aktivierung der farbauftragung. 0 = off, 1 = on
int farbMenge = 0;  // (0...255) bestimmt die geschwindigkeit der pumpen und somit die Farbmenge
int fmf = 1;  // Farbmengenfaktor zur regulierung der Farbmengenberechnung. PLATZHALTER. WERT MUSS NOCH EMPIRISCH ERMITTELT WERDEN 
int MotL;  // geschwindigkeit des Linken Antriebsmotors (0...255)
int MotR;  // geschwindigkeit des Rechten Antriebsmotors (0...255)
int Form; // gewünschte Form. 0 = Fahren mit Joystick, 1 = Quadrat, 2 = Kreis
int drMotL;  // drehrichtung des Linken Antriebsmotors. 2 = released, 1 = forwärts, 0 = rückwärts
int drMotR;  // drehrichtung des Rechten Antriebsmotors. 2 = released, 1 = forwärts, 0 = rückwärts
int vd;  // geschwindigkeit an der düse
boolean reinigungON = 0;  // löst den reinigungszyklus aus. 0 = off, 1 = on
int reinigungszyklen = 3;  // setzt anzahl der reinigungszyklen fest, welche bei aufruf der reinigungsfunktion durchgeführt werden
int spueldauer = 3000;  // dauer der spühlung einer einzelnen düse im reinigungsmodus (zeit in ms)
boolean wechslerOben;  // zeigt ob wechsler angehoben ist. 0 = unten, 1 = oben
int tFarbrueckzug = 2000;  // dauer des farbrückzuges bei düsenwechseln in ms

//Pins für nRF24L01 Kommunikation

int pin_CSN = 7;
int pin_CE = 8;
int pin_MOSI = 11;
int pin_MISO = 12;
int pin_SCK = 13;


RF24 radio(pin_CE,pin_CSN);    //Pin definition CE,CSN

const byte addresse[6] = "00001";      //Adresse für die Kommunikation, Nummer frei wählbar "00000"-"99999", Wichtig!-> Sender und Empfänger selbe Nummer

int empfangen[8] = {0,0,0,0,0,0,0,0};

void setup() 
{
  Serial.begin(9600);  // serial gedöns nur zu testzwecken
  Serial.println("setup gestartet");
  
  AFMS0.begin(); // Startet das untere Shield
  AFMS1.begin(); // Startet das mittlere Shield
  AFMS2.begin(); // Startet das obere Shield

  P1->run(RELEASE); // modus der Pumpe definieren
  P1->setSpeed(0); // geschwindigkeit auf 0 setzen (0...255). Wobei werte kleiner als 20 aus mechanischen gründen nicht funktionieren
  P2->run(RELEASE); // modus der Pumpe definieren
  P2->setSpeed(0); // geschwindigkeit auf 0 setzen (0...255). Wobei werte kleiner als 20 aus mechanischen gründen nicht funktionieren
  P3->run(RELEASE); // modus der Pumpe definieren
  P3->setSpeed(0); // geschwindigkeit auf 0 setzen (0...255). Wobei werte kleiner als 20 aus mechanischen gründen nicht funktionieren
  P4->run(RELEASE); // modus der Pumpe definieren
  P4->setSpeed(0); // geschwindigkeit auf 0 setzen (0...255). Wobei werte kleiner als 20 aus mechanischen gründen nicht funktionieren
  P5->run(RELEASE); // modus der Pumpe definieren
  P5->setSpeed(0); // geschwindigkeit auf 0 setzen (0...255). Wobei werte kleiner als 20 aus mechanischen gründen nicht funktionieren
  P6->run(RELEASE); // modus der Pumpe definieren
  P6->setSpeed(0); // geschwindigkeit auf 0 setzen (0...255). Wobei werte kleiner als 20 aus mechanischen gründen nicht funktionieren
  MotorR->run(RELEASE); // modus des Antriebs definieren
  MotorR->setSpeed(0); // geschwindigkeit auf 0 setzen (0...255). Wobei werte kleiner als 20 aus mechanischen gründen nicht funktionieren
  MotorL->run(RELEASE); // modus des Antriebs definieren
  MotorL->setSpeed(0); // geschwindigkeit auf 0 setzen (0...255). Wobei werte kleiner als 20 aus mechanischen gründen nicht funktionieren
  WMot->run(RELEASE); // modus des motors definieren
  WMot->setSpeed(40);  // geschwindigkeit setzen (0...255). Wobei werte kleiner als 20 aus mechanischen gründen nicht funktionieren
  servo1.attach(10);  // pin 10 gehört zu servo 1 auf shield
  wechslerHeben();  // setzt servo1 auf grundposition
  
  attachInterrupt(digitalPinToInterrupt(2), counter, RISING);  // definiert pin 2 als hardware interrupt. getrigerrt mit einer steigenden flanke(RISING) wird die funktion counter aufgerufen.

  pinMode(3, INPUT);  // anschluss des Endtasters
  pinMode(6, OUTPUT);  // Status LED

  digitalWrite(6, HIGH);  // setzt Status LED auf High

  nullpunkt();  // aufruf der nullpunktfunktion des wechslers

//Setup WiFi Kommunikation
  radio.begin();
  radio.openReadingPipe(1,addresse[0]);   //Empfangen auf "00001"
  radio.setPALevel(RF24_PA_MIN);     // Power Amplifier level {RF24_PA_MIN ,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX}
  radio.startListening();


  Serial.println("setup abgeschlossen");
}


void nullpunkt()  // fährt den nullpunkt des wechslers an und setzt die position
{
 Serial.println("nullpunkt gestartet");
 MotAus(); 
 PAus();
 wechslerHeben();
    
 while (endtaster != 1)  // fährt den Wechsler an den Endschalter um die Position zu bestimmen
  {
   drehrichtung = 1;
   WMot->run(BACKWARD);
   endtaster = digitalRead(3);
   encoder = 60;  // setzt den encoder auf den Wert der endschalterposition. WERT NOCH PLATZHALTER. MUSS NOCH BESTIMMT WERDEN
  }
  
 WMot->run(RELEASE);

 wechslerSenken();

 Serial.println("nullpunkt abgeschlossen");
 
}


void counter()  // Interrupt funktion um die schritte des Wechslermotors zu zählen zur positionsbestimmung
{
  if (drehrichtung == 0) encoder++;
  if (drehrichtung == 1) encoder--;

  Serial.println(encoder);
  Serial.println(duese);
}


void dueseWechseln()  // wechselt die düse
{
 Serial.println("dueseWechseln gestartet");
 MotAus();
 farbrueckzug();  
 PAus();
 wechslerHeben();

 while (Ewert[duese] > encoder)     // dreht wechsler vorwärts
  {
   drehrichtung = 0;
   WMot->run(FORWARD);
  }
  
 while (Ewert[duese] < encoder)     // dreht wechsler rückwärts
  {
   drehrichtung = 1;
   WMot->run(BACKWARD);
  }
  
 WMot->run(RELEASE);                // stopt den wechsler
   
 wechslerSenken();
 duesenstand = duese;
 farbvorschub();

 Serial.println("dueseWechseln abgeschlossen");
}


void malen()  // führ die Farbauftragung aus
{
  Serial.println("malen");
  
  wechslerSenken();
  farbMenge =  v()*fmf;
  
  switch (duese)
   {
    case 1:
     P1->setSpeed(farbMenge);
     P1->run(FORWARD);
     break;
    case 2:
     P2->setSpeed(farbMenge);
     P2->run(FORWARD);
     break;
    case 3:
     P3->setSpeed(farbMenge);
     P3->run(FORWARD);
     break;
    case 4:
     P4->setSpeed(farbMenge);
     P4->run(FORWARD);
     break;
    case 5:
     P5->setSpeed(farbMenge);
     P5->run(FORWARD);
     break;
    case 6:
     P6->setSpeed(farbMenge);
     P6->run(FORWARD);
     break;
   }
 // Serial.println("malen abgeschlossen");
}


void farbrueckzug()  // zieht farbe aus der düse zurück um auslaufen beim nichtgebrauch zu verhindern
{
  Serial.println("farbrueckzug");
  
  switch (duesenstand)
   {
    case 1:
     P1->setSpeed(250);
     P1->run(BACKWARD);
     break;
    case 2:
     P2->setSpeed(250);
     P2->run(BACKWARD);
     break;
    case 3:
     P3->setSpeed(250);
     P3->run(BACKWARD);
     break;
    case 4:
     P4->setSpeed(250);
     P4->run(BACKWARD);
     break;
    case 5:
     P5->setSpeed(250);
     P5->run(BACKWARD);
     break;
    case 6:
     P6->setSpeed(250);
     P6->run(BACKWARD);
     break;
   }
  delay(tFarbrueckzug);
  PAus();
   
  //Serial.println("farbrueckzug abgeschlossen");
}


void farbvorschub()  // drückt die farbe nachdem wechslen wieder an die düsenspitze, damit direkt wieder voll gemalt werden kann
{
  Serial.println("farbvorschub");
  
  switch (duese)
   {
    case 1:
     P1->setSpeed(125);
     P1->run(FORWARD);
     break;
    case 2:
     P2->setSpeed(125);
     P2->run(FORWARD);
     break;
    case 3:
     P3->setSpeed(125);
     P3->run(FORWARD);
     break;
    case 4:
     P4->setSpeed(125);
     P4->run(FORWARD);
     break;
    case 5:
     P5->setSpeed(125);
     P5->run(FORWARD);
     break;
    case 6:
     P6->setSpeed(125);
     P6->run(FORWARD);
     break;
   }
  delay(2*tFarbrueckzug);
  PAus();
   
  //Serial.println("farbvorschub abgeschlossen");
}


void wechslerHeben()  // hebt den Wechsler an
{
 PAus();
 while (Spos > servoheben) // fährt auf 20 grad in 1 grad schritten um Wechsler anzuheben  
  {
  Spos--; 
  servo1.write(Spos);  
  delay(15);  // verfahrzeit            
  }
 wechslerOben = 1;
}


void wechslerSenken()  // senkt den Wechsler in malposition
{
 wechslerOben = 0;
 PAus();
 while (Spos < servosenken) // fährt 120 grad in 1 grad schritten um Wechsler abzusenken
  { 
   Spos++;
   servo1.write(Spos);
   delay(15);              
  } 
}


int v()  // geschwindigkeit an der düse
{
 if (drMotL == 1 && drMotR == 1) vd = (MotL + MotR)/2;
 if (drMotL == 1 && drMotR == 0) vd = (MotL - MotR)/2;
 if (drMotL == 0 && drMotR == 1) vd = (MotR - MotL)/2;
 if (drMotL == 0 && drMotR == 0) vd = (-MotR - MotL)/2;
 return(vd);
}


void reinigung()  // reinigt alle düsen "reinigungszyklen"-mal jeweils für "spueldauer" ms 
{
 Serial.println("reinigung gestartet");
 
 MotAus(); 
 duese = 1;
 dueseWechseln();
  
 for (int i = 0; i < reinigungszyklen*6; i++)
 {
  if (duese <= 5)  //von duese 1 bis 6 durchwechseln
  {
   switch (duese)
   {
    case 1:
     P1->setSpeed(250);
     P1->run(FORWARD);
     break;
    case 2:
     P2->setSpeed(250);
     P2->run(FORWARD);
     break;
    case 3:
     P3->setSpeed(250);
     P3->run(FORWARD);
     break;
    case 4:
     P4->setSpeed(250);
     P4->run(FORWARD);
     break;
    case 5:
     P5->setSpeed(250);
     P5->run(FORWARD);
     break;
    case 6:
     P6->setSpeed(250);
     P6->run(FORWARD);
     break;
   }
   delay(spueldauer);
   PAus();
   dueseWechseln();
   duese++;
  } 
 }
 Serial.println("reinigung abgeschlossen");
}


void PAus()  // schaltet alle pumpen aus
{
 P1->run(RELEASE);
 P2->run(RELEASE);
 P3->run(RELEASE);
 P4->run(RELEASE);
 P5->run(RELEASE);
 P6->run(RELEASE);
}

void WiFi_Empfangen(){
  
  if (radio.available()){
    radio.read(&empfangen, sizeof(empfangen));

  Serial.println(empfangen[0]);
  Serial.println(empfangen[1]);
  Serial.println(empfangen[2]);
  Serial.println(empfangen[3]);
  Serial.println(empfangen[4]);
  Serial.println(empfangen[5]);
  Serial.println(empfangen[6]);

//  Serial.println(empfangen[0]);
//  Serial.println(empfangen[1]);
//  Serial.println(empfangen[2]);
//  Serial.println(empfangen[3]);
//  Serial.println(empfangen[4]);
//  Serial.println(empfangen[5]);
//  Serial.println(empfangen[6]);

  Serial.println();
  
  }
  duese = empfangen[0];
  farbON = empfangen[1];
  reinigungON = empfangen[2];
  MotR = empfangen[3];
  MotL = empfangen[4];
  drMotR = empfangen[5];
  drMotL = empfangen[6];
  Form = empfangen[7];
  
}



void Joystick_Fahren()
{
  if (farbON == 1) MotorR->setSpeed(MotR/2);  // verringert geschwindigkeit zum malen
  else MotorR->setSpeed(MotR);
  

  if(empfangen[5]==0){
    MotorR->run(BACKWARD);
  }
  else if(empfangen[5]==1){
    MotorR->run(FORWARD);
  }

  if (farbON == 1) MotorL->setSpeed(MotL/2);  // verringert geschwindigkeit zum malen
  else MotorL->setSpeed(MotL);
 
  if(empfangen[6]==0){
    MotorL->run(BACKWARD);
  }
  else if(empfangen[6]==1){
    MotorL->run(FORWARD);
  }
}

void Quadrat_Fahren(){
  int Geschwindigkeit_Linie = 80;
  int Geschwindigkeit_Rotation = 20;
  for(int i = 0; i < 4; i++){
    MotorR->run(FORWARD);
    MotorL->run(FORWARD);
    MotorR->setSpeed(Geschwindigkeit_Linie);
    MotorL->setSpeed(Geschwindigkeit_Linie);
    delay(3000);
    MotorR->run(FORWARD);
    MotorL->run(BACKWARD);
    MotorR->setSpeed(Geschwindigkeit_Rotation);
    MotorL->setSpeed(Geschwindigkeit_Rotation);
    delay(1000);
  }
  
}

void Kreis_Fahren(){
  int Geschwindigkeit_R = 80;
  int Geschwindigkeit_L = 50;
  MotorR->run(FORWARD);
  MotorL->run(FORWARD);
  MotorR->setSpeed(Geschwindigkeit_R);
  MotorL->setSpeed(Geschwindigkeit_L);
  delay(10000);
}



void MotAus()  // setzt die geschwindigkeit der Bewegungsmotoren auf 0
{
 MotorL->setSpeed(0);
 MotorR->setSpeed(0);

 Serial.println("MotAus");
}



void loop() 
{
 WiFi_Empfangen();

 Serial.println(vd);

 if (v() == 0) PAus();  // stellt sicher das bei stillstand keine farbe gepumpt wird
  

 if (duesenstand != duese) dueseWechseln();  // kontroliert düsenposition
 
 if (v() < 0 || farbON != 1) wechslerHeben();  // hebt die düsen bei rückwärtsfahrt oder nichtgebrauch
 
 if (duesenstand == duese && farbON == 1 && v() > 0) malen();  // kontroliert die bedingungen um farbe aufzutragen
 
 if (reinigungON == 1) reinigung();  // löst den reinigungszyklus aus


 switch (Form){
  case 0:
  Joystick_Fahren();
  break;
  case 1:
  Quadrat_Fahren();
  break;
  case 2:
  Kreis_Fahren();
  break;
 }

 delay(15);
 

}
