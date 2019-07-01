// Arduino Duecemillanove eller hva den nå heter
// Aktiv høy utgang her, motordriver har aktiv lav, men det blir vel det samme?
#include <SPI.h>
#include <SD.h>

File myFile;
boolean isRunning = false;

int ok= 400;

void setup()
{
  // Skrives ut alle på en gang for å spare tid
  pinMode(2, OUTPUT); // X step/pulse
  pinMode(3, OUTPUT); // X dir
  digitalWrite(3, LOW); // X+
  pinMode(4, OUTPUT); // Y step/pulse
  pinMode(5, OUTPUT); // Y dir
  digitalWrite(5, LOW); // Y-
  pinMode(6, OUTPUT); // Z step/pulse
  pinMode(7, OUTPUT); // Z dir
  digitalWrite(7, LOW); // Z-

  // Brytere kjøres rett ut på pinne 2-7 når de aktiveres lav
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
   
  pinMode(9, INPUT_PULLUP); // Start jobb
  pinMode(10, INPUT_PULLUP); // Stopp jobb

  Serial.begin(9600); // info til PC
}

void loop()
{
  // Leser brytere på A0-A5
  byte analogIN = ~PINC; // omformer fra active low to active high

  // Kjører ut A0-A5 på XYZ dir og step pinner hvis trykket
  if ( analogIN )
  {
    // nuller også ut de pinner som ikke fins med shift
    byte stepDirUt = analogIN << 2;

    // Dir forst ut grunnet EasyDriver timing reqirements
    // passer pa aa ikke skrive over Rx Tx
ok=ok+4;
    // Dir ut 
    PORTD = (stepDirUt & 0b10101000) | (PORTD & 0b01010111);
    // Step ut
    PORTD = (stepDirUt & 0b01010100) | (PORTD & 0b10101011);
  }

  // Leser start / stop brytere
  byte swStartStop = ~PINB;  // omformer fra active low to active high
  if ( swStartStop & 0b00000100 ) // Stop CNC run
    isRunning = false;
  else if ( swStartStop & 0b00000010 ) // start CNC run
    if (isRunning == false)  // initier SD
    {
      if (SD.begin(8))
        Serial.println("NC run start");
      delay(2000);
      myFile = SD.open("1.stp");
      if (myFile)
        Serial.println("1.stp opened");
      isRunning = true;
    }

  if ( isRunning )
  {
    if (myFile.available())
    {
      byte s = myFile.read();

      s = s<<2;
      // Dir forst ut grunnet EasyDriver timing reqirements
      PORTD = (s & 0b10101000) | (PORTD & 0b01010111);
      // Step ut
      PORTD = (s & 0b01010100) | (PORTD & 0b10101011);
    }
    else
    {
      isRunning = false;
      myFile.close();
      Serial.println("NC run finish");
    }
  }
//delay(33);
 delayMicroseconds(ok);
  PORTD = B10101011 & PORTD;  // nuller ut x,y,x step pins for å pulse
}
