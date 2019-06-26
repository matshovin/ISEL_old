#include <SPI.h>
#include <SD.h>

File myFile;
boolean isRunning = false;

void setup()
{
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);

  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);

  Serial.begin(9600);
}

void loop()
{
  // Leser brytere på A0-A5
  byte analogIN = ~PINC; // active low to active high

  // Kjører ut A0-A5 på XYZ dir og step pinner hvis trykket
  if ( analogIN )
  {
    // nuller også ut de pinner som ikke fins med shift

    byte stepDirUt = analogIN << 2;

    // Dir forst ut grunnet EasyDriver timing reqirements
    // passer pa aa ikke skrive over Rx Tx

    PORTD = (stepDirUt & 0b10101000) | (PORTD & 0b01010111);
    // Step ut
    PORTD = (stepDirUt & 0b01010100) | (PORTD & 0b10101011);
  }

  // Leser start / stop brytere
  byte swStartStop = ~PINB;  // active low to active high
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
        Serial.println("0.stp opened");
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

  delayMicroseconds(666);
  PORTD = B10101011 & PORTD;  // nuller ut x,y,x step pins
}
