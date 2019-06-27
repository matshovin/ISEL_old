// Arduino Duemilanove ATmega328
// Aktiv høy utgang her, motordriver har aktiv lav, men det blir vel det samme?

// RING:
// Komm byte: [ checkSum , sisteFrameByte , DirZ , PulsZ, DirY, PulsY, DirX, PulsX ]

// dec ringbuffer(64*2)

// ved start: tøm UART
// send "mer" til PC
//    PC sender bytes
// sett modulo ringFyllePeker til 0
// kopier hele UART 64bytes til ringbuffer, ringFyllePeker = modulo(64) 
// send "mer" til PC
//    PC sender bytes
// kopier hele UART 64bytes til siste del av ringbuffer, ringFyllePeker = modulo(64*2) = 0
// Ringbuffer er nå full
// Sett modulo steppPeker til 63
// send "mer" til PC

// while 
// Hvis SteppUt ikke er for mye større enn ringFyllePeker: stepp ut, Inc modulo steppPeker
// Hvis tilgjengelig og ringFyllePeker er mindre enn steppPeker: les byte fra UART, kopier til ringFyllePeker, inc modulo ringFyllePeker 
// IF sisteFrameByte, send "mer" til PC
// Hvis tilgjengelig og ringFyllePeker er mindre enn steppPeker: les byte fra UART, kopier til ringFyllePeker, inc modulo ringFyllePeker 
// IF sisteFrameByte, send "mer" til PC
// while end



boolean isRunning = false;

void setup()
{
  // Skrives ut alle på en gang for å spare tid
  pinMode(2, OUTPUT); // X step/pulse
  pinMode(3, OUTPUT); // X dir
  pinMode(4, OUTPUT); // Y step/pulse
  pinMode(5, OUTPUT); // Y dir
  pinMode(6, OUTPUT); // Z step/pulse
  pinMode(7, OUTPUT); // Z dir

  // Brytere kjøres rett ut på pinne 2-7 når de aktiveres lav
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
   
  pinMode(9, INPUT_PULLUP); // Start jobb
  pinMode(10, INPUT_PULLUP); // Stopp jobb

  Serial.begin(9600); //  PC
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
      Serial.println("NC run finish");
    }
  }

  delayMicroseconds(666);
  PORTD = B10101011 & PORTD;  // nuller ut x,y,x step pins for å pulse
}
