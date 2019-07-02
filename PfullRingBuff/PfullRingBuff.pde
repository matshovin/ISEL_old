

boolean plott3D = false;
boolean plott3Di = true;

String[] gCodeArrayOnlyLinesOfG01;      // Ren G01 G code filtrert fra fil
double[][] lineArrayXYZT;               // [Line no][X Y Z endtime] (mm)(sec)
double[][] interpolPointArray;          // [Step no][X Y Z] (mm)
byte[] motorByteArray;                  // Motor control byte, 
// bit format: [notUsed notUsed dirZ stepZ dirY stepY dirX stepX]

double stepFrequency = 1500.0;          // loop frequency in Arduino (Hz)

double numbOfMicrostep = 4.0;                       // Can be set on motor driver (org8)                         
double stepSizeInMM = 4/(200.0*numbOfMicrostep);    // Screw pitch (mm/revolution) / 
// (motor step per revolution * numbOfMicrostep) (0.005) (org 2.5)

// Max possible linear speed = stepFrequency * stepSizeInMM - 7.5mm/sec
// Motor rotation speed = stepFrequency / (numbOfMicrostep * 200)
String gCodeFileName = "../NC/0.nc";
String stepFileName = "../1.stp";
float speedScale = 0.0003;              // typisk fra mm/min i F360 til mm/sec som er formatet i ROBIN CNC driver

float cameraXcent = 0; // punkt som camera rettes mot
float cameraYcent = 0;
float cameraZcent = 0;

float cameraAngHor = 1;
float cameraAngVert = 1;
float cameraDist = 2*50; // funker bare i camera perspective

float cameraX; // camera i dette punkt
float cameraY;
float cameraZ;

void setup()
{
  size(600, 500, P3D);
  frameRate(6); 
  background(11); 
  cameraX = cameraDist*cos(cameraAngHor)*cos(cameraAngVert) + cameraXcent; 
  cameraY = cameraDist*sin(cameraAngHor)*cos(cameraAngVert) + cameraYcent;    
  cameraZ = cameraDist*sin(cameraAngVert) + cameraZcent; 
  println(""+cameraAngVert);

  lesFil_FiltrerTil_gCodeArrayOnlyLinesOfG01(gCodeFileName);

  parse_gCodeArrayOnlyLinesOfG01_til_lineArrayXYZT(); 

  int totalNoOfSteps = (int)( lineArrayXYZT[lineArrayXYZT.length-1][3] * stepFrequency ); 
  println("Total no of steps = " + totalNoOfSteps);    
  System.out.printf("Total run time = %.2f sec / %.2f min \n", lineArrayXYZT[lineArrayXYZT.length-1][3], lineArrayXYZT[lineArrayXYZT.length-1][3]/60.0);
  println("Step size = " + stepSizeInMM + " (mm)");
  println("Step frequency = " + stepFrequency + " (Hz)");
  println("Max possible linear motion speed = " + (stepFrequency * stepSizeInMM) + " (mm/sec)");
  println("Max possible motor rotational speed = " + stepFrequency / (numbOfMicrostep * 200.0) + " (rev/sec)");

  interpolPointArray = new double[totalNoOfSteps][3];

  parse_lineArrayXYZT_to_interpolPointArray();
  parse_interpolPointArray_to_motorByteArray();  

  //print_gCodeArrayOnlyLinesOfG01(0,10); // debug
  //print_interpolPointArray(0,100); // debug
  //printMotorByteArray(); // debug
  //printEtc1(0, 100); // debug  
  printAll(0.54, 0.56); // debug
  //print_lineArrayXYZT(0, 30); // debug
  saveBytes(stepFileName, motorByteArray);
  println("File: " + stepFileName + " saved");
}

//***********************************************************************

void draw()
{
  // Lys
  lights();
  float dirY = (mouseY / float(height) - 0.5) * 2;
  float dirX = (mouseX / float(width) - 0.5) * 2;
  directionalLight(204, 204, 204, -dirX, -dirY, -1);
  spotLight(111, 111, 111, 111, 111, 111, -111, -111, -111, PI/2, 600); 
  lightSpecular(.2, .2, .8);
  background(11);  

  camera(cameraX, cameraY, cameraZ, cameraXcent, cameraYcent, cameraZcent, 0, 0, -1);
  perspective(PI/3.0, (float)width/height, 1, 100000);

  if (plott3D == true)
  {
    strokeWeight(1);
    stroke(255, 255, 255); 
    for (int i=0; i<lineArrayXYZT.length-1; i++) // lineArrayXYZT.length-2;
      line( (float)lineArrayXYZT[i][0], (float)lineArrayXYZT[i][1], (float)lineArrayXYZT[i][2], 
        (float)lineArrayXYZT[i+1][0], (float)lineArrayXYZT[i+1][1], (float)lineArrayXYZT[i+1][2] );
  }

  if (plott3Di == true)
  {
    strokeWeight(1);
    stroke(255, 255, 255); 
    for (int i=0; i<interpolPointArray.length-101; i=i+100) // lineArrayXYZT.length-2;
      line( (float)interpolPointArray[i][0], (float)interpolPointArray[i][1], (float)interpolPointArray[i][2], 
        (float)interpolPointArray[i+100][0], (float)interpolPointArray[i+100][1], (float)interpolPointArray[i+100][2] );
  }  

  strokeWeight(2);
  stroke(0, 0, 111);    
  line(0, 0, 0, 30, 0, 0); // blaa : x axe, pluss retn
  stroke(111, 0, 0);
  line(0, 0, 0, 0, 30, 0); // rod : y axe, pluss retn
  stroke(111, 111, 111);
  line(0, 0, 0, 0, 0, 30); // gul : z axe, pluss retn
}
