// 1)    Filtrer fra fil inn i gPureshort G01 kode String array
// 2)    Parse the string array gPureshort to "gLineArray", format: X Y Z T, line end positions (mm) and line endtime (sec)
// 3)    Calculate numb of CNC steps from max T
// 4)    Declare "interpArray" (interpolated XYZ values (mm)), format: X Y Z 
// 5)    Iterate all CNC steps and find corresponding G line, generate line equation and interpolation point
// 6)    Declare "motorByteArray" and generate motor control bytes from all interpolation points in "interpArray"
// 7)    Save motorByteArray to file "1.stp"

// G.code file format for all lines:  G01 X1.232 Y100.0 Z-33.12 F15  (X,Y,Z in mm, F in mm/sec)
// Tool tip is manually located i 0,0,0 (WCS) first

String[] gPureshort;                    // Ren G01 G code filtrert fra fil
double[][] gLineArray;                  // [G line no][X Y Z endtime] (mm)(sec)
double[][] interpArray;                 // [step no][X Y Z] (mm)
byte[] motorByteArray;                  // Motor control byte, 
                                        // bit format: [notUsed notUsed dirZ stepZ dirY stepY dirX stepX]

double stepFrequency = 1500.0;          // loop frequency in Arduino (Hz)
                                   
double numbOfMicrostep = 4.0;                       // Can be set on motor driver (org8)                         
double stepSizeInMM = 4/(200.0*numbOfMicrostep);  // Screw pitch (mm/revolution) / 
                                                    // (motor step per revolution * numbOfMicrostep) (0.005) (org 2.5)

                                        // Max possible linear speed = stepFrequency * stepSizeInMM - 7.5mm/sec
                                        // Motor rotation speed = stepFrequency / (numbOfMicrostep * 200)
String gCodeFileName = "../NC/0.nc";
float speedScale = 0.0003; // typisk fra mm/min i F360 til mm/sec som er formatet i ROBIN CNC driver

void setup()
{
    filtrerFilTilGcodeArrayAvRenG01(gCodeFileName);
  
    readAndParseGcodeFileTo_gLineArray(); // X Y Z T

    int totalNoOfSteps = (int)( gLineArray[gLineArray.length-1][3] * stepFrequency ); 
    println("Total no of steps = " + totalNoOfSteps);    
    System.out.printf("Total run time = %.2f sec / %.2f min \n", gLineArray[gLineArray.length-1][3], gLineArray[gLineArray.length-1][3]/60.0);
    println("Step size = " + stepSizeInMM + " (mm)");
    println("Step frequency = " + stepFrequency + " (Hz)");
    println("Max possible linear motion speed = " + (stepFrequency * stepSizeInMM) + " (mm/sec)");
    println("Max possible motor rotational speed = " + stepFrequency / (numbOfMicrostep * 200.0) + " (rev/sec)");

    interpArray = new double[totalNoOfSteps][3];

    parseToInterpArray();
    parseTomotorByteArray();  

    //printgLineArray(); // debug
    //printInterpArray(); // debug
    //printMotorByteArray(); // debug
    printEtc(74763,74765); // debug
    printgLineArray(0,3); // debug
}

//***********************************************************************
//***********************************************************************
void readAndParseGcodeFileTo_gLineArray()
{
    double maxF = 0; // Vil finne maxF, bare en sikkerhets skjekk

    gLineArray = new double[gPureshort.length][4];

    double lineLengthMM; // line length for each G01 line in mm

    for (int L=0; L < gPureshort.length; L++) // løper alle linjer i "1.nc"
    {
        gLineArray[L][0] = (finnTallVerdiFraLinje("X", gPureshort[L]) ); // setter inn X verdi
        gLineArray[L][1] = (finnTallVerdiFraLinje("Y", gPureshort[L]) ); // setter inn Y verdi
        gLineArray[L][2] = (finnTallVerdiFraLinje("Z", gPureshort[L]) ); // setter inn Z verdi

        double F = finnTallVerdiFraLinje("F", gPureshort[L]); // hastighet i mm/sec
        if (F>maxF)
            maxF = F;

        if (L == 0) // maskin står i 0,0,0  forst
        {
            lineLengthMM = Math.sqrt( (gLineArray[L][0] - 0) * (gLineArray[L][0] - 0) +
                (gLineArray[L][1] - 0) * (gLineArray[L][1] - 0) +
                (gLineArray[L][2] - 0) * (gLineArray[L][2] - 0) );

            gLineArray[L][3] = lineLengthMM / F; // setter inn slutt tid
        } else
        {
            lineLengthMM = Math.sqrt(
                (gLineArray[L][0] - gLineArray[L-1][0]) * (gLineArray[L][0] - gLineArray[L-1][0]) +
                (gLineArray[L][1] - gLineArray[L-1][1]) * (gLineArray[L][1] - gLineArray[L-1][1]) +
                (gLineArray[L][2] - gLineArray[L-1][2]) * (gLineArray[L][2] - gLineArray[L-1][2]) );

            gLineArray[L][3] = (lineLengthMM / F) + gLineArray[L-1][3]; // setter inn slutt tid
        }
    }   
    System.out.printf("Max F found in file =  %.2f mm/sec \n", maxF);
}

//***********************************************************************
void parseToInterpArray()
{
    // parsing from gLineArray (short array) to interpArray (long array)
    
    int gCodeLineNo = 0; // (første linjesegment)

    // running over each index in interpArray (long array)
    for (int index=0; index<interpArray.length; index++)
    {
        double currentTime = index * (1.0/stepFrequency);

        if (gCodeLineNo == 0) // (første linjesegment)
        {
            double[] startCoord = {(double)0, (double)0, (double)0};
            interpArray[index] = interpolateSinglePoint(currentTime,
                               startCoord, 0,
                               gLineArray[gCodeLineNo], gLineArray[gCodeLineNo][3]);
        } else // (gjeldende linjesegment current punkt ligger på underveis)
        {
            interpArray[index] = interpolateSinglePoint(currentTime,
                               gLineArray[gCodeLineNo-1], gLineArray[gCodeLineNo-1][3],
                               gLineArray[gCodeLineNo], gLineArray[gCodeLineNo][3]);
        }
        
        while (currentTime > gLineArray[gCodeLineNo][3]) // laster inn ny gCodeLinje (linjesegment)
            gCodeLineNo++;
    }
}

//***********************************************************************
double[] interpolateSinglePoint(double currentTime,double[] startCoord, double startTime,double[] endCoord, double endTime)
{
    // equation for y axis: y = a*t + b
    // a = ( endCoord[y] - startCoord[y] ) / (endTime - startTime)
    // b = startCoord[y] - a*startTime

    double p[] = {0, 0, 0};

    // X axis
    double a = ( endCoord[0] - startCoord[0] ) / (endTime - startTime);
    double b = startCoord[0] - a*startTime;
    p[0] = a * currentTime + b;

    // Y axis
    a = ( endCoord[1] - startCoord[1] ) / (endTime - startTime);
    b = startCoord[1] - a*startTime;
    p[1] = a * currentTime + b;

    // Z axis
    a = ( endCoord[2] - startCoord[2] ) / (endTime - startTime);
    b = startCoord[2] - a*startTime;
    p[2] = a * currentTime + b;   

    return p;
}

//***********************************************************************
void parseTomotorByteArray()
{
    // parsing from interpArray to motorByteArray, 
    // Koder bare rett fram over til bits, men må finne ut om det trengs et nytt stepp/dir osv
    // Arduino nuller ut bittene slik at man får pulser
    
    motorByteArray = new byte[interpArray.length];

    // current number of steps
    int x, y, z;

    // machine is starting in step no 0,0,0
    int xOld = 0;
    int yOld = 0;
    int zOld = 0;

    for (int index=0; index<interpArray.length; index++)
    {
        motorByteArray[index] = 0;

        x = (int)(interpArray[index][0] / stepSizeInMM);
        if ( x > xOld )
            motorByteArray[index] = (byte)(motorByteArray[index] + 3); // X step, dir=1
        else if ( x < xOld )
            motorByteArray[index] = (byte)(motorByteArray[index] + 1); // X step, dir=0

        y = (int)(interpArray[index][1] / stepSizeInMM);
        if ( y > yOld )
            motorByteArray[index] = (byte)(motorByteArray[index] + 12); // Y step, dir=1
        else if ( y < yOld )
            motorByteArray[index] = (byte)(motorByteArray[index] + 4); // Y step, dir=0

        z = (int)(interpArray[index][2] / stepSizeInMM);
        if ( z > zOld )
            motorByteArray[index] = (byte)(motorByteArray[index] + 48); // Z step, dir=1
        else if ( z < zOld )
            motorByteArray[index] = (byte)(motorByteArray[index] + 16); // Z step, dir=0
            
        if ( (x>xOld+1) || (x<xOld-1) || (y>yOld+1) || (y<yOld-1) || (z>zOld+1) || (z<zOld-1))
            println("ERROR: speed to high for stepsize - line no: " + index);    

        xOld = x;
        yOld = y;
        zOld = z;
    }
}

//***********************************************************************
double finnTallVerdiFraLinje(String parameterNavn, String linje)
{
    int parameterNavnIndex = linje.indexOf(parameterNavn);

    if (parameterNavn == "F") // siste parameter på linjen
        return Double.parseDouble(linje.substring(parameterNavnIndex + 1, linje.length()));
    else
    {
        int spaceIndex = linje.indexOf(" ", parameterNavnIndex); // finner avslutning paa tallverdi
        return Double.parseDouble(linje.substring(parameterNavnIndex + 1, spaceIndex));
    }
}
//***********************************************************************
double finnTallVerdiFraLinjeRaaFil(String parameterNavn, String linje)
{
    int parameterNavnIndex = linje.indexOf(parameterNavn);

    if (parameterNavn == "F") // siste parameter pÃ¥ linjen
        return Double.parseDouble(linje.substring(parameterNavnIndex + 1, linje.length()));
    else
    {       
        int spaceIndex = linje.indexOf(" ", parameterNavnIndex); // finner avslutning paa tallverdi

        if (spaceIndex == -1)
            return Double.parseDouble(linje.substring(parameterNavnIndex + 1, linje.length()));

        return Double.parseDouble(linje.substring(parameterNavnIndex + 1, spaceIndex));
    }
}

//**************************************************************************************************
//**************************************************************************************************

void filtrerFilTilGcodeArrayAvRenG01(String gCodeFileInName)
{
    String[] gFanuc = loadStrings(gCodeFileInName);
    println(gFanuc.length + " lines of Fanuc G code");
    
    String[] gPure = new String[gFanuc.length];

    int indexPure = 0;
    for (int i=0; i<gFanuc.length; i++)
    {
        if ( (gFanuc[i].indexOf("G90")>-1) || (gFanuc[i].indexOf("%")>-1) || (gFanuc[i].indexOf("(")>-1) ||
            (gFanuc[i].indexOf("O")>-1) || (gFanuc[i].indexOf("G21")>-1) ||
            (gFanuc[i].indexOf("G54")>-1) || (gFanuc[i].indexOf("G28")>-1) || (gFanuc[i].indexOf("M")>-1) ||
            (gFanuc[i].indexOf("G49")>-1) || (gFanuc[i].indexOf("G43")>-1) || (gFanuc[i].length()<2) )
            ; // Ikke G00/G01 bevegelse linje - overfører ikke linjen
        else 
        {   // G00/G01 bevegelse linje - overfører linjen
            gPure[indexPure] = gFanuc[i];
            indexPure++;
        }

        if ( (gFanuc[i].indexOf("G02")>-1) || (gFanuc[i].indexOf("G03")>-1) )
            println("****************** ERROR G02/3 found");
    }

    int numbOfmotionLines = indexPure;
            
    println(numbOfmotionLines + " lines of gPure G00/G01 motion");        

    // Making new short String array with empty tailing indexes removed from gPure for å kunne bruke "saveStrings"
    gPureshort = new String[numbOfmotionLines];
    for (int i=0; i<numbOfmotionLines; i++)
       gPureshort[i] = gPure[i];

    double x=0;
    double y=0;
    double z=0;
    double f=1;
    
    double maxF = 0; // vil finne maxF

    for (int i=0; i<gPureshort.length; i++)
    {
        if (gPureshort[i].indexOf("X")>-1)
            x = finnTallVerdiFraLinjeRaaFil("X", gPureshort[i]);

        if (gPureshort[i].indexOf("Y")>-1)
            y = finnTallVerdiFraLinjeRaaFil("Y", gPureshort[i]);

        if (gPureshort[i].indexOf("Z")>-1)
            z = finnTallVerdiFraLinjeRaaFil("Z", gPureshort[i]);

        if (gPureshort[i].indexOf("F")>-1)
            f = finnTallVerdiFraLinjeRaaFil("F", gPureshort[i]) * speedScale;

        if (f<0.01)
            f=1;
            
        if ( f > maxF)
            maxF = f;

        gPureshort[i] = "G01 X" + x + " Y" + y + " Z" + z + " F" + f; // denne lagres til fil
    }
}

//***********************************************************************
//***********************************************************************
//***********************************************************************
//***********************************************************************

void draw(){}

void printEtc(int star, int end)
{
    println("");
    for (int i=star; i<end; i++)
    {
        print( "step no " + i + ", time: " + r(i/stepFrequency) );
        print( "  " + r(interpArray[i][0]) );
        print( " " + r(interpArray[i][1]) );
        print( " " + r(interpArray[i][2]) );
        println( "  " + binary(motorByteArray[i]) );
    }
    println(" ");    
}

String r(double d)
{
    String s = "" + d + "                          ";
    return s.substring(0,6);
}

void printgLineArray(int star, int end)
{
    for (int i=star; i<end; i++)
    {
        print( r(gLineArray[i][0]) + " ");
        print( r(gLineArray[i][1]) + " ");
        print( r(gLineArray[i][2]) + "  ");
        println( r(gLineArray[i][3]) );
    }
    println(" ");    
}
