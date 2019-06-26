// 1)    Read all (pure motion) G01 lines from file "1.nc" to a String array
// 2)    Parse the string array to "gLineArray", format: X Y Z T, line end positions (mm) and line endtime (sec)
// 3)    Calculate numb of CNC steps from max T
// 4)    Declare "interpArray" (interpolated XYZ values (mm)), format: X Y Z 
// 5)    Iterate all CNC steps and find corresponding G line, generate line equation and interpolation point
// 6)    Declare "motorByteArray" and generate motor control bytes from all interpolation points in "interpArray"
// 7)    Save motorByteArray to file "1.stp"

// G.code file format for all lines:  G01 X1.232 Y100.0 Z-33.12 F15  (X,Y,Z in mm, F in mm/sec)
// Tool tip is manually located i 0,0,0 (WCS) first

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
String gCodeFileName = "../1.nc";
String stepFileName = "../1.stp";

void setup()
{
    readAndParseGcodeFileTo_gLineArray(); // X Y Z T

    int totalNoOfSteps = (int)( gLineArray[gLineArray.length-1][3] * stepFrequency ); 
    println("Total no of steps = " + totalNoOfSteps);    
    println("Total run time = " + gLineArray[gLineArray.length-1][3] + " (sec)");
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
    
    saveBytes(stepFileName, motorByteArray);
    println("File: " + stepFileName + " saved");
}

//***********************************************************************
//***********************************************************************
void readAndParseGcodeFileTo_gLineArray()
{
    double maxF = 0; // Vil finne maxF, bare en sikkerhets skjekk
    
    String[] rawS = loadStrings(gCodeFileName);
    println(rawS.length + " lines read from file: " + gCodeFileName);

    gLineArray = new double[rawS.length][4];

    double lineLengthMM; // line length for each G01 line in mm

    for (int L=0; L < rawS.length; L++) // løper alle linjer i "1.nc"
    {
        gLineArray[L][0] = (finnTallVerdiFraLinje("X", rawS[L]) ); // setter inn X verdi
        gLineArray[L][1] = (finnTallVerdiFraLinje("Y", rawS[L]) ); // setter inn Y verdi
        gLineArray[L][2] = (finnTallVerdiFraLinje("Z", rawS[L]) ); // setter inn Z verdi

        double F = finnTallVerdiFraLinje("F", rawS[L]); // hastighet i mm/sec
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
    println("Max F found in file = " + maxF);
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
            motorByteArray[index] = (byte)(motorByteArray[index] + 3); // X dir
        else if ( x < xOld )
            motorByteArray[index] = (byte)(motorByteArray[index] + 1); // X step

        y = (int)(interpArray[index][1] / stepSizeInMM);
        if ( y > yOld )
            motorByteArray[index] = (byte)(motorByteArray[index] + 12); // Y dir
        else if ( y < yOld )
            motorByteArray[index] = (byte)(motorByteArray[index] + 4); // Y step

        z = (int)(interpArray[index][2] / stepSizeInMM);
        if ( z > zOld )
            motorByteArray[index] = (byte)(motorByteArray[index] + 48); // Z dir
        else if ( z < zOld )
            motorByteArray[index] = (byte)(motorByteArray[index] + 16); // Z step
            
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
