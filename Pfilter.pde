// G01 FILTER
// Reads Fanuc G01-code, saves only (gPure) G01 move lines for use with INF4500 CNC controllers
// Output format for all lines: G01 X16.227 Y13.53 Z-0.124 F42.0 

String gCodeFileInName = "../0.nc";
String gCodeFileOutName = "../1.nc";

void setup()
{
    String[] gFanuc = loadStrings(gCodeFileInName);
    println(gFanuc.length + " lines of Fanuc code");
    
    String[] gPure = new String[gFanuc.length];

    int indexPure = 0;
    for (int i=0; i<gFanuc.length; i++)
    {
        if ( (gFanuc[i].indexOf("G90")>-1) || (gFanuc[i].indexOf("%")>-1) || (gFanuc[i].indexOf("(")>-1) ||
            (gFanuc[i].indexOf("O")>-1) || (gFanuc[i].indexOf("G21")>-1) ||
            (gFanuc[i].indexOf("G54")>-1) || (gFanuc[i].indexOf("G28")>-1) || (gFanuc[i].indexOf("M")>-1) ||
            (gFanuc[i].indexOf("G49")>-1) || (gFanuc[i].indexOf("G43")>-1) || (gFanuc[i].length()<2) )
            ;
        else // motion command
        {
            gPure[indexPure] = gFanuc[i];
            indexPure++;
        }

        if ( (gFanuc[i].indexOf("G02")>-1) || (gFanuc[i].indexOf("G03")>-1) )
            println("****************** ERROR G02/3 found");
    }

    int numbOfmotionLines = indexPure;
            
    println(numbOfmotionLines + " lines of gPure G01 motion");        

    // Making new short String array with empty indexes removed from gPure
    String[] gPureshort = new String[numbOfmotionLines];
    for (int i=0; i<numbOfmotionLines; i++)
       gPureshort[i] = gPure[i];

    double x=0;
    double y=0;
    double z=0;
    double f=1;
    
    double maxF = 0;

    for (int i=0; i<gPureshort.length; i++)
    {
        if (gPureshort[i].indexOf("X")>-1)
            x = finnTallVerdiFraLinje("X", gPureshort[i]);

        if (gPureshort[i].indexOf("Y")>-1)
            y = finnTallVerdiFraLinje("Y", gPureshort[i]);

        if (gPureshort[i].indexOf("Z")>-1)
            z = finnTallVerdiFraLinje("Z", gPureshort[i]);

        if (gPureshort[i].indexOf("F")>-1)
            f = finnTallVerdiFraLinje("F", gPureshort[i]);

        if (f<0.01)
            f=1;
            
        if ( f > maxF)
            maxF = f;

        gPureshort[i] = "G01 X" + x + " Y" + y + " Z" + z + " F" + f;
    }

    saveStrings(gCodeFileOutName, gPureshort);
    println("Max F found = " + maxF);
}

//***********************************************************************
double finnTallVerdiFraLinje(String parameterNavn, String linje)
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