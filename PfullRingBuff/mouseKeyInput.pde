void mouseDragged() 
{
  if (mouseButton==LEFT)
  {
    cameraAngHor = cameraAngHor + (mouseX-pmouseX)*0.01;
    cameraAngVert = cameraAngVert + (mouseY-pmouseY)*0.01;

    if (cameraAngVert>(PI/2.0-0.1))
      cameraAngVert = (PI/2.0-0.1);

    if (cameraAngVert<-(PI/2.0-0.1))
      cameraAngVert = -(PI/2.0-0.1);
  }

  cameraX = cameraDist*cos(cameraAngHor)*cos(cameraAngVert) + cameraXcent; 
  cameraY = cameraDist*sin(cameraAngHor)*cos(cameraAngVert) + cameraYcent;    
  cameraZ = cameraDist*sin(cameraAngVert) + cameraZcent;
}

void mouseWheel(MouseEvent event) 
{
  float e = event.getCount();

  if (e>0)
    cameraDist = cameraDist*1.04;
  else
    cameraDist = cameraDist*0.96; 

  cameraX = cameraDist*cos(cameraAngHor)*cos(cameraAngVert) + cameraXcent; 
  cameraY = cameraDist*sin(cameraAngHor)*cos(cameraAngVert) + cameraYcent;    
  cameraZ = cameraDist*sin(cameraAngVert) + cameraZcent;
}
