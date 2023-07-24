/*PROGETTO SCORBOT
Alexandru Cretu (0279390)
Gianmatteo Gabrielli (0279998)

--DISCUSSIONE SPAZIO DI LAVORO DELLO SCORBOT--

Avendo preso in considerazione l'assenza di collisioni tra i vari link del braccio studiato,il suo spazio di lavoro 
si riduce ad una sfera, di raggio uguale alla somma delle lunghezze dei link 2-3-4-5, centrata nel punto di coordinate (l1,d1).
Tutto ciò nell'ipotesi di lavoro con braccio proteso in avanti (tra l'altro studiato anche a lezione).
Diversamente, se si è nell'ipotesi di lavoro con braccio proteso all'indietro, tale sfera è centrata nel punto di coordinate 
(-l1,d1).
Si può notare quindi,che si tratta semplicemente della sfera iniziale, con centro traslato di una lunghezza pari a due volte
la distanza tra gli assi dei giunti 1 e 2.

--DISCUSSIONE SOLUZIONE BRACCIO PROTESO INDIETRO--

L'idea per la soluzione del braccio proteso indietro è nata da una semplice osservazione riguardante la matrice R05,
la quale esprime l'orientamento della mano del Robot rispetto alla terna di base L0: ciò che doveva essere capito è che,
indipendentemente dalla soluzione del braccio scelta, bisognava raggiungere la stessa posizione desiderata (xd,yd,zd) con lo stesso orientamento della mano.
E' chiaro quindi che tale matrice doveva presentare gli stessi valori in entrambe le configurazioni. 
Dopo aver aggiunto all'angolo theta1 180 gradi, sono state eguagliate le espressioni R11 e R22, arrivando ad una equazione
del tipo c1c5+c1c234s5=c1c234c5+s1s5. Tale espressione dipende unicamente da theta5 e betad (essendo theta1 già noto).
Risolvendo tale equazione, si è trovato come dovessero essere aggiunti 180 gradi a theta5 e betad.

--MODALITA' DI UTILIZZO--
X,Y,Z, B, W -> incremento delle coordinate e dell'orientamento desiderato della posizione finale
x,y,z, b, w -> decremento delle coordinate e dell'orientamento desiderato della posizione finale
Mouse sx -> cambiamento origine del sistema di riferimento e della base del robot (x0, y0, z0)
Freccia su / giù -> Movimento telecamera
Freccia sx -> Soluzione gomito alto
Freccia dx -> Soluzione gomito basso
A / a -> Soluzione braccio proteso in avanti
I / i -> Soluzione braccio proteso indietro
P -> Apertura pinza
p -> Chiusura pinza
*/

//posizione ed orientamento desiderato
float xDes=0;
float yDes=0;
float zDes=0;
float betaDes=0;
float omegaDes=0;

//coordinate base
float xBase;
float yBase;

// dimensioni link 0
float d0x = 100;
float d0y = 50;
float d0z = 100;

// dimensioni link 1
float d1x = 100; 
float d1y = 50; 
float d1z = 100;

// dimensioni giunto 2
float g2x = 30; 
float g2y = 30; 
float g2z = 50; 

//dimensioni link 2
float d2x = 200;
float d2y = 30; 
float d2z = 30;

// dimensioni giunto 3
float g3x = 30;
float g3y = 30;
float g3z = 50;

// dimensioni link 3
float d3x = 200;
float d3y = 30;
float d3z = 30;

// dimensioni giunto 4
float g4x = 30;
float g4y = 30;
float g4z = 50;

//dimensioni link 4
float d4x = 200;
float d4y = 30;
float d4z = 30;

//dimensioni pinza
int lato=50;
int lp=25;
float profondita=30;

//camera
float eyeY = 0;

//angoli desiderati
float theta1Des=0;
float theta2Des=0;
float theta3Des=0;
float theta4Des=0;
float theta5Des=0;

//kp del controllo
float kp=.1; 

//differenza tra angolo finale ed angolo desiderato
float precisione=.0001;

//variabili di lavoro
float [] theta={0,0,0,0,0};
int gomito=1;
int indietro=0;
int count = 0;
float argcos3=0;
float c1=0;
float s1=0;
float A1=0;
float A2=0;
float d1=2*d0y+g2y/2;
float c3=0;
float s3=0;
float c2=0;
float s2=0;
int ng1=0;
int ng2=0;
float l1=(d1x)/2;
float d5=d4x+lato/2+lp+lato;
//matrice di rotazione R05
float r11=0;
float r12=0;
float r13=0;
float r21=0;
float r22=0;
float r23=0;
float r31=0;
float r32=0;
float r33=0;

//apertura/chiusura pinza
float theta5=0;
int open=1;

void setup()
{
  size(1000, 800, P3D);
  stroke(255);
  strokeWeight(2);
  xBase = width/2;
  yBase = height/2;
}

void draw()
{
  background(255);
  lights();
  camera((width/2.0), height/2 - eyeY, (height/2.0) / tan(PI*60.0 / 360.0), width/2.0, height/2.0, 0, 0, 1, 0);
  
  //comandi robot
  commands();
  
  //calcolo angoli 
  pushMatrix();
    translate(xBase+yDes, yBase-zDes,xDes);
    inverse_kinematics(xDes, yDes, zDes);
  popMatrix();
  
  //disegno robot
  pushMatrix();
    robot(xBase,yBase);
  popMatrix();
  
  //testo a schermo
  text();
}

void commands()
{
  if (mousePressed)
  {
    xBase = mouseX;
    yBase = mouseY;
  }
  if (keyPressed)
  {
    // movimento camera
    if (keyCode == DOWN)
    {
      eyeY -= 5;
    }
    if (keyCode == UP)
    {
      eyeY += 5;
    }
    
    if (keyCode ==LEFT)
    {
      gomito=-1;
    }
    if (keyCode == RIGHT)
    {
      gomito=1;
    }
    
    if(key == 'x')
    {
      xDes--;
    }
    if(key == 'X')
    {
      xDes++;
    }
    if(key == 'y')
    {
      yDes--;
    }
    if(key == 'Y')
    {
      yDes++;
    }
    if(key == 'z')
    {
      zDes--;
    }
    if(key == 'Z')
    {
      zDes++;
    }
    if(key == 'b')
    {
      betaDes-=0.01;
    }
    if(key == 'B')
    {
      betaDes+=0.01;
    }
    if(key == 'w')
    {
      omegaDes -= 0.1;;
    }
    if(key == 'W')
    {
      omegaDes += 0.1;
    }
    
    if (key == 'p')
    {
      if (theta5 <= 0.3)
        theta5 += .01; 
    }
    if (key == 'P')
    {
      if (theta5 >= -0.3)
        theta5 -= .01; 
    }
    
    if (key == 'i' || key == 'I')
    {
      indietro=1;
      if (count == 0)
      {
        betaDes += +PI;
        count = 1;
      }
    }
    if (key == 'a' || key == 'A')
    {
      indietro=0;
      if (count == 1)
      {
        betaDes -= -PI;
        count = 0;
      }
    }
  }
}

void text()
{
  //matrice R05
  r11=cos(theta[0])*cos(theta[1]+theta[2]+theta[3])*cos(theta[4])+sin(theta[0])*sin(theta[4]);
  r12=-cos(theta[0])*cos(theta[1]+theta[2]+theta[3])*sin(theta[4])+sin(theta[0])*cos(theta[4]);
  r13=-cos(theta[0])*sin(theta[1]+theta[2]+theta[3]);
  r21=sin(theta[0])*cos(theta[1]+theta[2]+theta[3])*cos(theta[4])-cos(theta[0])*sin(theta[4]);
  r22=-sin(theta[0])*cos(theta[1]+theta[2]+theta[3])*sin(theta[4])-cos(theta[0])*cos(theta[4]);
  r23=-sin(theta[0])*sin(theta[1]+theta[2]+theta[3]);
  r31=-sin(theta[1]+theta[2]+theta[3])*cos(theta[4]);
  r32=sin(theta[1]+theta[2]+theta[3])*sin(theta[4]);
  r33=-cos(theta[1]+theta[2]+theta[3]);
  
  //testo
  textSize(15);
  fill(0);
  text("Sistema di Riferimento di Denavit-Hartenberg", width/2-100, 20);
  if(indietro==1)
  {
    text("Soluzione braccio indietro\n",width/2-100, 40);
    if (gomito==-1)
      text("Soluzione gomito basso\n", width/2-100, 60);
    else
      text("Soluzione gomito alto\n", width/2-100, 60);
  }
  else
  {
    text("Soluzione braccio avanti\n", width/2-100, 40); 
    if (gomito==1)
      text("Soluzione gomito basso\n", width/2-100, 60);
    else
      text("Soluzione gomito alto\n", width/2-100, 60);
  }
  text("--COORDINATE--",10,20);
  text("xDes: ",10,40);
  text(xDes,80,40);
  text("yDes: ",10,60);
  text(yDes,80,60);
  text("zDes: ",10,80);
  text(zDes,80,80);
  text("betaDes: ",10,100);
  text(degrees(betaDes)%360,80,100);
  text("omegaDes: ",10,120);
  text(degrees(omegaDes)%360,80,120);
  text("------ANGOLI------",10,140);
  text("theta[1]:",10,160);
  text(degrees(theta[0])%360,80,160);
  text("theta[2]:",10,180);
  text(degrees(theta[1])%360,80,180);
  text("theta[3]:",10,200);
  text(degrees(theta[2])%360,80,200);
  text("theta[4]:",10,220);
  text(degrees(theta[3])%360,80,220);
  text("theta[5]:",10,240);
  text(degrees(theta[4])%360,80,240);
  text("-----------MATRICE-R05-----------", 10,260);
  
  text("_",11,273);
  text("|\n",10,285);
  text("|\n",10,300);
  text("|\n",10,315);
  text("|\n",10,330);
  text("_",11,333);
  
  text("_",194,273);
  text("|\n",200,285);
  text("|\n",200,300);
  text("|\n",200,315);
  text("|\n",200,330);
  text("_",194,333);
  
  text(r11,20,290);
  text(r12,90,290);
  text(r13,160,290);
  text(r21,20,310);
  text(r22,90,310);
  text(r23,160,310);
  text(r31,20,330);
  text(r32,90,330);
  text(r33,160,330); 
  textSize(30);
  text("SCORBOT-ER III",width/2,height-30);
}

void inverse_kinematics(float x, float y , float z)
{  
  
  //angolo theta1->rotazione base
  theta1Des=atan2(y,x)+ng1*2*PI + PI*indietro;
  if(abs(theta[0]-theta1Des)>abs(theta[0]-(theta1Des + 2*PI)))
  {
    theta1Des+=2*PI;
    ng1++;
  }
  if (abs(theta[0]-theta1Des) > abs(theta[0]-(theta1Des - 2*PI)))
  {
    theta1Des-=2*PI;
    ng1--;
  }
  if (abs(theta[0]-theta1Des)>precisione)
  {
    theta[0] += kp*(theta1Des-theta[0]);
  }
  
  //angolo theta3->terzo giunto
  c1=cos(theta1Des);
  s1=sin(theta1Des);
  A1=x*c1+y*s1-l1-d5*cos(betaDes); 
  A2=d1-z-d5*sin(betaDes);
  argcos3=(pow(A1,2)+pow(A2,2)-pow(d2x,2)-pow(d3x,2))/(2*d2x*d3x);
  if(abs(argcos3)<=1)
  {
    theta3Des=gomito*acos(argcos3);
    if (abs(theta[2]-theta3Des)>=0)
    {
      theta[2] += kp*(theta3Des-theta[2]);
    }
  } else {
    pushMatrix();
      translate(0,0,0);
      fill(255,0,0);
      text("Posizione fuori dallo spazio di lavoro\n",0,-200);
    popMatrix();
  }
  
  //angolo theta2->secondo giunto
  c3=cos(theta3Des);
  s3=sin(theta3Des);
  theta2Des=atan2((d2x+d3x*c3)*A2-d3x*s3*A1, (d2x+d3x*c3)*A1+d3x*s3*A2)+ng2*2*PI;
  if(abs(theta[1]-theta2Des)>abs(theta[1]-(theta2Des + 2*PI)))
  {
    theta2Des+=2*PI;
    ng2++;
  }
  if (abs(theta[1]-theta2Des) > abs(theta[1]-(theta2Des - 2*PI)))
  {
    theta2Des-=2*PI;
    ng2--;
  }
 
  if(abs(theta[1]-theta2Des)>precisione)
  {
    theta[1]+=kp*(theta2Des-theta[1]);
  }
  
  //angolo theta4->quarto giunto
  theta4Des=betaDes-theta[1]-theta[2]-PI/2;
  if(abs(theta[3]-theta4Des)>precisione)
  {
    theta[3]+=kp*(theta4Des-theta[3]);
  }
  
  //angolo theta5->quinto giunto
  //theta5Des=omegaDes+indietro*PI;
  theta5Des = omegaDes + indietro*PI ;
  
  if(abs(theta[4]-theta5Des)>precisione)
  {
    theta[4]+=kp*(theta5Des-theta[4]);
  }
}

void robot(float x,float y)
{
  //disegno sdr base SCORBOT asse z0
  fill(255,0,0); 
  rect(xBase, yBase+25, 5,-150);
  text("z0", xBase, yBase-160);
  
  //disegno sdr base SCORBOT asse y0
  fill(0,255,0); 
  rect(xBase, yBase+25, 150,5);
  text("y0", xBase+160, yBase);
  
  //disegno sdr base SCORBOT asse x0
  fill(0,0,255); 
  pushMatrix();
    translate(xBase,yBase,0);
    rotateY(-PI/2);
    rect(0, 25, 150,5);
    text("x0", 160, 0);
  popMatrix();
  
  fill(255,117,20);
  stroke(2);
  
  //link 0
  translate(x,y,0);
  box(d0x,d0y,d0z);
  
  //giunto 1 
  rotateY(theta[0]-PI/2);
  
  //link1
  translate(0,-d0y,0);
  box(d1x,d1y,d1z);
  
  pushMatrix();
    
    //giunto 2
    fill(0);
    translate(d1x/2,-d1y+10,0);
    box(g2x,g2y,g2z);
    rotateZ(theta[1]);
    fill(255,117,20);
    //link 2   
    fill(255,117,20);
    translate(3*g2x-5,0,0);
    box(d2x,d2y,d2z);
    translate(d3x/2+g3x/2,0,0);
    
    //giunto 3
    fill(0);
    box(g3x,g3y,g3z);
    rotateZ(theta[2]);
    
    //link 3
    fill(255,117,20);
    translate(d2x/2-15,0,0);
    box(d3x,d3y,d3z);
   
   
    
    //giunto 4
    fill(0);
    translate((d3x+g4x)/2,0,0);
    box(g4x,g4y,g4z);
    rotateZ(theta[3] + PI/2);
   
    //link4
    fill(255,117,20);
    translate((-g4x+d4x)/2,0,0);
    box(d4x,d4y,d4z);
    
    
    //giunto 5
    rotateX(-theta[4]+PI/2);
    
    //pinza
    translate(d4x/2-13,d4y-5,profondita/2);
    rotateZ(-PI/2);
    pinza(0,0,0);
    
    //disegno sdr pinza SCORBOT asse z0
    pushMatrix();
      fill(255,0,0); 
      rotateZ(PI/2);
      rect(lato+(lato+lp)/2,0, 150,5);
      text("z5",250,0);
    popMatrix();
    
    //disegno sdr pinza SCORBOT asse y0
    fill(0,255,0); 
    rect(-150, (lato+(lato+lp)/2), 150,5);
    text("y5", -170,(lato+(lato+lp)/2));
    
    //disegno sdr pinza SCORBOT asse x0
    pushMatrix();
      fill(0,0,255); 
      rotateY(PI/2);
      rect(0, (lato+(lato+lp)/2), -150,5);
      text("x5", -170,(lato+(lato+lp)/2));
    popMatrix();
    
  popMatrix();
}

void pinza(float xP,float yP,float zP)
{
  fill(0);
  translate(xP+lato/2,yP+lato/2,zP-profondita/2);
  box(lato,lp,profondita);
  
  pushMatrix();
    
    translate(-lato/2,lp/2,profondita/2);
    pushMatrix();
    
      if(open==1)
      {
        rotateZ(theta5);
      }
      
      //collo sinistro
       
      //faccia anteriore
      beginShape();
        vertex(0,0,0);
        vertex(lp,0,0);
        vertex(0,lato/2,0);
        vertex(-lp,lato/2,0);
      endShape(CLOSE);
      
      //faccia  laterale dx
      beginShape();
        vertex(lp,0,0);
        vertex(lp,0,-profondita);
        vertex(0,lato/2,-profondita);
        vertex(0,lato/2,0);
      endShape(CLOSE);
    
      //faccia laterale sx
      beginShape();
        vertex(0,0,0);
        vertex(-lp,lato/2,0);
        vertex(-lp,lato/2,-profondita);
        vertex(0,0,-profondita);
      endShape(CLOSE);
     
      //faccia posteriore
      beginShape();
        vertex(0,0,-profondita);
        vertex(lp,0,-profondita);
        vertex(0,lato/2,-profondita);
        vertex(-lp,lato/2,-profondita);
      endShape(CLOSE);
      
      //punta sx
      translate(-lp,lato/2,0);
      beginShape();
        vertex(0,0,0);
        vertex(lp,0,0);    
        vertex(lp, lato,0);
      endShape(CLOSE); 
        
      //faccia posteriore
      beginShape();
        vertex(0,0,-profondita);
        vertex(lp,0,-profondita);    
        vertex(lp, lato,-profondita);
      endShape(CLOSE);
      
      // Faccia laterale sinistra
      beginShape();
        vertex(0, 0,0);
        vertex(0,0,-profondita);    
        vertex(lp,lato,-profondita);    
        vertex(lp, lato,0);
      endShape(CLOSE); 
    
      // Faccia laterale destra
      beginShape();
        vertex(lp,0,0);
        vertex(lp,0,-profondita); 
        vertex(lp,lato,-profondita);    
        vertex(lp,lato,0); 
      endShape(CLOSE);
    popMatrix();
    
    //collo dx
    translate(lato,0,0);

    if(open==1)
    {
      rotateZ(-theta5);
    }
 
    //faccia anteriore
    beginShape();
      vertex(0,0,0);
      vertex(-lp,0,0);
      vertex(0,lato/2,0);
      vertex(lp,lato/2,0);
    endShape(CLOSE);
    
    //faccia posteriore
    beginShape();
      vertex(0,0,-profondita);
      vertex(-lp,0,-profondita);
      vertex(0,lato/2,-profondita);
      vertex(lp,lato/2,-profondita);
    endShape(CLOSE);
    
   //faccia laterale dx
   beginShape();
      vertex(0,0,0);
      vertex(0,0,-profondita);
      vertex(lp,lato/2,-profondita);
      vertex(lp,lato/2,0);
   endShape(CLOSE); 
  
  //faccia laterale sx
   beginShape();
      vertex(-lp,0,0);
      vertex(0,lato/2,0);
      vertex(0,lato/2,-profondita);
      vertex(-lp,0,-profondita);
   endShape(CLOSE);
   
   translate(0,lato/2,0);

   //PUNTA DX
   
   //faccia anteriore
   beginShape();
      vertex(0,0,0);
      vertex(0,lato,0);
      vertex(lp,0,0);
   endShape(CLOSE);
   
   //faccia posteriore
   beginShape();
      vertex(0,0,-profondita);
      vertex(0,lato,-profondita);
      vertex(lp,0,-profondita);
   endShape(CLOSE);
   
   //faccia laterale sx
   beginShape();
      vertex(0,0,0);
      vertex(0,0,-profondita);
      vertex(0,lato,-profondita);
      vertex(0,lato,0);
   endShape(CLOSE);
   
   //faccia laterale dx
   beginShape();
      vertex(lp,0,0);
      vertex(lp,0,-profondita);
      vertex(0,lato,-profondita);
      vertex(0,lato,0);
   endShape(CLOSE);
  
  popMatrix();
 
}
