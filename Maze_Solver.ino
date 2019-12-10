int i=0, j=0, k=0, m=0, a=0, b=0;

int motor11=26, motor12=27, motor21=30, motor22=31, pwm1=6, pwm2=7;
int speed1, speed2,avgSpeed=230, TurnSpeed=230; //right    left
int counter = 0;
int seconds=500;
int return_time;
int Setpoint = 100;
int error=0, previous_error=0, targt=0, mes, integral, derivative, turn;

float right_flag, right, center, left, left_flag, right_flag_up, left_flag_up, center_up, right_up, left_up;
float right_flag_avg, right_avg, center_avg, left_avg, left_flag_avg, right_flag_up_avg, left_flag_up_avg, center_up_avg, right_up_avg, left_up_avg;
float right_flag_avg1=0, right_avg1=0, center_avg1=0, left_avg1=0, left_flag_avg1=0, right_flag_up_avg1=0, left_flag_up_avg1=0, center_up_avg1=0, right_up_avg1=0, left_up_avg1=0;
float right_flag_avg2=0, right_avg2=0, center_avg2=0, left_avg2=0, left_flag_avg2=0, right_flag_up_avg2=0, left_flag_up_avg2=0, center_up_avg2=0, right_up_avg2=0, left_up_avg2=0;

int multiplier=1;
char path[1000];

double Input, Output;
double kp=2, ki=3, kd=1;
unsigned long lastTime;
double ITerm, lastInput;
int SampleTime = 400; //1 sec
double outMin=0, outMax=250;
bool inAuto = false;
 
#define MANUAL 0
#define AUTOMATIC 1
 
#define DIRECT 0
#define REVERSE 1
int controllerDirection = DIRECT;

void setup()
{
  pinMode(13, OUTPUT);
  Serial.begin(9600); 
  delay(5000); 
}


void loop()
{  
  if(k==0)
  {
    Serial.println("calibrate");  
    calibrate();
    k=1;
  }
  if(m==0)
  {
    Serial.println("Maze_Analysis");  
    Maze_Analysis();
    m=1;
  }
}

void Track_Line()
{
 while(1)
  {
    
      left_flag=analogRead(A0);
      left=analogRead(A1);    
      center=analogRead(A2);
      right_flag=analogRead(A3);
      right=analogRead(A4);
     
      right_flag_up=analogRead(A5);
      right_up=analogRead(A6);
      center_up=analogRead(A7);
      left_up=analogRead(A8);    
      left_flag_up=analogRead(A9);
      
      if(right_flag_up<right_flag_up_avg)
      {
        a=1;
      }

      if(left_flag_up<left_flag_up_avg)
      {
        b=1;
      } 


      if(right_flag<right_flag_avg)
      {
        if((a==1)&&(b==0))
        {  
          turn_right();
          delay(seconds);
          while(right_up<right_up_avg)
          {
            turn_right();
          }
        }
        else
        {
          a=0;
          b=0; 
        }
      }
      if(left_flag<left_flag_avg)
      {
        if((a==0)&&(b==1))
        {  
          turn_left();
          delay(seconds);
          while(left_up<left_up_avg)
          {
            turn_left();
          }
        }
        else
        {
          a=0;
          b=0; 
        }
      }
      if((right_up<right_up_avg)&&(center_up>center_up_avg)&&(left_up>left_up_avg))
      {
      Input=100;
      }
      else if((left_up<left_up_avg)&&(center_up>center_up_avg)&&(right_up>right_up_avg))
      {
       Input=100;
      }
      else if((right>right_avg)&&(center<center_avg))
      {
         Input=80;
      }
  
      
     else if((left>left_avg)&&(center<center_avg))
    {
     Input=120;
    }
     Compute();Serial.println(Output);delay(000);
     if(Output>0)
    {
      speed1=speed1-(Output*multiplier);
      speed2=speed2;  
      digitalWrite(motor11, HIGH);
      digitalWrite(motor12, LOW);
      analogWrite(pwm1, speed1);
    
      digitalWrite(motor21, HIGH);
      digitalWrite(motor22, LOW);
      analogWrite(pwm2, speed2); 
    }
    else if(Output<0)
    {
      speed1=speed1;
      speed2=speed2+(Output*multiplier);
    
      digitalWrite(motor11, HIGH);
      digitalWrite(motor12, LOW);
      analogWrite(pwm1, speed1);
    
      digitalWrite(motor21, HIGH);
      digitalWrite(motor22, LOW);
      analogWrite(pwm2, speed2); 
    }
  }
     
}
void Short_path()
{
  while(1)
  {
    
    left_flag=analogRead(A0);
    left=analogRead(A1);    
    center=analogRead(A2);
    right_flag=analogRead(A3);
    right=analogRead(A4);
     
    right_flag_up=analogRead(A5);
    right_up=analogRead(A6);
    center_up=analogRead(A7);
    left_up=analogRead(A8);    
    left_flag_up=analogRead(A9);

  if(right_flag_up<right_flag_up_avg)
  {
    a=1;
  }

  if(left_flag_up<left_flag_up_avg)
  {
    b=1;
  }    
  if((right_flag<right_flag_avg)||(left_flag<left_flag_avg)||((right_flag<right_flag_avg)&&(left_flag<left_flag_avg)))
  {
      if((right_flag<right_flag_avg)&&(left_flag>left_flag_avg))
        {
          if(path[j]=='R')
          {
             Serial.println("c0b1");
             while(left_flag_up>left_flag_up_avg)
             {
                left_flag_up=analogRead(A9);
                turn_left();
                Serial.println("L");
             }
             while(center_up>center_up_avg)
             {
                 center_up=analogRead(A7);
                 turn_left();Serial.println(center_up);
                 Serial.println("L");
             }   
             j=j-1;
             a=0;
             b=0;      
          }
          else if(path[j]=='L')
          {
              Serial.println("c0b2");
              while(right_flag_up>right_flag_up_avg)
              {
                  right_flag_up=analogRead(A5);
                  turn_right();
                  Serial.println("R");
             }
             while(center_up>center_up_avg)
             {
                 center_up=analogRead(A7);
                 turn_right();Serial.println(center_up);
                 Serial.println("R");
             }   
             j=j-1;
             a=0;
             b=0; 
          }
          else if(path[j]=='S')
          {
             Serial.println("c0a1");
             while(right_flag<right_flag_avg)
             {
                right_flag=analogRead(A3);
                straight();
                Serial.println("S");
            }
            j=j-1;
            a=0;
            b=0;
          }
        
        }
        if((right_flag>right_flag_avg)&&(left_flag<left_flag_avg))
        {
          if(path[j]=='R')
          {
             Serial.println("c1b1");
             while(left_flag_up>left_flag_up_avg)
             {
                left_flag_up=analogRead(A9);
                turn_left();
                Serial.println("L");
             }
             while(center_up>center_up_avg)
             {
                 center_up=analogRead(A7);
                 turn_left();Serial.println(center_up);
                 Serial.println("L");
             }   
             j=j-1;
             a=0;
             b=0;       
          }
          else if(path[j]=='L')
          {
              Serial.println("c1b2");
              while(right_flag_up>right_flag_up_avg)
              {
                  right_flag_up=analogRead(A5);
                  turn_right();
                  Serial.println("R");
             }
             while(center_up>center_up_avg)
             {
                 center_up=analogRead(A7);
                 turn_right();Serial.println(center_up);
                 Serial.println("R");
             }   
             j=j-1;
             a=0;
             b=0; 
          }
          else if(path[j]=='S')
          {
             Serial.println("c2a1");
             while(left_flag<left_flag_avg)
             {
                right_flag=analogRead(A3);
                straight();
                Serial.println("S");
            }
            j=j-1;
            a=0;
            b=0;
          }
        }
        if((right_flag<right_flag_avg)&&(left_flag<left_flag_avg))
        {
          if(path[j]=='R')
          {
             Serial.println("c2b1");
             while(left_flag_up>left_flag_up_avg)
             {
                left_flag_up=analogRead(A9);
                turn_left();
                Serial.println("L");
             }
             while(center_up>center_up_avg)
             {
                 center_up=analogRead(A7);
                 turn_left();Serial.println(center_up);
                 Serial.println("L");
             }   
             j=j-1;
             a=0;
             b=0;      
          }
          else if(path[j]=='L')
          {
              Serial.println("c2b2");
              while(right_flag_up>right_flag_up_avg)
              {
                  right_flag_up=analogRead(A5);
                  turn_right();
                  Serial.println("R");
             }
             while(center_up>center_up_avg)
             {
                 center_up=analogRead(A7);
                 turn_right();Serial.println(center_up);
                 Serial.println("R");
             }   
             j=j-1;
             a=0;
             b=0; 
          }
          else if(path[j]=='S')
          {
             Serial.println("c2a1");
             while(right_flag<right_flag_avg)
             {
                right_flag=analogRead(A3);
                straight();
                Serial.println("S");
            }
            j=j-1;
            a=0;
            b=0;
          }
        }
     }
  if((right<right_avg)&&(center>center_avg))
  {
    mes=-3;
    Serial.println("plus++");
  }   
  if((right<right_avg)&&(center<center_avg))
  {
    mes=-1;
    Serial.println("Plus");
  }
  if((right>right_avg)&&(center<center_avg)&&(left>left_avg))
  {
     mes=0;
     Serial.println("zero");
  }  
  if((left<left_avg)&&(center<center_avg))
  {
    mes=1;
    Serial.println("minus");
  }
  if((left<left_avg)&&(center>center_avg))
  {
    mes=3;
    Serial.println("minus--");
  }
  
  
  error=targt-mes;

  integral=integral+error;
  
  if(integral > 255)
  {
    integral=255;
  }
  if(integral < -255)
  {
    integral=-255;
  }
  
  derivative = error - previous_error;
 
  turn = (kp*error) + (ki*integral) + (kd*derivative); 
  
  previous_error = error;
  
//  delay(10);

  if(turn>=avgSpeed)
  turn=avgSpeed;
  
  if(turn<=-avgSpeed)
  turn=-avgSpeed;
  
  if(turn>=0)
  {
    speed1=avgSpeed-turn;
    speed2=avgSpeed;
  }
  else
  {
    speed1=avgSpeed;
    speed2=avgSpeed+turn;
  }
      speed1=speed1;
      speed2=speed2+(Output*multiplier);
    
      digitalWrite(motor11, HIGH);
      digitalWrite(motor12, LOW);
      analogWrite(pwm1, speed1);
    
      digitalWrite(motor21, HIGH);
      digitalWrite(motor22, LOW);
      analogWrite(pwm2, speed2); 
    
  }
}
void Maze_Analysis()
{
  while(1)
  {
    left_flag=analogRead(A0);
    left=analogRead(A1);    
    center=analogRead(A2);
    right_flag=analogRead(A3);
    right=analogRead(A4);
     
    right_flag_up=analogRead(A5);
    right_up=analogRead(A6);
    center_up=analogRead(A7);
    left_up=analogRead(A8);    
    left_flag_up=analogRead(A9);

  if(left_flag_up<left_flag_up_avg)
  {
    a=1;
    Serial.println("                      a==1");
    Serial.println("                      b==");
    Serial.print(b);
  }
  if(right_flag_up<right_flag_up_avg)
  {
    b=1;
    Serial.println("                      b==1");
    Serial.println("                      a==");
    Serial.print(a);
  }
  
  if((right_flag<right_flag_avg)||(left_flag<left_flag_avg)||((right_flag<right_flag_avg)&&(left_flag<left_flag_avg)))
  {
      if((right_flag<right_flag_avg)&&(left_flag>left_flag_avg))
      {
          if((left_up<left_up_avg)||(center_up<center_up_avg)||(right_up<right_up_avg))
          {
              if((a==0)&&(b==1))
              {
                  Serial.println("c0a1");
                  while(right_flag<right_flag_avg)
                  {
                      right_flag=analogRead(A3);
                      straight();
                      Serial.println("S");
                  }
                  path[j]='S';Serial.println(j);Serial.println("path==S");delay(000);
                  j=j+1;
                  a=0;
                  b=0;
              }
              else if((a==1)&&(b==1))
              {
                  Serial.println("c0b1");
                  while(left_flag_up>left_flag_up_avg)
                  {
                       left_flag_up=analogRead(A9);
                       turn_left();
                       Serial.println("L");
                  }
                  while(center_up>center_up_avg)
                  {
                      center_up=analogRead(A7);
                      turn_left();Serial.println(center_up);
                      Serial.println("L");
                  }   
                  path[j]='L';Serial.println(j);Serial.println("path==L");delay(000);
                  j=j+1;
                  a=0;
                  b=0;
            }
      }
      else if(center_up>center_up_avg)
      {
          if((a==0)&&(b==1))
          {
              Serial.println("c0b2");
              Serial.println("c1b1");
              while(right_flag_up>right_flag_up_avg)
              {
                  right_flag_up=analogRead(A5);
                  turn_right();
                  Serial.println("R");
             }
             while(center_up>center_up_avg)
             {
                 center_up=analogRead(A7);
                 turn_right();Serial.println(center_up);
                 Serial.println("R");
             }   
             path[j]='R';Serial.println(j);Serial.println("path==R");delay(000);
             j=j+1;
             a=0;
             b=0;
         }
         else if((a==1)&&(b==1))
         {
            Serial.println("c0b2");
            Serial.println("c1b2");
            while(left_flag_up>left_flag_up_avg)
            {
               left_flag_up=analogRead(A9);
               turn_left();
               Serial.println("L");
            }
            while(center_up>center_up_avg)
            {
               center_up=analogRead(A7);
               turn_left();Serial.println(center_up);
               Serial.println("L");
            }   
            path[j]='L';Serial.println(j);Serial.println("path==L");delay(000);
            j=j+1;
            a=0;
            b=0;
         }
         else
         {
            a=0;
            b=0;
         }
      }
   }
   else if((left_flag<left_flag_avg)&&(right_flag>right_flag_avg))
   {   
          Serial.println("c2b1");
          while(left_flag_up>left_flag_up_avg)
          {
              Serial.println("c2b1a");
              left_flag_up=analogRead(A9);
              turn_left();
              Serial.println("L");
          }
          while(center_up>center_up_avg)
          {
              Serial.println("c2b1b");
              center_up=analogRead(A7);
              turn_left();
              Serial.println(center_up);
              Serial.println(center_up);
              Serial.println("L");
          }
          path[j]='L';Serial.println(j);Serial.println("path==L");delay(000);
          j=j+1;
          a=0;
          b=0;
   }
  else if((right_flag<right_flag_avg)&&(left_flag<left_flag_avg))
  {
       if(center_up>center_up_avg)
       {
          while(left_flag_up>left_flag_up_avg)
          {
             left_flag_up=analogRead(A9);
             turn_left();
             Serial.println("L");delay(000);
          }
          while(center_up>center_up_avg)
          {
            center_up=analogRead(A7);
            turn_left();Serial.println(center_up);
            Serial.println("L");
          }
          path[j]='L';Serial.println(j);Serial.println("path==L");delay(000);
          j=j+1;
          a=0;
          b=0;
      }
      else if((right_flag_up<right_flag_up_avg)&&(left_flag_up<left_flag_up_avg)&&(right<right_avg)&&(left<left_avg)&&(center_up<center_up_avg))
      {
        Serial.println("c5a");
        while(left_up<left_up_avg)
        {
          straight();
        }
        Maze_Solve();
      }  
    }
  }
  else if((right_up>right_up_avg)&&(left_up>left_up_avg)&&(right>right_avg)&&(left>left_avg)&&(center>center_avg)&&(right_flag>right_flag_avg)&&(left_flag>left_flag_avg)&&(center_up>center_up_avg))
  {
       Serial.println("c4a");
       while(left_flag_up>left_flag_up_avg)
       {
           left_flag_up=analogRead(A9);
           turn_left();
           Serial.println("L");delay(000);
       }
       while(center_up>center_up_avg)
      {
         center_up=analogRead(A7);
         turn_left();
         Serial.println("L");delay(000);
      }
      path[j]='B';Serial.println(j);Serial.println("path==B");delay(000);
      j=j+1;
      a=0;
      b=0;
   }
  if((right<right_avg)&&(center>center_avg))
  {
    mes=-3;
    Serial.println("plus++");
  }   
  if((right<right_avg)&&(center<center_avg))
  {
    mes=-1;
    Serial.println("Plus");
  }
  if((right>right_avg)&&(center<center_avg)&&(left>left_avg))
  {
     mes=0;
     Serial.println("zero");
  }  
  if((left<left_avg)&&(center<center_avg))
  {
    mes=1;
    Serial.println("minus");
  }
  if((left<left_avg)&&(center>center_avg))
  {
    mes=3;
    Serial.println("minus--");
  }
  
  
  error=targt-mes;

  integral=integral+error;
  
  if(integral > 255)
  {
    integral=255;
  }
  if(integral < -255)
  {
    integral=-255;
  }
  
  derivative = error - previous_error;
 
  turn = (kp*error) + (ki*integral) + (kd*derivative); 
  
  previous_error = error;
  
//  delay(10);

  if(turn>=avgSpeed)
  turn=avgSpeed;
  
  if(turn<=-avgSpeed)
  turn=-avgSpeed;
  
  if(turn>=0)
  {
    speed1=avgSpeed-turn;
    speed2=avgSpeed;
  }
  else
  {
    speed1=avgSpeed;
    speed2=avgSpeed+turn;
  }
    Serial.println(speed1);
    Serial.println(speed2);delay(000);
    digitalWrite(motor11, HIGH);
    digitalWrite(motor12, LOW);
    analogWrite(pwm1, speed1);
    
    digitalWrite(motor21, HIGH);
    digitalWrite(motor22, LOW);
    analogWrite(pwm2, speed2); 
  
  }
}

void Maze_Solve()
{
   for (i = 0; path[i]!= 0; i++)
    {
      counter++;
    }
  for(i=counter; i>=0; i--)
    {
        if(path[i]=='B')
        {
            if((path[i-1]=='L')&&(path[i+1]=='R'))
            {
               i=i-1;
               path[i]='B';
               for(i=i+1; path[i]!='\0'; i++)
               {
                   path[i]=path[i+2];
               }
            }
            else if((path[i-1]=='L')&&(path[i+1]=='S'))
            {
               i=i-1;
               path[i]='R';
               for(i=i+1; path[i]!='\0'; i++)
               {
                   path[i]=path[i+2];
               }
            }
            else if((path[i-1]=='R')&&(path[i+1]=='L'))
            {
               i=i-1;
               path[i]='B';
               for(i=i+1; path[i]!='\0'; i++)
               {
                   path[i]=path[i+2];
               }
            }
            else if((path[i-1]=='S')&&(path[i+1]=='L'))
            {
               i=i-1;
               path[i]='R';
               for(i=i+1; path[i]!='\0'; i++)
               {
                   path[i]=path[i+2];
               }
            }
            else if((path[i-1]=='S')&&(path[i+1]=='S'))
            {
               i=i-1;
               path[i]='B';
               for(i=i+1; path[i]!='\0'; i++)
               {
                   path[i]=path[i+2];
               }
            }
            else if((path[i-1]=='L')&&(path[i+1]=='L'))
            {
               i=i-1;
               path[i]='S';
               for(i=i+1; path[i]!='\0'; i++)
               {
                   path[i]=path[i+2];
               }
            }
        }
        else
        {
            path[i]=path[i];
        }
    }
    counter=0;
    for (i = 0; path[i]!= 0; i++)
    {
      counter++;
    }
    j=counter;
    delay(5000);
    Short_path();
}

void turn_right()
{
  digitalWrite(motor11, LOW);
  digitalWrite(motor12, HIGH);
  analogWrite(pwm1, TurnSpeed);
  
  digitalWrite(motor21, HIGH);
  digitalWrite(motor22, LOW);
  analogWrite(pwm2, TurnSpeed);
  
}

void turn_left()
{  
  digitalWrite(motor11, HIGH);
  digitalWrite(motor12, LOW);
  analogWrite(pwm1, TurnSpeed);
 
  digitalWrite(motor21, LOW);
  digitalWrite(motor22, HIGH);
  analogWrite(pwm2, TurnSpeed);
}
void straight()
{
  digitalWrite(motor11, HIGH);
  digitalWrite(motor12, LOW);
  analogWrite(pwm1, TurnSpeed);
  
  digitalWrite(motor21, HIGH);
  digitalWrite(motor22, LOW);
  analogWrite(pwm2, TurnSpeed);
}

void calibrate()
{
for(i=0; i<5; i++)
  {  
    digitalWrite(13, HIGH);   
    delay(1000);             
  
  }
for(i=0; i<100;i++)
  {

    left_flag=analogRead(A0);
    left=analogRead(A1);    
    center=analogRead(A2);
    right_flag=analogRead(A3);
    right=analogRead(A4);
     
    right_flag_up=analogRead(A5);
    right_up=analogRead(A6);
    center_up=analogRead(A7);
    left_up=analogRead(A8);    
    left_flag_up=analogRead(A9);
 


    left_flag_avg1=left_flag_avg1+left_flag;
    left_avg1=left_avg1+left;   
    center_avg1=center_avg1+center; 
    right_flag_avg1=right_flag_avg1+right_flag;
    right_avg1=right_avg1+right;
    
    right_flag_up_avg1=right_flag_up_avg1+right_flag_up;
    left_flag_up_avg1=left_flag_up_avg1+left_flag_up;
    center_up_avg1=center_up_avg1+center_up;
    right_up_avg1=right_up_avg1+right_up;
    left_up_avg1=left_up_avg1+left_up;
    
  }
   
    left_flag_avg1=left_flag_avg1/100;
    left_avg1=left_avg1/100;   
    center_avg1=center_avg1/100; 
    right_flag_avg1=right_flag_avg1/100;
    right_avg1=right_avg1/100;
    
    right_flag_up_avg1=right_flag_up_avg1/100;
    left_flag_up_avg1=left_flag_up_avg1/100;
    center_up_avg1=center_up_avg1/100;
    right_up_avg1=right_up_avg1/100;
    left_up_avg1=left_up_avg1/100;

digitalWrite(13, LOW);  
delay(10000); 

for(i=0; i<5;i++)
  {  
    digitalWrite(13, HIGH);   
    delay(1000);             

  }
    
  for(i=0; i<100;i++)
  {
    left_flag=analogRead(A0);
    left=analogRead(A1);    
    center=analogRead(A2);
    right_flag=analogRead(A3);
    right=analogRead(A4);
     
    right_flag_up=analogRead(A5);
    right_up=analogRead(A6);
    center_up=analogRead(A7);
    left_up=analogRead(A8);    
    left_flag_up=analogRead(A9);
 


    left_flag_avg2=left_flag_avg2+left_flag;
    left_avg2=left_avg2+left;   
    center_avg2=center_avg2+center; 
    right_flag_avg2=right_flag_avg2+right_flag;
    right_avg2=right_avg2+right;
    
    right_flag_up_avg2=right_flag_up_avg2+right_flag_up;
    left_flag_up_avg2=left_flag_up_avg2+left_flag_up;
    center_up_avg2=center_up_avg2+center_up;
    right_up_avg2=right_up_avg2+right_up;
    left_up_avg2=left_up_avg2+left_up;
  }
    
    left_flag_avg2=left_flag_avg2/100;
    left_avg2=left_avg2/100;   
    center_avg2=center_avg2/100; 
    right_flag_avg2=right_flag_avg2/100;
    right_avg2=right_avg2/100;
    
    right_flag_up_avg2=right_flag_up_avg2/100;
    left_flag_up_avg2=left_flag_up_avg2/100;
    center_up_avg2=center_up_avg2/100;
    right_up_avg2=right_up_avg2/100;
    left_up_avg2=left_up_avg2/100;

    
    left_flag_avg=(left_flag_avg2+left_flag_avg1)/2;
    left_avg=(left_avg2+left_avg1)/2;
    center_avg=(center_avg2+center_avg1)/2;
    right_flag_avg=(right_flag_avg2+right_flag_avg1)/2;
    right_avg=(right_avg2+right_avg1)/2;

    
    right_flag_up_avg=(right_flag_up_avg2+right_flag_up_avg1)/2;
    right_up_avg=(right_up_avg2+right_up_avg1)/2;
    center_up_avg=(center_up_avg2+center_up_avg1)/2;
    left_up_avg=(left_up_avg2+left_up_avg1)/2;
    left_flag_up_avg=(left_flag_up_avg2+left_flag_up_avg1)/2;
      Serial.print(left_flag_up_avg);
  Serial.print("  ");
  Serial.print(left_up_avg);
  Serial.print("  ");
  Serial.print(center_up_avg);
  Serial.print("  ");
  Serial.print(right_up_avg);
  Serial.print("  ");
  Serial.print(right_flag_up_avg);  
  Serial.println("  ");
  
  
  Serial.print(left_flag_avg);
  Serial.print("  ");
  Serial.print(left_avg);
  Serial.print("  ");
  Serial.print(center_avg);
  Serial.print("  ");
  Serial.print(right_avg);
  Serial.print("  ");
  Serial.print(right_flag_avg);
  Serial.print("  ");
  Serial.println("  "); 
  Serial.println("  "); 
    digitalWrite(13, LOW); 
    delay(5000); 
    Serial.println("Done");    
    delay(000);
}

void Compute()
{
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      if(dInput>0)
      {
        Output = kp * error + ITerm- kd * dInput;
      }
      else if(dInput<0)
      {
        Output = kp * error + ITerm-(- (kd * dInput));
      }
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}



