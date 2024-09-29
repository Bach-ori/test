/*
  Basic Leanbot Motion

  Wait for TB1A+TB1B touch signal, then go straight for 100 mm, then stop.

  More Leanbot examples at  https://git.pythaverse.space/leanbot/Examples
*/


#include <Leanbot.h>                    // use Leanbot library
//-----------PID 1 -------------------
float Kp=50,Ki=0,Kd=100;
float error;
float P;
float I;
float D; 
float pre_error = 0;
int gt_dau=600;
int PID_value = 0;
int PID_P,PID_T;

//-----------PID 2 -------------------
float Kp_f=50, Ki_f=0, Kd_f=100;
float error_f;
int hieu = 0;
float P_f;
float I_f;
float D_f; 
float pre_error_f = 0;
int gt_dau_f=600;
int PID_value_f = 0;
int PID_P_f, PID_T_f;

long distance = 0;
long distanceMm = 0;

int touch_state = 0;

typedef enum
{
  TURN_LEFT,
  TURN_RIGHT
}TURN_t;
TURN_t turn = TURN_LEFT;

typedef enum
{
  DOAN1,
  DOAN2,
  DOAN3
}DOAN_t;

DOAN_t doan = DOAN1;

void setup() 
{
  Leanbot.begin();                      // initialize Leanbot
}

void touch_turn()
{
  touch_state = LbTouch.read(TB1B);
  LbRGB[ledO] = CRGB(0, 0, 0);
  LbRGB[ledF] = CRGB(0, 0, 0);
  LbRGB[ledE] = CRGB(0, 0, 0);
  LbRGB[ledB] = CRGB(0, 0, 0);
  LbRGB[ledC] = CRGB(0, 0, 0);
  
  if(touch_state == 1)
  {
    turn = TURN_RIGHT;
  }
  if(turn == TURN_LEFT)
  {
    LbRGB[ledD] = CRGB::Red;
    LbRGB[ledA] = CRGB(0, 0, 0);

    LbRGB.show();
  }
  else
  {
    LbRGB[ledA] = CRGB::Red;
    LbRGB[ledD] = CRGB(0, 0, 0);
    LbRGB.show();
  }
}

int dem_quay = 0;
int distanceCm = 0;

void quayPhai90(){
  int speed = 1000;                       // Speed of movement
  int rotation90 = 1600;
    LbMotion.runLR(+speed, -speed);       // let Leanbot turn right
    LbMotion.waitRotation(+rotation90);   // wait for full 1600 steps
    LbMotion.stopAndWait();               // let Leanbot stop slowly
}

void quayTrai90(){
  int speed = 1000;                       // Speed of movement
  int rotation90 = 1600;
  // Rotate from right to left 4 times, approximately 90 degrees each time:
    LbMotion.runLR(-speed, +speed);       // let Leanbot turn left
    LbMotion.waitRotation(rotation90);   // wait for full 1600 steps
    LbMotion.stopAndWait();
}

void quayTrai75(){
  int speed = 1000;                       // Speed of movement
  int rotation90 = 1150;
  // Rotate from right to left 4 times, approximately 90 degrees each time:
    LbMotion.runLR(-speed, +speed);       // let Leanbot turn left
    LbMotion.waitRotation(rotation90);   // wait for full 1600 steps
    LbMotion.stopAndWait();
}

void quayTrai80(){
  int speed = 1000;                       // Speed of movement
  int rotation90 = 1400;
  // Rotate from right to left 4 times, approximately 90 degrees each time:
    LbMotion.runLR(-speed, +speed);       // let Leanbot turn left
    LbMotion.waitRotation(rotation90);   // wait for full 1600 steps
    LbMotion.stopAndWait();
}

void quayPhai80(){
  int speed = 1000;                       // Speed of movement
  int rotation90 = 1400;
  // Rotate from right to left 4 times, approximately 90 degrees each time:
    LbMotion.runLR(speed, -speed);       // let Leanbot turn left
    LbMotion.waitRotation(rotation90);   // wait for full 1600 steps
    LbMotion.stopAndWait();
}

void quayPhai45(){
  int speed = 1000;                       // Speed of movement
  int rotation45 =800;
    LbMotion.runLR(+speed, -speed);       // let Leanbot turn right
    LbMotion.waitRotation(+rotation45);   // wait for full 1600 steps
    LbMotion.stopAndWait();               // let Leanbot stop slowly
    
}

void quayTrai45(){
  int speed = 1000;                       // Speed of movement
  int rotation45 = 800;
  // Rotate from right to left 4 times, approximately 90 degrees each time:
    LbMotion.runLR(-speed, +speed);       // let Leanbot turn left
    LbMotion.waitRotation(rotation45);   // wait for full 1600 steps
    LbMotion.stopAndWait();
}

void doan1_ne()
{
  distanceCm = Leanbot.pingCm();
  uint32_t time_detect = 0;
  if(distanceCm > 2 && distanceCm <= 11 && dem_quay == 0)
  {
    DetectOjt();
    time_detect = millis();
    //Serial.println(time_detect);
  }
  else 
  {
    run();
    int gap_state = 0;
    byte valueLine = LbIRLine.read(120); // 0001 1000


    if( dem_quay == 1 && gap_state == 0)
    {
        gap_state = 1;
    }
    if(valueLine == 0b0000 && dem_quay == 1  && gap_state == 1 )
    {
        //Serial.println(time_detect);
        LbMotion.runLR(500,500);
        LbMotion.waitDistanceMm(55);
        GapVat();
        LbMotion.runLR(500,500);
        LbMotion.waitDistanceMm(55);
        LbMotion.runLR(500,500);
        quayTrai90();
        LbMotion.runLR(500,500);
        LbMotion.waitDistanceMm(200);
        quayTrai90();
        doan = DOAN2;
    }
  }
  //run();
}

int check2 = 0;
void doan2_ne()
{
  byte valueLine = LbIRLine.read(120);
  run();
  if(distanceMm >= 2003)
  {
    if(valueLine == 0b1111 && check2 == 0)
    {
      LbMotion.runLR(500, 500);
      LbMotion.waitDistanceMm(50);
      LbMotion.stopAndWait();
      quayTrai90();
      check2 = 1;
    }
    run();
    if(valueLine == 0b0000 && check2 == 1)
    {
      LbMotion.stopAndWait();// lui tha do
      LbMotion.runLR(-500, -500);
      LbMotion.waitDistanceMm(10);
      LbMotion.stopAndWait();
      LbGripper.open();

      LbMotion.stopAndWait();
      LbMotion.runLR(-500, -500);
      LbMotion.waitDistanceMm(50);

      if(turn == TURN_LEFT)
      {
        // touch
        quayTrai80();
        LbMotion.runLR(500 , 500);// quay trai 1
        LbMotion.waitDistanceMm(25);
        LbGripper.close();
        LbMotion.runLR(-500, -500);
        LbMotion.waitDistanceMm(25);
        quayTrai90();
        LbMotion.runLR(500, 500);
        LbMotion.waitDistanceMm(100);
        quayTrai90();
        check2 = 2;
        run();
      }
      else
      {
        quayPhai80();
        LbMotion.runLR(500 , 500);// quay trai 1
        LbMotion.waitDistanceMm(25);
        LbGripper.close();
        LbMotion.runLR(-500, -500);
        LbMotion.waitDistanceMm(25);
        quayPhai90();
        LbMotion.runLR(500, 500);
        LbMotion.waitDistanceMm(100);
        quayTrai90();
        check2 = 2;
        run();
      }
    }
    if(check2 == 2 && valueLine == 0b1111 && distanceMm >= 3200)
    {
      quayPhai90();
      LbMotion.runLR(500 , 500);// quay phai
      LbMotion.waitDistanceMm(290);
      quayPhai90();
      check2 = 3;
    }
    if(check2 == 3 && valueLine == 0b0000 ){
      LbMotion.stopAndWait();
      LbGripper.open();
      LbMotion.runLR(-500 , -500);// tha xong lui ra
      LbMotion.waitDistanceMm(235);
      quayPhai90();
      check2 = 4;
      run();
    }
    if(check2 == 4 && valueLine == 0b0000 && distanceMm >= 3750 ){ // co the them distanceMm
      LbMotion.stopAndWait();
      LbMotion.runLR(500 , 500);// SAFE
      LbMotion.waitDistanceMm(100);
      quayTrai90();
      doan = DOAN3;
    }
  }
  //
}

void loop() 
{
  touch_turn();
  distanceMm = LbMotion.getDistanceMm();
  Serial.println(distanceMm);
  if(doan == DOAN1)
  {
    doan1_ne();
  }
  else if(doan == DOAN2)
  {
    doan2_ne();
  }
  else if(doan == DOAN3)
  {
    run_f();
  }
}

String toString1(byte a)
{
  String s = "....";
  for(int i =3 ; i >= 0; i--)
  {
    if(a % 2 == 1)
    {
      s[i] = '1';
      a/= 2;
    }
    else 
    {
      s[i] = '0';
      a/= 2;
    }
  }
  return s;
}

void PID()
{
  byte valueLine = LbIRLine.read(120); // 0001 1000
  String s = toString1(valueLine);
  if(s == "1000")
  {
    error = -10000;
  }
  else if(s == "1100")
  {
    error = -20;
  }
  else if(s == "0110")
  {
    error = 0;
  }
  else if(s == "0011")
  {
    error = 20;
  }
  else if(s == "0001")
  {
    error = 10000;
  }
  else 
  {
     error = 0;
  }
}

void run()
{
  PID();
  P = error;
  I = I + error;
  D = error-pre_error;
  PID_value= P*Kp + I*Ki + D*Kd;
  pre_error=error;
   
  float tgt,tgp;
  tgt=gt_dau+PID_value;
  tgp=gt_dau-PID_value;

  PID_T=constrain(tgt,0,2000);
  PID_P=constrain(tgp,0,2000);

  LbMotion.runLR(PID_T, PID_P);  
}

void DetectOjt()
{
  quayPhai45();
  LbMotion.runLR(1000,1000);
  LbMotion.waitDistanceMm(130);

  quayTrai75();
  LbMotion.runLR(1000,1000);
  LbMotion.waitDistanceMm(165);

  LbMotion.runLR(1000,-1000);
  LbMotion.waitRotationDeg(13);
  Serial.println("buoc 1");
  dem_quay = 1;
}

void GapVat()
{
  LbMotion.stopAndWait();
  LbGripper.close();
  delay(2000);
}

void PID_f()
{
    int value4 = LbIRArray.read(ir4L);    
  Serial.print(value4);     
    int value5 = LbIRArray.read(ir5R);    
  Serial.print(value5);     
    int value6 = LbIRArray.read(ir6L);    
  Serial.print(value6);     
    int value7 = LbIRArray.read(ir7R);    
  Serial.println(value7);     
  
  hieu = value7 - value6;
  if(hieu >= -20 && hieu <= 20)
  {
    error_f = 0;
  }
  else if(hieu > 20 && hieu < 100)
  {
    error_f = 15;
  }
  else if(hieu < -20 && hieu > -100)
  {
    error_f = -15;
  }
  else if(hieu > 100)
  {
    error_f = 15;
  }
  else if(hieu < -100)
  {
    error_f = -15;
  }  
}

void run_f()
{
  PID_f();
  P_f = error_f;
  I_f = I_f + error_f;
  D_f = error_f - pre_error_f;
  PID_value_f = P_f*Kp_f + I_f*Ki_f + D_f*Kd_f;
  pre_error_f = error_f;
   
  float tgt,tgp;
  tgt=gt_dau_f+PID_value_f;
  tgp=gt_dau_f-PID_value_f;

  PID_T_f =constrain(tgt,0,2000);
  PID_P_f =constrain(tgp,0,2000);

  LbMotion.runLR(PID_T_f , PID_P_f);  
}