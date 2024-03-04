#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

float C_Variable;
float propBand;
float integralTime;
float derivativeTime;
bool LM35_1=true;
bool LM35_2;
bool Forward=false;
bool Reverse=true;
bool controlAction;
String command;

int tempLocal;
int tempRemote;
int potA1;
int potA3;

float TR_C;
float TL_C;
float TR_C_Filtered;
float TL_C_Filtered;

bool Auto=true;
bool Manual=false;

void LM35_A2();
void LM35_A7();
void Potentiometer(bool action);
void printText();
void SerialPlotter(int controllerOutput);

float PID_output(float process, float setpoint, float Prop, float Integ, float deriv, int Interval, bool action);
float SetpointGenerator();
float TR_C_filterFunction(float timeConstant, float processGain,float blockIn, float intervalTime);
float TL_C_filterFunction(float timeConstant, float processGain,float blockIn, float intervalTime);
float DerivativefilterFunction(float timeConstant, float processGain,float blockIn, float intervalTime);   

void setup() {
    lcd.init();
    lcd.backlight();
  
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    
    Serial.begin(9600);
      
    printText();
    
    C_Variable=TR_C_Filtered;
    propBand = 7;
    integralTime=40;
    derivativeTime=10;
    //analogReference(INTERNAL);

}

void loop() {
    float heaterSetting;
    float contOutNorm;
    

    if (Serial.available()) {
      command = Serial.readStringUntil('\n');
      command.trim();
      
      if (command.equals("Auto")) {
        Auto=true;
        Manual=false;
      }
  
      else if (command.equals("Manual")) {
        Manual =true;
        Auto=false;
      }

      else if (command.equals("LM35_1")) {
        LM35_1=true;
        LM35_2=false;
        propBand = 7;
        integralTime=40;
        derivativeTime=10;
      }

      else if (command.equals("LM35_2")) {
        LM35_2=true;
        LM35_1=false;
        propBand = 1.8;
        integralTime=128;
        derivativeTime=32;
      }

      else if (command.equals("Forward")) {
        Forward=true;
        Reverse=false;
      }

      else if (command.equals("Reverse")) {
        Forward=false;
        Reverse=true;
      }
      else 
        Serial.println("bad command");

    Serial.print("Command: ");
    Serial.println(command);
   }
    //analogReference(INTERNAL);
    LM35_A7();
    LM35_A2();
    
    //analogReference(DEFAULT);
    Potentiometer(controlAction);
    
    
  
    if(Reverse==true)
      controlAction =true;
    else if(Forward==true)
      controlAction=false;
    if(LM35_1 ==true)
     C_Variable=TR_C_Filtered;
    else if (LM35_2==true)
     C_Variable =TL_C_Filtered;
    if(Auto==true ) {
      heaterSetting=SetpointGenerator();
      contOutNorm=PID_output(C_Variable/500, heaterSetting/500,propBand, integralTime, derivativeTime, 2000, controlAction);
      if (controlAction==true)
        analogWrite(10,255*contOutNorm);
      else if(controlAction==false)
        analogWrite(11,255*contOutNorm);
      
      lcd.setCursor(0, 3);
      lcd.print("Set Pt C ");
    } 

    if (Manual==true ) {
     potA1=analogRead(A1);
     if (controlAction==true)
      analogWrite(10,((float)potA1/1023*255*2.90));
    } 
    else if (controlAction==false)
      analogWrite(11,((float)potA1/1023*255));
     
    lcd.setCursor(9 , 3); 
    lcd.print("    ");    
    lcd.setCursor(9 , 3); 
    lcd.print (int((float)potA1/1023*100*2.90));

     lcd.setCursor(0, 3);
     lcd.print("Manual % ");
    
    
    delay(2000);
    
    SerialPlotter(contOutNorm);
}

void printText() {
 
  lcd.setCursor(0, 0);
  lcd.print("Temp R C "); 
  lcd.setCursor(0, 1);
  lcd.print("Temp L C ");
  lcd.setCursor(0, 2);
  lcd.print("C Out  % ");
  lcd.setCursor(0, 3);
  lcd.print("Set Pt C  ");

  lcd.setCursor(13, 0);
  lcd.print("P "); 
  lcd.setCursor(13, 1);
  lcd.print("I ");
  lcd.setCursor(13, 2);
  lcd.print("D ");
  lcd.setCursor(13, 3);
  lcd.print("Fn% ");
}

void SerialPlotter(float controllerOutput) {
   if (Auto==true & Manual==false) {
        Serial.print(150.0);
        Serial.print(",");
        Serial.print(0.0);
        Serial.print(",");
        Serial.print(((float)potA1*0.4887585));
        Serial.print(",");
        Serial.print(TR_C_Filtered);
        Serial.print(",");
        Serial.println(TL_C_Filtered);
    }
  
   else if (Manual==true & Auto==false) {
        Serial.print(150.0);
        Serial.print(",");
        Serial.print(0.0);
        Serial.print(",");
        Serial.print(int((float)potA1*0.09775*2.90));
        Serial.print(",");
        Serial.print(TL_C_Filtered);
        Serial.print(",");
        Serial.println(TR_C_Filtered);
    }

          
    //Serial.println(100*controllerOutput);
    //Serial.println(","); 
    //Serial.print(",");
    //Serial.print("tempRemote = ");
    //Serial.print(","); 
    //Serial.println(tempRemote);
    //Serial.print(",");  
    //Serial.print("TR_C_Filtered = ");
    //Serial.print(","); 
      
    //Serial.print(100*controllerOutput);
    //Serial.print(","); 
    //Serial.print("tempLocal = ");
    //Serial.print(","); 
    //Serial.println(tempLocal);
    //Serial.print(","); 
    //Serial.print("TL_C_Filtered = ");
    //Serial.print(","); 
    
    //Serial.print("potA1 =");
    //Serial.print(",");
    //Serial.println(potA1);
 
}

void LM35_A2() {
  tempRemote=analogRead(A2);
  TR_C=(float)tempRemote*0.4887585; 
  TR_C_Filtered=TR_C_filterFunction(5, 1.0,TR_C, 2000);
  
  lcd.setCursor(9, 0); 
  lcd.print("    ");
  lcd.setCursor(9, 0); 
  lcd.print((int)TR_C_Filtered); 
}

void LM35_A7() { 
  tempLocal=analogRead(A7);
  TL_C= (float)tempLocal*0.4887585;
  TL_C_Filtered=TL_C_filterFunction(25, 1.0,TL_C, 2000 );
  
  lcd.setCursor(9, 1); 
  lcd.print("    ");
  lcd.setCursor(9, 1); 
  lcd.print((int)TL_C_Filtered); 
   
}

void Potentiometer(bool action) {
  float speedPercent;
  potA3=analogRead(A3);

  speedPercent=(float)potA3*100/1023;
  if (action==false)
   analogWrite(10,speedPercent/100*255); 
  else if (action==true)
   analogWrite(11,speedPercent/100*255);  
  
  lcd.setCursor(17, 3); 
  lcd.print("   ");
  lcd.setCursor(17, 3); 
  lcd.print ((int) (speedPercent));
  
}

float SetpointGenerator() {
  float setTemp;
  
  potA1=analogRead(A1);
  //if (potA1<=300) {
    setTemp=(float)potA1*0.4887585;
    //setTemp=(float)potA1*0.14174;

    lcd.setCursor( 9, 3); 
    lcd.print("    ");
    lcd.setCursor(9 , 3); 
    lcd.print((int)setTemp); 
    return setTemp;
  //} 
  /*else {
    setTemp=146;
    lcd.setCursor( 9, 3); 
    lcd.print("    ");
    lcd.setCursor(9 , 3); 
    lcd.print((int)setTemp); 
    return setTemp;
  }*/
  
}

float PID_output(float process, float setpoint, float Prop, float Integ, float deriv, int Interval, bool action) {
  float Er;
  static float Olderror, Cont;
  static int Limiter_Switch;
  static float Integral;
  float derivative;
  float proportional;
  float deltaT;
  float filteredDerivative;
  deltaT=float(Interval)/1000;
  Limiter_Switch = 1;
  //delay(Interval); 

  if (action==false)
    Er = (process-setpoint);
  else if (action==true)
    Er=(setpoint-process);

  if ((Cont >= 1 && Er > 0) || (Cont <= 0 && Er < 0) || (Integ >= 3600)) 
          Limiter_Switch = 0;
  else
          Limiter_Switch = 1;     
    
  Integral = Integral + 100 / Prop / Integ * Er *deltaT * Limiter_Switch;// Integral calculator
  derivative = 100 / Prop * deriv * (Er - Olderror) / deltaT;// Derivative calculator
  filteredDerivative=DerivativefilterFunction(5, 1.0,derivative, 1000);
  proportional = 100 / Prop * Er;// Proportional calculator
          
  Cont = proportional + Integral + filteredDerivative;
  Olderror = Er;

  if (Cont > 1) 
      Cont = 1;

  if (Cont < 0) 
      Cont = 0;
  lcd.setCursor(9 , 2); 
  lcd.print("    ");    
  lcd.setCursor(9 , 2); 
  lcd.print((int)(Cont*100.0));

  lcd.setCursor(15 , 0); 
  lcd.print("     ");    
  lcd.setCursor(15 , 0);
  lcd.print((int)(proportional*100.0));
  lcd.setCursor(15 , 1); 
  lcd.print("    ");    
  lcd.setCursor(15 , 1); 
  lcd.print((int)(Integral*100.0));

  lcd.setCursor(15 , 2); 
  lcd.print("     ");    
  lcd.setCursor(15 , 2); 
  lcd.print((int)(filteredDerivative*100.0));
  /*Serial.print("Error = ");
  Serial.println(Er);
  Serial.print("setpoint = ");
  Serial.println(setpoint);
  Serial.print("Process = ");
  Serial.println(process);
  Serial.print("Proportional = ");
  Serial.println(proportional);
  Serial.print("Intgral = ");
  Serial.println(Integral);
  Serial.print("Forward = ");
  Serial.println(Forward);
  Serial.print("Reverse = ");
  Serial.println(Reverse);*/

  return  Cont;
}

float TR_C_filterFunction(float timeConstant, float processGain,float blockIn, float intervalTime) {
  float static blockOut;
  blockOut=blockOut+(intervalTime/1000/(timeConstant+intervalTime/1000))*(processGain*blockIn-blockOut);
  return blockOut;  
}

float TL_C_filterFunction(float timeConstant, float processGain,float blockIn, float intervalTime) {
  float static blockOut;
  blockOut=blockOut+(intervalTime/1000/(timeConstant+intervalTime/1000))*(processGain*blockIn-blockOut);
  return blockOut;  
}

float DerivativefilterFunction(float timeConstant, float processGain,float blockIn, float intervalTime) {
  float static blockOut;
  blockOut=blockOut+(intervalTime/1000/(timeConstant+intervalTime/1000))*(processGain*blockIn-blockOut);
  return blockOut;  
}
