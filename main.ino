/********************************************************
 * CHANGELOG
 * v1.0 testing for lcd, pid and dht
 * v1.1 temperature with filtering
 * v1.2 add menu, custom character
 * v1.3 eeprom and some menus
 * v1.4 add dht22
 * v1.5 mode pengendalian
 ********************************************************/


#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <SimpleDHT.h>
#include <EEPROM.h>
#include "customchar.h"
#include "definition.h"
#include "buttons.h"
#include "settings.h"

const byte ADR_setPoint=1,//0x01
           ADR_setPoint2=7,//0x02
           ADR_tempCal=8,//0x03
           ADR_tempDeadzone=2,//0x04
           ADR_tempTimeout=3,//0x05
           ADR_humidMax=4,//0x06
           ADR_humidMin=5,//0x07
           ADR_tempSensorType=6,//0x08
           ADR_humidCal=9,//0x09
           ADR_deadzoneMode=10,//0x0A
           ADR_responType=11;//0x0B


//lcd init
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//DHT22 and humidity init
SimpleDHT22 dht22(pin_DHT22);
byte humidCalInt=0;
float humidCal=0;
float humidity=0;
byte humidMax=0;
byte humidMin=0;

//temp init
int Vo;
const float R1 = 10000;
float logR2, R2, T;
const float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
int tempCalInt=0;
int tempDeadzone=0;
float tempCal=0;
bool tempSensorType=0;//0 for ntc, 1 for dht;
double tmpVal[10]={0,0,0,0,0,0,0,0,0,0};
double tmpAvg=0;
byte tmpCount=0;

//for buttons init
byte longPress=0,blOff=0;
unsigned long previousMillis = 0;


//PID init
byte setPoint2;
double setPoint, temperature, Output;

//Define the aggressive and conservative Tuning Parameters
char responType=0;
bool deadzoneMode=0;
//const double slowKp=7, aggKi=1, aggKd=2;
//const double aggKp=7, aggKi=1, aggKd=2;
const double consKp[3]={1,7,30}, consKi[3]={0.05,1,10}, consKd[3]={0.25,2,8};

//Specify the links and initial tuning parameters
PID myPID(&temperature, &Output, &setPoint, consKp[responType], consKi[responType], consKd[responType], DIRECT);

float readTemp(float cal)
{
  Vo = analogRead(PIN_NTC);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15 + cal;
  return T;
}

float readTempDHT(float cal)
{
  float temp = 0;
  float humid = 0;
  if (dht22.read2(&temp, &humid, NULL) != SimpleDHTErrSuccess) {
    Serial.println("Read DHT11 failed, err=");
    //return 0; 
  }
  else
  {
    return (float)temp+cal;
  } 
}

float FilteredReadTemp(float cal, bool sensorType)
{
  tmpAvg=0;
  
  if(tmpCount<10) tmpCount++;
  else tmpCount=0;
  
  if(sensorType)
  {
    //tmpVal[tmpCount]=readTempDHT(cal);
    return readTempDHT(cal);
  }
  else 
  {
    tmpVal[tmpCount]=readTemp(cal);    
    for(byte a=0;a<10;a++)
    {
      tmpAvg=tmpAvg+tmpVal[a];
    }
    return tmpAvg/10;
  }
  
}

float readHumid(float cal)
{
  float temp = 0;
  float humid = 0;
  if (dht22.read2(&temp, &humid, NULL) != SimpleDHTErrSuccess) {
    Serial.println("Read DHT11 failed, err=");
  }
  else
  {
    return humid+cal;
  } 
}

void setting()
{
  digitalWrite(PIN_BL,1);
  analogWrite(PIN_PWM,0);
  lcd.clear();
  byte menu[5]={1,0,0,0,0},menuSize[5]={5,5,3,4,1},selectedMenu=0,btn=0;
  bool s=1,menuSelecting=1,clearScreen=0,decision=0;
  bool activated=0;
  byte sm=0;
  while(s)
  {
    if(menuSelecting==1){
      clearScreen=0;
      switch (getButton()){
      case DN:      
      menu[selectedMenu]--;
      if(menu[selectedMenu]==0) menu[selectedMenu]=menuSize[selectedMenu];
      lcd.clear();
      break;
      case UP:
      menu[selectedMenu]++;
      if(menu[selectedMenu]>menuSize[selectedMenu]) menu[selectedMenu]=1;
      lcd.clear();
      break;
      case OK:
      if(menu[0]==1) selectedMenu=1;
      if(menu[0]==2) selectedMenu=2;
      if(menu[0]==3) selectedMenu=3;
      if(menu[0]==4) {selectedMenu=4;menuSelecting=0;}
      if(menu[0]==5) s=0;
      menu[0]=0;
      
     
      //menu keluar
      if(menu[selectedMenu]==menuSize[selectedMenu])
      {
        menu[selectedMenu]=0;
        menu[0]=selectedMenu;
        selectedMenu=0;        
      }
      else if(menu[selectedMenu]>0) {menuSelecting=0;clearScreen=1;}
      if(selectedMenu>0 && menu[selectedMenu]==0) menu[selectedMenu]=1;
      lcd.clear();
      
      //Serial.println(selectedMenu);
      break;
    }
    lcd.setCursor(0,0);
    switch (selectedMenu){
      case 0:
      lcd.print(">PENGATURAN");
      break;
      case 1:
      lcd.print(">>SUHU");
      break;
      case 2:
      lcd.print(">>KELEMBAPAN");
      break;
      case 3:
      lcd.print(">>PEMANAS");
      break;
    }
    
    lcd.setCursor(0,1);
    switch (menu[0]){
      case 1:
      lcd.print("1.Suhu");
      break;
      case 2:
      lcd.print("2.Kelembapan");
      break;
      case 3:
      lcd.print("3.Pemanas");
      break;
      case 4:
      lcd.print("4.Reset Pabrik");
      break;
      case 5:
      lcd.print("5.Keluar");
      break;
  
    }
    switch (menu[1]){
      case 1:
      lcd.print("1.Tipe Sensor");
      break;
      case 2:
      lcd.print("2.Set Poin");
      break;
      case 3:
      lcd.print("3.Sesuaikan");
      break;
      case 4:
      lcd.print("4.Deadzone");
      break;
      case 5:
      lcd.print("5.Keluar");
      break;
    }

    switch (menu[2]){
 
      case 1:
      lcd.print("1.Sesuaikan");
      break;
      case 2:
      lcd.print("2.Alarm Kritis");
      break;
      case 3:
      lcd.print("3.Keluar");
      break;
 
    }
    
    switch (menu[3]){

      case 1:
      lcd.print("1.Tes Output");
      break;
      case 2:
      lcd.print("2.Tipe Respon");
      break;
      case 3:
      lcd.print("3.DeadzoneCutoff");
      break;
      case 4:
      lcd.print("4.Keluar");
      break;
    }
    if(clearScreen) lcd.clear();
    }
    else
    {
      if(menu[1]==1)
      {     
      btn=getButton();
      lcd.setCursor(0,0);
      lcd.print(">>>Sensor Tipe");
      lcd.setCursor(0,1);
      if(tempSensorType) lcd.print("DHT22   ");
      else lcd.print("NTC 10K");
      if(btn==UP)
      {
        tempSensorType=1;
      }
      if(btn==DN)
      {
        tempSensorType=0;
      }
      if(btn==OK)
      {
        EEPROM.update(ADR_tempSensorType,tempSensorType);
        menuSelecting=1;
        lcd.setCursor(0,1);
        lcd.print("TERSIMPAN!");
        delay(1000);
        lcd.clear();
      }
    }
      if(menu[1]==2)
      {
      btn=getButton();
      lcd.setCursor(0,0);
      lcd.print(">>>Tmp Set Poin");
      lcd.setCursor(0,1);
      if(btn==UP) setPoint=setPoint+0.1;
      else if(btn==DN) setPoint=setPoint-0.1;
      lcd.print(setPoint,1);
      lcd.write(4);
      lcd.print("C");
      if(btn==OK)
      {
        lcd.setCursor(0,1);
        lcd.print("TERSIMPAN!");
        setPoint2=setPoint;
        setPoint2=(setPoint-setPoint2)*10;
        EEPROM.update(ADR_setPoint, setPoint);
        EEPROM.update(ADR_setPoint2, setPoint2);
        Serial.println(setPoint2);
        delay(1000);
        menuSelecting=1;
        lcd.clear();
      }
    }
      if(menu[1]==3)
      { 
      if(millis()-previousMillis>=500)
        {
          previousMillis=millis();          
          temperature=FilteredReadTemp(0,tempSensorType);
        }
      btn=getButton();
      lcd.setCursor(0,0);
      lcd.print(">>>Sesuaikan Tmp");
      lcd.setCursor(0,1);
      if(btn==UP) {tempCalInt++;lcd.clear();}
      else if(btn==DN) {tempCalInt--;lcd.clear();}
      lcd.print(temperature,1);
      if(tempCalInt-128>=0)lcd.print("+");
      lcd.print((float(tempCalInt)-128)/10,1);
      lcd.print("=");
      lcd.print(temperature+(float(tempCalInt)-128)/10,1);
      if(btn==OK)
      {
        lcd.setCursor(0,1);
        lcd.print("TERSIMPAN!      ");
        EEPROM.update(ADR_tempCal,tempCalInt);
        Serial.println(tempCalInt);
        tempCal=(float(tempCalInt)-128)/10;
        delay(1000);
        menuSelecting=1;
        lcd.clear();
      }
    }
      if(menu[1]==4)
      { 
      //Output=FilteredReadTemp(0,tempSensorType);
      btn=getButton();
      lcd.setCursor(0,0);
      lcd.print(">>>Tmp Deadzone");
      lcd.setCursor(0,1);
      lcd.print("Deadzone:");
      if(btn==UP) {lcd.clear(); tempDeadzone++;}
      else if(btn==DN) {lcd.clear(); tempDeadzone--;}
      lcd.print(tempDeadzone);
      lcd.write(4);
      lcd.print("C");
      if(btn==OK)
      {
        lcd.setCursor(0,1);
        lcd.print("TERSIMPAN!      ");
        EEPROM.update(ADR_tempDeadzone,tempDeadzone);
        Serial.println(tempDeadzone);
        delay(1000);
        menuSelecting=1;
        lcd.clear();
      }
    }
      
      if(menu[2]==1)
      {
        if(millis()-previousMillis>=500)
        {
          previousMillis=millis();
          humidity=readHumid(0);
        }
        btn=getButton();
        lcd.setCursor(0,0);
        lcd.print(">>>Sesuaikan Hmd");
        lcd.setCursor(0,1);
        if(btn==UP) {humidCalInt++;lcd.clear();}
        else if(btn==DN) {humidCalInt--;lcd.clear();}
        lcd.print(humidity,1);
        if(humidCalInt-128>=0)lcd.print("+");
        lcd.print(float(humidCalInt-128)/10,1);
        lcd.print("=");
        lcd.print((humidity-float(humidCalInt-128)/10),1);
        if(btn==OK)
        {
          lcd.setCursor(0,1);
          lcd.print("TERSIMPAN!      ");
          EEPROM.update(ADR_humidCal,humidCalInt);
          Serial.println(humidCalInt);
          humidCal=float(humidCalInt-128)/10;
          delay(1000);
          menuSelecting=1;
          lcd.clear();
        }
      }
      
      if(menu[2]==2) //kelembapan min
      {
        btn=getButton();
        lcd.setCursor(0,0);
        lcd.print(">>>Alarm Hmd");
        lcd.setCursor(0,1);
        if(btn==UP)
        {
          humidMin++;
          if(humidMin>=100)humidMin=99;
          lcd.clear();
        }
        else if(btn==DN)
        {
          humidMin--;
          if(humidMin<1)humidMin=1;
          lcd.clear();
        }
        lcd.print("Minimal:");
        lcd.print(humidMin);
        lcd.print("%");
        if(btn==OK)
        {
          lcd.setCursor(0,1);
          lcd.print("TERSIMPAN!      ");
          EEPROM.update(ADR_humidMin,humidMin);
          Serial.println(humidMin);
          delay(1000);
          menuSelecting=1;
          lcd.clear();
        }
      }
      
      if(menu[3]==1) //tes output
      {
        btn=getButton();
        lcd.setCursor(0,0);
        lcd.print(">>>Test Output");
        lcd.setCursor(0,1);
        if(btn==UP)
        {
          lcd.write(3);
          lcd.print(":ON ");
          analogWrite(PIN_PWM,100);
        }
        else if(btn==DN)
        {
          lcd.write(3);
          lcd.print(":OFF");
          analogWrite(PIN_PWM,0);
        }
        //lcd.print(humidMin);
        //lcd.print("%");
        if(btn==OK)
        {
          menuSelecting=1;
          analogWrite(PIN_PWM,0);
          lcd.clear();
        }
      }
      if(menu[3]==2) //tipe respon
      {
        btn=getButton();
        lcd.setCursor(0,0);
        lcd.print(">>>Tipe Respon");
        lcd.setCursor(0,1);
        if(btn==UP)
        {
          responType++;
          if(responType>2)responType=0;
          lcd.clear();
        }
        else if(btn==DN)
        {
          responType--;
          if(responType<0)responType=2;
          lcd.clear();
        }
    
        switch(responType)
        {
          case 0:
          lcd.print("Santai ");
          break;
          case 1:
          lcd.print("Sedang ");
          break;
          case 2:
          lcd.print("Agresif");
          break;
        }
    
        if(btn==OK)
        {
          EEPROM.update(ADR_responType,responType);
          lcd.setCursor(0,1);
          lcd.print("TERSIMPAN!      ");
          EEPROM.update(ADR_humidMin,humidMin);
          Serial.println(humidMin);
          delay(1000);
          menuSelecting=1;
          lcd.clear();
        }
      }
      if(menu[3]==3) //deadzone cut mode
      {
        btn=getButton();
        lcd.setCursor(0,0);
        lcd.print(">>>DeadzoneCut");
        lcd.setCursor(0,1);
        if(btn==UP)
        {
          deadzoneMode=1;
          lcd.clear();
        }
        else if(btn==DN)
        {
          deadzoneMode=0;
          lcd.clear();
        }
        if(deadzoneMode) lcd.print("ON ");
        else lcd.print("OFF");    

        if(btn==OK)
        {
          EEPROM.update(ADR_deadzoneMode,deadzoneMode);
          lcd.setCursor(0,1);
          lcd.print("TERSIMPAN!      ");
          delay(1000);
          menuSelecting=1;
          lcd.clear();
        }
      }  
     
      if(selectedMenu==4) //reset pabrik
      {
        btn=getButton();
        lcd.setCursor(0,0);
        lcd.print(">>>RESET PABRIK?");
        
        if(btn==UP)
        {
          decision=1;
          lcd.clear();
        }
        else if(btn==DN)
        {
          decision=0;
          lcd.clear();
        }
        
        lcd.setCursor(0,1);
        if(decision) lcd.print("YA   ");
        else lcd.print("TIDAK");  

        if(btn==OK && decision)
        {
          EEPROM.update(ADR_setPoint,37);
          EEPROM.update(ADR_setPoint2,5);
          EEPROM.update(ADR_tempDeadzone,42);
          EEPROM.update(ADR_tempCal,128);
          EEPROM.update(ADR_humidCal,128);
          EEPROM.update(ADR_tempSensorType,0);
          EEPROM.update(ADR_deadzoneMode,1);
          EEPROM.update(ADR_responType,1);
          EEPROM.update(ADR_humidMin,40);  
          humidMin=EEPROM.read(ADR_humidMin); 
          setPoint=EEPROM.read(ADR_setPoint);
          setPoint2=EEPROM.read(ADR_setPoint2);
          tempDeadzone=EEPROM.read(ADR_tempDeadzone);
          tempCalInt=EEPROM.read(ADR_tempCal);
          humidCalInt=EEPROM.read(ADR_humidCal);
          tempSensorType=EEPROM.read(ADR_tempSensorType);
          deadzoneMode=EEPROM.read(ADR_deadzoneMode);
          responType=EEPROM.read(ADR_responType);
          lcd.setCursor(0,1);
          lcd.print("TERSIMPAN!      ");
          delay(1000);
          s=0;
          lcd.clear();
        }
        else if(btn==OK)
        {
          lcd.setCursor(0,1);
          lcd.print("DIBATALKAN!     ");
          delay(1000);
          s=0;
          lcd.clear();
        }
      }
    }
  }
  lcd.clear();
  delay(100);
}

void buzzer(bool out)
{
  if(out) digitalWrite(PIN_BZR,1);
  else digitalWrite(PIN_BZR,0);
}

void setup()
{  
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.createChar(1, termometer);
  lcd.createChar(2, air);
  lcd.createChar(3, listrik);
  lcd.createChar(4, derajat);
  lcd.createChar(5, smile);
  lcd.createChar(6, okey);
  lcd.createChar(7, sad);
  lcd.createChar(8, box);
  lcd.setCursor(0,0);
  lcd.print("Halo, Bos! ");
  lcd.write(5);
  //memory eeprom reading
  //EEPROM.update(ADR_setPoint,37);
  //EEPROM.update(ADR_tempCal,1);
  //EEPROM.update(ADR_humidCal,humidCalInt);
  setPoint=EEPROM.read(ADR_setPoint);
  setPoint2=EEPROM.read(ADR_setPoint2);
  tempDeadzone=EEPROM.read(ADR_tempDeadzone);
  tempCalInt=EEPROM.read(ADR_tempCal);
  humidCalInt=EEPROM.read(ADR_humidCal);
  tempSensorType=EEPROM.read(ADR_tempSensorType);
  deadzoneMode=EEPROM.read(ADR_deadzoneMode);
  responType=EEPROM.read(ADR_responType);
  humidMin=EEPROM.read(ADR_humidMin);

  
  
  humidCal=float(humidCalInt-128)/10;
  tempCal=float(tempCalInt-128)/10;
  
  //Serial.println(tempCal);
  setPoint = (setPoint*10 + setPoint2)/10;
  
  Serial.println("Berikut adalah data parameter:");
  Serial.print("Temperatur Set Poin\t:");Serial.println(setPoint);
  Serial.print("Parameter Kalibrasi Temperatur\t:");Serial.println(tempCal);
  Serial.print("Deadzone Temperatur:\t");Serial.println(tempDeadzone);
  Serial.print("Parameter Kalibrasi Kelembapan\t:");Serial.println(humidCal);
  Serial.print("Alarm Kelembapan Minimum\t:");Serial.println(humidMin);
  Serial.print("Deadzone Mode\t:");Serial.println(deadzoneMode);
  Serial.print("Tipe Sensor Suhu\t:");if(tempSensorType) Serial.println("DHT22"); else Serial.println("NTC 10K");
  Serial.print("Tipe Respon Pengendalian\t:");
  switch (responType)
  {
    case 0:
    Serial.println("Santai");
    break;
    case 1:
    Serial.println("Sedang");
    break;
    case 2:
    Serial.println("Agresif");
    break;
  }
  Serial.println("==========================");
  pinMode(BTN_UP,INPUT_PULLUP);
  pinMode(BTN_OK,INPUT_PULLUP);
  pinMode(BTN_DN,INPUT_PULLUP);

  pinMode(PIN_BZR,OUTPUT);
  pinMode(PIN_LED,OUTPUT);
  //pinMode(PIN_NTC,INPUT);
  pinMode(PIN_PWM,OUTPUT);

  pinMode(PIN_BL, OUTPUT);

  digitalWrite(PIN_LED,1);
  digitalWrite(PIN_BL,1);
  analogWrite(PIN_PWM,0);
  digitalWrite(PIN_BZR,0);
  //initialize the variables we're linked to
  //temperature = readTemp();
  //setPoint = 37;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  //dht_read(dht_temp,dht_humid);
  Serial.println("Berikut adalah data keluaran:");
  Serial.print("Temperature");Serial.print("\t");
  Serial.print("Kelembapan");Serial.print("\t");
  Serial.println("Output(pwm)");
  for(byte a=0;a<=20;a++)
  {
    lcd.setCursor(0,1);
    lcd.print("Memulai ");
    lcd.print(a*5);
    lcd.print("%");
    if(tempSensorType==0) temperature=FilteredReadTemp(tempCal,tempSensorType);
    if(a<5 && a%2 == 0) buzzer(1);
    else buzzer(0);
    if(a%2 == 0) digitalWrite(PIN_LED,1);
    else digitalWrite(PIN_LED,0);
    delay(200);
  }
  lcd.setCursor(0,1);
  lcd.print("Set=");
  lcd.print(setPoint);
  lcd.write(4);
  lcd.print("C  ");
  delay(2000);
}

void loop()
{
  int err = SimpleDHTErrSuccess;
  float temp = 0;
  float humid = 0;
  bool dhtError=1;
  double gap;
  if(getButton()==OK)
  {
    if(digitalRead(PIN_BL)) digitalWrite(PIN_BL,0);
    else digitalWrite(PIN_BL,1);
  }
  
  if(millis()-previousMillis>=500)
  {
    
    previousMillis=millis();
    if ((err = dht22.read2(&temp, &humid, NULL)) != SimpleDHTErrSuccess) {
      dhtError=1;
      }
    else dhtError=0;
    
    if(tempSensorType)
    {      
      temperature = temp+tempCal;
      humidity=humid+humidCal;
    }
    else
    {
      temperature = FilteredReadTemp(tempCal,tempSensorType);
      if(dhtError==0) humidity = humid+humidCal;
    }
    
    gap = abs(setPoint-temperature);
    lcd.clear();    
    if(digitalRead(PIN_BL)==1) blOff++;
    if(blOff>120) {digitalWrite(PIN_BL,0); blOff=0;}  
    if(digitalRead(BTN_DN)==0 && digitalRead(BTN_UP)==0) longPress++;
    else longPress=0;
    if(longPress>4) { longPress=0; digitalWrite(PIN_BZR,0); setting();}    
    lcd.setCursor(0,0);
    lcd.write(1);
    lcd.print(":");
    if(dhtError==0 || tempSensorType==0) lcd.print(temperature,1);
    else lcd.print("-");
    lcd.write(4);
    lcd.print("C");
    lcd.setCursor(10,0);
    lcd.write(8);
    lcd.print(":");  
    if(gap<0.5) lcd.write(5);
    else if(gap<1) lcd.write(6);
    else lcd.write(7);
    lcd.setCursor(0,1);
    lcd.write(2);
    lcd.print(":");
    if(dhtError==0)
    {
      if(humidity<humidMin) if(digitalRead(PIN_BZR)) digitalWrite(PIN_BZR,0); else digitalWrite(PIN_BZR,1);
      lcd.print(humidity,1);
    }
    else lcd.print("-");
    lcd.print("%");
    lcd.setCursor(10,1);
    lcd.write(3);
    lcd.print(":");
    lcd.print(Output*100/255,0);
    lcd.print("%");
    if (gap < 0.2) myPID.SetTunings(consKp[0], consKi[0], consKd[0]);
    else myPID.SetTunings(consKp[responType], consKi[responType], consKd[responType]);
    myPID.Compute();
    if(deadzoneMode && temperature>=tempDeadzone) Output=0;
    analogWrite(PIN_PWM, Output);
    Serial.print(temperature,1);
    Serial.print("\t");
    Serial.print(humidity,1);
    Serial.print("\t");
    Serial.println(Output,0);
  }
}
