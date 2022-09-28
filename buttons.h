//define for button
#define BTN_UP 4
#define BTN_OK 3
#define BTN_DN 2
#define OK 2
#define DN 3
#define UP 1

bool btnState=0;
uint8_t getButton()
{
  if(digitalRead(BTN_UP)==0 && btnState==0)
  {
    //Serial.println("UP");
    //lcd.clear();
    btnState=1;
    //digitalWrite(PIN_BZR,1);
    //delay(20);
    //digitalWrite(PIN_BZR,0);
    return UP;
  }
  if(digitalRead(BTN_DN)==0 && btnState==0)
  {
    //Serial.println("DN");
    //lcd.clear();
    btnState=1;
//    digitalWrite(PIN_BZR,1);
//    delay(20);
//    digitalWrite(PIN_BZR,0);
    return DN;
  }
  if(digitalRead(BTN_OK)==0 && btnState==0)
  {
    //Serial.println("OK");
    //lcd.clear();
    btnState=1;
//    digitalWrite(PIN_BZR,1);
//    delay(20);
//    digitalWrite(PIN_BZR,0);
    return OK;
  }

  
  if(digitalRead(BTN_OK) && digitalRead(BTN_UP) && digitalRead (BTN_DN) && btnState)
  {
    btnState=0;
    return 0;
  }
  else
  {
    //lcd.clear();
  }
  //delay(300);//for stability
  
}
