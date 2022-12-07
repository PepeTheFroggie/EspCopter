

void buf_to_rc()
{
  uint8_t seq;
  rcValue[0] = RCdata.chans.Ch1;
  rcValue[1] = RCdata.chans.Ch2;
  rcValue[2] = RCdata.chans.Ch3;
  rcValue[3] = RCdata.chans.Ch4;
  rcValue[4] = RCdata.chans.Ch5;
  rcValue[5] = RCdata.chans.Ch6;
  rcValue[6] = RCdata.chans.Ch7;
  rcValue[7] = RCdata.chans.Ch8;
  seqno = RCdata.chans.spare;
}

static uint16_t servo[4];

void mix()
{
  if (armed & (rcValue[THR] > MINTHROTTLE))
  {
    servo[0] = constrain(rcValue[THR] - axisPID[ROLL] + axisPID[PITCH] - axisPID[YAW],1000,2000);
    servo[1] = constrain(rcValue[THR] - axisPID[ROLL] - axisPID[PITCH] + axisPID[YAW],1000,2000);
    servo[2] = constrain(rcValue[THR] + axisPID[ROLL] + axisPID[PITCH] + axisPID[YAW],1000,2000);
    servo[3] = constrain(rcValue[THR] + axisPID[ROLL] - axisPID[PITCH] - axisPID[YAW],1000,2000);
  }
  else 
  { 
    axisPID[0] = 0; axisPID[1] = 0; axisPID[2] = 0;
    servo[0] = 1000; servo[1] = 1000; servo[2] = 1000; servo[3] = 1000;
  }
}

#if defined PWMOUT //----------------------------------------------

#define pwmpin1 14
#define pwmpin2 12
#define pwmpin3 13
#define pwmpin4 15

uint8_t  pwmActChan = 0;
uint32_t pwmServo[4] = {80000,80000,80000,80000}; // use 160000 for 16mhz
uint32_t next;

void inline PWM_ISR(void)
{
  next += pwmServo[pwmActChan];
  timer0_write(next);
  switch (pwmActChan)
  {
    case 0:
      digitalWrite(pwmpin4,LOW);
      digitalWrite(pwmpin1,HIGH);
      pwmActChan = 1;
      break;
    case 1:
      digitalWrite(pwmpin1,LOW);
      digitalWrite(pwmpin2,HIGH);
      pwmActChan = 2;
      break;
    case 2:
      digitalWrite(pwmpin2,LOW);
      digitalWrite(pwmpin3,HIGH);
      pwmActChan = 3;
      break;
    case 3:
      digitalWrite(pwmpin3,LOW);
      digitalWrite(pwmpin4,HIGH);
      pwmActChan = 0;
      break;
  }
}

// factor 80 is for 80mhz, use 160 for 160mhz.
void writeServo() 
{
  pwmServo[0] = servo[0]*80;  
  pwmServo[1] = servo[1]*80;  
  pwmServo[2] = servo[2]*80;  
  pwmServo[3] = servo[3]*80;  
}

void initServo() 
{
  pinMode(pwmpin1, OUTPUT); 
  pinMode(pwmpin2, OUTPUT); 
  pinMode(pwmpin3, OUTPUT); 
  pinMode(pwmpin4, OUTPUT); 
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(PWM_ISR);
  next=ESP.getCycleCount()+100000;
  timer0_write(next);
  interrupts();
}

/*
// use those functions before/after writing to eeprom
// in acc calib and pid store. Or it will crash.
// dunno why, some esp8266 internal problem
void resumeServo()
{
  timer0_attachInterrupt(PWM_ISR);
  next=ESP.getCycleCount()+F_CPU/100;
  timer0_write(next);
}
void stopServo()
{
  timer0_detachInterrupt();
}
*/

#else //----------------------------------------------

uint8_t outmsg[5];

void writeServo()
{
  outmsg[0] = 0xF5;
  outmsg[1] = constrain((servo[0]-1000)>>2,0,0xF4);
  outmsg[2] = constrain((servo[1]-1000)>>2,0,0xF4);
  outmsg[3] = constrain((servo[2]-1000)>>2,0,0xF4);
  outmsg[4] = constrain((servo[3]-1000)>>2,0,0xF4);
  Serial1.write(outmsg,5);
}

void initServo()
{
  Serial1.begin(128000);
}

#endif //----------------------------------------------
