
#define yawRate 90
#define rollPitchRate 20

static int8_t P_Level_PID = 40;  // P8
static int8_t I_Level_PID = 20;  // I8
static int8_t D_Level_PID = 10;  // D8
static int8_t P_PID[3] = { 20, 20, 20 };     // P8
static int8_t I_PID[3] = { 10, 10, 10 };     // I8
static int8_t D_PID[3] = { 10, 10,  0 };     // D8

static int16_t axisPID[3];
static int16_t lastError[3] = {0,0,0};
static int32_t errorGyroI[3] = {0,0,0};

//----------PID controller----------
// Angle is Level_P
// Horizon is Level_I
// Level_D is unused
// P 3.3, 0.010, 13
// L 4.5, 0.010, 100
    
#define GYRO_I_MAX 50

# if defined STD

void pid()
{
  uint8_t axis;
  int16_t errorAngle;
  int16_t PTerm,ITerm,DTerm;
  int16_t delta,deltaSum;
  static int16_t delta1[3],delta2[3];
  int16_t AngleRateTmp, RateError;
  
      //----------PID controller----------
      for(axis=0;axis<3;axis++) 
      {
        //-----Get the desired angle rate depending on flight mode
        if (axis == 2) 
        {//YAW is always gyro-controlled 
          AngleRateTmp = (((int32_t) (yawRate + 27) * rcCommand[YAW]) >> 5);
          RateError = AngleRateTmp - gyroData[axis];
        } 
        else 
        {
          if (flightmode == STABI) 
          { 
            // calculate error and limit the angle to 45 degrees max inclination
            errorAngle = constrain(rcCommand[axis],-450,+450) - angle[axis]; //16 bits is ok here           
            //it's the ANGLE mode - control is angle based, so control loop is needed
            AngleRateTmp = ((int32_t) errorAngle * P_Level_PID)>>4;
            RateError = AngleRateTmp;
                        
            delta = - gyroData[axis]; 
            DTerm = ((int32_t)delta*D_Level_PID)>>7;
          }
          else 
          {//control is GYRO based (ACRO - direct sticks control is applied to rate PID
            AngleRateTmp = ((int32_t) (rollPitchRate + 27) * rcCommand[axis]) >> 4;
            RateError = AngleRateTmp - gyroData[axis];
            
            //-----calculate D-term
            delta           = RateError - lastError[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
            lastError[axis] = RateError;
            //add moving average here to reduce noise
            deltaSum       = delta1[axis]+delta2[axis]+delta;
            delta2[axis]   = delta1[axis];
            delta1[axis]   = delta;        
            DTerm = (deltaSum*D_PID[axis])>>6;
          } 
        }
         
        //-----calculate P component scaled to 4ms cycle time
        PTerm = ((int32_t) RateError * P_PID[axis])>>7;

        //-----calculate I component
        //there should be no division before accumulating the error to integrator, because the precision would be reduced.
        //Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
        //Time correction (to avoid different I scaling for different builds based on average cycle time)
        //is normalized to cycle time = 2048.
        errorGyroI[axis]  += (int32_t)RateError * I_PID[axis];
        //limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        //I coefficient (I8) moved before integration to make limiting independent from PID settings
        errorGyroI[axis]  = constrain(errorGyroI[axis], (int32_t) -GYRO_I_MAX<<13, (int32_t) +GYRO_I_MAX<<13);
        ITerm = errorGyroI[axis]>>13;

        //-----calculate total PID output
        axisPID[axis] =  PTerm + ITerm + DTerm;
        //axisPID[axis] =  PTerm + DTerm;
      }
}

#else

void pid()
{
  uint8_t axis;
    for(axis=0;axis<3;axis++) 
    {
      //-----Get the desired angle rate depending on flight mode
      if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis<2 ) 
      { // MODE relying on ACC
        // calculate error and limit the angle to 50 degrees max inclination
        errorAngle = constrain((rcCommand[axis]<<1),-500,+500) - angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
        PTermACC = ((int32_t)errorAngle*conf.P8[PIDLEVEL])>>7;                          // 32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
        PTermACC = constrain(PTermACC,-conf.D8[PIDLEVEL]*5,+conf.D8[PIDLEVEL]*5);
        
        errorAngleI[axis]     = constrain(errorAngleI[axis]+errorAngle,-10000,+10000);    // WindUp     //16 bits is ok here
        ITermACC  = ((int32_t)errorAngleI[axis]*conf.I8[PIDLEVEL])>>12;            // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
      }
      if ( !f.ANGLE_MODE || f.HORIZON_MODE || axis == 2 ) 
      { // MODE relying on GYRO or YAW axis
        error = ((int32_t)rcCommand[axis]<<6)/conf.P8[axis] ; // 32 bits is needed for calculation
        error -= gyroData[axis]/4;  // error = rcCommand - gyro
        
        PTermGYRO = rcCommand[axis];
      
        errorGyroI[axis]  = constrain(errorGyroI[axis]+error,-16000,+16000);         // WindUp   16 bits is ok here
        if (abs(gyroData[axis]>>2)>640) errorGyroI[axis] = 0;
        ITermGYRO = ((errorGyroI[axis]>>7)*conf.I8[axis])>>6;                        // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
      }
      
      if (axis<2)
      {
        if (flightmode == STABI)  
        {
          PTerm = PTermACC;
          ITerm = ITermACC;
        } 
        else // Acro
        {
          PTerm = PTermGYRO;
          ITerm = ITermGYRO;
        }
      }
      else // yaw axis
      {
        PTerm = PTermGYRO;
        ITerm = ITermGYRO;
      }

      PTerm -= ((int32_t)gyroData[axis]*dynP8[axis])>>8; // 32 bits is needed for calculation   

      delta          = gyroData[axis] - lastGyro[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
      lastGyro[axis] = gyroData[axis];
      deltaSum       = delta1[axis]+delta2[axis]+delta;
      delta2[axis]   = delta1[axis];
      delta1[axis]   = delta;
 
      DTerm = ((int32_t)deltaSum*dynD8[axis])>>7;        // 32 bits is needed for calculation
                      
      axisPID[axis] =  PTerm + ITerm - DTerm;
    }
}

#endif

void zeroGyroI()
{
  errorGyroI[ROLL] = 0; errorGyroI[PITCH] = 0; errorGyroI[YAW] = 0;
}


