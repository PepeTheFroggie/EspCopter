#include <ESP8266WiFi.h>
#include <Wire.h>
#include <EEPROM.h>

extern "C" { 
  #include <espnow.h> 
}

#include "RC.h"

#define WIFI_CHANNEL 4
//#define PWMOUT  // normal esc, uncomment for serial esc
//#define ARMSWITCH

volatile boolean recv;
volatile int peernum = 0;

void recv_cb(u8 *macaddr, u8 *data, u8 len)
{
  recv = true;
  //Serial.print("recv_cb ");
  //Serial.println(len); 
  if (len == RCdataSize) 
  {
    for (int i=0;i<RCdataSize;i++) RCdata.data[i] = data[i];
  }
  if (!esp_now_is_peer_exist(macaddr))
  {
    Serial.println("adding peer ");
    esp_now_add_peer(macaddr, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);
    peernum++;
  }
};

void send_cb(uint8_t* mac, uint8_t sendStatus) 
{
  //Serial.print("send_cb ");
};

#define ACCRESO 4096
#define CYCLETIME 4
#define MINTHROTTLE 1050
#define MIDRUD 1519
#define THRCORR 19

enum ang { ROLL,PITCH,YAW };

static int16_t gyroADC[3];
static int16_t accADC[3];
static int16_t gyroData[3];
static float angle[2]    = {0,0};  
extern int calibratingA;

#define ROL 0
#define PIT 1
#define THR 2
#define RUD 3
#define AU1 4
#define AU2 5
static int16_t rcCommand[] = {0,0,0};

#define GYRO     0
#define STABI    1
#define RTH      2
static int8_t flightmode;
static int8_t oldflightmode;

boolean armed = false;
uint8_t armct = 0;

void setup() 
{
  Serial.begin(115200); Serial.println();

  MPU6050_init();
  MPU6050_readId(); // must be 0x68, 104dec
  
  EEPROM.begin(64);
  if (EEPROM.read(63) != 0x55) Serial.println("Need to do ACC calib");
  ACC_Read();
  
  WiFi.mode(WIFI_STA); // Station mode for esp-now 
  WiFi.disconnect();

  Serial.printf("This mac: %s, ", WiFi.macAddress().c_str()); 
  Serial.printf(", channel: %i\n", WIFI_CHANNEL); 

  if (esp_now_init() != 0) Serial.println("*** ESP_Now init failed");

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

  esp_now_register_recv_cb(recv_cb);
  esp_now_register_send_cb(send_cb);
  
  delay(1000); 
  initServo();
}

uint32_t rxt; // receive time, used for falisave

void loop() 
{
  uint32_t now,diff; 
  
  //now = millis(); // actual time

  if (recv)
  {
    recv = false;    
    buf_to_rc();

    if      (rcValue[AU1] < 1300) flightmode = GYRO;
    else if (rcValue[AU1] > 1700) flightmode = RTH;
    else                          flightmode = STABI;   
    if (oldflightmode != flightmode)
    {
      zeroGyroI();
      oldflightmode = flightmode;
    }

    #if defined (ARMSWITCH)
      if (armed) 
      {
        if (rcValue[AU2] <= 1400) { armed = false; armct = 0; }
        rcValue[THR]    -= THRCORR;
        rcCommand[ROLL]  = rcValue[ROL] - MIDRUD;
        rcCommand[PITCH] = rcValue[PIT] - MIDRUD;
        rcCommand[YAW]   = rcValue[RUD] - MIDRUD;
      }  
      else if (rcValue[AU2] >= 1600)
      {  
        if (rcValue[THR] < MINTHROTTLE) armct++;
        if (armct >= 25) armed = true;
      } 
    #else
      if (armed) 
      {
        rcValue[THR]    -= THRCORR;
        rcCommand[ROLL]  = rcValue[ROL] - MIDRUD;
        rcCommand[PITCH] = rcValue[PIT] - MIDRUD;
        rcCommand[YAW]   = rcValue[RUD] - MIDRUD;
      }  
      else
      {  
        if (rcValue[THR] < MINTHROTTLE) armct++;
        if (armct >= 25) armed = true;
      }
    #endif 
    
    //Serial.println(rcValue[AU2]    );
    //Serial.print(rcValue[THR]    ); Serial.print("  ");
    //Serial.print(rcCommand[ROLL] ); Serial.print("  ");
    //Serial.print(rcCommand[PITCH]); Serial.print("  ");
    //Serial.print(rcCommand[YAW]  ); Serial.println();

    //diff = now - rxt;
    //Serial.print(diff); Serial.println();
    rxt = millis();

    if (peernum > 0) 
    {
      //t_angle();
      t_gyro();
      //t_acc();

      //esp_now_send(NULL, (u8*)hello, sizeof(hello)); // NULL means send to all peers
    }
  }

  Gyro_getADC();
  //Serial.print(gyroADC[0]); Serial.print("  ");
  //Serial.print(gyroADC[1]); Serial.print("  ");
  //Serial.print(gyroADC[2]); Serial.println("  ");
  
  ACC_getADC();
  //Serial.print(accADC[0]); Serial.print("  ");
  //Serial.print(accADC[1]); Serial.print("  ");
  //Serial.print(accADC[2]); Serial.println("  ");

  getEstimatedAttitude();
  //Serial.print(angle[0]); Serial.print("  ");
  //Serial.print(angle[1]); Serial.println("  ");

  pid();

  mix();

  writeServo();
  
  // Failsave part
  if (now > rxt+90)
  {
    rcValue[THR] = MINTHROTTLE;
    rxt = now;
    //Serial.println("FS");
  }

  // parser part
  if (Serial.available())
  {
    char ch = Serial.read();
    // Perform ACC calibration
    if (ch == 'A')
    { 
      Serial.println("Doing ACC calib");
      calibratingA = 64; // CALSTEPS
      while (calibratingA != 0)
      {
        delay(CYCLETIME);
        ACC_getADC(); 
      }
      ACC_Store();
      Serial.println("ACC calib Done");
    }
  }
  
  delay(CYCLETIME-1);
  //diff = millis() - now;
  //Serial.print(diff); Serial.println();
}


