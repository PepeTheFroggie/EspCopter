// Spec of RC data

#define CHANNELS 8
#define IN_PIN 4 //GPIO4, D2 on wemos D1 mini

typedef struct 
{
  uint16_t Ch1     : 11; 
  uint16_t Ch2     : 11;
  uint16_t Ch3     : 11;
  uint16_t Ch4     : 11;
  uint16_t Ch5     : 11;
  uint16_t Ch6     : 11;
  uint16_t Ch7     : 11;
  uint16_t Ch8     : 11;
  uint8_t spare    : 8;
}Payload;

#define RCdataSize 12
typedef union
{
  Payload chans;
  uint8_t data[RCdataSize];
} RCdataTY;

RCdataTY RCdata;

int16_t rcValue[CHANNELS];  // in us, center = 1500
uint8_t seqno;
volatile boolean gotRC;

void init_RC();
void buf_to_rc();
void rc_to_buf();
