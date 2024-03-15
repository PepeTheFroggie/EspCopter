
//------------------------------------------------------------------

#define PPMIN_CHANNELS 6  // dont raise this

volatile uint32_t last = 0;
volatile uint8_t  chan = 0;

ICACHE_RAM_ATTR void rxInt() 
{
    uint32_t now,diff; 
    now = micros();
    diff = now - last;
    last = now;

    if      (diff > 3000) chan = 0; // Sync gap
    else if (chan < CHANNELS)
    {
      if (950<diff && diff<2050)
      {
        rcValue[chan] = diff;
        chan++;
      }
    }
    if (chan == PPMIN_CHANNELS) gotRC = true;
}

void init_RC()
{
  attachInterrupt(IN_PIN,rxInt,RISING);
}

//------------------------------------------------------------------

void rc_to_buf()
{
  RCdata.chans.Ch1 = rcValue[0];   
  RCdata.chans.Ch2 = rcValue[1];   
  RCdata.chans.Ch3 = rcValue[2];   
  RCdata.chans.Ch4 = rcValue[3];   
  RCdata.chans.Ch5 = rcValue[4];   
  RCdata.chans.Ch6 = rcValue[5];   
  RCdata.chans.Ch7 = rcValue[6];   
  RCdata.chans.Ch8 = rcValue[7];
  RCdata.chans.spare = seqno;       
}

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

