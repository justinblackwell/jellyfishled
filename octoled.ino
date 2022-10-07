#include <Wire.h>
#include <FastLED.h>

#define LED_TYPE WS2812 // type of LED circuit
#define COLOR_ORDER GRB // unique to LED spec (I think)
#define NUM_RINGS 32 // rings on the costume 
#define NUM_LEDS_PER_RING 6 // leds in each ring
#define NUM_LEDS NUM_RINGS * NUM_LEDS_PER_RING // needed by some FastLed functions(?)
#define NUM_PINS 4 // number of pins to which strips are connected

// visual vars
#define FADE_RATE 250 // how fast to fade a ring
#define BRIGHTNESS 80
#define FRAMES_PER_SECOND 128
//#define RING_PIN_1 3
//#define RING_PIN_2 4
#define LIT_RING_QUANTITY NUM_RINGS/8 // how many rings to illuminate on each loop
#define FADE_RING_QUANTITY NUM_RINGS/3 // how many rings to fade on each loop

// array of rings (CRGBArray objects)
CRGBArray<NUM_LEDS_PER_RING> rings[NUM_RINGS];

// stack of lit rings to fade out on interval
short lit_stack[NUM_RINGS];

// pins to ring map (might use this)
short ring_map[NUM_PINS][2] = {
  {0, 3},
  {8, 15},
  {16, 23},
  {24, 31}
};

short last_lit = 0; // last index of lit ring

void setup()
{
  short x;
  
  // first pin
  for( x = ring_map[0][0] ; x < ring_map[0][1] ; x++ )
  {
    FastLED.addLeds<LED_TYPE, 3, COLOR_ORDER>(rings[x], NUM_LEDS_PER_RING);
  }

  // second pin 
  for( x = ring_map[1][0] ; x < ring_map[1][1] ; x++ )
  {
    FastLED.addLeds<LED_TYPE, 4, COLOR_ORDER>(rings[x], NUM_LEDS_PER_RING);
  }

  // third pin 
  for( x = ring_map[2][0] ; x < ring_map[2][1] ; x++ )
  {
    FastLED.addLeds<LED_TYPE, 5, COLOR_ORDER>(rings[x], NUM_LEDS_PER_RING);
  }
  
  // fourth pin 
  for( x = ring_map[3][0] ; x < ring_map[3][1] ; x++ )
  {
    FastLED.addLeds<LED_TYPE, 6, COLOR_ORDER>(rings[x], NUM_LEDS_PER_RING);
  }

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  // power mgmt
  set_max_power_in_volts_and_milliamps(5, 500);
}

void loop()
{
    EVERY_N_MILLISECONDS( 1000/FRAMES_PER_SECOND ) {  } // @todo fade out the rings at random 
    
    short x = 0;
    while(LIT_RING_QUANTITY > x++)
    {
      // strobe a random ring
      lightRing( random8(0, NUM_RINGS) );
    }
    
    x = 0; 
    while(FADE_RING_QUANTITY > x++)
    {
      fadeRandomRing();
    }

}

bool inArray(short v, short * ary[])
{
  for(int x = 0 ; x < sizeof(ary) ; x++)
  {
    if(ary[x] == v) return true;
  }
  return false;
}

short randomUnlitRingNumber()
{
  
}

short nextAvailableUnlitRingNumber()
{
  
}

short randomLitRingNumber()
{
  
}

void lightRandomUnlitRing()
{
  
}

void lightRing(short ringNumber)
{
  // @todo Use += operator to fade up to blue
  rings[ringNumber].fill_solid(CRGB::DarkTurquoise);

  // queue it to be faded
  addToLitStack(ringNumber);
}

void addToLitStack(short ringNumber)
{
  short x = 0;
  short nextAvailable = -1;
  
  // check if already exists in stack
  while(x < LIT_RING_QUANTITY)
  {
    if(lit_stack[x] == ringNumber)
    {
      // already in stack 
      return;
    }
    if(lit_stack[x] == -1)
    {
      nextAvailable = x;
//      // increase last_lit IF larger than we have now
//      last_lit = x > last_lit ? x : last_lit;
    }
    x++;
  }

  /**
   * no spaces left in lit_stack. 
   * This would only happen if we're lighting rings faster than we're fading them out. 
   * It should have no consequences as sizeof(lit_stack) is NUM_RINGS and only contains 
   * distinct ringNumbers, so all rings will be faded eventually.
   */
  if(nextAvailable == -1)
  {
    Serial.println("no space left in lit_stack");
    return;
  }
  
  // store in empty spot
  lit_stack[nextAvailable] = ringNumber;
  Serial.println(sprintf("storing ring %d in lit_stack[%d]", ringNumber, nextAvailable));
}

void fadeRing(short ringNumber)
{
  
}

void fadeRandomRing()
{
  short x = 0;
  while(x < FADE_RING_QUANTITY)
  {
    if(lit_stack[x] != -1)
    {
      rings[lit_stack[x]].fadeToBlackBy(FADE_RATE);
      lit_stack[x] = -1;
      return;
    }
    x++;
  }
}
