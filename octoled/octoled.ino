#include <Wire.h>
#include <FastLED.h>

#define LED_TYPE WS2812 // type of LED circuit
#define COLOR_ORDER GRB // unique to LED spec (I think)
#define NUM_RINGS 32 // rings on the costume 
#define NUM_LEDS_PER_RING 6 // leds in each ring
#define NUM_LEDS NUM_RINGS * NUM_LEDS_PER_RING // needed by some FastLed functions(?)
#define NUM_PINS 8 // number of pins to which strips are connected

// visual vars
#define FADE_RATE 250
#define BRIGHTNESS 80
#define FRAMES_PER_SECOND 128
#define RING_PIN_1 3
#define RING_PIN_2 4
#define LIT_RING_QUANTITY NUM_RINGS/8

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
  // put your main code here, to run repeatedly:
    EVERY_N_MILLISECONDS( 1000/FRAMES_PER_SECOND ) {  } // @todo fade out the rings at random 
    
    for(short x = 0 ; x < LIT_RING_QUANTITY ; x++)
    {
      // strobe a random ring
      strobeRing( random8(0, NUM_RINGS) );
    }
}

void strobeRing(short ringNumber)
{
  // @todo Use += operator to fade up to blue
  rings[ringNumber].fill_solid(CRGB::DarkTurquoise);
  
  FastLED.delay(FADE_RATE);

  // fade it out
  rings[ringNumber].fadeToBlackBy(FADE_RATE);
}
