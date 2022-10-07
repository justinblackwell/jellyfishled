#include <MPU6050.h>
#include <Wire.h>
#include <FastLED.h>
#include "EasingLib.h"

FASTLED_USING_NAMESPACE

#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    18
#define NUM_STRIPS 8
#define LEDS_PER_STRIP 18 
#define BRIGHTNESS          80
#define FRAMES_PER_SECOND  128
#define MIN_LIT 5
#define RAINBOW_ANGLE -12

#define FADE_RATE 250
#define DRIP_RATE_FAST 250
#define DRIP_RATE_SLOW 2500
#define DRIP_MODE EASE_IN_QUINT
#define TILT_ANGLE_LOW -15
#define TILT_ANGLE_HIGH 25

CRGB leds[NUM_STRIPS][NUM_LEDS]; // holder of all LED CRGB values

uint8_t gHue = 0; // rotating "base color" used by many of the patterns
uint8_t rainbowHue = 0;

bool doLightning = false;
MPU6050 mpu; // gyro device
bool noGyro = false;

uint8_t coords[8] = { // default coords set to all leds lit
  NUM_LEDS, NUM_LEDS, NUM_LEDS, NUM_LEDS, 
  NUM_LEDS, NUM_LEDS, NUM_LEDS, NUM_LEDS
};

int8_t angles[8] = { // init all angles to equator 
  0, 0, 0, 0,
  0, 0, 0, 0 
};

// @todo should be uint8_t[]
uint8_t ledsToLight = NUM_LEDS-1; // limits for leds to light 

Easing *easings[8] = {
  new Easing(),
  new Easing(),
  new Easing(),
  new Easing(),
  new Easing(),
  new Easing(),
  new Easing(),
  new Easing()
};

void setup() {
  
  delay(3000); 
  Serial.begin(115200);
  
  // init all strips on their respective data pins  
  FastLED.addLeds<LED_TYPE,10, COLOR_ORDER>(leds[0], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 8, COLOR_ORDER>(leds[1], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 4, COLOR_ORDER>(leds[2], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 9, COLOR_ORDER>(leds[3], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 7, COLOR_ORDER>(leds[4], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 3, COLOR_ORDER>(leds[5], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 6, COLOR_ORDER>(leds[6], NUM_LEDS);
  FastLED.addLeds<LED_TYPE, 5, COLOR_ORDER>(leds[7], NUM_LEDS);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  // power mgmt
  set_max_power_in_volts_and_milliamps(5, 500);

  Serial.println("Initialize MPU6050");

  uint8_t gyro_tries = 10; // attempt gyro detect for 5 seconds
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G) && --gyro_tries > 0)
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  if(gyro_tries < 1){
    noGyro = true;
  }

  for(int strip = 0 ; strip < 8 ; strip++){ 
    easings[strip]->Init(0);
    easings[strip]->SetSetpoint(NUM_LEDS-1);
    easings[strip]->SetMode(DRIP_MODE);
    easings[strip]->SetMillisInterval( (DRIP_RATE_SLOW - DRIP_RATE_FAST) / 2);
  }
  
}

void loop() {

  EVERY_N_MILLISECONDS( 1000/FRAMES_PER_SECOND ) { gHue += 1; rainbowHue +=16; } // slowly cycle the "base color" through the rainbow
  EVERY_N_MILLISECONDS( 100 ) { calculateCoords(); } // calculate tilt angles
//  EVERY_N_MILLISECONDS( 5000 ) { doLightning = doLightning ? false : true; } // calculate tilt angles

  for(int x = 0; x < NUM_STRIPS ; x++){
    
//    rainbow(x);
    if(false && angles[x] <= RAINBOW_ANGLE){
      rainbow(x, coords[x]);
    } else {
      if(doLightning){
        lightning(60, x);
      } else {
        waterdrop(x);
      }
    }
    

// alternative examples
//    ledsToLight = coords[x];
//    ledsToLight = NUM_LEDS;
//    fill_rainbow( leds[x], NUM_LEDS, gHue, 7);
//    addGlitter(x, .5);
//    addGlitter(map(coords[x], 0, NUM_LEDS, 15, 95), x); // glitter with intensity matching tilt
//    leds[x][coords[x]] = CRGB::Red; // test pattern showing gyro orientation
//    fill_rainbow(leds[x], x+1, gHue, 7); // test pattern - identify strips

    FastLED.show();
  }
    
  
  FastLED.delay(1000/FRAMES_PER_SECOND);

//  Serial.print("max_brightness_for_power_mW: "); Serial.println( calculate_max_brightness_for_power_mW(leds[0], NUM_LEDS, 5, 500));
}

void calculateCoords(){
  if(!noGyro){
    Vector normAccel = mpu.readNormalizeAccel(); // Read normalized values 

//  Alternative way of reading gyro 
//  Vector normAccel = mpu.readNormalizeGyro();
//  // Calculate Pitch, Roll and Yaw
//  pitch = pitch + norm.YAxis * timeStep;
//  roll = roll + norm.XAxis * timeStep;
//  yaw = yaw + norm.ZAxis * timeStep;

    // Calculate Pitch & Roll
    angles[2] = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
    angles[0] = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
    
    angles[4] = -1 * angles[0];
    angles[6] = -1 * angles[2];
    
    angles[1] = (angles[0] + angles[2]) / 2;
    angles[5] = (angles[4] + angles[6]) / 2;
    
    angles[3] = (angles[2] + angles[4]) / 2;
    angles[7] = (angles[0] + angles[6]) / 2;
  }

  
  for(uint8_t x = 0 ; x < NUM_STRIPS ; x++){
     // map angles to led positions
     coords[x] = map(angles[x], -90, 90, MIN_LIT, NUM_LEDS);
  }
  
}

void waterdrop(int strip)
{
  fadeToBlackBy( leds[strip], NUM_LEDS, FADE_RATE);

  long inter = map(constrain(angles[strip], TILT_ANGLE_LOW, TILT_ANGLE_HIGH), TILT_ANGLE_LOW, TILT_ANGLE_HIGH, DRIP_RATE_FAST, DRIP_RATE_SLOW);
  easings[strip]->SetMillisInterval( inter ); // map angle of tilt to interval
//  easings[strip]->SetMode(EASE_IN_CUBIC);
//  easings[strip]->SetSetpoint(NUM_LEDS);
  float val = easings[strip]->GetValue();
  int pos = constrain(round(val), 0, NUM_LEDS-1);
  
//  leds[strip][pos] += CRGB::Aqua;
  // random color from pallete 
  CRGBPalette16 palette = CloudColors_p;
  leds[strip][pos] |= ColorFromPalette(palette, gHue, 200);
  
  if(val == easings[strip]->GetSetpoint(1)){ // @todo Any other way to know when "done"; (bool) _active is private member
    easings[strip]->Init(0);
    easings[strip]->SetSetpoint(NUM_LEDS-1 * .00001); // adding multiplier to ensure we miss setPoint while easing past for "bounce" affect
  }

//  Serial.print("strip: ");
//  Serial.print(strip);
//  Serial.print(" pos: ");
//  Serial.println(pos);
  
}

// lightning simulator
void lightning( fract8 chanceOfLightning, int strip) 
{
  if( random8() < chanceOfLightning) {
    leds[strip][ random16(NUM_LEDS) ] += CRGB::White;
  }
  fadeToBlackBy( leds[strip], NUM_LEDS, FADE_RATE);
}



/* old code */


void rainbow(int strip, uint8_t howfar) 
{
  fill_rainbow( leds[strip], howfar ? howfar : NUM_LEDS, rainbowHue, 7);
}

//void rainbowWithGlitter(int strip) 
//{
//  // built-in FastLED rainbow, plus some random sparkly glitter
//  rainbow(strip);
//  addGlitter(80, strip);
//}

//void flame(uint8_t strip, uint8_t startPosition, uint8_t flameHeight){
//  uint8_t beat = cubicwave8(32);
//  CRGBPalette16 palette = HeatColors_p;
//  for( int i = startPosition; i <= flameHeight && i < NUM_LEDS; i++) { 
//    leds[strip][i] = ColorFromPalette(palette, i*10, beat+(i*10));
////    leds[strip][max(startPosition - i,0)] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
//  }
//}

//void bpm(int strip, int angle)
//{
//  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
////  uint8_t BeatsPerMinute = 100;
//  uint8_t BeatsPerMinute = map(angle, -90, 90, 50, 80);
////  CRGBPalette16 palette = PartyColors_p;
////  CRGBPalette16 palette = OceanColors_p; // CloudColors_p
//  CRGBPalette16 palette = CloudColors_p;
//
//  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
//  for( int i = 0; i < NUM_LEDS; i++) { //9948
//    leds[strip][i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
//    
//  }
//}
