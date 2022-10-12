#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_TCS34725.h>
#include <FastLED.h>

FASTLED_USING_NAMESPACE

#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    144
#define NUM_STRIPS 1
#define LEDS_PER_STRIP 144
#define BRIGHTNESS          80
#define FRAMES_PER_SECOND  128
#define MIN_LIT 5

#define RAINBOW_MIN 5
#define RAINBOW_MAX 144
#define GRAVITY_LOW 8
#define GRAVITY_HIGH 20
#define FADE_RATE 100
#define GRAVITY_SAMPLE_RATE 1000/128


#define RGB_SENSOR_BUTTON_PIN A0

CRGB leds[NUM_STRIPS][NUM_LEDS]; // holder of all LED CRGB values

uint8_t gHue = 0; // rotating "base color" used by many of the patterns
uint8_t rainbowHue = 0;

float jumpVelocity = 0.0; // +- gravity multiplier
//float hardestJump = 0.0; // max G
//float lightestJump = 0.0; // min G

MPU6050 mpu; // gyro device
bool noGyro = false; // gyro detected

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);
bool noRgbSensor = false; // no RGB sensor found
byte gammatable[256]; // our RGB -> eye-recognized gamma color

bool senseMode = false;

CRGB lastColor = CRGB::Orange;

void setup() {
  
  delay(3000); 
  Serial.begin(115200);
  
  // init all strips on their respective data pins  
  FastLED.addLeds<LED_TYPE,10, COLOR_ORDER>(leds[0], NUM_LEDS);
//  FastLED.addLeds<LED_TYPE, 8, COLOR_ORDER>(leds[1], NUM_LEDS);
//  FastLED.addLeds<LED_TYPE, 4, COLOR_ORDER>(leds[2], NUM_LEDS);
//  FastLED.addLeds<LED_TYPE, 9, COLOR_ORDER>(leds[3], NUM_LEDS);
//  FastLED.addLeds<LED_TYPE, 7, COLOR_ORDER>(leds[4], NUM_LEDS);
//  FastLED.addLeds<LED_TYPE, 3, COLOR_ORDER>(leds[5], NUM_LEDS);
//  FastLED.addLeds<LED_TYPE, 6, COLOR_ORDER>(leds[6], NUM_LEDS);
//  FastLED.addLeds<LED_TYPE, 5, COLOR_ORDER>(leds[7], NUM_LEDS);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.setCorrection(TypicalLEDStrip);
  FastLED.setTemperature(HighNoonSun);
  FastLED.setDither(BRIGHTNESS < 255);

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
  } else {
    Serial.println("found MPU6050 gyro");
  }

  if (tcs.begin()) {
    Serial.println("Found RGB sensor");
    tcs.setInterrupt(true);
  } else {
    noRgbSensor = true;
    Serial.println("No TCS34725 found ... check your connections");
  }

  pinMode(RGB_SENSOR_BUTTON_PIN, INPUT_PULLUP);

  setupGammaTable();
  
}

void setupGammaTable(){
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    gammatable[i] = x;

    //Serial.println(gammatable[i]);
  }
}

void loop() {

  EVERY_N_MILLISECONDS( 1000/FRAMES_PER_SECOND ) { gHue += 1; rainbowHue +=1; } // slowly cycle the "base color" through the rainbow
  EVERY_N_MILLISECONDS( GRAVITY_SAMPLE_RATE ) { detectJump(); } // calculate tilt angles
//  EVERY_N_SECONDS( 15 ) { senseMode = false; }
  int howfar = 0;
  
  senseColor();
  
  for(int x = 0; x < NUM_STRIPS ; x++){
    
    howfar = map(constrain(jumpVelocity, GRAVITY_LOW, GRAVITY_HIGH), GRAVITY_LOW, GRAVITY_HIGH, MIN_LIT, NUM_LEDS);

    fadeToBlackBy(leds[x], NUM_LEDS, FADE_RATE);
    
    if(true || senseMode){
      fill_gradient_RGB(leds[x], 0, lastColor, howfar-1, CRGB::Black);
//      fill_solid(leds[x], howfar-1, lastColor);
    } else {
      rainbow(x, howfar);
//    rainbow(x, NUM_LEDS);
    }

    FastLED.show();
  }
  
  FastLED.delay(1000/FRAMES_PER_SECOND);
  
//  Serial.print("max_brightness_for_power_mW: "); Serial.println( calculate_max_brightness_for_power_mW(leds[0], NUM_LEDS, 5, 500));
}

void detectJump(){
  
  if(noGyro){
    return false;
  }

  jumpVelocity = mpu.readNormalizeAccel().ZAxis; // Read normalized Z value
  
//  Serial.println(jumpVelocity);
  
//  if(jumpVelocity > hardestJump){
//    hardestJump = jumpVelocity; 
////    Serial.print("hardestJump:"); Serial.println(hardestJump);
//  }
//  
//  if(jumpVelocity < lightestJump){
//    lightestJump = jumpVelocity; 
////    Serial.print("lightestJump:"); Serial.println(lightestJump);
//  }

}

void rainbow(int strip, uint8_t howfar){
  fill_rainbow( leds[strip], howfar ? howfar : NUM_LEDS, rainbowHue, 7);
}

void senseColor(){
  
  // check analog switch 
  if(digitalRead(RGB_SENSOR_BUTTON_PIN)){
    return;
  }

  if(noRgbSensor) {
    Serial.println("no RGB sensor found");
    senseMode = false;
    return;
  }
  
  uint32_t sum;
  uint16_t red, green, blue, clr;
  float r, g, b;

  senseMode = true;
  
  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read
  
//  tcs.getRGB(&r, &g, &b);
  tcs.getRawData(&red, &green, &blue, &clr);
  
  tcs.setInterrupt(true);  // turn off LED

  // Figure out some basic hex code for visualization
  sum = red;
  sum += green;
  sum += blue;
  
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;


  Serial.println("From sensor:");
  Serial.print("R:\t"); Serial.print(int(red));
  Serial.print("\tG:\t"); Serial.print(int(green));
  Serial.print("\tB:\t"); Serial.print(int(blue));
  Serial.print("\t");
  Serial.print((int)red, HEX); Serial.print((int)green, HEX); Serial.print((int)blue, HEX);
  Serial.print("\n");

  Serial.println("Modified:");
  Serial.print("R:\t"); Serial.print((int)r);
  Serial.print("\tG:\t"); Serial.print((int)g);
  Serial.print("\tB:\t"); Serial.print((int)b);
  Serial.print("\t");
  Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  Serial.print("\n");

  Serial.println("Gamma corrected:");
  Serial.print("R:\t"); Serial.print(gammatable[(int)r]);
  Serial.print("\tG:\t"); Serial.print(gammatable[(int)g]);
  Serial.print("\tB:\t"); Serial.print(gammatable[(int)b]);
  Serial.print("\t");
  Serial.print(gammatable[(int)r], HEX); Serial.print(gammatable[(int)g], HEX); Serial.print(gammatable[(int)b], HEX);
  Serial.print("\n");
  Serial.print("\n");

  lastColor = CRGB(gammatable[(int)r], gammatable[(int)g], gammatable[(int)b]);
//  lastColor = CRGB((int)r, (int)g, (int)b);

}
