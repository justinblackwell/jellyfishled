#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_TCS34725.h>
#include <FastLED.h>

FASTLED_USING_NAMESPACE

#define DEBUG false
#define DEBUG_POWER true
#define CALIBRATE_GYRO false

#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    15
#define NUM_STRIPS 1
#define BRIGHTNESS 255
#define FRAMES_PER_SECOND 128
#define MIN_LIT 7
#define GAMMA_MULTIPLIER 2.5

#define LEG_LEDS  15 // leds per tentacle
#define HEAD_LEDS 36 // leds per head segment
#define RING_LEDS 47 // leds in ring around head

#define LEG_LED_PIN  10
#define HEAD_LED_PIN 11
#define RING_LED_PIN 12

#define GRAVITY_LOW 0
#define GRAVITY_HIGH 18
#define FADE_RATE 120
#define GRAVITY_SAMPLE_RATE 1000/128

#define RGB_SENSOR_BUTTON_PIN A0

CRGB ledsl[LEG_LEDS]; // holder of all LED CRGB values for tentacles
CRGB ledsh[HEAD_LEDS]; // holder of each head segment leds
CRGB ledsr[RING_LEDS]; // holds all leds for ring

CRGBSet legleds(ledsl, LEG_LEDS);
CRGBSet headleds(ledsh, HEAD_LEDS);
CRGBSet ringleds(ledsr, RING_LEDS);

uint8_t rainbowHue = 0;

volatile float jumpVelocity = 9.8; // +- gravity multiplier
volatile float xVelocity = 0.0;
volatile float yVelocity = 0.0;

#ifdef DEBUG
  float hardestJump = 0.0; // max G
  float lightestJump = 0.0; // min G
#endif

MPU6050 mpu; // gyro device
bool noGyro = false; // no gyro detected

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);
bool noRgbSensor = false; // no RGB sensor found

CRGB lastColor = CRGB::SkyBlue;
CRGBPalette16 lastPalette = OceanColors_p;

void setup() {
  
  delay(3000);
  Serial.begin(115200);
  
  // init all strips on their respective data pins

  // add tentacle led strip(s)
  FastLED.addLeds<LED_TYPE, LEG_LED_PIN, COLOR_ORDER>(legleds, LEG_LEDS);

  // add head led strip(s)
  FastLED.addLeds<LED_TYPE, HEAD_LED_PIN, COLOR_ORDER>(headleds, HEAD_LEDS);

  // add ring led strip
  FastLED.addLeds<LED_TYPE, RING_LED_PIN, COLOR_ORDER>(ringleds, RING_LEDS);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.setCorrection(TypicalLEDStrip);
  FastLED.setTemperature(Candle);
  FastLED.setDither(BRIGHTNESS < 255);

  // power mgmt
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);

  Serial.print("Initialize MPU6050...");

  uint8_t gyro_tries = 10; // attempt gyro detect for 5 seconds
  
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G) && --gyro_tries > 0)
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  if(gyro_tries < 1){
    noGyro = true;
    Serial.println("No Gyro found");
  } else {
    Serial.println("found MPU6050 gyro.");
    #ifdef CALIBRATE_GYRO
    Serial.print("Calibrating gyro...");
    mpu.calibrateGyro();
    Serial.println("calibration complete.");
    #endif
  }

  Serial.print("Initialize TCS34725...");
  if (tcs.begin()) {
    Serial.println("Found RGB sensor");
    tcs.setInterrupt(true);
  } else {
    noRgbSensor = true;
    Serial.println("No TCS34725 found ... check your connections");
  }

  pinMode(RGB_SENSOR_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("all systems go.");
}

/**
 * Convert RGB -> eye-recognized gamma color
 * Very inefficient to call this frequently, but this is only used when the 
 * "sense color" button is pressed. The freeing of 256B of memory was worth 
 * the performance hit upon infrequent user input. 
 */
uint8_t getGamma(float x){
  x /= 255;
  x = pow(x, GAMMA_MULTIPLIER);
  x *= 255;
  return (uint8_t) x;
}

void loop() {

  static uint8_t howfar = MIN_LIT; // start with the minimum leds lit

  static uint32_t headled_mw, ringled_mw, legled_mw;

  EVERY_N_MILLISECONDS( 1000/FRAMES_PER_SECOND ) { 
    rainbowHue +=1; // slowly cycle the "base color" through the rainbow
  }

  #if DEBUG_POWER
  EVERY_N_SECONDS( 2 ) {

    headled_mw = scale8(8 * calculate_unscaled_power_mW(headleds, HEAD_LEDS), BRIGHTNESS);
    ringled_mw = scale8(calculate_unscaled_power_mW(ringleds, RING_LEDS), BRIGHTNESS);
    legled_mw = scale8(8 * calculate_unscaled_power_mW(legleds, howfar), BRIGHTNESS);

    Serial.print("headleds:");
    Serial.println(headled_mw);
    
    Serial.print("ringleds:");
    Serial.println(ringled_mw);

    Serial.print("legleds:");
    Serial.println(legled_mw);

    Serial.print("total:");
    Serial.println(legled_mw + ringled_mw + headled_mw);

    Serial.println("");
  }
  #endif
  
  // EVERY_N_MILLISECONDS( GRAVITY_SAMPLE_RATE ) { 
  //   detectJump();
  //   howfar = (uint8_t) map(constrain(jumpVelocity, GRAVITY_LOW, GRAVITY_HIGH), GRAVITY_LOW, GRAVITY_HIGH, MIN_LIT, LEG_LEDS);
  // }

  detectJump();
  howfar = (uint8_t) map(constrain(jumpVelocity, GRAVITY_LOW, GRAVITY_HIGH), GRAVITY_LOW, GRAVITY_HIGH, MIN_LIT, LEG_LEDS);

  #if DEBUG
    Serial.print("How Far:"); Serial.print(howfar); Serial.print(",");
  #endif

  // fade out unused legleds
  legleds(howfar, LEG_LEDS).fadeToBlackBy(FADE_RATE);
//  legleds(howfar, LEG_LEDS-howfar+1).nscale8(192);
//  legleds(howfar, LEG_LEDS-howfar).fill_solid(CRGB::Black);
//  fadeToBlackBy(ledsl, LEG_LEDS, FADE_RATE);    
//  legleds.fill_solid(CRGB::Black);

  // set the active leg leds
  colorwaves(legleds, howfar, lastPalette);
  // waveit(legleds, howfar, lastPalette);
  // alternative ways of filling leg leds
//  fill_solid(legleds, howfar, lastColor);
//  fill_gradient_RGB(legleds, 0, lastColor, howfar, CRGB::Black);
//  fill_rainbow(legleds, howfar, rainbowHue, 7);

  // set the ring leds
  colorwaves(ringleds, RING_LEDS, lastPalette);
  //fill_solid(ringleds, RING_LEDS, lastColor);

  // set the head leds 
  colorwaves(headleds, HEAD_LEDS, lastPalette);
  // waveit(headleds, HEAD_LEDS, lastPalette);

  FastLED.show();
  
  FastLED.delay(1000/FRAMES_PER_SECOND);

  senseColor();

  #if DEBUG
  Serial.println("");
  #endif

}

void detectJump(){
  
  if(noGyro){
    return false;
  }

  jumpVelocity = mpu.readNormalizeAccel().ZAxis; // Read normalized Z value
  xVelocity = mpu.readNormalizeAccel().XAxis; // Read normalized X value
  yVelocity = mpu.readNormalizeAccel().YAxis; // Read normalized Y value

  #if DEBUG

    if(jumpVelocity > hardestJump){
      hardestJump = jumpVelocity; 
      Serial.print("hardestJump:"); Serial.println(hardestJump);
    }
    
    if(jumpVelocity < lightestJump){
      lightestJump = jumpVelocity; 
      Serial.print("lightestJump:"); Serial.println(lightestJump);
    }
    
    Serial.print("Jump Velocity:"); Serial.print(jumpVelocity); Serial.print(",");

  #endif 

}


/**
 * Check for analog button press, interrupt RGB sensor (to light LED), read values, then disable LED. 
 * Use consumed values to create new CRGB color and Palette object for consumption by main loop.
 */
void senseColor(){

  if(noRgbSensor) {
    Serial.println("no RGB sensor found");
    return;
  }
  
  // check analog switch 
  if(digitalRead(RGB_SENSOR_BUTTON_PIN)){
    return;
  }
  
  uint32_t sum;
  uint16_t red, green, blue, clr;
  float r, g, b;
  
  tcs.setInterrupt(false);  // turn on LED

  delayMicroseconds(60 * 1000); // takes 50ms to read

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

  #if DEBUG
  // Serial.println("From sensor:");
  // Serial.print("R:\t"); Serial.print(int(red));
  // Serial.print("\tG:\t"); Serial.print(int(green));
  // Serial.print("\tB:\t"); Serial.print(int(blue));
  // Serial.print("\t");
  // Serial.print((int)red, HEX); Serial.print((int)green, HEX); Serial.print((int)blue, HEX);
  // Serial.print("\n");

  // Serial.println("Modified:");
  // Serial.print("R:\t"); Serial.print((int)r);
  // Serial.print("\tG:\t"); Serial.print((int)g);
  // Serial.print("\tB:\t"); Serial.print((int)b);
  // Serial.print("\t");
  // Serial.print((int)r, HEX); Serial.print((int)g, HEX); Serial.print((int)b, HEX);
  // Serial.print("\n");

  // Serial.println("Gamma corrected:");
  
  Serial.print("R:"); Serial.print(getGamma(r));
  Serial.print(",");
  Serial.print("G:"); Serial.print(getGamma(g));
  Serial.print(",");
  Serial.print("B:"); Serial.print(getGamma(b));
  Serial.print(",");
  
  // Serial.print("\t");
  // Serial.print(getGamma(r), HEX); Serial.print(getGamma(g), HEX); Serial.print(getGamma(b), HEX);
  // Serial.print("\n");
  // Serial.print("\n");
  #endif

  lastColor = CRGB(getGamma(r), getGamma(g), getGamma(b));
  lastPalette = CRGBPalette16(lastColor);
}

// This function draws color waves with an ever-changing,
// widely-varying set of parameters, using a color palette.
void colorwaves( CRGB* ledarray, int howfar, CRGBPalette16& palette) {
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;
 
  // uint8_t sat8 = beatsin88( 87, 220, 255);
  uint8_t brightdepth = beatsin88( 341, 96, 255);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beat8(147); //beatsin88(147, 23, 60); - creates a more dynamic pattern [IH]
  //uint8_t msmultiplier = beatsin88(map(constrain(jumpVelocity, GRAVITY_LOW, GRAVITY_HIGH), GRAVITY_LOW, GRAVITY_HIGH, 24, 256), 24, 256); //- creates a more dynamic pattern [IH]
 
  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 300, 1500);
  
  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5,9);
  uint16_t brightnesstheta16 = sPseudotime;
  
  for( uint16_t i = 0 ; i < howfar; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;
    uint16_t h16_128 = hue16 >> 7;
    if( h16_128 & 0x100) {
      hue8 = 255 - (h16_128 >> 1);
    } else {
      hue8 = h16_128 >> 1;
    }
 
    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;
 
    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);
    
    uint8_t index = hue8;
    //index = triwave8( index);
    index = scale8( index, 64);
    #if DEBUG
      Serial.print("brightdepth:"); Serial.print(brightdepth); Serial.print(",");
      Serial.print("bri8:"); Serial.print(bri8); Serial.print(",");
      Serial.print("brightdepth:"); Serial.print(brightdepth); Serial.print(",");
    #endif 
 
    CRGB newcolor = ColorFromPalette( palette, index, bri8);
 
    uint16_t pixelnumber = i;
    pixelnumber = (howfar-1) - pixelnumber;
    
    nblend( ledarray[pixelnumber], newcolor, 128);
  }
}

void waveit(CRGB* ledarray, int howfar, CRGBPalette16& palette){
  //int rate = map(constrain(jumpVelocity, GRAVITY_LOW, GRAVITY_HIGH), GRAVITY_LOW, GRAVITY_HIGH, 20, 150);
  // uint8_t bri = beatsin8(32, 32, 248);
  
  uint8_t index = beatsin8(40, 200, 255);
  CHSV newcolor = rgb2hsv_approximate(lastColor);
  newcolor.saturation = index;
  for(uint8_t i = 0 ; i < howfar ; i++){
    // uint8_t bri = inoise8(i);
    // uint16_t index = scale8(128, i);
    // Serial.print("scale:"); Serial.print(scale); Serial.print(",");
    // CRGB newcolor = ColorFromPalette(palette, 0, scale);
    // ledarray[i] = newcolor;
    // ledarray[i] = ColorFromPalette(palette, index, bri);
    // nblend( ledarray[i], ColorFromPalette(palette, index, bri), 128 );
    ledarray[i] = newcolor;

  }
  
}
