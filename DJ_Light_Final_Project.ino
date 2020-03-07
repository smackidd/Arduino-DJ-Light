#include <Adafruit_NeoPixel.h>
#include <FastLED.h>
//#include <SPI.h>
//#include <arduinoFFT.h>
FASTLED_USING_NAMESPACE;
//bring in FastLED and FFT libraries?

#define LED_PIN         12   // input pin Neopixel is attached to
#define BUTTON_PIN      2    // input pin for mode button
#define MODE_MAX        5    // maximum number of modes
#define DEBOUNCE        50   // button debounce delay
#define POT_PIN         A1   // Analog pin of the Potentiometer
#define MIC_PIN         A5
#define SAMPLE_WIN      50
#define NOISE           330                      
    
#define NUMPIXELS       120  // Number of pixels for the entire board
#define NUMPIX_STRIP    12   // Number of pixels per strip
#define NUM_STRIPS      10   // Number of strips
#define GRAVITY         1    // Gravity of falling dot

#define BAUD            9600 // Baud rate for the serial port

#define STANDBY_DELAY   500  // delay before cycling the standby leds


CRGB leds[NUMPIXELS];           // Sets up a multidimensional array of 10 strips of 12 pixels each.



Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, MIC_PIN, NEO_GRB + NEO_KHZ800);


            
int buttonState = 0;
int lastButtonState = 0;
int modeCounter = 0;
int16_t value = 0;             // initializes the brightness value

bool ascending = true;         // sets whether a loop is ascending or descending.

uint8_t peakDot[NUM_STRIPS] = {0};              // peak dot
uint8_t pixelOn = 0; 
uint8_t dotCount[NUM_STRIPS] = {0};             // delay for falling dot
float hue = 0;
unsigned long startMillis = 0; // ms. delay counter
uint8_t randPixel[60] = {0};   // an array of 60 random pixels generated
uint8_t index = 0;             // array index

//OUTPUT THE PIXEL COLORS
//void pixelShow (uint32_t potValue) {
//  for (int i = 0; i < NUMPIXELS; i++) {
    
//  }
//Set the colors and pixel numbers to turn on
//set the pixel numbers to turn off
//show and clear pixels.
//}

//ANALYZE FFT
//

//READ THE MICROPHONE
unsigned int micRead(){
  unsigned int signalMax = 0;
  unsigned int signalMin = 1023; 
  unsigned int peakToPeak = 0;   // peak-to-peak level
  unsigned int sample = 0;
  
  startMillis = millis();  // Start of sample window
  Serial.println("collecting samples");
  //collect samples in an array from 0-1024, 50ms at a time
  while (millis() - startMillis < SAMPLE_WIN) {
    
    sample = analogRead(MIC_PIN);
    Serial.print("sample: "); Serial.print(sample); Serial.print(" ");
    
       if (sample > signalMax)
       {
          signalMax = sample;  // save just the max levels
       }
       if (sample < signalMin)
       {
          signalMin = sample;  // save just the min levels
       }
    
     
  }
  
  Serial.println();
  Serial.print("signalMax: "); Serial.print(signalMax); Serial.print(" "); 
  Serial.print("signalMin: "); Serial.println(signalMin); 
    
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  Serial.print("peakToPeak: "); Serial.print(peakToPeak);
  double volts = (peakToPeak * 5.0) / 1024;  // convert to volts 
  Serial.print("volts: "); Serial.println(volts);  
  return peakToPeak;
}

//MODE 5 - OFF
void mode5(uint32_t potValue){
  value = 0;

  for (int i = 0; i < NUMPIXELS; i++) {
    leds[i] = CHSV(hue,255,value);
  }
  FastLED.show();
}

//MODE 4
//random leds turning on 
int mode4(uint32_t potValue) {
  const int randomWindow = 100;
  if (startMillis == 0) {                 // if this is the first time entering the mode
    startMillis = millis(); //
  }
  
  while (millis() - startMillis < randomWindow) return 0; // delay for a period of time before initializing a new pixel
  startMillis = 0;
  if (index == 60) {                      // include only 20 pixels at a time, wrap-around the index after 20.
    index = 0;
    leds[randPixel[index]] = CRGB::Black;
  }
  randPixel[index] = random(0,NUMPIXELS);
  uint8_t pallette = map(potValue, 0, 1023, 0, 150);
  leds[randPixel[index]] = CHSV(pallette + index, random(0,255), 255);
  FastLED.show();
  for (int i = 0; i < 60; i++) {          // run through the array of 20 random pixels and fade them a bit more.
    leds[randPixel[i]].fadeToBlackBy(16);
  }
  index++;
  leds[randPixel[index]] = CHSV(pallette, 255, 10);
  
  return 0;
    
}

//MODE 3
//void mode3(uint32_t potValue){
//read the microphone
//  Serial.println("Reading microphone");
//  unsigned int RMS = micRead();
//  Serial.println();
//  Serial.println(RMS);
//analyze the fft
//output a snaking pattern of colors that, each pattern reacting to the frequency band output.
//}

//MODE 2 - VU Meter
void mode2(uint32_t potValue) {
//read the microphone
//  Serial.println("Reading microphone");
  unsigned int RMS = micRead();
  uint8_t scale = map(potValue, 0, 1023, 15, 100);
  int displayPeak = map(RMS, 0, 20, 0, NUMPIX_STRIP) - 1;
  Serial.print("displayPeak: "); Serial.println(displayPeak);
  for (int strip = 1; strip <= NUM_STRIPS; strip++) {
    
    int centerBias = strip - 6;                      // bias the display peak away from the center strips of 5 and 6
    if (centerBias < 0) {
      centerBias += 1;
      centerBias = abs(centerBias);
    }
    
    for (int pixel = 0; pixel < NUMPIX_STRIP; pixel++) {
      pixelOn = (strip - 1) * NUMPIX_STRIP + pixel;
      
      if (pixel <= displayPeak - centerBias) 
        leds[pixelOn] = CHSV(210, 150 + abs(pixel * -1 * 6), 100);
      else
        leds[pixelOn] = CRGB::Black;
    }
    int top = displayPeak - centerBias;
    if (top > peakDot[strip])
      peakDot[strip] = top;
    Serial.print("peakDot: "); Serial.print(peakDot[strip]); Serial.print(" ");
    if (peakDot[strip] > 0 && peakDot[strip] <= NUMPIX_STRIP) {
      pixelOn = (strip - 1) * NUMPIX_STRIP + peakDot[strip];
      Serial.print("pixelOn: "); Serial.print(pixelOn); Serial.print(" ");
      leds[pixelOn] = CHSV(160, 150, 255);
    }
    if (++dotCount[strip] > GRAVITY) {
      if (peakDot[strip] > 0) peakDot[strip]--;
      dotCount[strip] = 0;
    }
  }
  Serial.println();
  FastLED.show();

    // peakDot reset to the top
  

  

  
//  Serial.println();
//  Serial.println(RMS);
//analyze the rms peak value
//output the volume as a horizontal mirror effect
}

//MODE 1 - full color cycle
void mode1(uint32_t potValue){
  
  value = potValue * 255 / 1023;
  hue += 0.5;
  Serial.print("hue: "); Serial.println(hue);
  if (hue >= 255) hue = 1;
  for (int i = 0; i < NUMPIXELS; i++) {
    leds[i] = CHSV(hue,255,value);
  }
  FastLED.show();
  //analyze the fft 
  //output the frequencies to the different bands of light strips
}

//MODE 0 - standby mode
void mode0(uint32_t potValue) {
  
  //if (value == 0) delay(STANDBY_DELAY); 
  if (ascending) {value += 10;} else {value -= 10;}  //increase or decrease the brightness value
  Serial.print("ascending: "); Serial.println(ascending); 
  if (value > 255) {
    ascending = false; // if the value has reached the end of its loop, switch the ascending/desceding mode
    value = 255;
  } else if (value <= 0) {
    ascending = true;
    value = 0;
  }
  
  for (int pixel = 6; pixel <= 7; pixel++) {
    for (int strip = 5; strip <= 6; strip++){
      pixelOn = (strip - 1) * NUMPIX_STRIP + (pixel - 1);  // find the led value for the current pixel in the current strip
      Serial.print("pixel: "); Serial.print(pixelOn); Serial.print(" ");
      Serial.print("value: "); Serial.print(value); Serial.print(" ");
      leds[pixelOn] = CHSV(96,255,value);
    }
  }
  FastLED.show();
  Serial.println();
  
}




void setup() {
   Serial.begin(BAUD); //initialize the serial port
   //pixels.begin();      //initialize the pixel strips
   ADCSRA |= bit (ADPS0);      // set ADC to free running mode and set pre-scalar to 16 
   //ADMUX = MIC_PIN;       // use pin A0 and external voltage reference
    
   FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUMPIXELS);
   pinMode(BUTTON_PIN, INPUT);
   //setup power and stand-by LED pin
   //turn on stand-by LED
   Serial.println("start");
   randomSeed(analogRead(5));
}

void loop() {
 //read the power button

//if power-on:
  //turn on power LED
  //turn off stand-by LED
  //output the servo to bring up LCD screen
  //output initial power-on pattern
//if power-off:
  //ouput the servo to let the LCD screen down.
  //output the power-off pattern
  //turn off power LED
  //turn on stand-by LED
//read the dimmer pot
  uint32_t potValue = analogRead(POT_PIN);
  float dimValue = potValue * 255 / 1023;
//read the mode button
  buttonState = digitalRead(BUTTON_PIN);
  if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {
      modeCounter++;
      if (modeCounter == MODE_MAX) modeCounter = 0;    // loop the mode counter back to 0 if past the last mode
      FastLED.clear();
    }
  }
  lastButtonState = buttonState; 
  Serial.print("mode: ");Serial.println(modeCounter);
  //delay(DEBOUNCE);
 
  
//run the current mode
  switch (modeCounter) {
    case 0:
      mode0(potValue);
      break;
    case 1:
      mode1(potValue);
      break;
    case 2:
      mode2(potValue);
      break;
    case 3:
      mode4(potValue);
      break;
    case 4:
      mode5(potValue);
      break;
    default: 
      mode0(potValue);
  }
  

}
