/*********
  Cochlea.ino       Visual Ear. 
  (c) Phil Malone
  
  This version has the following attributes:
  LED Type          DotStar
  LED Num           25
  LED Data          12
  LED Clk           14
  Number of LEDs    42
  
  Mic Type          I2S
  Mic Data          33
  Mic Clk           26
  Mic WS            25
  Sample Freq       36 kHz
  Samples per burst 1024
  Number of Bursts  6
  Bursts per FFT    4
  
*********/

#define  FASTLED_INTERNAL
#include <FastLED.h>
#include "driver/i2s.h"
#include "arduinoFFT_float.h"

TaskHandle_t AudioTask;
TaskHandle_t FFTTask;

///=================  Shared Data =================
#define SAMPLING_FREQ   36000
#define BURST_SAMPLES    1024
#define BURST_SAMPLES_2    10
#define SIZEOF_BURST     (BURST_SAMPLES << 2)
#define NUM_BURSTS          6
#define BURSTS_PER_AUDIO    4
#define UNUSED_AUDIO_BITS  16  // Was 18

#define FFT_SAMPLES      4096
#define FFT_SAMPLES_2      11
#define FREQ_BINS        2048

#define NUM_BANDS          42
#define AMPLITUDE        1400         // Depends on the audio source level (Increase is levels are higher)  1600
#define LED_DATA_PIN       12
#define LED_CLOCK_PIN      14
#define BRIGHTNESS        255         // Brightness 0 - 255, 

#define START_NOISE_FLOOR 3000         // Used as a crude noise filter (Increase for noisy signals)
#define BASE_NOISE_FLOOR  1000         // Used as a crude noise filter (Increase for noisy signals)

#define BAND_HUE_STEP   (200/NUM_BANDS)

#define NUM_LEDS       NUM_BANDS      // Total number of LEDs
///=================  Shared Data =================
#define BUFFER_READY   0
#define BUFFER_FILLING 1
#define BUFFER_FULL    2

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

const   i2s_port_t I2S_PORT = I2S_NUM_0;

uint8_t windowing_type = FFT_WIN_TYP_HANN ;
/*
 * Windowing types
 *  FFT_WIN_TYP_RECTANGLE
 *  FFT_WIN_TYP_HAMMING
 *  FFT_WIN_TYP_HANN
 *  FFT_WIN_TYP_TRIANGLE
 *  FFT_WIN_TYP_NUTTALL
 *  FFT_WIN_TYP_BLACKMAN
 *  FFT_WIN_TYP_BLACKMAN_NUTTALL
 *  FFT_WIN_TYP_BLACKMAN_HARRIS
 *  FFT_WIN_TYP_FLT_TOP
 *  FFT_WIN_TYP_WELCH
 */

// Data written by Task 0 
int32_t   audioBuffer0[BURST_SAMPLES];
int32_t   audioBuffer1[BURST_SAMPLES];
int32_t   audioBuffer2[BURST_SAMPLES];
int32_t   audioBuffer3[BURST_SAMPLES];
int32_t   audioBuffer4[BURST_SAMPLES];
int32_t   audioBuffer5[BURST_SAMPLES];

int32_t  *audioBuffers[NUM_BURSTS] = {audioBuffer0, audioBuffer1, audioBuffer2, audioBuffer3, audioBuffer4, audioBuffer5}; 

int8_t    nowFilling = -1;
int8_t    nowProcessing = -1;
bool      goodAudioRead = true;

float     vReal[FFT_SAMPLES];
float     vImag[FFT_SAMPLES];
float     weights[FFT_SAMPLES];

uint32_t  bandValues[NUM_BANDS];
uint16_t  bandMaxBin[NUM_BANDS] = {7,8,9,10,12,13,15,18,20,23,27,31,35,41,47,54,62,71,82,94,108,124,142,163,187,215,247,284,326,375,430,494,568,652,749,861,989,1136,1304,1498,1721,2047};

arduinoFFT_float FFT = arduinoFFT_float(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQ);
CRGB       leds[NUM_LEDS];

void setup() {
  Serial.begin(1000000); 
  for (int i=0; i< NUM_BURSTS; i++) {
    memset(audioBuffers[i], 0, sizeof(audioBuffer0));
  }

  Serial.println("\nCochlea Started.");
  
  // init_wifi("HACKYOU", "RobotsRule", "ORGAN");
  // webota.init(80, "/organ");
  
  xTaskCreatePinnedToCore(
                    AudioSample,   /* Task function. */
                    "AudioSample", /* name of task. */
                    20000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &AudioTask,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */                  
  delay(2000); 

  xTaskCreatePinnedToCore(
                    FFTcode,     /* Task function. */
                    "FFT",       /* name of task. */
                    40000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &FFTTask,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */
  delay(100); 
}

// ###############################################################
//  AudioSample: Sample the microphone and fill buffer with amplitude data
// ###############################################################
void AudioSample( void * pvParameters ){

  esp_err_t err;
  int32_t   sample;
  size_t    bytesRead;
  
  // The I2S config as per the example
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),  // Receive, not transfer
      .sample_rate = SAMPLING_FREQ,                       // 40960 Hz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,       // could only get it to work with 32bits
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,        // use right channel
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,           // Interrupt level 1
      .dma_buf_count = 4,                                 // number of buffers
      .dma_buf_len = 128                                  // 128 samples per buffer (minimum)
  };

  // The pin config as per the setup
  const i2s_pin_config_t pin_config = {
      .bck_io_num = 26,                                   // Serial Clock (SCK)
      .ws_io_num = 25,                                    // Word Select (WS)
      .data_out_num = I2S_PIN_NO_CHANGE,                  // not used (only for speakers)
      .data_in_num = 33                                   // Serial Data (SD)
  };

  FastLED.addLeds<DOTSTAR, LED_DATA_PIN, LED_CLOCK_PIN, BGR>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  // preload the hue into each LED and set the saturation to full and brightness to low.
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(i * BAND_HUE_STEP , 255, 10);
  }
  FastLED.show();

  //  ######## SETUP Audio Sample #########
  Serial.print("SampleAudio Started on core ");
  Serial.println(xPortGetCoreID());

  // Configuring the I2S driver and pins.
  // This function must be called before any I2S driver read/write operations.
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true);
  }
  
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed setting pin: %d\n", err);
    while (true);
  }
  delay(10);

  //  ######## Audio Sample LOOP #########
  nowFilling    = 0;
  goodAudioRead = true;
  
  for(;;){
    // Start filling the next available Burst Buffer (with wrap around)
    i2s_read(I2S_PORT, (char *)(audioBuffers[nowFilling]), SIZEOF_BURST, &bytesRead, 100);

    // indicate full if all the samples were taken.
    if (bytesRead == SIZEOF_BURST)
      nowFilling = (nowFilling + 1) % NUM_BURSTS ;
    delayMicroseconds(5);

    //while (nowProcessing >= 0){   //
    //  delay(10);                  //  DEBUG
    //}                             //
  } 
}

// ###############################################################
// FFTcode: Process a buffer when it's ready
// ###############################################################
void FFTcode( void * pvParameters ){
  
  uint32_t  lastTime = millis();
  uint32_t  thisTime = millis();
  
  Serial.print("FFT Started on core ");
  Serial.println(xPortGetCoreID());

  // Load up the windowing weights
  for (int i=0; i < FFT_SAMPLES; i++) {
    weights[i] = 1.0;
  }
  FFT.Windowing(weights, FFT_SAMPLES, windowing_type, FFT_FORWARD);

  //  ######## LOOP #########
  for(;;){
    if (nowProcessing != nowFilling) {
      nowProcessing = nowFilling;
      delay(1);

      // Process the 
      ComputeMyFFT();
      updateDisplay();
      // delay(1000);
    }
    
    delayMicroseconds(50);
  }
}

void loop() {
 //  webota.handle();
}

void ComputeMyFFT(void) {
  
  int32_t bufferDC ;
  int32_t * burstP;
  float tempF;

  // Determine DC component from Burst 0
  bufferDC = 0;
  burstP = audioBuffers[0];
  for (uint16_t i = 0; i < BURST_SAMPLES; i++) {
    bufferDC += (*burstP++ >> UNUSED_AUDIO_BITS);
  }      
  bufferDC = bufferDC >> BURST_SAMPLES_2;

  // transfer audio buffer into real values while removing DC component.  One burst at a time.
  for (uint8_t b = 0; b < BURSTS_PER_AUDIO; b++) {
    burstP = audioBuffers[(nowProcessing + NUM_BURSTS + b - BURSTS_PER_AUDIO) % NUM_BURSTS];
    uint16_t  burstOffset = BURST_SAMPLES * b;
    
    for (uint16_t i = 0; i < BURST_SAMPLES; i++) {
      tempF = (float)((*burstP++ >> UNUSED_AUDIO_BITS ) - bufferDC) * weights[burstOffset + i];
      vReal[burstOffset + i] = tempF;
      // Serial.println(tempF);
    }      
  }

  // Clear out imaginary values;
  memset((void *)vImag, 0, sizeof(vImag));

  // Now do the FFT
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
}

void  updateDisplay (void){
  uint16_t ledBrightness;

  // Allocate FFT results into LED buckets
  fillBuckets ();
  
  // Process the LED buckets into LED Intensities
  for (byte band = 0; band < NUM_BANDS; band++) {
    
    // Scale the bars for the display
    ledBrightness = bandValues[band] / AMPLITUDE;
    if (ledBrightness > BRIGHTNESS) 
      ledBrightness = BRIGHTNESS;
  
    // Display LED bucket in the correct Hue.
    leds[band].setHSV(band * BAND_HUE_STEP, 255, ledBrightness);
  }
  
  // Update LED display
  FastLED.show();
}

void  fillBuckets (void){
  uint16_t  frequency; 
  uint16_t  bucket; 
  uint16_t  minBucket = 1; // skip over the DC level, and start with second freq.
  uint16_t  maxBucket = 0; 
  uint32_t  bandValue;
  uint32_t  noiseFloor;     

  //  zero out all the LED band magnitudes.
  memset(bandValues, 0, sizeof(bandValues));

  // Serial.println("\nBucket\tValue\t\tBand\tTotal\tNoiseFloor ");
  
  // Cycle through each of the LED bands.  Set noise threshold high and drop down.
  noiseFloor = START_NOISE_FLOOR;
  minBucket = bandMaxBin[0];
  
  for (int band = 0; band < NUM_BANDS; band++){
    // get the new maximum freq for this band.
    maxBucket = bandMaxBin[band];


    // Accumulate freq values from all bins that match this LED band,
    for (int bucket = minBucket; bucket <= maxBucket; bucket++) {
      bandValue = (uint32_t)vReal[bucket];          
      if (bandValue > noiseFloor) {
        bandValues[band] += bandValue;  
      }

      /*      
      Serial.print(bucket);
      Serial.print("\t");
      Serial.print(bandValue);
      Serial.print("\t\t");
      Serial.print(band);
      Serial.print("\t");
      Serial.print(bandValues[band]);
      Serial.print("\t");
      Serial.println(noiseFloor);
      */
    }
    
    // slide the max of this band to the min of next band.
    minBucket = maxBucket + 1;

    // Adjust Noise Floor
    if (noiseFloor > BASE_NOISE_FLOOR) {
      noiseFloor = 95 * noiseFloor / 100;  // equiv 0.95 factor.
    }
  }
}
