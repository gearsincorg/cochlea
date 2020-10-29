/*********
  Phil Malone
  This version has the following attributes:
  LED Type  DotStar
  LED Num   25
  LED Data  12
  LED Clk   14
  
  Mic Type  I2S
  Mic Data  33
  Mic Clk   26
  Mic WS    25
  Sample Freq 32 kHz
  Samples   1024
  
*********/

#define FASTLED_INTERNAL
#include <FastLED.h>
#include "driver/i2s.h"
#include "arduinoFFT.h"
#include "WebOTA.h"

void  fillBuckets (void);

TaskHandle_t Task1;
TaskHandle_t Task2;

///=================  Shared Data =================
#define SAMPLING_FREQ   32000
#define AUDIO_SAMPLES    1024
#define FREQ_BINS         512
#define AUDIO_SAMPLES_2    10
#define NUM_BANDS          25
#define AMPLITUDE         300         // Depending on your audio source level, you may need to alter this value. Can be used as a 'sensitivity' control.
#define LED_DATA_PIN       12
#define LED_CLOCK_PIN      14
#define BRIGHTNESS        255         // Brightness 0 - 255, 
#define NOISE             100         // Used as a crude noise filter, values below this are ignored
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

int32_t audioBuffer[AUDIO_SAMPLES];
int8_t  bufferStatus = BUFFER_READY;
double  vReal[AUDIO_SAMPLES];
double  vImag[AUDIO_SAMPLES];
uint32_t bandValues[NUM_BANDS];
uint16_t bandMaxBin[NUM_BANDS] = {1,2,3,4,5,6,8,10,13,16,20,25,32,40,51,64,80,101,127,160,202,255,321,404,512};

arduinoFFT FFT = arduinoFFT(vReal, vImag, AUDIO_SAMPLES, SAMPLING_FREQ);
CRGB       leds[NUM_LEDS];

void setup() {
  Serial.begin(250000); 
  memset(audioBuffer, 1, sizeof(audioBuffer));
  init_wifi("HACKYOU", "RobotsRule", "ORGAN");
  webota.init(80, "/organ");
  
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    SampleAudio,   /* Task function. */
                    "SampleAudio", /* name of task. */
                    20000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */                  
  webota.delay(2000); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    FFTcode,     /* Task function. */
                    "FFT",       /* name of task. */
                    40000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 1 */
  webota.delay(500); 
}

// ###############################################################
//  SampleAudio: Sample the microphone and fill buffer with amplitude data
// ###############################################################
void SampleAudio( void * pvParameters ){

  esp_err_t err;
  int32_t   sample;
  size_t    bytesRead;
  
  // The I2S config as per the example
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),  // Receive, not transfer, Power Density Modulation
      .sample_rate = SAMPLING_FREQ,                       // 
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,       // could only get it to work with 32bits
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,        // use right channel
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,           // Interrupt level 1
      .dma_buf_count = 4,                                 // number of buffers
      .dma_buf_len = AUDIO_SAMPLES                        // 
  };

  // The pin config as per the setup
  const i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_PIN_NO_CHANGE,                    // Serial Clock (SCK)
      .ws_io_num = 26,                                    // Word Select (WS)
      .data_out_num = I2S_PIN_NO_CHANGE,                  // not used (only for speakers)
      .data_in_num = 33                                   // Serial Data (SD)
  };

  //  ######## SETUP #########
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
  delay(1000);

  //  ######## LOOP #########
  for(;;){
    // Wait for flag indicating that the last buffer has been transfered and is ready for sampling again.
    if (bufferStatus == BUFFER_READY) {
      //Update the buffer status and start sampling.
      bufferStatus = BUFFER_FILLING;
      i2s_read(I2S_PORT, (char *)&audioBuffer, sizeof(audioBuffer), &bytesRead, 100);

      // indicate full if all the samples were taken.
      if (bytesRead == sizeof(audioBuffer)){
        bufferStatus = BUFFER_FULL;
      } else {
        bufferStatus = BUFFER_READY;
      }
    }

    // yeild the CPU to enable the other CPU core to transfer out the date to start FFT process.
    delay(1);
  } 
}

// ###############################################################
// FFTcode: Process a buffer when it's ready
// ###############################################################
void FFTcode( void * pvParameters ){
  
  uint16_t ledBrightness;
  int32_t  bufferDC ;
  unsigned long startFFT = micros();
  unsigned long lastFFT = micros();

  Serial.print("FFT Started on core ");
  Serial.println(xPortGetCoreID());
  
  FastLED.addLeds<DOTSTAR, LED_DATA_PIN, LED_CLOCK_PIN, BGR>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  // preload the hue into each LED and set the saturation to full and brightness to low.
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(i * BAND_HUE_STEP , 255, 10);
  }
  FastLED.show();

  //  ######## LOOP #########
  for(;;){
    // Wait till the sample buffer is filled by other CPU core.
    if (bufferStatus == BUFFER_FULL) {

      startFFT = micros();

      // Remove DC component
      bufferDC = 0;
      for (uint16_t i = 0; i < AUDIO_SAMPLES; i++) {
        bufferDC += (audioBuffer[i] >>= 18);
      }      
      bufferDC = bufferDC >> AUDIO_SAMPLES_2;

      // transfer buffer into real values and remove dc component
      for (uint16_t i = 0; i < AUDIO_SAMPLES; i++) {
        vReal[i]  = audioBuffer[i] - bufferDC;
      }      

      // Release the buffer so the other CPU can start taking samples again.
      bufferStatus = BUFFER_READY;
      delay(1);

      // Now do the FFT while recording new samples
      memset((void *)vImag, 0, sizeof(vImag));
      FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.Compute(FFT_FORWARD);
      FFT.ComplexToMagnitude();

      // Allocate FFT results into LED buckets
      fillBuckets ();
      
      // Process the LED buckets into bar heights
      for (byte band = 0; band < NUM_BANDS; band++) {
        
        // Scale the bars for the display
        ledBrightness = bandValues[band] / AMPLITUDE;
        if (ledBrightness > BRIGHTNESS) 
          ledBrightness = BRIGHTNESS;

        Serial.println(ledBrightness);

        // Display LED bucket in the correct Hue.
        leds[band].setHSV(band * BAND_HUE_STEP, 255, ledBrightness);
      }
      Serial.println("-10 256");

      // Update LED display
      FastLED.show();

      // Display processing times
      uint32_t temp = micros();
      //Serial.print("Cycle = ");
      //Serial.print(temp - lastFFT );
      //Serial.print(" (");
      //Serial.print(startFFT - lastFFT);
      //Serial.println(")");
      
      lastFFT = micros();
      
    } else {
      delay(1);
    }
  }
}

void loop() {
  webota.handle();
}

void  fillBuckets (void){
  uint16_t  bucket; 
  uint16_t  minBucket = 1; // skip over the DC level, and start with second freq.
  uint16_t  maxBucket = 0; 
  uint32_t  bandValue;

  //  zero out all the LED band magnitudes.
  memset(bandValues, 0, sizeof(bandValues));
  
  // Cycle through each of the LED bands.
  for (int band = 0; band < NUM_BANDS; band++){
    // get the new maximum freq for this band.
    maxBucket = bandMaxBin[band];

    // Accumulate freq values from all bins that match this LED band,
    for (int bucket = minBucket; bucket <= maxBucket; bucket++) {
      bandValue = (uint32_t)vReal[bucket];          
      if (bandValue > NOISE) {
        bandValues[band] += bandValue;  
      }
    }
    // slide the max of this band to the min of next band.
    minBucket = maxBucket + 1;
  }
}
