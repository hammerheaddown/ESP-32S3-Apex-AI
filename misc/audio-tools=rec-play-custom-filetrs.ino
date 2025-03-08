#include "driver/i2s_std.h"
#include "Arduino.h"
#include <SPI.h>
#include <SD.h>
#include "FS.h"
#include "AudioTools.h"
#include "AudioTools/AudioLibs/AudioRealFFT.h"
#include "AudioTools/AudioCodecs/CodecWAV.h"

// -------------------------- Pin Setup --------------------------------------
#define SD_MISO GPIO_NUM_13
#define SD_MOSI GPIO_NUM_11
#define SD_SCK GPIO_NUM_12
#define SD_CS GPIO_NUM_10

#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_42  
#define I2S_MIC_SERIAL_DATA GPIO_NUM_17
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_18

#define I2S_BCLK GPIO_NUM_5
#define I2S_DOUT GPIO_NUM_6
#define I2S_LRC GPIO_NUM_14

#define PUSH_BUTTON_PIN GPIO_NUM_21
#define MY_DEBOUNCE_DELAY 100  									// Reduced to 100ms for faster response

// -------------------------- Config --------------------------------------
#define SAMPLE_RATE 16000
#define REC_BUFFER_SIZE 1024
const char* wavFilename = "/audio,wav";
AudioInfo info(SAMPLE_RATE, 1, 16); 							// Mono, centralized config

// -------------------------- Filter Config --------------------------------------
#define VOLUME_SCALE 1.0
#define LIMITER_THRESHOLD 25000
#define NOISE_GATE_THRESHOLD 500
#define HIGH_PASS_FREQ 100
#define LOW_PASS_FREQ 6000

// Filter State
float lastSampleHP = 0;
float lastSampleLP = 0;

// Custom Filter Stream (for limiter, noise gate, band-pass)
class FilteredAudioStream : public AudioStream {
public:
    FilteredAudioStream(AudioStream &source) : source(source) {}

    size_t readBytes(uint8_t *buffer, size_t length) override {
        size_t bytesRead = source.readBytes(buffer, length);
        if (bytesRead > 0) {
            int16_t *samples = (int16_t *)buffer;
            size_t sampleCount = bytesRead / sizeof(int16_t);
            for (size_t i = 0; i < sampleCount; i++) {
                int16_t sample = samples[i] * VOLUME_SCALE;

                // Noise Gate
                if (abs(sample) <= NOISE_GATE_THRESHOLD) {
                    sample = 0;
                } else {
                    // High-Pass Filter
                    float hp = highPassFilter(sample, &lastSampleHP, HIGH_PASS_FREQ, SAMPLE_RATE);
                    // Low-Pass Filter (Band-Pass = HP + LP)
                    float lp = lowPassFilter(hp, &lastSampleLP, LOW_PASS_FREQ, SAMPLE_RATE);
                    sample = (int16_t)lp;

                    // Limiter
                    if (sample > LIMITER_THRESHOLD) sample = LIMITER_THRESHOLD;
                    else if (sample < -LIMITER_THRESHOLD) sample = -LIMITER_THRESHOLD;
                }
                samples[i] = sample;
            }
        }
        return bytesRead;
    }

    // Required overrides (corrected to match base class)
    size_t write(const uint8_t *, size_t) override { return 0; }  
    int available() override { return source.available(); }
    bool begin() override { return source.begin(); } 

private:
    AudioStream &source;
    
    float highPassFilter(int16_t sample, float *lastSample, float cutoffFreq, float sampleRate) {
        float RC = 1.0 / (cutoffFreq * 2 * M_PI);
        float dt = 1.0 / sampleRate;
        float alpha = RC / (RC + dt);
        *lastSample = sample * (1.0 - alpha) + (*lastSample) * alpha;
        return sample - *lastSample;
    }

    float lowPassFilter(float sample, float *lastSample, float cutoffFreq, float sampleRate) {
        float RC = 1.0 / (cutoffFreq * 2 * M_PI);
        float dt = 1.0 / sampleRate;
        float alpha = dt / (RC + dt);
        *lastSample = alpha * sample + (1 - alpha) * (*lastSample);
        return *lastSample;
    }
};

// -------------------------- Audio Tools Setup --------------------------------------
I2SStream i2s_in;         										// Mic input
VolumeStream mic_volume(i2s_in); 								// Boost mic input 
FilteredAudioStream filteredStream(mic_volume); 				// Wrap mic_volume with filters

I2SStream i2s_out;
VolumeStream amp_volume(i2s_out);

File file;                										// SD file
WAVEncoder encoder;       										// WAV encoder
WAVDecoder decoder;       										// WAV decoder

AudioRealFFT fft;         										// FFT analysis
VolumeMeter volumeMeter;  										// Volume meter

EncodedAudioStream encoderStream(&file, &encoder); 				// Recording stream to file
StreamCopy copier;        										// Generic copier

// -------------------------- Global Variables --------------------------------------
bool isRecording = false;
bool interruptRequested = false;  								// Global interrupt flag for playback
String serialCommand = "";  									// Declare serialCommand globally

// --------------------------  Action enum  --------------------------------------
enum Action {
  NONE,
  RECORD,
  STOP,
  PLAY
};

// -------------------------- FFT Callback --------------------------------------
void fftResult(AudioFFTBase &fft) {
  auto result = fft.result();
  if (result.magnitude > 100) {
    //Serial.print("Frequency: ");  							// Commented out to reduce Serial noise
    //Serial.print(result.frequency);							// Commented out to reduce Serial noise
    //Serial.print(" Hz | Magnitude: ");						// Commented out to reduce Serial noise
    //Serial.println(result.magnitude);							// Commented out to reduce Serial noise
  }
}

// -------------------------- Setup Functions --------------------------------------
void SD_setup() {
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  delay(10);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  SPI.setFrequency(4000000);
  delay(100);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card mount failed!");
    while (1);
  }
  //Serial.println("SD card mounted successfully.");  			// Commented out to reduce Serial noise			
}

void mic_setup() {
  AudioToolsLogger.begin(Serial, AudioToolsLogLevel::Error);
  auto cfg_in = i2s_in.defaultConfig(RX_MODE);
  cfg_in.copyFrom(info);  										// Use centralized AudioInfo
  cfg_in.port_no = 0;
  cfg_in.pin_bck = I2S_MIC_SERIAL_CLOCK;
  cfg_in.pin_ws = I2S_MIC_LEFT_RIGHT_CLOCK;
  cfg_in.pin_data = I2S_MIC_SERIAL_DATA;
  cfg_in.buffer_size = 1024;
  cfg_in.buffer_count = 8;
  i2s_in.begin(cfg_in);

  auto vcfg = mic_volume.defaultConfig();
  vcfg.copyFrom(info);  										// Use centralized AudioInfo
  vcfg.allow_boost = true;
  mic_volume.begin(vcfg);
  mic_volume.setVolume(15.5); 									// Match working code, check for clipping
  //Serial.println("Mic setup complete.");  					// Commented out to reduce Serial noise
}

void amp_setup() {
    auto cfg_out = i2s_out.defaultConfig(TX_MODE);
    cfg_out.copyFrom(info);
    cfg_out.port_no = I2S_NUM_1;
    cfg_out.pin_bck = I2S_BCLK;
    cfg_out.pin_ws = I2S_LRC;
    cfg_out.pin_data = I2S_DOUT;
    i2s_out.begin(cfg_out);

    auto vcfg = amp_volume.defaultConfig();
    vcfg.copyFrom(info);
    vcfg.allow_boost = true;
    amp_volume.begin(vcfg);
    amp_volume.setVolume(10.0);
    Serial.println("Amp setup complete. Volume set to: (Value: " + String(10.0) + ")");
}

void pipeline_setup() {
  auto fftCfg = fft.defaultConfig();
  fftCfg.copyFrom(info);  										// Use centralized AudioInfo
  fftCfg.length = 1024;
  fftCfg.callback = &fftResult;
  fft.begin(fftCfg);

  volumeMeter.begin(info);  									// Use centralized AudioInfo
  //Serial.println("Pipeline setup complete.");  				// Commented out to reduce Serial noise

}

void setup() {
  delay(100);
  Serial.begin(115200);
  pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP);

  delay(10);
  SD_setup();
  #ifdef DEBUG_MODE
    Serial.println("Free Heap B4 we continue " + String(ESP.getFreeHeap()));
  #endif

  delay(10);
  mic_setup();
  #ifdef DEBUG_MODE
    Serial.println("Free Heap B4 we continue " + String(ESP.getFreeHeap()));
  #endif

  delay(10);
  amp_setup();
   #ifdef DEBUG_MODE
    Serial.println("Free Heap B4 we continue " + String(ESP.getFreeHeap()));
  #endif

  delay(10);
  pipeline_setup();
  #ifdef DEBUG_MODE
    Serial.println("Free Heap B4 we continue " + String(ESP.getFreeHeap()));
  #endif

  delay(10);
  printCommands();
  #ifdef DEBUG_MODE
    Serial.println("Free Heap B4 we continue " + String(ESP.getFreeHeap()));
  #endif
}

// -------------------------- Helper functions --------------------------------------
void printCommands() {   
    Serial.println("Commands: '1' record, '2' stop, '3' play, '8' or 'interrupt' interrupt");
    Serial.println("Commands:  Button toggles recording/interrupts playback via polling.");  
}

// -------------------------- Action Handling --------------------------------------
Action handleMenu() {
  // Button polling with debouncing (mirroring your working loop)
  static bool buttonState = HIGH;
  static bool lastButtonState = HIGH;
  int currentButtonState = digitalRead(PUSH_BUTTON_PIN);

  if (currentButtonState != lastButtonState) {
    delay(MY_DEBOUNCE_DELAY); 																	// 100ms debounce for stability
    currentButtonState = digitalRead(PUSH_BUTTON_PIN);
    Serial.print("Button 21 state: "); Serial.println(currentButtonState);  					// Debug for GPIO 21
    if (currentButtonState == LOW && lastButtonState == HIGH) {
      if (!isRecording) {
        interruptAudioPlayback();  																// Button press interrupts playback
        return NONE;  																			// No action in enum, handled in playAudio
      }
      else return RECORD;  																		// Button press starts recording
    }
    else if (currentButtonState == HIGH && lastButtonState == LOW && isRecording) return STOP;  // Button release stops recording
    lastButtonState = currentButtonState;
  }

  // Serial command handling 
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    Serial.print("Received command: "); Serial.println(command);  								// Debug to track input
    if (command == "1" && !isRecording) return RECORD;
    else if (command == "2" && isRecording) return STOP;
    else if (command == "3" && !isRecording) {
      delay(500); 																				// Delay to stabilize state after interrupt
      return PLAY;
    }
    else if (command == "8" || command.equalsIgnoreCase("interrupt")) {
      interruptAudioPlayback();  																// Serial '8' or "interrupt" interrupts playback
      return NONE;  																			// No action in enum, handled in playAudio
    }
    else Serial.println("Invalid command or state!");
  }
  return NONE;
}

// -------------------------- Interrupt Functions --------------------------------------
void interruptAudioPlayback() {
  interruptRequested = true;
  // Add a small delay for debounce
  delay(100);
  //Serial.println("WAV playback interrupted.");  												// Commented out to reduce Serial noise
}

// -------------------------- Playback Function --------------------------------------
void playAudio() {
  file = SD.open(wavFilename, FILE_READ);
  if (!file) {
    Serial.println("Failed to open WAV file for playback!");
    return;
  }

  interruptRequested = false;  																	// Reset interrupt flag at playback start
  Serial.println("Playing: " + String(wavFilename));  
  copier.begin(i2s_out, file); 																	// Direct to amp for now
  copier.setCheckAvailableForWrite(false);

  while (copier.copy()) {  																		// Use copier.copy() for reliable playback
    uint8_t buffer[REC_BUFFER_SIZE];
    size_t bytesRead = file.read(buffer, REC_BUFFER_SIZE);
    if (bytesRead > 0) {
      fft.write(buffer, bytesRead);       														// SD -> FFT
      volumeMeter.write(buffer, bytesRead); 													// FFT -> VolumeMeter
      i2s_out.write(buffer, bytesRead);    														// VolumeMeter -> Amp
      //Serial.print("Raw sample (playback): ");  												// Commented out to reduce Serial noise
      //Serial.println(((int16_t*)buffer)[0]); 													// Debug playback data (commented out)
    }
    float volume = volumeMeter.volume(0);
    //Serial.print("Volume: ");  																// Commented out to reduce Serial noise
    //Serial.println(volume);

    // Check for interrupts during playback, mirroring your example
    if (interruptRequested) {
      //Serial.println("Interrupt Block Executed!");  											// Debug (commented out)
      //Serial.println("Free Heap before interrupt " + String(ESP.getFreeHeap()));  			// Commented out to reduce Serial noise
      copier.end();
      file.close();
      interruptRequested = false;
      Serial.println("WAV playback interrupted.");  // Commented out
      printCommands();
      return;
    }

    // Check for serial input more frequently for interrupt
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      Serial.print("Playback command: "); Serial.println(command);  							// Debug to track input
      if (command == "8" || command.equalsIgnoreCase("interrupt") || command == "3") {
        interruptAudioPlayback();
      }
    }

    // Poll button state during playback for interrupt
    int buttonState = digitalRead(PUSH_BUTTON_PIN);
    static bool lastButtonState = HIGH;
    if (buttonState != lastButtonState) {
      delay(MY_DEBOUNCE_DELAY); 																// 100ms debounce
      buttonState = digitalRead(PUSH_BUTTON_PIN);
      Serial.print("Button 21 state during playback: "); Serial.println(buttonState);  			// Debug for GPIO 21
      if (buttonState == LOW && lastButtonState == HIGH) {
        interruptAudioPlayback();  																// Button press interrupts playback
      }
      lastButtonState = buttonState;
    }
  }

  file.close();
  copier.end();
  //Serial.println("Playback finished.");  														// Commented out to reduce Serial noise
  printCommands();
}

// -------------------------- Recording Functions --------------------------------------
void recordAudio() {
  if (SD.exists(wavFilename)) SD.remove(wavFilename);
  file = SD.open(wavFilename, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open WAV file for recording!");
    return;
  }

  Serial.println("Recording to: " + String(wavFilename));  										// Commented out to reduce Serial noise
  encoder.begin();
  copier.begin(encoderStream, filteredStream); 													// Use filteredStream instead of mic_volume
  copier.setCheckAvailableForWrite(false);

  isRecording = true;
}

void mic_volume_meter() {
  if (isRecording) {
    uint8_t buffer[REC_BUFFER_SIZE];
    size_t bytesRead = filteredStream.readBytes(buffer, REC_BUFFER_SIZE); 						// Read from filteredStream
    if (bytesRead > 0) {
      fft.write(buffer, bytesRead);       														// Mic (filtered) -> FFT
      volumeMeter.write(buffer, bytesRead); 													// FFT -> VolumeMeter
      encoderStream.write(buffer, bytesRead); 													// VolumeMeter -> SD
      //Serial.print("Raw sample (record): ");  												// Commented out to reduce Serial noise
      //Serial.println(((int16_t*)buffer)[0]); 													// Debug mic input (commented out)
    }
    float volume = volumeMeter.volume(0);
    // Serial.print("Volume: ");  																// Commented out to reduce Serial noise
    // Serial.println(volume);																	// Commented out to reduce Serial noise
  }
}

void stopRecording() {
  if (isRecording) {
    isRecording = false;
    copier.end();
    encoder.end();
    file.close();
    //Serial.println("Recording stopped.");  													// Commented out to reduce Serial noise
  }
  printCommands();
}

// -------------------------- Loop --------------------------------------
void loop() {
  // Check interrupt first, mirroring your working loop
  if (interruptRequested) {
    interruptRequested = false;
    //Serial.println("Interrupt handled in loop.");  											// Debug (commented out)
  }

  Action action = handleMenu();
  switch (action) {
    case RECORD:
      recordAudio();
      break;
    case STOP:
      stopRecording();
      break;
    case PLAY:
      playAudio();
      break;
    case NONE:
      break;
  }
  delay(10);
  mic_volume_meter();
  #ifdef DEBUG_MODE
    Serial.println("Free Heap B4 we continue " + String(ESP.getFreeHeap()));
  #endif
  delay(10);  
}
