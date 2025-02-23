//  change these depending on your set up
#include "driver/i2s_std.h"                
#include "Arduino.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <UrlEncode.h>
#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"

URLStream url; 
I2SStream i2s;
EncodedAudioStream decoder(&i2s, new MP3DecoderHelix());

// ----- Declare global playback variables
bool audioPlaying = false;

const int MAX_CHUNK_LENGTH = 195;            //  Googles limit for FREE is 200, stay below it
const int MIN_CHUNK_LENGTH = 100;             // Start cutting after a minimum of 100 characters so less shorter sentences

void speakGoogleTTSInChunk(String text);


speakGoogleTTSInChunks("What can I help you with today? ");  //  how to call this



void speakGoogleTTSInChunks(String text) {
  rgbLedWrite(RGB_BUILTIN, 0, 0, 0); // Reset to Black

  // Re-initialize decoder
  if (decoder) {
    decoder.begin();
  }

  // Re-initialize copier (unconditionally)
  copier.begin();

  if (decoder) {
    copier.copy();
    #ifdef DEBUG_MODE
      Serial.println("Free Heap after copy: " + String(ESP.getFreeHeap()));
    #endif
  } else {
    copier.copy();
    decoder.begin();
    #ifdef DEBUG_MODE
      Serial.println("Free Heap after setup: " + String(ESP.getFreeHeap()));
    #endif
  }

  audioPlaying = true;

  int start = 0;

  while (start < text.length()) {
    int end = start;

    if (text.length() - start <= MAX_CHUNK_LENGTH) {                              // If remaining text is short enough, take it all
      end = text.length();
    } else {
      end = start + MIN_CHUNK_LENGTH;

      while (end < text.length() && end - start < MAX_CHUNK_LENGTH) {
        if (text[end] == '.' || text[end] == '!' || text[end] == '?') {
          end++;                                                                  // Include the punctuation
          break;                                                                  // Stop at the first sentence ending punctuation.
        }
        end++;
      }

      //If no punctuation is found, find the last space
      if (end - start >= MAX_CHUNK_LENGTH) {
        int prevSpace = end;
        while (prevSpace > start && text[prevSpace] != ' ') {
          prevSpace--;
        }
        if (prevSpace > start) {
          end = prevSpace + 1;
        }
      }

      //If still no punctuation or space is found, split at MAX_CHUNK_LENGTH
      if (end - start >= MAX_CHUNK_LENGTH) {
        end = start + MAX_CHUNK_LENGTH;
      }

      //Handle edge case where only one word is longer than MAX_CHUNK_LENGTH
      if (end == start) {
        end = start + MAX_CHUNK_LENGTH;
        if (end > text.length()) {
          end = text.length();
        }
      }
    }

    // --------  Start of chunk cleanup    ---------------

    if (end > start) {
      int chunkLength = end - start;
      char* chunkChars = new char[chunkLength + 1];

      if (chunkLength > 0) {
        text.substring(start, end).toCharArray(chunkChars, chunkLength + 1);
        chunkChars[chunkLength] = '\0';
      } else {
        chunkChars[0] = '\0';
      }

      String chunk = String(chunkChars);
      chunk.trim();
      delete[] chunkChars;

      #ifdef DEBUG_MODE
        Serial.println("Free Heap before speaking chunk: " + String(ESP.getFreeHeap()));
      #endif

      chunk.replace("“", "\"");
      chunk.replace("”", "\"");
      chunk.replace("‘", "'");
      chunk.replace("’", "'");

      while (chunk.indexOf("##") != -1) {
        chunk.replace("##", " ");
      }

      chunk.replace(" ' ", " '");
      chunk.replace(" \" ", " \"");
      chunk.replace(" . ", ". ");
      chunk.replace(" , ", ", ");
      chunk.replace(" ! ", "! ");
      chunk.replace(" ? ", "? ");
      chunk.replace("( ", "(");
      chunk.replace(" )", ")");
      chunk.replace("[ ", "[");
      chunk.replace(" ]", "]");
      chunk.replace("{ ", "{");
      chunk.replace(" }", "}");

      while (chunk.indexOf("  ") != -1) {
        chunk.replace("  ", " ");
      }

      chunk.trim();

      // --------   End of chunk cleanup now lets get to playback ---------------

      #ifdef DEBUG_MODE
        Serial.println("Free Heap before speaking chunk: " + String(ESP.getFreeHeap()));
      #endif

      // Serial.println(chunk);                                                     // in case you want to see the chunks info

      String encodedChunk = urlEncode(chunk);
      int charCount = countCharacters(chunk);                                        // in case we want to use later

      delay(10);
      #ifdef DEBUG_MODE
        Serial.println("Free Heap b4 counting chrachters : " + String(ESP.getFreeHeap()));
      #endif

    
      String url_str = "http://translate.google.com/translate_tts?ie=UTF-8&tl=" + String(TTS_GOOGLE_LANGUAGE) + "&client=tw-ob&ttsspeed=1&q=" + encodedChunk;

      url.begin(url_str.c_str(), "audio/mp3");
    
      // --- Read from URLStream into the decoder directly ---
      uint8_t buffer[256];  
      size_t bytesRead = 0;
      while (url.available()) {
        bytesRead = url.readBytes(buffer, sizeof(buffer));
        decoder.write(buffer, bytesRead); // Write in chunks
        if (speakTextFirstPlayTime == 0) {
          speakTextFirstPlayTime = micros();
        }

        if (interruptRequested) {
          url.end();
          copier.end();
          decoder.end();
          interruptRequested = false;
          audioPlaying = false;
		  
          #ifdef DEBUG_MODE
            Serial.println("Interrupt Block Executed!");
            Serial.println("Free Heap after interupt: " + String(ESP.getFreeHeap()));
          #endif
          return;
        }

        // Check for serial input more frequently for interrupt or other
        if (Serial.available() > 0) {
          String command = Serial.readStringUntil('\n');
          command.trim();

          if (command.equalsIgnoreCase("interrupt") || command.equalsIgnoreCase("3")) {
            interruptAudioPlayback();       
          }
        }
      }
    start = end;
  }


  //---  lets be sure to tidy up
  url.end();
  copier.end();
  decoder.end();
  decoder.flush();
  audioPlaying = false;
  delay(10);
    #ifdef DEBUG_MODE
      Serial.println("Free Heap after cleanup : " + String(ESP.getFreeHeap()));
    #endif

}

//  i like to be able to use other places if need be, will expand on it
//  set some flags so i know which processes to shut down
void interruptAudioPlayback() {
  interruptRequested = true;
  delay(10); 
 
}
