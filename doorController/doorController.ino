

//#include <pin_magic.h>
//#include <registers.h>

#include <Wire.h>
//#include "PN532_I2C.h"
//#include <PN532.h> //replaced by newer one below
#include <Adafruit_PN532.h>
#include <TaskScheduler.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#define ENABLE_BUILTIN_BADGES

#ifdef ENABLE_BUILTIN_BADGES
  #include "builtinBadges.h"
#endif

/* ========================================================================== *
 *  Pinout
 * ========================================================================== */
// NB: Do not use D0 or D2, these need to be pulled high by 10k resistors to ensure correct
//     boot sequence following watchdog/internal reset
#define I2C_DATA_PIN      A4
#define I2C_CLOCK_PIN     A5
#define PN532_RESET_PIN   4
//#define PN532_IRQ_PIN     ?
#define DOOR_SENSOR_PIN   5
#define OUTPUT_PIN        6
#define EXIT_BUTTON_PIN   3  // normally open, pulled high
#define NEOPIXEL_PIN      8
#define DOORBELL_PIN      A0
#define DOORBELL_ALARM_PIN  9
#define BUILTIN_LED       13
#define ESP_RESET_PIN     12

#define DEBUG_CAPSENSE
#define NUM_LEDS 24
/* ========================================================================== *
 *  Enums
 * ========================================================================== */


/* ========================================================================== *
 *  Configuration
 * ========================================================================== */

#define VERSION           "0.1"
#define EEPROM_MAGIC      3  // update to clear EEPROM on restart
#define ESP_CONNECTION_TASK_INTERVAL   10000 // milliseconds
#define RFID_CONNECTION_TASK_INTERVAL   10000 // milliseconds
#define LOOKFORCARD_TASK_INTERVAL       100   // milliseconds
#define SYNC_CACHE_TASK_INTERVAL        600000 // milliseconds
#define MONITORDOORSENSOR_TASK_INTERVAL 500   // milliseconds
#define MONITOROUTPUT_TASK_INTERVAL     500   // milliseconds
#define CARD_DEBOUNCE_DELAY             2000  // milliseconds
#define PN532_READ_TIMEOUT              50   // milliseconds
#define CACHE_SIZE        32    // number of tokens held in cache (memory and EEPROM)
#define CACHE_SYNC        240   // resync cache after <value> x 10 minutes

#define OUTPUT_ENABLE_DURATION          10000 // milliseconds

#define MAXINPUTCHARS   20   // size of ESP serial incoming buffer

#define COLOR_RED     strip.Color(255,0,0)
#define COLOR_GREEN   strip.Color(0,255,0)
#define COLOR_BLUE    strip.Color(0,0,255)
#define COLOR_WHITE   strip.Color(255,255,255)
#define COLOR_PURPLE  strip.Color(255,0,255)
#define COLOR_YELLOW  strip.Color(255,255,0)
#define COLOR_PINK    strip.Color(255,100,200)
#define COLOR_ORANGE  strip.Color(245,100,10)
#define COLOR_OFF     strip.Color(0,0,0)


#define PATTERN_WHITE    0
#define PATTERN_GREEN    1
#define PATTERN_PURPLE   2
#define PATTERN_BLUE     3
#define PATTERN_PINK     4
#define PATTERN_ORANGE   5
#define PATTERN_RAINBOW  17
#define PATTERN_PYB      18
#define PATTERN_FWHITE    24
#define PATTERN_FGREEN    25
#define PATTERN_FPURPLE   26
#define PATTERN_FBLUE     27
#define PATTERN_FPINK     28
#define PATTERN_FORANGE   29
#define PATTERN_RED       32
#define PATTERN_REDBLUE     33
#define PATTERN_REDORANGE   34
#define PATTERN_REDGREEN    35
#define PATTERN_REDPURPLE   36
#define PATTERN_REDPINK     37

/* ========================================================================== *
 *  Types / Structures
 * ========================================================================== */

 /*
  * struct for token cache
  * token - 7 bytes
  * length - unsigned byte - length of token
  * flags - 1 byte, encodes permission (bit0) and cache status (bit1) and color code (bit 2-7)
  * scan count - 2 bytes (unsigned int) - number of scans
  *
  * cache is fixed sized array
  * selection sorted on token
  * if cache size exceeded then another item is removed,
  * starting with badges that have no access, followed by smallest scan count
  *
  * cache can be debugged over serial, but not wifi, to prevent snooping of valid token numbers
  */

// tokens are 4 or 7-byte values, held in a fixed 7-byte array
typedef uint8_t TOKEN[7];

// flags for TOKEN_CACHE_ITEM
#define TOKEN_ACCESS    0x01
#define TOKEN_TRAINER   0x02
#define TOKEN_ERROR     0x04

// struct for items in the token cache
struct TOKEN_CACHE_ITEM {
    TOKEN token;      // the token uid
    uint8_t length;   // length of token in bytes
    uint8_t flags;    // permission bits
    uint16_t count;   // scan count
    uint8_t sync;     // countdown to resync with cache with server
};
// in memory = 12 bytes, EEPROM size = 9 bytes


/* ========================================================================== *
 *  Prototypes for task callbacks, etc
 * ========================================================================== */

// tasks
void keepESPConnected();
void keepRFIDConnected();
void lookForCard();
void displayUptime();
void syncCache();
void monitorDoorSensor();
void monitorOutput();
void monitorExitButton();
void animation();

// other prototypes
uint8_t queryServer();
uint8_t handleSerial();
boolean isDoorUnlocked();

void syncEEPROM();

/* ========================================================================== *
 *  Global Variables / Objects
 * ========================================================================== */

//PN532_I2C pn532i2c(Wire, I2C_DATA_PIN, I2C_CLOCK_PIN);  // data, clock
Adafruit_PN532 nfc(I2C_DATA_PIN, PN532_RESET_PIN);
//PN532 nfc(pn532i2c);
uint16_t PN532Resets = 0;  // reset counter

// ESP serial
SoftwareSerial ESPSerial(10, 11);
unsigned long lastHB;

// the cache
TOKEN_CACHE_ITEM cache[CACHE_SIZE];

// number of items in cache
uint8_t cacheSize = 0;

// Serial handling
//char inputString[20] = "";         // a string to hold incoming data
uint8_t inputChars = 0;
String inputString;
boolean stringComplete = false;  // whether the string is complete
uint8_t colorPattern = 0; //current pattern its displaying
// tasks
Task ESPConnectionTask(ESP_CONNECTION_TASK_INTERVAL, TASK_FOREVER, &keepESPConnected);
Task RFIDConnectionTask(RFID_CONNECTION_TASK_INTERVAL, TASK_FOREVER, &keepRFIDConnected);
//Task lookForCardTask(LOOKFORCARD_TASK_INTERVAL, TASK_FOREVER, &lookForCard);
Task displayUptimeTask(60000, TASK_FOREVER, &displayUptime);
Task syncCacheTask(SYNC_CACHE_TASK_INTERVAL, TASK_FOREVER, &syncCache);
//Task monitorDoorSensorTask(MONITORDOORSENSOR_TASK_INTERVAL, TASK_FOREVER, &monitorDoorSensor);

// scheduler
Scheduler runner;

// door
boolean doorOpen = false;

// output
unsigned long outputEnableTimer;
unsigned long unlockCount = 0;
boolean exitPressed = false;

// neopixel
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// doorbell
unsigned long doorbellTimer = 0;
boolean doorbellOn = false;

// token as hex string
char tokenStr[14];

/* ========================================================================== *
 *  Utility Functions
 * ========================================================================== */

void updateTokenStr(const uint8_t *data, const uint32_t numBytes) {
  const char * hex = "0123456789abcdef";
  uint8_t b = 0;
  for (uint8_t i = 0; i < numBytes; i++) {
        tokenStr[b] = hex[(data[i]>>4)&0xF];
        b++;
        tokenStr[b] = hex[(data[i])&0xF];
        b++;
  }

  // null remaining bytes in string
  for (uint8_t i=numBytes; i < 7; i++) {
        tokenStr[b] = 0;
        b++;
        tokenStr[b] = 0;
        b++;
  }
}

void uptime(Stream& serial, boolean display = true)
{
  static uint16_t rollover = 0;  // to count timer rollovers
  static boolean highMillis = false;
  long days=0;
  long hours=0;
  long mins=0;
  long secs=0;
  unsigned long ms = millis();

  // track rollover
  if (ms > 100000) {
    highMillis = true;
  }
  if (ms < 100000 && highMillis) {
    rollover++;
    highMillis = false;
  }

  secs = millis()/1000;
  mins=secs/60;
  hours=mins/60;
  days=(rollover * 50) + hours/24;
  secs=secs-(mins*60); //subtract the coverted seconds to minutes in order to display 59 secs max
  mins=mins-(hours*60); //subtract the coverted minutes to hours in order to display 59 minutes max
  hours=hours-(days*24); //subtract the coverted hours to days in order to display 23 hours max

  //Display results
  if (display) {
    serial.print(F("Uptime:"));
    if (days>0) // days will displayed only if value is greater than zero
    {
        serial.print(days);
        serial.print("d");
    }
    serial.print(hours);
    serial.print(':');
    serial.print(mins);
    serial.print(':');
    serial.print(secs);

    // unlock count
    serial.print(F("_unlocked:"));
    serial.print(unlockCount);
    serial.print('\n');
  }
}

void displayUptime() {
  uptime(Serial, true);
}


inline int clamp(int v, int minV, int maxV) {
  return min(maxV, max(v, minV));
}

/* ========================================================================== *
 *  NEOPIXEL
 * ========================================================================== */

void colorWipe(uint8_t pattern) {
  colorPattern = pattern;
  Serial.print(F("Setting Pattern to "));  Serial.println(colorPattern);
  for(uint16_t i=0; i<strip.numPixels(); i++) {
        
    strip.setPixelColor(i, flag2color(pattern,i));
    // if (wait >0 ) {
    //   strip.show();
    //   delay(wait);
    // }
  }
  //if (show) { //sometimes dont want to show as we will still changing some of the pixels with segments() or sprial()
    strip.show();
  //}
}


uint32_t flag2color(uint8_t pattern, int pos) {
  //patterns from the 6 bit code
  uint8_t val = pos % 24;
  switch (pattern){
    case PATTERN_WHITE: //solid white
      return(COLOR_WHITE);
    case PATTERN_GREEN: //solid green
      return(COLOR_GREEN);
    case PATTERN_PURPLE: //solid purple
      return(COLOR_PURPLE);
    case PATTERN_BLUE: //solid blue
      return(COLOR_BLUE);
    case PATTERN_PINK: //solid pink
      return(COLOR_PINK);
    case PATTERN_ORANGE: //solid orange
      return(COLOR_ORANGE);
    case PATTERN_RAINBOW: //rainbow spiral
      switch (val>>2){
        case 1: return(COLOR_RED);
        case 2: return(COLOR_ORANGE);
        case 3: return(COLOR_YELLOW);
        case 4: return(COLOR_GREEN);
        case 5: return(COLOR_BLUE);      
        default: return(COLOR_PURPLE);
      }
    case PATTERN_PYB: //PYB spiral
      switch (val/8){
        case 0: return (COLOR_PINK);      
        case 1: return (COLOR_ORANGE);  
        default: return (COLOR_BLUE);      
      }
    case PATTERN_FWHITE: //flash white
      return(COLOR_WHITE);
    case PATTERN_FGREEN: //flash green
      return(COLOR_GREEN);
    case PATTERN_FPURPLE: //flash purple
      return(COLOR_PURPLE);
    case PATTERN_FBLUE: //flash blue
      return(COLOR_BLUE);
    case PATTERN_FPINK: //flash pink
      return(COLOR_PINK);
    case PATTERN_FORANGE: //flash orange
      return(COLOR_ORANGE);
    case PATTERN_RED: //error solid red
      return(COLOR_RED);
    case PATTERN_REDBLUE: //error half red/blue
      if (pos < NUM_LEDS/2) 
        return(COLOR_RED);
      else
        return(COLOR_BLUE);
    case PATTERN_REDORANGE: //error half red/yellow
      if (pos < NUM_LEDS/2) 
        return(COLOR_RED);
      else
        return(COLOR_ORANGE);
    case PATTERN_REDGREEN: //error half reg/green
      if (pos < NUM_LEDS/2) 
        return(COLOR_RED);
      else
        return(COLOR_GREEN);
    case PATTERN_REDPURPLE: //error half reg/purple
      if (pos < NUM_LEDS/2) 
        return(COLOR_RED);
      else
        return(COLOR_PURPLE);
    case PATTERN_REDPINK: //error half red/pink
      if (pos < NUM_LEDS/2) 
        return(COLOR_RED);
      else
        return(COLOR_PINK);
    default: //invalid color code
      if (pos < NUM_LEDS/2) 
        return(COLOR_RED);
      else
        return(COLOR_WHITE);
  }
}
void segment(uint8_t pattern, uint32_t c2, int start, int end) {
  // turn all LEDs off (c2)
  
  for(uint16_t i=0; i<strip.numPixels(); i++) {
     strip.setPixelColor(i,c2);
  }
  
  // turn on the segment (c1)
  for(int i=start; i<end+1; i++) {
    int pix = i;
    if (i < 0) i+=strip.numPixels();
    strip.setPixelColor(i % strip.numPixels(), flag2color(pattern, i));
  }
  strip.show();
}

void spinner(uint8_t pattern, uint8_t from, uint8_t reduce) {
  // stating at LED from, fades from c to black, by reduce each LED
  
  uint32_t c1 = 0;
  int r;
  for (uint8_t i=0; i<strip.numPixels(); i++) {
    uint32_t c = flag2color(pattern, i+from);
    r = (23 - i);
    c1 = strip.Color(
      clamp(((c >> 16) & 0xff) * r / 23, 0, 255),
      clamp(((c >> 8) & 0xff) * r / 23, 0, 255),
      clamp((c & 0xff) * r / 23, 0, 255)
    );
    int pix = from - i;
    if (pix < 0) pix += 24;
    strip.setPixelColor(pix % strip.numPixels(), c1);
  }
  strip.show();
}

void flash(uint8_t from){
  uint8_t onoff = from; //slow down flashing by 4x to prevent nightclub strobing
  Serial.print("FLASH");
  Serial.println(onoff);
  if (onoff % 2){ //on
    for (uint8_t i=0; i<strip.numPixels(); i++) {
      uint32_t c = flag2color(colorPattern, i);
      strip.setPixelColor(i, c);
    }
  }else{ //off
    for (uint8_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, COLOR_OFF);
    }
  }
  strip.show();  
  
}

void animation() {
   static int pos = 0;

   // animation changes based on lock and doorbell states

   if (isDoorUnlocked()) {
      // countdown animation

      if (millis() < outputEnableTimer) {
        pos =  23 * (outputEnableTimer - millis()) / OUTPUT_ENABLE_DURATION;
      } else {
        pos = 0;
      }
        switch (colorPattern) {
          case 0 ... 7: 
            spinner(colorPattern, pos, 5);
            break;
         case PATTERN_RAINBOW :
         case PATTERN_PYB :
            spinner(colorPattern, pos, 0); //spinner works with no fade reduction
            break;
         case 24 ... 31:
           flash(pos);
           break;
         case 32 ... 40 : //half colours
           spinner(colorPattern, 0, 0); //spiral with no movement
           break;
         default : //shouldnt happen
           spinner(colorPattern, pos, 5);
           break;     
       }
      //segment(colorPattern, 0, 4, pos + 4);

      //reset pos to top postion, so that spinner always resumes from right position
      pos = 4;
      
   } else if (doorbellOn) {
    // doorbell on

    // do nothing, LEDs should already be blue as set by the doorbell task
    
   } else {
    
     // normal spinner animation

     // clockwise if ESP connected
     if (true) {
        pos++;
        if (pos > 23) pos = 0;
     } else {
        // anti-clockwise if not
        pos--;
        if (pos < 0) pos = 23;
     }
  
     colorPattern = PATTERN_ORANGE;
     if (doorOpen) {
        colorPattern = PATTERN_RED;  // red if doorOpen
     }

     //do appropriate animation
     spinner(colorPattern, pos, 5);
     
     
     //Serial.print(F("Setting Pattern to"));  Serial.println(colorPattern);
   }
}

/* ========================================================================== *
 *  Output
 * ========================================================================== */

// Maglock connected to NC terminals on relay, output:  true = unlocked, false = locked

// duration in milliseconds
void unlockDoor(unsigned long duration, uint8_t color_flag) {
  unlockCount++;
  Serial.print(F("Door Unlocked, "));  Serial.println(unlockCount);
  digitalWrite(OUTPUT_PIN, LOW);
  outputEnableTimer = millis() + duration;
  // green when unlocked
  colorWipe(color_flag);
}

void lockDoor() {
  Serial.println(F("Door Locked"));
  digitalWrite(OUTPUT_PIN, HIGH);
  // return to orange
  //colorWipe(strip.Color(50, 25, 0), 50);
}

// returns true if door unlocked
boolean isDoorUnlocked() {
  return !digitalRead(OUTPUT_PIN);
}

// task to monitor output status, disable when necessary
void monitorOutput() {
  if (isDoorUnlocked() && (millis() > outputEnableTimer)) {
    lockDoor();
  }
}

/* ========================================================================== *
 *  Token Cache
 * ========================================================================== */

// Get cache item by token, returns null if not in cache
TOKEN_CACHE_ITEM* getTokenFromCache(TOKEN* token, uint8_t length) {
  // search through items to find a match to token
  // TODO: binary search?

  TOKEN_CACHE_ITEM* item = NULL;

  uint8_t i;
  for (i=0; i<cacheSize; i++) {
    if ((length == cache[i].length) && (memcmp(token, &cache[i], length)==0)) {
      item = &cache[i];
      break;
    }
  }

  return item;
}


// add a token to the cache, returns pointer to new cache item
TOKEN_CACHE_ITEM* addTokenToCache(TOKEN* token, uint8_t length, uint8_t flags) {
  // if cache not full, then add a new item to end of array
  uint8_t pos = cacheSize;
  uint8_t i;

  // check for existing
  TOKEN_CACHE_ITEM* t = getTokenFromCache(token, length);
  if (t != NULL) {
    // update flags
    t->flags = flags;
    return t;
  }

  // else, find item with least scans
  if (cacheSize == CACHE_SIZE) {
    cacheSize = 0;  // start at the beginning

    for (i=0; i < cacheSize; i++) {
      if (cache[i].count < cache[pos].count) {
        pos = i;
      }
    }
  } else {
    cacheSize++;
  }

  // write new token into the cache
  memcpy(&cache[pos].token, token, length);
  cache[pos].length = length;
  cache[pos].flags = flags;
  cache[pos].count = 1;
  cache[pos].sync = CACHE_SYNC;

  // TODO: selection sort all items?

  // write new info to EEPROM
  syncEEPROM();

  // print number of cache slots used
  Serial.print(F("Cache used: "));
  Serial.print(cacheSize);
  Serial.print('/');
  Serial.println(CACHE_SIZE);

  // return new item
  return &cache[pos];
}

// pass item to remove
void removeTokenFromCache(TOKEN_CACHE_ITEM* item) {
  item->length = 0;
  item->flags = 0;
  item->count = 0;
  item->sync = CACHE_SYNC;
}

void initCache() {
  Serial.println(F("Loading cache from EEPROM..."));

  EEPROM.begin();

  // read magic
  if (EEPROM.read(0) != EEPROM_MAGIC) {
    Serial.println(F("Magic changed, resetting cache"));
    // reset stored cache size
    EEPROM.write(0, EEPROM_MAGIC);
    EEPROM.write(1, 0);

    cacheSize = 0;
  } else {
    cacheSize = EEPROM.read(1);
    Serial.print(cacheSize);
    Serial.println(F(" items"));
  }

  // read token items from cache
  uint8_t i = 0, j = 0;
  uint16_t addr = 2;
  for (i=0; i<cacheSize; i++) {
    // token
    for (j=0; j<7; j++) {
      cache[i].token[j] = EEPROM.read(addr + j);
    }

    // length
    cache[i].length = EEPROM.read(addr + 7);

    // flags
    cache[i].flags = EEPROM.read(addr + 8);

    updateTokenStr(cache[i].token, cache[i].length);

    Serial.print(' ');
    Serial.print(tokenStr);
    Serial.print(':');
    Serial.println(cache[i].flags);

    cache[i].count = 0;
    cache[i].sync = 1 + i;  // resync everything soon-ish

    addr += 9;
  }

#ifdef ENABLE_BUILTIN_BADGES
  TOKEN *token;
  for (i=0; i < BUILTIN_BADGE_COUNT; i++) {
    token = (TOKEN*)&builtinBadges[i];
    addTokenToCache(token, 4, 3);
  }
#endif
}

// task to update permission flags in cache, e.g. if someones access has changed
// called every 10min ish
void syncCache() {
  // for each item in cache
  uint8_t i;
  for (i=0; i<cacheSize; i++) {
    // dec sync counter
    cache[i].sync--;

    // if reached zero
    if (cache[i].sync == 0 && cache[i].length > 0) {
      Serial.print(F("Syncing cached flags for: "));
      updateTokenStr(cache[i].token, cache[i].length);
      Serial.println(tokenStr);

      // query permission flags from server
      uint8_t flags = queryServer();

      if (flags != TOKEN_ERROR) {
        if (flags > 0) {
         // if successful, update flags and reset sync counter
         cache[i].flags = flags;
         cache[i].sync = CACHE_SYNC;
        } else {
          // else remove token
          removeTokenFromCache(&cache[i]);
        }
      } else {
        // else try again next cycle
        cache[i].sync = 1;
      }

    }
  }

  // sync changes to EEPROM
  syncEEPROM();

  // send uptime log to server via ESP
  ESPSerial.print('!');
  uptime(ESPSerial, true);
}


// update a byte of EEPROM memory, return true if changed
boolean updateEEPROM(int address, uint8_t value) {
  boolean changed = EEPROM.read(address) != value;
  // TODO: rework for Arduino EEPROM
  if (changed) {
    EEPROM.write(address, value);
  }
  return changed;
}

// update EEPROM to match cache
void syncEEPROM() {
  boolean changed = false;

  changed |= updateEEPROM(1, cacheSize);

  // update items from cache
  uint8_t i = 0, j = 0;
  uint16_t addr = 2;
  for (i=0; i<cacheSize; i++) {
    // token
    for (j=0; j<7; j++) {
      changed |= updateEEPROM(addr + j, cache[i].token[j]);
    }

    // length
    changed |= updateEEPROM(addr + 7, cache[i].length);

    // flags
    changed |= updateEEPROM(addr + 8, cache[i].flags);

    addr += 9;
  }

  if (changed) {
    Serial.println(F("Updating EEPROM"));
  }
}


/* ========================================================================== *
 *  ESP Interface
 * ========================================================================== */

// query server for token, and return flags
uint8_t queryServer() {
   uint8_t flags = 0;

   // send query to ESP
   ESPSerial.print('?');
   ESPSerial.print(tokenStr);
   ESPSerial.print('\n');

   // wait for reply, no more than 4 seconds
   unsigned long giveUp = millis() + 4000;
 
   do {
      flags = handleSerial();
   } while (flags ==0 && millis() < giveUp);
   
   return flags;
}


// send a log msg to the server, fire and forget
void sendLogMsg(String msg1, String msg2) {
   ESPSerial.print('!');
   ESPSerial.print(msg1);
   ESPSerial.print(msg2);
   ESPSerial.print('\n');
}


/* ========================================================================== *
 *  PN532
 * ========================================================================== */

void resetPN532() {
  digitalWrite(PN532_RESET_PIN, HIGH);
  delay(10);
  digitalWrite(PN532_RESET_PIN, LOW);
  delay(10);
  digitalWrite(PN532_RESET_PIN, HIGH);

  // inc reset counter
  PN532Resets++;

  Serial.print(F("PN532 reset: "));  Serial.println(PN532Resets);

  nfc.begin();

  // added ref issue: https://github.com/Seeed-Studio/PN532/issues/44
  nfc.setPassiveActivationRetries(0x19);

  nfc.SAMConfig();
  nfc.getFirmwareVersion();

  //Serial.println(nfc.getGeneralStatus());
  //Serial.println(nfc.getFirmwareVersion());

  nfc.SAMConfig();
}

// TODO: Get IRQ working to avoid polling for card?
void cardAvailable() {
  Serial.println("IRQ change");
}

// Task to keep RFID connected - i.e. reset PN532 if goes weird
void keepRFIDConnected() {

   // seem to need to call this before a getFirmwareVersion to get reliable response!?!
   nfc.SAMConfig();

   uint32_t versiondata = nfc.getFirmwareVersion();

   if (! versiondata) {
      Serial.println(F("PN532 -> Error - Resetting"));

      // Reset the PN532 if it locks up
      resetPN532();

   } else {
      //Serial.println("PN532 -> OK");

      // configure board to read RFID tags - again and again
      nfc.SAMConfig();

      RFIDConnectionTask.setInterval(RFID_CONNECTION_TASK_INTERVAL);
   }
}

// Task to poll for card
void lookForCard() {
  uint8_t success;
  TOKEN uid;  // Buffer to store the returned UID
  uint8_t uidLength; // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  static TOKEN luid;  // last scanned uid, for debounce
  static unsigned long lastChecked;

  TOKEN_CACHE_ITEM* item;

  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, PN532_READ_TIMEOUT);

  if (success && (memcmp(luid, uid, uidLength)!=0 || (millis() > lastChecked + CARD_DEBOUNCE_DELAY))) {
    uint8_t flags = 0xFC; //default value meaning no info found
    // store the uid to permit debounce
    memcpy(luid, uid, uidLength);

    lastChecked = millis();

    updateTokenStr(uid, uidLength);
    
    Serial.print(F("Card found: "));  Serial.println(tokenStr);
    Serial.println(uidLength);

    // check cache
    item = getTokenFromCache(&uid, uidLength);
    //if not found, then query server and add to cache if has permission
    if (item == NULL) {
      Serial.println(F("Not in cache"));
      
      // bright orange - found card, querying server
      colorWipe(PATTERN_ORANGE);
      flags = queryServer();
      Serial.println("gotflags");
      Serial.println(flags);
      uint8_t flag_access = flags & 0x01; //first bit is access allowed
      uint8_t flag_cache  = (flags & 0x02) >> 1; //second bit is cache           
      if ((flag_access) && (flag_cache)) { //if given access and is a token to be cached (weekend tier not to be cached)
        item = addTokenToCache(&uid, uidLength, flags);
      }
    } else {
      Serial.println(F("In cache"));
      // double check it's a valid item
      if (item->flags == 0) {
        removeTokenFromCache(item);
        item = NULL;
      }
      flags = item->flags;
    }

    // if got valid details and permission given, then open/power the thing, if not, don't
    if (flags != 0xFC) {
      uint8_t flag_access = (flags & 0x02) >> 1; //second bit is access allowed
      uint8_t flag_color = (flags & 0xFC) >> 2; //remaining bits for colour code
      // take action and send log to server
      if (flag_access) {
        // permission given, so open/power the thing
        item->count++;

        Serial.print(F("Permission granted: "));
        Serial.println(item->count);
        unlockDoor(OUTPUT_ENABLE_DURATION, flag_color);

        sendLogMsg(F("Permission%20granted%20to:%20"), tokenStr);
        
        //sendTelegramMsg("Door opened");

      } else {
        // permission denied!
        Serial.println(F("Permission denied"));
        sendLogMsg(F("Permission%20denied%20to:%20"), tokenStr);

        // show error code (color sent from server or esp8266)
        colorWipe(flag_color);

        delay(1000);
      }

    } else {
      // permission denied!
        Serial.println(F("No Response"));
        sendLogMsg(F("Permission%20denied%20to:%20"), tokenStr);

        // 
        colorWipe(PATTERN_REDGREEN); //half red/green 

        delay(1000);
    }

  }
}


/* ========================================================================== *
 *  ESP Client
 * ========================================================================== */

void resetESP() {
  digitalWrite(ESP_RESET_PIN, HIGH);
  delay(10);
  digitalWrite(ESP_RESET_PIN, LOW);
  delay(10);
  digitalWrite(ESP_RESET_PIN, HIGH);
  // reset the HB timer...  give the ESP a chance to reboot
  lastHB = millis();
}

// Task to keep ESP connected
void keepESPConnected() {
   // send HB
   ESPSerial.print("~\n");
   Serial.println("HB");

   // NB: Heartbeats are received in the serial handling loop

   // check how long since last HB
   if (millis() > lastHB + 30000) {
      // been ages, so reset ESP
      Serial.println(F("Resetting ESP..."));
      resetESP();
   }
}


/* ========================================================================== *
 *  Door Sensor
 * ========================================================================== */

// Task to keep an eye on the door sensor (reed switch)
void monitorDoorSensor() {
  // Switch is to ground, with internal pullup enabled...
  // sensor value: 0 = closed, 1 = open
  boolean newDoorOpen = digitalRead(DOOR_SENSOR_PIN);

  // see if state has changed
  if (newDoorOpen != doorOpen) {
    doorOpen = newDoorOpen;

    if (doorOpen) {
      Serial.println(F("Door opened"));

      // Compare to lock status...  scream if not expected to be opened!!
      if (!isDoorUnlocked() ) {
        Serial.println(F("AAARRGH: Door opened unexpectedly!"));
        //sendTelegramMsg("AARGH someone has forced the door open");
      }

    } else {
      Serial.println(F("Door closed"));
    }
  }
}

/* ========================================================================== *
 *  Exit Button
 * ========================================================================== */

void exitButtonISR() {
  // can't do too much in the ISR, or it causes an excpetion
  // so just set a flag, and actually unlock the door from a
  // normal task
  
  // debounce
  delay(2);
  if (!digitalRead(EXIT_BUTTON_PIN)) {
    exitPressed = true;
  }
}

void monitorExitButton() {
    if (exitPressed) {
      exitPressed = false;
      Serial.println("Exit button pressed");
      unlockDoor(OUTPUT_ENABLE_DURATION, COLOR_GREEN);
    }
}


/* ========================================================================== *
 *  Doorbell
 * ========================================================================== */

void monitorDoorbell() {
  boolean bellPressed = !digitalRead(DOORBELL_PIN);

  // turn doorbell on?
  if (bellPressed) {
    colorWipe(PATTERN_BLUE);
    
    Serial.println(F("Bing bong"));

    digitalWrite(DOORBELL_ALARM_PIN, LOW);

    doorbellTimer = millis() + 500; 
    doorbellOn = true;
  }

  // turn doorbell off?
  if (doorbellOn && millis() > doorbellTimer) {
    Serial.println("Doorbell off");
    doorbellOn = false;

    digitalWrite(DOORBELL_ALARM_PIN, HIGH);
  }
}


/* ========================================================================== *
 *  Serial handling
 * ========================================================================== */

// call to process serial, will return non-zero value if it gets a query reply
uint8_t handleSerial() {
  uint8_t flags = 0;
  
  while (ESPSerial.available()) {
    char inChar = (char)ESPSerial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else if (inChar != '\r') {
      inputChars++;
      if (inputChars < MAXINPUTCHARS) {
        inputString += inChar;
      }
    }
  
    if (stringComplete) {
      Serial.print("ESP:");
      Serial.println(inputString);    
  
      // reply to query?
      if (inputString[0] == '?') {
        // look at second character to see if door should open!
        flags = (uint8_t)inputString[1];// - 48;  // convert from ascii representation
      } else if (inputString[0] == '~') {
        // received reply to heartbeat
        lastHB = millis();
      } else {
        
      }
      
      // clear the string:
      inputChars = 0;
      inputString = "";
      stringComplete = false;
    }

    // reset watchdog
    wdt_reset();

  }
  return flags;
}


/* ========================================================================== *
 *  Setup
 * ========================================================================== */

void setup(void) {
  // configure pins
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(PN532_RESET_PIN, OUTPUT);
  //pinMode(PN532_IRQ_PIN, INPUT_PULLUP);
  pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(EXIT_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(EXIT_BUTTON_PIN, HIGH);
  pinMode(ESP_RESET_PIN, OUTPUT);

  // reset ESP on start
  resetESP();

  // doorbell
  pinMode(DOORBELL_PIN, INPUT_PULLUP);
  pinMode(DOORBELL_ALARM_PIN, OUTPUT);
  digitalWrite(DOORBELL_ALARM_PIN, HIGH);

  //attachInterrupt(digitalPinToInterrupt(PN532_IRQ_PIN), cardAvailable, CHANGE);

  inputString.reserve(MAXINPUTCHARS);

  // make sure door is locked
  lockDoor();

  // start serial
  Serial.begin(9600);

  Serial.println();
  Serial.println(F("Door Controller"));
  Serial.print("V:");  Serial.println(VERSION);

  Serial.println(F("Listening for exit button..."));
  attachInterrupt(digitalPinToInterrupt(EXIT_BUTTON_PIN), exitButtonISR, FALLING);

  // Start PN532
  Serial.println(F("Connecting to PN532..."));
  resetPN532();

  // init EEPROM and load cache
  initCache();

  Serial.println(F("Starting fancy LEDs..."));
  strip.begin();
  colorWipe(PATTERN_WHITE);

  Serial.println(F("Connecting to ESP..."));
  ESPSerial.begin(9600);

  // Setup scheduler
  Serial.println(F("Configuring tasks..."));
  runner.init();
  runner.addTask(ESPConnectionTask);
  runner.addTask(displayUptimeTask);
  runner.addTask(syncCacheTask);
  runner.addTask(RFIDConnectionTask);

  // Enable tasks
  ESPConnectionTask.enable();
  RFIDConnectionTask.enable();
  displayUptimeTask.enable();
  syncCacheTask.enable();

  Serial.println(F("Ready"));
  Serial.println();

  // enable watchdog, 8 sec timeout
  wdt_enable(WDTO_8S);
}


/* ========================================================================== *
 *  Main Loop
 * ========================================================================== */

void loop(void) {
  // execute tasks
  runner.execute();

  lookForCard();
  monitorDoorSensor();

  // keep track of uptime
  uptime(Serial, false);

  // TODO: Apply fix for millis overflows around 49 days

  // TODO:
  // I've rebooted log msg
  // send heartbeat to server - 10min?

  animation();
  monitorOutput();
  monitorDoorbell();
  monitorExitButton();

  handleSerial();
  
  // visual comfort
  digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));

  // reset watchdog
  wdt_reset();
}
