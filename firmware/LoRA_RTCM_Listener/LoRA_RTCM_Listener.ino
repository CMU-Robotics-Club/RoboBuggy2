/*
   RadioLib SX127x Transmit with Frequency Hopping Example

   This example transmits packets using SX1276 LoRa radio module.
   Each packet contains up to 256 bytes of data, in the form of:
    - Arduino String
    - null-terminated char array (C-string)
    - arbitrary binary data (byte array)

   Other modules from SX127x/RFM9x family can also be used.

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/

   SX127x supports FHSS or Frequency Hopping Spread Spectrum.
   Once a hopping period is set and a transmission is started, the radio
   will begin triggering interrupts every hop period where the radio frequency
   is changed to the next channel.
*/

#include <RadioLib.h>

// SX1276 has the following connections:
// NSS pin:   8
// DIO0 pin:  3
// RESET pin: 4
// DIO1 pin:  14
SX1276 radio = new Module(8, 3, 4, 14);

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// flag to indicate frequency must be changed
volatile bool fhssChangeFlag = false;

// the channel frequencies can be generated randomly or hard coded
// NOTE: The frequency list MUST be the same on both sides!
float channels[] = {       902.3, 902.5, 902.7, 902.9, 
                    903.1, 903.3, 903.5, 903.7, 903.9,
                    904.1, 904.3, 904.5, 904.7, 904.9,
                    905.1, 905.3, 905.5, 905.7, 905.9,
                    906.1, 906.3, 906.5, 906.7, 906.9,
                    907.1, 907.3, 907.5, 907.7, 907.9,
                    908.1, 908.3, 908.5, 908.7, 908.9,
                    909.1, 909.3, 909.5, 909.7, 909.9,
                    910.1, 910.3, 910.5, 910.7, 910.9,
                    911.1, 911.3, 911.5, 911.7, 911.9,
                    912.1, 912.3, 912.5, 912.7, 912.9,
                    913.1, 913.3, 913.5, 913.7, 913.9,
                    914.1, 914.3, 914.5, 914.7, 914.9 };
int numberOfChannels = sizeof(channels) / sizeof(float);
uint8_t channel_indices[sizeof(channels) / sizeof(float)];

// counter to keep track of how many frequency hops were performed
int hopsCompleted = 0;

// cache for transmitting over radio
byte packetCache[255];

// this function is called when a complete packet
// is received by the module
void setRxFlag(void) {
  receivedFlag = true;
}

// this function is called when FhssChangeChannel interrupt occurs
// (at the beginning of each transmission)
void setFHSSFlag(void) {
  fhssChangeFlag = true;
}

void setup() {
  Serial.begin(57600);

  delay(5000);

  // generate LFSR indexes (psuedorandom non-repeating [0, 63])
  memset(&channel_indices[0], 0, sizeof(channel_indices));
  channel_indices[0] = 29; // seed value
  for (int i = 1; i < numberOfChannels-1; i++) {
    bool mask = channel_indices[i-1] & 0x1;
    channel_indices[i] = channel_indices[i-1] >> 1;
    if (mask)
      channel_indices[i] ^= 0x30;
  }

  // begin radio on home channel
  // Serial.println("[SX1276] Initializing ... ");
  int state = radio.begin(channels[channel_indices[0]], 125.0, 7, 5, RADIOLIB_SX127X_SYNC_WORD, 17, 8, 0);
  if (state == RADIOLIB_ERR_NONE) {
    // Serial.println("success!");
  } else {
    // Serial.print("failed, code ");
    // Serial.println(state);
    while (true);
  }

  // set the CRC to be used
  state = radio.setCRC(true);
  if (state == RADIOLIB_ERR_NONE) {
    // Serial.println(F("success!"));
  } else {
    // Serial.print(F("failed, code "));
    // Serial.println(state);
    while (true);
  }

  // set hop period in symbols
  // this will also enable FHSS
  state = radio.setFHSSHoppingPeriod(8);
  if (state == RADIOLIB_ERR_NONE) {
    // Serial.println("success!");
  } else {
    // Serial.print("failed, code ");
    // Serial.println(state);
    while (true);
  }

  // set the function to call when reception is finished
  radio.setDio0Action(setRxFlag);

  // set the function to call when we need to change frequency
  radio.setDio1Action(setFHSSFlag);

  // start listening for LoRa packets
  // Serial.print("[SX1276] Starting to listen ... ");
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    // Serial.println("success!");
  } else {
    // Serial.print("failed, code ");
    // Serial.println(state);
    while (true);
  }
}

void loop() {
  // check if the reception flag is set
  if (receivedFlag == true) {
    // you can read received data as an Arduino String
    // String str;
    byte data[255];
    unsigned int length = radio.getPacketLength();
    int state = radio.readData(data, min(255, length));

    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int state = radio.readData(byteArr, 8);
    */

    if (state == RADIOLIB_ERR_NONE) {
      // packet was successfully received

      // print data of the packet
      //Serial.print("[SX1276] Data:\t\t");
      // Serial.printf("%d\n", length);
      Serial.write(data, length);
      Serial.flush();

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      // Serial.println("[SX1276] CRC error!");

    } else {
      // some other error occurred
      // Serial.print("[SX1276] Failed, code ");
      //Serial.println(state);

    }

    // print the number of hops it took
    //Serial.print("[SX1276] Hops completed: ");
    //Serial.println(hopsCompleted);
    //Serial.printf("[SX1276] SnR %f RSSI %f\n",radio.getSNR(), radio.getRSSI());

    // reset the counter
    hopsCompleted = 0;

    // put the module back to listen mode
    radio.startReceive();

    // we're ready to receive more packets, clear the flag
    receivedFlag = false;
  }

  // check if we need to do another frequency hop
  if (fhssChangeFlag == true) {    
    // we do, change it now
    int state = radio.setFrequency(channels[channel_indices[radio.getFHSSChannel() % numberOfChannels]]);
    if (state != RADIOLIB_ERR_NONE) {
      //Serial.print("[SX1276] Failed to change frequency, code ");
      //Serial.println(state);
    }

    // increment the counter
    hopsCompleted++;

    // clear the FHSS interrupt
    radio.clearFHSSInt();

    // we're ready to do another hop, clear the flag
    fhssChangeFlag = false;
  }
}
