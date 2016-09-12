/*
* OpenBikeLocker is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of
* the License, or(at your option) any later version.
*
* OpenBikeLocker is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with OpenBikeLocker.  If not, see
* <http://www.gnu.org/licenses/>.
*/

/* Connections
*  Autonomo D0 <=> MFRC522 RESET
*  Autonomo D5 <=> MFRC522 MISO
*  Autonomo D6 <=> MFRC522 SS
*  Autonomo D7 <=> MFRC522 MOSI
*  Autonomo D8 <=> MFRC522 CLK
*
*  Autonomo D1 <=> RED LED
*  Autonomo D2 <=> GREEN LED
*  
*  Autonomo D3 <=> SWITCH
*  
*  Autonomo D4 <=> SERVO
* 
*/

#include <TheThingsNetwork.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h> 

#define debugSerial Serial
#define loraSerial Serial1

// defines for MFRC lib
#define SS_PIN 7
#define RST_PIN 8

MFRC522 mfrc522(SS_PIN, RST_PIN);  
TheThingsNetwork ttn;
Servo myservo;

// defines for status leds/switches
#define LED_RED 2
#define LED_GREEN 3
#define SWITCH1_PIN 4

// define for servo motor
#define SERVO_PIN 5
#define RELAY_PIN 6

/* defines for low level lock API (in uplink direction) 
*  
* CMD_GET_SERVER_TIME get current time from server
* command = 1 byte
* response from server: 6 bytes with datetime structure (to be defined)
* 
* CMD_INVOKE_DOWNLINK see if server has new data for us
* command = 1 byte
* response from server: 1 byte with command, x bytes with command parameters (eg. open lock in 1 minute, to be defined)   
* 
* CMD_SET_LOCKER_STATE sets state of the locker
* command = 1 byte
* param1 1 byte (required) state => maps 1:1 to veiligstallen; 0 = vrij, 1 = bezet, 2 = geblokkeerd, 3 = gereserveerd, 4 =  buiten werking 
* param2 10 bytes (optional) mifareid
* 
* CMD_UPDATE_LOCKER_USER_ID sets the mifareid of the locker that is currently used (eg. anonymity expires after 24h)
* command = 1 byte
* param1 10 bytes (required) mifareid
*/
#define CMD_GET_SERVER_TIME 1
#define CMD_INVOKE_DOWNLINK 2
#define CMD_SET_LOCKER_STATE 3
#define CMD_REQUEST_LOCKER_UNBLOCK 4

//Function for simulation of lock (servo based, can alco be a relay)
void openLock(int degrees)
{
  unsigned long timeout=millis();
 
  digitalWrite(RELAY_PIN, HIGH);
  myservo.attach(SERVO_PIN,1000,2000); 
  myservo.write(degrees); 
  delay(1000);
  myservo.detach();
  digitalWrite(RELAY_PIN, LOW);
}

// Functions, variables for led control
uint8_t ledStates[3];

void ledSet(uint8_t ledId, uint8_t state, uint8_t updateState=1)
{
  digitalWrite(ledId, state);
  if (updateState)
  {
    switch(ledId)
    {
      case LED_BUILTIN:
        ledStates[0]=state;
        break;
      case LED_RED:
        ledStates[1]=state;
        break;
      case LED_GREEN:
        ledStates[2]=state;
        break;
    }
  }
}

void ledReset(uint8_t ledId)
{
  switch(ledId)
  {
    case LED_BUILTIN:
      digitalWrite(ledId,ledStates[0]);
      break;
    case LED_RED:
      digitalWrite(ledId,ledStates[1]);
      break;
    case LED_GREEN:
      digitalWrite(ledId,ledStates[2]);
      break;
  }
}

void ledBlink(uint8_t ledId, uint8_t blinks, uint8_t repeat=1)
{
    ledSet(ledId,0,0);
    for (uint8_t r=0;r<repeat;r++)
    {
      if (r)
        delay(600);
      for (uint8_t b=0;b<blinks;b++)
      {
           ledSet(ledId,1,0);
           delay(200);
           ledSet(ledId,0,0);
           delay(200);
      }
    }
    ledReset(ledId);
}

// Functions for switch readout  
uint8_t readSwitch(uint8_t switchId)
{
  // check for pressed switch (debounce for 20ms, active low)
  for (int zz=0;zz<10;zz++)
  {  
      if (digitalRead(switchId))
        return 0;
      delay(2);
  }
  return 1; 
}

// Functions, variables for keeping/setting the state of the locker 
uint8_t lockerInUse=0;
unsigned long lockerInUseTime=0;
const uint8_t uidLength=4;
uint8_t storedUid[uidLength];

void uidCopy(uint8_t *buf, uint8_t offset, uint8_t *uid)
{
  for (int zz=0;zz<uidLength;zz++)
    buf[zz+offset]=uid[zz];
}

void uidStore(uint8_t *uid)
{
  for (int zz=0;zz<uidLength;zz++)
    storedUid[zz]=uid[zz];
}

uint8_t uidCompare(uint8_t *uid)
{
  for (int zz=0;zz<uidLength;zz++)
    if (storedUid[zz]!=uid[zz])
      return 0;
  return 1;
}

void uidShow(uint8_t *uid)
{
  for (int zz=0;zz<uidLength;zz++)
  { 
    if (zz)
      debugSerial.print('.');
    if (uid[zz]<16)
      debugSerial.print('0');
    debugSerial.print(uid[zz],HEX);
  }
}

// Functions, variables for thingsnetwork (ABP assumed)
const byte devAddr[4] = { 0xBA, 0xC1, 0xF5, 0x77 }; 
const byte nwkSKey[16] = { 0x3B, 0x91, 0xEB, 0xC5, 0x4C, 0xD6, 0x2F, 0xCC, 0xE0, 0x61, 0xC7, 0x89, 0xE0, 0x84, 0xBC, 0xEA }; 
const byte appSKey[16] = { 0x24, 0xD7, 0xAF, 0x21, 0x23, 0xE3, 0x67, 0x05, 0xE9, 0xE6, 0x86, 0xC3, 0x46, 0x6B, 0x85, 0x1C }; 

uint8_t sendPayload[32];
uint8_t receivePayload[32];
uint16_t receivedBytes;
uint8_t sendRetries=0;

void rawRN2483Cmd(const uint8_t* command)
{
  uint8_t t=0;
  debugSerial.print("CMD : ");
  while (command[t])
  {
    debugSerial.write(command[t]);
    loraSerial.write(command[t++]);
  }
  
  unsigned long time=millis();;

  while(((millis()-time)<1000)&&(!loraSerial.available()))
    delay(10);

  if (loraSerial.available())
  {
    debugSerial.print("RSP : ");
    while (loraSerial.available()) 
      debugSerial.write(loraSerial.read());
  }
  else
    debugSerial.println("No response!");
}

void clearReceivePayload(void)
{
  for (uint8_t zz;zz<32;zz++)
    receivePayload[zz]=0;
}

uint8_t ttnSend(const uint8_t* payload, uint8_t size, uint8_t reqAck=0, uint8_t retries=0)   
{  
  uint8_t resp, ret=0;

/*  rawRN2483Cmd((uint8_t*)"radio get freq\r\n");
  rawRN2483Cmd((uint8_t*)"radio get sync\r\n");
  rawRN2483Cmd((uint8_t*)"radio get cr\r\n");
  rawRN2483Cmd((uint8_t*)"radio get sf\r\n");
  rawRN2483Cmd((uint8_t*)"radio get crc\r\n");
  rawRN2483Cmd((uint8_t*)"radio get prlen\r\n");
*/
  reqAck=0;
  ttn.sendCommand("mac set dr 5");
  if (reqAck)
    resp=ttn.sendBytes(payload,size, 1, 1);
    
  else
    resp=ttn.sendBytes(payload,size, 1, 0);  

  return resp;
}  

void setup()
{
  // initialise debug serial
  while ((!debugSerial) && (millis() < 10000));
  debugSerial.begin(57600);

  // initialise servo
  // todo
  
  // initialise relay
  pinMode(RELAY_PIN, OUTPUT);
  
  // initialise status leds
  pinMode(LED_BUILTIN, OUTPUT);     
  pinMode(LED_RED, OUTPUT);     
  pinMode(LED_GREEN, OUTPUT);     
  
  ledSet(LED_BUILTIN,0);
  ledSet(LED_RED,0);
  ledSet(LED_GREEN,1);

  // initialise switch
  pinMode(SWITCH1_PIN, INPUT_PULLUP);     
  
  loraSerial.begin(57600);
  ttn.init(loraSerial, debugSerial);
  ttn.reset();
  
  if (ttn.personalize(devAddr, nwkSKey, appSKey))
  {
    debugSerial.println("Connected to network.");
    ledSet(LED_BUILTIN,1);
    //ttn.sendCommand("mac set rx2 3 869525000");
    ttn.showStatus();
  }
  else
  {  
    debugSerial.println("Connection to network failed. Halting!");
    while (1)
      ledBlink(LED_BUILTIN,2,3);
  }

  // initialise card reader
  SPI.begin();      
  mfrc522.PCD_Init(); 
}

void loop()
{
  if (readSwitch(SWITCH1_PIN)) // if pressed we need to communicate with server 
  {
    debugSerial.println("Switch pressed, invoking downlink");
    sendPayload[0]=CMD_INVOKE_DOWNLINK;
    clearReceivePayload();
    receivedBytes = (ttnSend(sendPayload,1,1));
    if (receivedBytes)
    {
      debugSerial.println("Acknowledge packet received!");
      if(receivedBytes)
      {  
      	debugSerial.print("I received ");
      	debugSerial.print(receivedBytes);
      	debugSerial.print(" byte(s) : ");
      	for (uint8_t zz=0;zz<receivedBytes;zz++)
          debugSerial.println(ttn.downlink[zz],HEX);
      	
      	if ((ttn.downlink[0]==1)&&(ttn.downlink[1]==0xff))
      	{
      	  debugSerial.println("Open lock command received from downlink, locker is no longer in use!");
          ledSet(LED_RED,0);
          ledSet(LED_GREEN,1);

          lockerInUse=0;
          lockerInUseTime=0;
          openLock(0);

          sendPayload[0]=CMD_SET_LOCKER_STATE;
          sendPayload[1]=0; // vrij
          if (ttnSend(sendPayload,2,1))
            ledBlink(LED_GREEN,3);
      	}
        else
          ledBlink(LED_GREEN,1);
      }
      else    
      	debugSerial.println("No return data received!");
    }
  }
 
  if (mfrc522.PICC_IsNewCardPresent()) 
  {
    if (!mfrc522.PICC_ReadCardSerial()) 
    {
      debugSerial.println("Could not read info of detected card!");
      ledBlink(LED_RED,5);
    }
    else
    {  
      debugSerial.print("Card detected, uid = ");
      uidShow((uint8_t *)mfrc522.uid.uidByte);
      debugSerial.println("");  
      //mfrc522.PICC_DumpToSerial(&(mfrc522.uid));

      switch(lockerInUse)
      {
        case 0: // locker vrij
        debugSerial.println("Locker is now in use!");
        uidStore((uint8_t *)mfrc522.uid.uidByte);
        ledSet(LED_RED,1);
        ledSet(LED_GREEN,0);

        lockerInUse=1;
          lockerInUseTime=millis();
        openLock(180);

        sendPayload[0]=CMD_SET_LOCKER_STATE;
        sendPayload[1]=1; // bezet
          //uidCopy(sendPayload,2,(uint8_t *)mfrc522.uid.uidByte);
          ttnSend(sendPayload,2,1);
  
          delay(2000); // guard time
          break;
        case 1: // locker bezet
          if (uidCompare((uint8_t *)mfrc522.uid.uidByte))
          {
            debugSerial.println("Locker is no longer in use!");

            ledSet(LED_RED,0);
            ledSet(LED_GREEN,1);
  
            lockerInUse=0;
            lockerInUseTime=0;
            openLock(0);
  
            sendPayload[0]=CMD_SET_LOCKER_STATE;
            sendPayload[1]=0; // vrij
            //uidCopy(sendPayload,2,(uint8_t *)mfrc522.uid.uidByte);
            ttnSend(sendPayload,2,1);

        delay(2000); // guard time
      }
      else
      {
            debugSerial.print("Safe is in use by another card (");
            uidShow((uint8_t *)storedUid);
            debugSerial.println(")!");
            
            ledBlink(LED_RED,5);
          }
          break;
        case 2: // locker geblokkeerd
        if (uidCompare((uint8_t *)mfrc522.uid.uidByte))
        {
            debugSerial.println("Locker is blocked, requesting open lock from server!");
            sendPayload[0]=CMD_REQUEST_LOCKER_UNBLOCK;
            uidCopy(sendPayload,1,(uint8_t *)mfrc522.uid.uidByte);
            clearReceivePayload();
              receivedBytes=ttnSend(sendPayload,1+uidLength,1);
            if (receivedBytes)
            {
              debugSerial.println("Acknowledge packet received!");
              if(receivedBytes)
              {  
                debugSerial.print("I received ");
                debugSerial.print(receivedBytes);
                debugSerial.print(" byte(s) : ");
                for (uint8_t zz=0;zz<receivedBytes;zz++)
                  debugSerial.println(ttn.downlink[zz]);
                
                if ((ttn.downlink[0]==1)&&(ttn.downlink[1]==0xff))
                {
                   debugSerial.println("Unblock clearance received from downlink, locker is no longer in use!");
  
          ledSet(LED_RED,0);
          ledSet(LED_GREEN,1);

          lockerInUse=0;
                    lockerInUseTime=0;
          openLock(0);

                    lockerInUse=0;
                    lockerInUseTime=0;
          sendPayload[0]=CMD_SET_LOCKER_STATE;
          sendPayload[1]=0; // vrij
                    if (ttnSend(sendPayload,2,1))
                      ledBlink(LED_GREEN,3);
                  }
                  else
                    ledBlink(LED_GREEN,5);
                }
                else    
                  debugSerial.println("No return data received!");
              }
        }
        else
        {
          debugSerial.print("Safe is in use by another card (");
          uidShow((uint8_t *)storedUid);
          debugSerial.println(")!");
            
          ledBlink(LED_RED,5);
        }
            break;
          default:
            // do nothing
            break;
      }
    }
    delay(500);
  }
  
  if (lockerInUse==1)
  {
    if ((millis()-lockerInUseTime)>120000) // block the safe after 2 minutes (24 hours)
    {
      debugSerial.println("Locker is now blocked!");
      lockerInUse=2;
      lockerInUseTime=0;
      
      ledSet(LED_RED,1);
      ledSet(LED_GREEN,1);

      sendPayload[0]=CMD_SET_LOCKER_STATE;
      sendPayload[1]=2; // geblokkeerd
      uidCopy(sendPayload,2,(uint8_t *)mfrc522.uid.uidByte);
      if (ttnSend(sendPayload,2+uidLength,1))
  ledBlink(LED_GREEN,3);

    }
  }   
}


