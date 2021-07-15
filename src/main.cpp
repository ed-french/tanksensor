/*


        Tank depth sensor node

        Based on Bluepill wired to VL52L0X sensor




        Wiring
        ======

                                  
        3v3 -------T----------------Bat ++*** For LFePo4 only (Lion goes to 5v)
                   |
                   L----------------Sensor Vcc
                   |
                   L----------------Lora Vcc (may need Capacitor)

        Gnd -------T----------------Bat --
                   |
                   L----------------Sensor Gnd
                   |
                   L----------------Lora Gnd

        PB7-------------------------Sensor SDA

        PB6-------------------------Sensor SCL


        PA7-------------------------Lora MOSI
        PA6-------------------------Lora MISO
        PA5-------------------------Lora SCK
        PA4-------------------------Lora NSS
        PB10------------------------Lora DI0
        PB11------------------------Lora rst


                                    Bat++

        PA2------------------



        Optionally
        ----------




        Sensor address = 0x29



        3.2V = 347
        3.33V = 631
        







*/

#include <Arduino.h>

#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <SPI.h>
#include <LoRa.h>
#include <crc16.h>
#include "STM32LowPower.h"


#define LED_BUILTIN PC13
#define SENSOR_TIMING_BUDGET 200000 //ms
#define AVE_OVER_READINGS 10
#define NODE_ID 9
#define LORA_RESENDS 2 // Number of times each message is sent
#define SLEEPTIME_SECS 3*60 // Send every three mins



#define ARM_DELAY 10 // Seconds 


#ifdef BLACKPILL

#define BATTERY_LEVEL_PIN PA2
#define LORA_SS_PIN PA4
#define LORA_RESET_PIN PC14
#define LORA_DIO0_PIN PB10
#define PIN_MEASURE PB4 // Noise used for RNG seed

#define PIN_SENSOR_SDA PB7
#define PIN_SENSOR_SCL PB6

#else
#define HARDWARE_SLEEP
#define BATTERY_LEVEL_PIN PA2
#define LORA_SS_PIN PA4
#define LORA_RESET_PIN PC14
#define LORA_DIO0_PIN PB10
#define PIN_MEASURE PB4 // Noise used for RNG seed

#define PIN_SENSOR_SDA PB7
#define PIN_SENSOR_SCL PB6

#endif

#define SENSOR_I2C_ADDRESS 0x29 //VL53X0 sensor

#define MESSAGE_DESTINATION_NODE 0x01 // Broadcast only to master node, all others will ignore


/* message format is:

Offset range        Content
============        ==============================
0                   Destination (0xFF=BROADCAST, 0x01=MASTER,0x02=ERROR REPORTING)
1-4                 Message ID (32 bit random no.)
5                   Sender (0x01 MASTER, 0xFF is illegal)
6                   Type - see enum below
7-51                45 bytes of message content
52-53               16 bit CRC of message              




*/

#define CORE_MESSAGE_LENGTH 52
#define BROADCAST_ADDRESS 0xFF
#define MASTER_ADDRESS 0x01
#define ERROR_REPORTING_ADDRESS 0x02
#define STILL_ALIVE_MASK 0x01FF// For frequent set to 0x007F





enum  MessageType : uint8_t
{
  LOWER_ENUM_BOUND=1,
  NORMAL=2,
  BROADCAST=3,
  ACKNOWLEDGEMENT=4,
  ERROR_=6,
  RESERVED=7,
  RESERVED2=8,
  RESERVED3=9,
  UPPER_ENUM_BOUND=10
};
enum MessageState
{
  READY_TO_SEND,
  SENT_NOT_ACKNOWLEDGED,
  ACKNOWLEDGED_NOT_DELETED
};

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void(* resetFunc) (void) = 0; //declare reset function @ address 0

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}



void seedRNG()
{
  // By itself random isn't
  // So building a 32 bit seed using analog reads

  uint32_t randseed=0;
  for (uint8_t i=0;i<32;i++)
  {
    uint8_t onebit=analogRead(PIN_MEASURE) & 0x01; // Just take least significant bit
    randseed=(randseed<<1) | onebit; // Add the next bit
  }

  Serial.printf("Made a new random number seed of : %d\n",randseed);

  randomSeed(randseed);
}

class  LoraMessage
{
  public:
    uint8_t destination;
    uint8_t sender;
    MessageType type;
    uint32_t message_id;
    uint16_t checksum;
    char content[45];

    LoraMessage()
    {//Declare an empty message, normal for sending
        // Generate an ID
        //for (int i=0;i<20;i++)
        //{
            message_id=random(2147483647);
            //Serial.print("Message id set to : ");
            //Serial.println(message_id);
        //}
    }

    LoraMessage(char * received, uint8_t messagelength)
    {// Constructor used with a received message
    // tries to parse it into a valid object
    // If it fails it sets the MessageType to error

      // Reset the message to error
      memset(content,0,sizeof content);
      char * defaultcontent="Error parsing message";
      strcpy(content,defaultcontent);
      type=(MessageType)255;
      destination=2;
      sender=2;
      message_id=999;

    // Deserialization here with lots of error checks!
    
      // Do we have the right number of bytes
      if (messagelength!=CORE_MESSAGE_LENGTH+2)
      {
        type=ERROR_;
        Serial.print("Wrong message length : ");
        Serial.println(strlen(received));
        return;
      }


      // does the CRC match
      uint16_t calcedcrc=calcCRC(received,messagelength-2);
      //Serial.print("Calculated CRC should be : "); //Correct
      //Serial.println(calcedcrc);

      uint16_t rxedcrc=(uint16_t)(uint8_t)received[CORE_MESSAGE_LENGTH]+256*(uint16_t)(uint8_t)received[CORE_MESSAGE_LENGTH+1];
      //Serial.print("Received CRC was : ");
      //Serial.println(rxedcrc);
      //Serial.println((uint16_t)(uint8_t)received[CORE_MESSAGE_LENGTH]);
      //Serial.println((uint16_t)(uint8_t)received[CORE_MESSAGE_LENGTH+1]);

      if (calcedcrc!=rxedcrc)
      {
        type=ERROR_;
        Serial.println("CRC does not match.");
        return;
      }
      // Enough with the checks, now just deserialize the bits
      uint8_t * p=(uint8_t *)received; // Pointer to traverse the rx'ed data
      memcpy(&destination,p,1);
      //Serial.print("Destination : ");
      //Serial.println(destination);
      p++;
      memcpy(&message_id,p,4);
      //Serial.print("ID : ");
      //Serial.println(message_id);
      p=p+4;
      memcpy(&sender,p,1);
      //Serial.print("Sender : ");
      //Serial.println(sender);
      p++;
      memcpy(&type,p,1);
      //Serial.print("Type : ");
      //Serial.println(type);
      p++;
      memcpy(&content,p,45);
      // is type a valid enum type
      if ((type<=LOWER_ENUM_BOUND) | (type>=UPPER_ENUM_BOUND))
      {
        type=ERROR_;
        Serial.print("Out of bounds for message type : ");
        Serial.println(type);
        return;
      }




    

    } // End of constructor using rx'ed string

    void setDestination(uint8_t dest)
    {
        destination=dest;
    }  
    void setSender(uint8_t send_to)
    {
        sender=send_to;
    }
    void setType(MessageType typ)
    {
        type=typ;
    }
    void setContent(char * cont)
    {
        strncpy(content,cont,45);
    }

    uint16_t calcCRC(char* str,uint8_t length)
      {
        uint16_t crc=0; // starting value as you like, must be the same before each calculation
        for (int i=0;i<length;i++) // for each character in the string
        {
          crc= _crc16_update (crc, str[i]); // update the crc value
        }
        return crc;
      }

    char * serialize()
    {// Returns a complete string for the message including checksum
        char *result=(char *)malloc(CORE_MESSAGE_LENGTH+2);
        char * p=result; // move this pointer along as we go
        *p=destination;
        p++;
        memcpy(p,&message_id,4);
        p=p+4;
        *p=sender;
        p++;
        *p=type;
        p++;
        memcpy(p,&content,45);
        p=p+45;
        uint16_t crc=calcCRC(result,CORE_MESSAGE_LENGTH);
        memcpy(p,&crc,2);
        return result;
    }

    void print()
    {// Returns a printable version of the message
        Serial.print("Message ID : ");
        Serial.println(message_id);
        Serial.print("Type : ");
        Serial.println(type);
        Serial.print("Sender : ");
        Serial.println(sender);
        Serial.print("Destination :");
        Serial.println(destination);
        Serial.println("Content :");
        Serial.println(content);

    }

    
};


void sendMessage(LoraMessage mess) {
  Serial.println("About to send ...");
  for (uint8_t sendcount=0;sendcount<LORA_RESENDS;sendcount++)
  {
    mess.print();
    LoRa.beginPacket();                   // start packet
    //char * rawbytes=testmsg.serialize();
    char * serialized=mess.serialize();
    LoRa.write((uint8_t*)serialized,CORE_MESSAGE_LENGTH+2);              // add destination address
    free(serialized);
    LoRa.endPacket();                     // finish packet and send it
    delay(random(2000));// Random gap between resends
  }                 // increment message ID
}


void onReceive(int packetSize) 
{// Handles incoming packets and parses out a message from the packet
  if (packetSize == 0) return;          // if there's no packet, return
  Serial.println("Found something....");
  static char buffer[CORE_MESSAGE_LENGTH+2];// Somewhere to put the answer for a bit

  for (int i=0;(i<CORE_MESSAGE_LENGTH+2 && LoRa.available());i++)
  {
    buffer[i]=LoRa.read();
  }
  if (LoRa.available())
  {
    // Dump extra bytes
    while(LoRa.available())
    {
      Serial.print("xx"+LoRa.read());
    }
  }
Serial.println(buffer);

  LoraMessage mess=LoraMessage((char *)buffer,CORE_MESSAGE_LENGTH+2);
  
  // For now just print it out

  Serial.println("Received new message....");

  // Note could still be an error!

  mess.print();

  if (mess.type!=NORMAL)
  {
    Serial.println("Not a normal message, could be an error");
  }

  // Check message is intended for the gate
  if (mess.destination!=NODE_ID) 
  {
    Serial.println("Not for this node");
    return; // We can ignore other packets, we are a gate!
  }
  // Process the received message, if any



}






void sendLora(char * message)
{
  
  Serial.println("Starting up lora");
  delay(20);
  #ifdef BUILD_FOR_BLACK_PILL
    SPIClass spiiii=SPIClass(LORA_MOSI_PIN,LORA_MISO_PIN,LORA_SCLK_PIN,LORA_SS_PIN);
    LoRa.setSPI(spiiii);
  #else
    
  #endif
  
  LoRa.setPins(LORA_SS_PIN,LORA_RESET_PIN, LORA_DIO0_PIN);// should be 8
  Serial.println("Set pins");


  LoRa.setCodingRate4(8);// Default is 5, should improve link budget at cost of b/w
  Serial.println("Set coding rate");
  LoRa.setSpreadingFactor(10);// Default is 7 so link budget should be improved
  Serial.println("Set spreading factor");
  LoRa.setSignalBandwidth(20.8E3);// defaults to 125kHz 125E3
  Serial.println("Set bandwidth");
  LoRa.setTxPower(20);// default is 17

  Serial.println("Set LoRa parameters");
  
  delay(20);
  if (!LoRa.begin(433E6)) 
  {             // initialize ratio at 433 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa established");
  delay(20);


  
  Serial.printf("Sending %s\n",message);

  // Typical message: DRIVE:x:120.3,y:-43.2,z:0.02,battery:3.432

  
  LoraMessage mess=LoraMessage();
  mess.setType(NORMAL);
  mess.setDestination(MASTER_ADDRESS);
  mess.setSender(NODE_ID);
  mess.setContent(message);
  sendMessage(mess);
}


bool initialize_sensor(bool closefirst=false)
{
  /*
      Tries to initialise the distance sensor, returning true if it suceeded
      or false otherwise
  */
      // Initialise the distance sensor
    if (closefirst) Wire.end();

    int retries=10;
    

    //Wire.begin((uint8_t)PIN_SENSOR_SDA,(uint8_t)PIN_SENSOR_SCL); // SDA, SCL
    //Wire.setClock(10000);// Set slow mode to cope with long cables

    //sensor.setAddress(SENSOR_I2C_ADDRESS);

    //sensor.setTimeout(500);
    
    Serial.println("Sensor I2C set, up");
    while (true)
    {
      //TwoWire tw=TwoWire((uint8_t)PIN_SENSOR_SDA,(uint8_t)PIN_SENSOR_SCL);
      if (lox.begin())//lox.begin(0x29,true,&tw))
      {
        Serial.println("sensor OK");
        break;
      }
      //tw.end();
      Serial.println(F("Failed to boot VL53L0X"));

        
        delay(400);
    }
    Serial.println("Sensor initialised OK");

    
// // lower the return signal rate limit (default is 0.25 MCPS)
//   sensor.setSignalRateLimit(0.1);
//   // increase laser pulse periods (defaults are 14 and 10 PCLKs)
//   sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
//   sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);


// increase timing budget to 200 ms to inc accuracy
  //sensor.setMeasurementTimingBudget(SENSOR_TIMING_BUDGET);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
    //sensor.startContinuous();
  
  return true;
}


float getReading()
{
  initialize_sensor(true);
  uint8_t count=0;
  float accum=0;
  //sensor.startContinuous();
  Serial.print("(");
  uint16_t bad_count=0;
  VL53L0X_RangingMeasurementData_t measure;
  uint16_t reading;
  while (count<AVE_OVER_READINGS)
  {
    
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    //uint16_t reading=sensor.readRangeSingleMillimeters();//readRangeContinuousMillimeters();
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      reading=measure.RangeMilliMeter;
    } else {
      Serial.println(" out of range ");
      reading=65535;
    }
    if (reading==65535) 
    {
      Serial.print("!");
    }
    Serial.print(reading);
    Serial.print(",");

    if (reading<8000)
    {
      // Valid reading
      
      accum+=reading;
      count++;
      bad_count=0;
    } else {
      // Bad reading
      
      bad_count++;
      Serial.printf(" BAD#%d ",bad_count);
      if (bad_count>300)
      {
        sendLora("TANK: RESTARTING -SENSOR STUCK");
        delay(2000);
        resetFunc();
      }
      if (bad_count>10)
      {
        
        Serial.println("Restarting the sensor");
        delay(2000);// Pause
        initialize_sensor();// Try to restart the sensor
      }
    }
  }
  Serial.println(")");
  //sensor.stopContinuous();

  return accum/count;
}

// void trigger_relay()
// {
//     digitalWrite(PIN_RELAY,LOW);
//     pinMode(PIN_RELAY,OUTPUT);
//     digitalWrite(PIN_RELAY,LOW);
//     Serial.write("Relay triggered");
//     delay(500);
//     pinMode(PIN_RELAY,INPUT);
//     digitalWrite(PIN_RELAY,HIGH);
// }


uint16_t getBatterymv()
{// returns level in mv
  uint16_t reading=analogRead(BATTERY_LEVEL_PIN);
  uint16_t millivolts=(uint16_t)(7552.6-(float)reading*13.158);//(uint16_t)((float)reading*3928.0/384.0);
  return millivolts;
}

void setup()
{
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);
    Serial.begin(115200);
    Serial.println("Starting:");
    for (int i=ARM_DELAY;i>0;i--)
    {
        Serial.println(i);
        delay(1000);
    }
    seedRNG();
    Serial.println("Starting to try and initialise the sensor...");
    while (!initialize_sensor(false))
    {
      Serial.println("Failed to initialise the tank depth sensor");
      sendLora("TANK: SENSOR INIT FAILED");
      delay(5*60*1000);// Wait 5 mins
    }
    
    sendLora("TANK: BOOTED");
    #ifdef HARDWARE_SLEEP
      LowPower.begin();
    #endif
}

void i2ctestloop()
{
  Wire.begin((uint8_t)PIN_SENSOR_SDA,(uint8_t)PIN_SENSOR_SCL); // SDA, SCL
  Wire.setClock(10000);// Set slow mode to cope with long cables
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 

 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.printf("Error: %d at address 0x",error);
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for next scan
}


void loop()
{
    while(true)
    {
        digitalWrite(LED_BUILTIN,LOW);
        float new_reading=getReading();//getReading();
        Serial.println(new_reading);
        char reading_str[10];
        dtostrf(new_reading,4,2,reading_str);



        // Get a battery level measurement
        uint16_t bat=getBatterymv();


        // Send the result

        char buf[60];

        sprintf(buf,"TANK:LEVEL:%s,BATTERY:%d",reading_str,bat);
        Serial.println(buf);
        
        sendLora(buf);
        
        Serial.printf("Free memory : %d\n",freeMemory());
        Serial.println("Switching LED off");
        digitalWrite(LED_BUILTIN,HIGH);
        delay(1000);
        Serial.println("Going to sleep");



        #ifdef HARDWARE_SLEEP
          LowPower.deepSleep((uint32_t)SLEEPTIME_SECS*1000);
        #else
          delay(4000);
        #endif


        digitalWrite(LED_BUILTIN,LOW);
        Serial.begin(115200);
        Serial.println("Woken");
        
        ///delay(200);
        //trigger_relay();
        //delay(5000); // Will be the sleep part eventually
    }
}
