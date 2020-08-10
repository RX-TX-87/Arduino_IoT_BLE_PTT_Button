/*
  Example for how to connect a Zello BLE PTT Button (NEXTAV PTT-U2-L0) to an Arduino Nano 33 IOT
  https://github.com/RX-TX-87/Arduino_IoT_BLE_PTT_Button
  8/10/2020
*/

#include <ArduinoBLE.h>

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

//--------------------------------------------------------------------------------------------
// Prototypes

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();

void BLE_PTT_Button_start_scan();
void BLE_PTT_Button_check_connect_and_subscribe();

//--------------------------------------------------------------------------------------------
// Variables

BLEDevice BLE_PTT_Button;
BLECharacteristic simpleKeyCharacteristic;

bool isLEDOn = false;

bool timer_flag_5ms = false;
bool timer_flag_1s = false;
int timer_5ms_overflow_count = 0;

int PTT_Button_Val = 0; // external PTT button
byte BLE_PTT_Button_Val = 0; // BLE PTT button
int BLE_PTT_Button_Status = 0; // 0=scan for BLE device; 1=subscribe; 2=sucessfully subscribed/operational


//========================================================================================================================================
//========================================================================================================================================
//========================================================================================================================================


void setup() {

  //-----------------------------------------------------------------------------------
  // USB Serial
  Serial.begin(9600);
  //while (!Serial);
 
  //-----------------------------------------------------------------------------------
  // Configure built-in LED

  pinMode(LED_BUILTIN, OUTPUT);

  //-----------------------------------------------------------------------------------
  // Define input and output pins 

  // External LED (D3)
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);  

  //-----------------------------------------------------------------------------------
  // TIMER

  startTimer(200); // 200 Hz = interrupt every 5 ms

  //-----------------------------------------------------------------------------------
  // initialize the BLE hardware
  
  BLE.begin();

  //-----------------------------------------------------------------------------------
  
}

//========================================================================================================================================
//========================================================================================================================================
//========================================================================================================================================

void loop() {


  //********************************************************************************************************************
  if (timer_flag_5ms == true){

    //------------------------------------------------------------
   
    if (BLE_PTT_Button_Status == 2){

      if (BLE_PTT_Button.connected()) {

        // check if the value of the simple key characteristic has been updated
        if (simpleKeyCharacteristic.valueUpdated()) {
          
          // if yes get the value, characteristic is 1 byte therefore use byte value
          BLE_PTT_Button_Val = 0;          
          simpleKeyCharacteristic.readValue(BLE_PTT_Button_Val);
          Serial.println(BLE_PTT_Button_Val);

          if ( BLE_PTT_Button_Val == 0 ) {
            // Switch off external LED if button is not pressed
            digitalWrite(3, LOW);
          } else {
            // Switch on external LED if button is pressed
            digitalWrite(3, HIGH);
          }
    
        }     
        
      } else {

        // BLE connection was lost -> initiate new scan
         BLE_PTT_Button_Status = 0;
         Serial.println("BLE connection lost");
                 
      }
      
    }
    
    //------------------------------------------------------------
    // Reset timer flag

    timer_flag_5ms = false;

    //------------------------------------------------------------


  }

  //********************************************************************************************************************
  // Timer flag is set every one second
  
  if (timer_flag_1s == true){

    //------------------------------------------------------------
    // Manage BLE PTT button connection

    if (BLE_PTT_Button_Status == 0){
      BLE_PTT_Button_Status=99; // wait for completion
      BLE_PTT_Button_start_scan();
    }
    
    if (BLE_PTT_Button_Status == 1){
       BLE_PTT_Button_Status=99; // wait for completion
       BLE_PTT_Button_check_connect_and_subscribe();
    }
 
    //------------------------------------------------------------
    // Toggle LED

    digitalWrite(LED_BUILTIN, isLEDOn);
    isLEDOn = !isLEDOn;

    //------------------------------------------------------------
    // Reset timer flag

    timer_flag_1s = false;

    //------------------------------------------------------------
  }
  //********************************************************************************************************************


}

//========================================================================================================================================
//========================================================================================================================================
//========================================================================================================================================

void BLE_PTT_Button_start_scan(void) {

  // BLE PTT Button (Zello)  
  BLE.scanForAddress("ff:32:77:c4:fe:69");
  //BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");

  Serial.println("BLE scan started ");

  BLE_PTT_Button_Status=1;

}

//========================================================================================================================================
//========================================================================================================================================
//========================================================================================================================================

void BLE_PTT_Button_check_connect_and_subscribe(void){

  // check if a BLE PTT Button peripheral has been discovered
  BLE_PTT_Button = BLE.available();

  if (BLE_PTT_Button) {
    
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("BLE found ");
    Serial.print(BLE_PTT_Button.address());
    Serial.print(" '");
    Serial.print(BLE_PTT_Button.localName());
    Serial.print("' ");
    Serial.print(BLE_PTT_Button.advertisedServiceUuid());
    Serial.println();

    // stop scanning
    BLE.stopScan();

  } else {
    Serial.println("BLE scan: no device available");
    BLE_PTT_Button_Status=1;
    return;
  }


  // connect to the peripheral
  Serial.println("Connecting ...");

  if (BLE_PTT_Button.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    BLE_PTT_Button_Status=1;
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (BLE_PTT_Button.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    BLE_PTT_Button.disconnect();
    BLE_PTT_Button_Status=1;
    return;
  }


  // retrieve the Button characteristic
  //if (BLE_PTT_Button.discoverService("ffe0")) {
  if (BLE_PTT_Button.discoverService("65504")) {
    Serial.println("Discovered SimpleKey service");
  }
  else {
    Serial.println("Could not discover SimpleKey service");
    BLE_PTT_Button.disconnect();
    BLE_PTT_Button_Status=1;
    return;
  }

  //int characteristicCount = peripheral.characteristicCount();
  //Serial.print("CharacteristicsCount: ");
  //Serial.println(characteristicCount);
  
  simpleKeyCharacteristic = BLE_PTT_Button.service("ffe0").characteristic(0);

  if (simpleKeyCharacteristic) {
    Serial.println("Peripheral has SimpleKey characteristic!");
  }
  else {
    Serial.println("Peripheral does NOT have SimpleKey characteristic!");
    BLE_PTT_Button_Status=1;
    return;
  }

  /*
  if (simpleKeyCharacteristic.canRead()) {
    Serial.println("characteristic is readable");
  } else {
    Serial.println("characteristic is not readable");
    BLE_PTT_Button_Status=1;
    return;
  }
  */

  if (simpleKeyCharacteristic.canSubscribe()) {
    Serial.println("characteristic is subscribable");
  } else {
    Serial.println("characteristic is not subscribable");
    BLE_PTT_Button_Status=1;
    return;
  }


  if (simpleKeyCharacteristic.subscribe()) {
    Serial.println("SimpleKey is subscribed");
    BLE_PTT_Button_Status=2;
  } else {
    Serial.println("SimpleKey is not subscribed");
    BLE_PTT_Button_Status=1;
    return;
  }

  
}


//========================================================================================================================================
//========================================================================================================================================
//========================================================================================================================================


void setTimerFrequency(int frequencyHz) {
  
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
  
}


//========================================================================================================================================
//========================================================================================================================================
//========================================================================================================================================

/*
This code is by Nebs Petrovic and can be found at: 
http://github.com/nebs/arduino-zero-timer-demo.
It is a slightly modified version of the timer setup found at:
https://github.com/maxbader/arduino_tools
 */
 
void startTimer(int frequencyHz) {
  
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID (GCM_TCC2_TC3)) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 );

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1);

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1);
  
}

//========================================================================================================================================
//========================================================================================================================================
//========================================================================================================================================

void TC3_Handler() {
  
  TcCount16* TC = (TcCount16*) TC3;
  
  // If this interrupt is due to the compare register matching the timer count
  // the flags timer_flag_5ms and timer_flag_1s are set accordingly
  
  if (TC->INTFLAG.bit.MC0 == 1) {
    
    TC->INTFLAG.bit.MC0 = 1;

    timer_flag_5ms = true;
    timer_5ms_overflow_count++;

    if (timer_5ms_overflow_count == 200){
     timer_flag_1s = true;
     timer_5ms_overflow_count = 0;
    }
        
    //digitalWrite(LED_BUILTIN, isLEDOn);
    //isLEDOn = !isLEDOn;
    
  }

}

//========================================================================================================================================
//========================================================================================================================================
//========================================================================================================================================
