//  ESP RainMaker - for MasterBedroom room Geyer , device name MasterBedroom Geyser
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "esp_timer.h"
#include "./ACS712.h"


// remove 2nd step
Param current("Current", "custom.param.text", value(15), PROP_FLAG_READ ); // param to show current

const char *Current="Current"; // remove it later

// remove all above

const char *OTAName = "MasterBedroomGeyser01" ; // name of OTA service, change it

#define DEFAULT_POWER_MODE false
#define DEFAULT_DURATION_LEVEL 5
const char *service_name = "PROV_2342";
const char *pop = "xyzv1232";
 
//GPIO for virtual device
static int gpio_0 = 0; // reset pin
static int gpio_relay = 2 ; // use the correct GPIO where the relay is attached, change this to 2 ( actual)

// for current sensor 

#define AMP 3000 //5Amp

const int acsPin = 34; // Input pin for acs712 current sensor

ACS712  ACS(acsPin,3.3, 4096, 66); //  current sensor 4096


volatile int isCurrent=1; // default 15A is on


int remaining_time=DEFAULT_DURATION_LEVEL; // used to track remaining time using timer

bool relay_state = false;  // intial state of relay

Param time_paramR("Remaining", "custom.param.text", value(DEFAULT_DURATION_LEVEL), PROP_FLAG_READ ); 

Param no_Power("PowerState", "custom.param.text", value(""), PROP_FLAG_READ ); // param to show there is no power


static Device my_device("MasterBedroom Geyser", "custom.device.geyser", &gpio_relay);  // change name of the device here

// timer related variables

volatile int onceonly=0; // just to ensure the timer interrupt does not get called after relay is off.

esp_timer_handle_t timer; // esp soft timer 

void IRAM_ATTR  onTimer(); //forward declaration

const esp_timer_create_args_t timerParameters = { .callback = reinterpret_cast<esp_timer_cb_t>( onTimer) };

// using these constants to be able to update the client mobile app as needed outside of the writecall back

const char *remaining ="Remaining"; // variable for time remaining - made global
const char *powerS = "Power";  // // variable for time remaining - made global
const char *PowerState="PowerState"; // power state 15A is there or not
const char *Duration="Duration"; // duration UI bar


bool computeCurrent () { // computes if there is 15A current using acs712

  int i=0;
  int mA =0;
 
  delayMicroseconds(100);
 
  mA = ACS.mA_AC();
   
  Serial.printf("mA=%d\n", mA);
  my_device.updateAndReportParam(Current, mA);
      
return mA > AMP ? true : false ;
    
}


void sysProvEvent(arduino_event_t *sys_event)
{
    switch (sys_event->event_id) {      
        case ARDUINO_EVENT_PROV_START:

        Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
        printQR(service_name, pop, "ble");
        break;
        default:;
    }
}
void IRAM_ATTR stop_timer() {
        int error = esp_timer_is_active(timer);
        Serial.printf("return code esp_time_is_active= %d, onceonly : %d, isCurrent:%d \n", error,onceonly,isCurrent);
        delay(100);
        
         if (esp_timer_is_active(timer)) 
        {
           
            ESP_ERROR_CHECK(esp_timer_stop(timer));
           
            int err2=esp_timer_delete (timer);
         
            delay(500);
            isCurrent=1;
            relay_state=false; // important line
           
        }
}

// timer interrupt for checking every 1 minute
void IRAM_ATTR  onTimer() {

 Serial.printf("OnTimer: is again getting called\n");
  remaining_time--;
  my_device.updateAndReportParam(remaining, remaining_time);
  
  if (  !computeCurrent()) {  // if 15A power is not there, switch off relay and update the UI.
    
    my_device.updateAndReportParam(powerS, DEFAULT_POWER_MODE); // required to turn off power on app and report it back

    digitalWrite(gpio_relay, LOW);
   
    Serial.printf("Low current, switching of the Geyser\n");
    my_device.updateAndReportParam(PowerState, "*** No Power ***");
    isCurrent=0; // no Power
    relay_state = false;
     
  } 
  if ( remaining_time == 0 ) { 
    digitalWrite(gpio_relay, LOW);
    my_device.updateAndReportParam(powerS, DEFAULT_POWER_MODE); // required to turn off power on app
     
  }
   
}

// call back function gets called whenever we modify state in app

void IRAM_ATTR write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
    const char *device_name = device->getDeviceName();
    const char *param_name = param->getParamName();

     if(strcmp(param_name, "Power") == 0) {
        Serial.printf("Received value = %s for %s - %s\n", val.val.b? "true" : "false", device_name, param_name);
        relay_state = val.val.b;
                
        (relay_state == false ) ? digitalWrite(gpio_relay, LOW) : digitalWrite(gpio_relay, HIGH);
        
        if ( !computeCurrent()) { // if 15A is not there ,  set relay_switch to false to get power off 
          relay_state = false;
          Serial.printf("Low current, switching of the Geyser from write_callback\n");
          my_device.updateAndReportParam(PowerState, "*** No Power ***");
          digitalWrite(gpio_relay, LOW);

        }
        else
            my_device.updateAndReportParam(PowerState, "");

            
         my_device.updateAndReportParam(powerS, relay_state);

         Serial.printf(" write_callback:onceonly : %d, isCurrent:%d : relay_state=%d remaining_time :%d\n", onceonly,isCurrent, relay_state, remaining_time);

       
         if ((relay_state==false) ) // turn off the timer once the geyser switch is off
               remaining_time=0;                  
         else
            my_device.updateAndReportParam(remaining, remaining_time);
            
         if ((relay_state == true) && (remaining_time==0)) {
            onceonly=0;  
            remaining_time=DEFAULT_DURATION_LEVEL;
         }
                            
    } else if (strcmp(param_name, "Duration") == 0) {
        Serial.printf("\nReceived value = %d for %s - %s\n", val.val.i, device_name, param_name);
        param->updateAndReport(val);
        remaining_time = val.val.i; 
        my_device.updateAndReportParam(remaining, remaining_time); // update remaining time to the value given
    }
}



void setup()
{
    Serial.begin(115200);
    pinMode(gpio_0, INPUT);
    pinMode(gpio_relay, OUTPUT);
    digitalWrite(gpio_relay, LOW);

    // set the midpoint for ACS712
    ACS.autoMidPoint();

    Node my_node;    
    my_node = RMaker.initNode("ESP RainMaker Geyser2");

    my_device.addNameParam();
    my_device.addPowerParam(DEFAULT_POWER_MODE);
    my_device.assignPrimaryParam(my_device.getParamByName(ESP_RMAKER_DEF_POWER_NAME));

     no_Power.addUIType(ESP_RMAKER_UI_TEXT);
     my_device.addParam(no_Power);

    //Create and add a custom level parameter
    Param level_paramD("Duration", "custom.param.level", value(DEFAULT_DURATION_LEVEL), PROP_FLAG_READ | PROP_FLAG_WRITE);
    level_paramD.addBounds(value(0), value(30), value(1));
    level_paramD.addUIType(ESP_RMAKER_UI_SLIDER);
    my_device.addParam(level_paramD);

    // add another parameter for time remaining

     
     time_paramR.addUIType(ESP_RMAKER_UI_TEXT);
     my_device.addParam(time_paramR);

     // remove this later +++++++++++++++++
     current.addUIType(ESP_RMAKER_UI_TEXT);
     my_device.addParam(current);
 
    my_device.addCb(write_callback);  // attach the callback to the device
    
    //Add custom geyser device to the node   
    my_node.addDevice(my_device);

     //If you want to enable scheduling, set time zone for your region using setTimeZone(). 
    //The list of available values are provided here https://rainmaker.espressif.com/docs/time-service.html

    
   // RMaker.setTimeZone("Asia/Kolkata");

     
    // Alternatively, enable the Timezone service and let the phone apps set the appropriate timezone
    RMaker.enableTZService();

    RMaker.enableSchedule();

    RMaker.start();

    WiFi.onEvent(sysProvEvent);

    // provision over BLE

    WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);

    my_device.updateAndReportParam(powerS, DEFAULT_POWER_MODE);   
    digitalWrite(gpio_relay, LOW);



// Classic OTA

ArduinoOTA.setHostname(OTAName);  

// OTA code added
ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
       
}


void loop()
{
    ArduinoOTA.handle();
    
    delay(100);
    
    if (relay_state==true && onceonly==0 ) {
      
      // Timer initialize and set for 1 minute wakeups
        
        Serial.printf("Main Loop: Starting our timer\n");
        
        esp_timer_create(&timerParameters, &timer);
        esp_timer_start_periodic(timer, 60000000); // 1 minute
          onceonly=1;
          isCurrent=1;       
 
      
      }
      if ((onceonly == 1 && remaining_time == 0) || isCurrent == 0 ) {
        Serial.printf("Main loop:Came to stop_timer loop\n");
        stop_timer();
        onceonly=0;
        remaining_time=DEFAULT_DURATION_LEVEL;
        // set the default duration level
        my_device.updateAndReportParam(Duration, DEFAULT_DURATION_LEVEL); // required to turn off power on app and report it back


      }  
     
     // mainly for resetting using gpio_o , normally not used
        
    if(digitalRead(gpio_0) == LOW) { //Push button pressed

        // Key debounce handling
        delay(100);
        int startTime = millis();
        while(digitalRead(gpio_0) == LOW) delay(50);
        int endTime = millis();

        if ((endTime - startTime) > 10000) {  
          // If key pressed for more than 10secs, reset all
          Serial.printf("Reset to factory.\n");
          RMakerFactoryReset(2);
        } else if ((endTime - startTime) > 3000) {
          Serial.printf("Reset Wi-Fi.\n");
          // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
          RMakerWiFiReset(2);
        } else {
          // Toggle device state
          relay_state = !relay_state;
          Serial.printf("Toggle State to %s.\n", relay_state ? "true" : "false");
          my_device.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, relay_state);
          (relay_state == false) ? digitalWrite(gpio_relay, LOW) : digitalWrite(gpio_relay, HIGH);
      }
    }
  
    
    delay(100);
}
