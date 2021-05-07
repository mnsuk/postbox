//#define DEBUG
#ifdef DEBUG
#define DB if(1)
#else
#define DB if(0)
#endif
/*
Postbox
=======
Designed to work with HomeAssistant. Send a MQTT message when the
postbox top flap is opened. Go into deep sleep when it is closed.


Hardware Connections
======================
Top postal flap magnetic reed switch connected to GPIO 25. 
Owner's door magnetic reed switch connected to GPIO 26. 

Both require external 100K pullup resistors.

Author:
Martin Saunders <mnsaunders@gmail.com>
*/
#include "EspMQTTClient.h"
#include "driver/adc.h"
#define SSID "Penguins"
#define SSID_PASSWORD "9EF382ABCD"
#define MQTT_USER "mq_user"
#define MQTT_PASSWORD "SPQR69tt"
#define MQTT_CLIENTNAME "ESP32_postboxV2"

#define TOPIC_HOME "homeassistant/binary_sensor/postbox/"
#define AVAILABILITY_TOPIC TOPIC_HOME "availability"
#define FLAP_STATE_TOPIC TOPIC_HOME "flapstate"
#define DOOR_STATE_TOPIC TOPIC_HOME "doorstate"

#define ST_INIT 0
#define ST_FIRSTOPEN 1
#define ST_STILLOPEN 2
#define ST_STUCK 3
#define ST_CLOSED 4

#define MAX_OPENFLAP_TIME 15000 // 15 seconds
#define MAX_STILL_OPEN_COUNT 3
#define STILL_OPEN_TIMER_SLEEP_MICROSECS 120 * 1000000 // 2 minutes
#define STUCK_TIMER_SLEEP_MICROSECS 1800 * 1000000     // 30 minutes


#define GPIO_25_BITMASK 0x2000000
#define GPIO_26_BITMASK 0x4000000
#define GPIO_FLAP_TRIGGER_BITMASK GPIO_25_BITMASK
#define GPIO_DOOR_TRIGGER_BITMASK GPIO_26_BITMASK
#define GPIO_FLAP_AND_DOOR_TRIGGER_BITMASK 0x6000000



EspMQTTClient client(
    SSID,
    SSID_PASSWORD,
    "192.168.0.7",  // MQTT Broker server ip
    MQTT_USER,      // Can be omitted if not needed
    MQTT_PASSWORD,     // Can be omitted if not needed
    MQTT_CLIENTNAME // Client name that uniquely identify your device
);

// PIN for magnetic open sensors
gpio_num_t flapSensorPin = GPIO_NUM_25;
gpio_num_t doorSensorPin = GPIO_NUM_26;
gpio_num_t GPIO_FLAP_IO_TRIGGER = flapSensorPin;
gpio_num_t GPIO_DOOR_IO_TRIGGER = doorSensorPin;
int GPIO_FLAP_CLOSED_STATE = LOW;
int GPIO_FLAP_OPEN_STATE = !GPIO_FLAP_CLOSED_STATE;
int GPIO_DOOR_CLOSED_STATE = LOW;
int GPIO_DOOR_OPEN_STATE = !GPIO_FLAP_CLOSED_STATE;

// RTC data retained across sleep cycles
RTC_DATA_ATTR int rtc_currentState = ST_INIT;
RTC_DATA_ATTR int rtc_bootCount = 0;
RTC_DATA_ATTR int rtc_stillOpenCount = 0;
RTC_DATA_ATTR int rtc_lastFlapState = -99;
RTC_DATA_ATTR long rtc_timeAwakeMillis = 0;

int flapState = LOW;
int doorState = LOW;
long currentMillis;
int startTime = 0;
char totalTimeAwake[21];

/*
 Print the reason that the ESP32 has been awoken from sleep
*/
void print_wakeup_reason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_EXT0:
        Serial.println("Wakeup caused by external signal using RTC_IO");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        Serial.println("Wakeup caused by external signal using RTC_CNTL");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        Serial.println("Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        Serial.println("Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        Serial.println("Wakeup caused by ULP program");
        break;
    default:
        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        break;
    }
}

/*
 * Determine which GPIO triggered ext1 wakeup
 */
int get_wakeup_gpio(){
  int bitpattern = esp_sleep_get_ext1_wakeup_status();
  return log(bitpattern)/log(2);
}

/* formatting function to convert milliseconds to h:m:s */
void msToTimeStr(char *str, unsigned long ms)
{
    unsigned long runMillis = ms;
    unsigned long allSeconds = runMillis / 1000;
    int runHours = allSeconds / 3600;
    int secsRemaining = allSeconds % 3600;
    int runMinutes = secsRemaining / 60;
    int runSeconds = secsRemaining % 60;
    sprintf(str, "%02d:%02d:%02d", runHours, runMinutes, runSeconds);
}

/* Save state variables and go into deep sleep */
void esp32Sleep()
{
    rtc_timeAwakeMillis = rtc_timeAwakeMillis + (millis() - currentMillis);
    rtc_lastFlapState = digitalRead(flapSensorPin);
    DB msToTimeStr(totalTimeAwake, rtc_timeAwakeMillis);
    DB Serial.printf("Flap is %s, awake %s, going to sleep.\n", rtc_lastFlapState ? "open" : "closed", totalTimeAwake);
    adc_power_off();
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
}

void publishFlapState(int st)
{
    DB Serial.print("Waiting for net:");
    int loopCount = 0;
    while (loopCount < 10)
    {
        client.loop();
        DB Serial.print(".");
        delay(50);
        if (client.isConnected())
        {
            if (loopCount == 0)
            {
                DB Serial.print("p");
                if (st == GPIO_FLAP_OPEN_STATE)
                {
                    client.publish(FLAP_STATE_TOPIC, "open");
                }
                else
                {
                    client.publish(FLAP_STATE_TOPIC, "closed");
                }
            }
            loopCount++;
        }
    }
    DB Serial.println(".");
}

void publishDoorState(int st)
{
    DB Serial.print("Waiting for net:");
    int loopCount = 0;
    while (loopCount < 10)
    {
        client.loop();
        DB Serial.print(".");
        delay(50);
        if (client.isConnected())
        {
            if (loopCount == 0)
            {
                DB Serial.print("p");
                if (st == GPIO_DOOR_OPEN_STATE)
                {
                    client.publish(DOOR_STATE_TOPIC, "open");
                }
                else
                {
                    client.publish(DOOR_STATE_TOPIC, "closed");
                }
            }
            loopCount++;
        }
    }
    DB Serial.println(".");
}

void waitForDoorClose(){
  doorState = digitalRead(doorSensorPin);
  while (doorState == GPIO_DOOR_OPEN_STATE) {
     delay(50);
     doorState = digitalRead(doorSensorPin);
  }
}

void onConnectionEstablished()
{
    client.publish(AVAILABILITY_TOPIC, "online", true);
}

void print_status() {
    switch(rtc_currentState) {
        case ST_INIT:
        Serial.print("State ST_INIT, ");
        break;
        case ST_FIRSTOPEN:
        Serial.print("State ST_FIRSTOPEN, ");
        break;
        case ST_STILLOPEN:
        Serial.print("State ST_STILLOPEN, ");
        break;
        case ST_CLOSED:
        Serial.print("State ST_CLOSED, ");
        break;
        case ST_STUCK:
        Serial.print("State ST_STUCK, ");
        break;    
    }
    Serial.print(" flapState: " + rtc_lastFlapState ? "closed" : "open");
    Serial.print(", bootCount: " + String(rtc_bootCount));
    Serial.println(", stillOpenCount " + String(rtc_stillOpenCount));
}

void setup()
{
    currentMillis = millis();
    Serial.begin(115200);
    btStop();
    client.enableLastWillMessage(AVAILABILITY_TOPIC, "offline", true);
    client.setKeepAlive(300);
    // client.enableDebuggingMessages();
    // mns
    pinMode(GPIO_FLAP_IO_TRIGGER, INPUT);
    pinMode(GPIO_DOOR_IO_TRIGGER, INPUT);
    gpio_pullup_dis(GPIO_FLAP_IO_TRIGGER);
    gpio_pulldown_dis(GPIO_FLAP_IO_TRIGGER);
    gpio_pullup_dis(GPIO_DOOR_IO_TRIGGER);
    gpio_pulldown_dis(GPIO_DOOR_IO_TRIGGER);
    flapState = digitalRead(flapSensorPin);
    doorState = digitalRead(doorSensorPin);
    ++rtc_bootCount;

    DB print_wakeup_reason();
    DB print_status();

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
      int gpiopin = get_wakeup_gpio();
      if (gpiopin == doorSensorPin) {
        publishDoorState(doorState);
        switch (rtc_currentState) {
        case ST_STILLOPEN:
          esp_sleep_enable_timer_wakeup(STILL_OPEN_TIMER_SLEEP_MICROSECS);
          esp_sleep_enable_ext1_wakeup(GPIO_DOOR_TRIGGER_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
          break;
        case ST_STUCK:
          esp_sleep_enable_timer_wakeup(STUCK_TIMER_SLEEP_MICROSECS);
          esp_sleep_enable_ext1_wakeup(GPIO_DOOR_TRIGGER_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
          break;
        case ST_CLOSED:
          esp_sleep_enable_ext1_wakeup(GPIO_FLAP_AND_DOOR_TRIGGER_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
          break;
        default:
           esp_sleep_enable_ext1_wakeup(GPIO_DOOR_TRIGGER_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
          break;               
        }
        waitForDoorClose();
        esp32Sleep();
      } 
    } else {
        if (doorState == GPIO_DOOR_OPEN_STATE) {
          publishDoorState(doorState);
          waitForDoorClose();
        }
    }

    switch (rtc_currentState)
    {
    case ST_INIT:
        if (flapState == GPIO_FLAP_OPEN_STATE)
        {
            DB Serial.println("Transitiion ST_INIT to ST_FIRSTOPEN");
            firstopen();
            DB print_status();
            if (flapState == GPIO_FLAP_OPEN_STATE)
            {
                DB Serial.println("Transitiion ST_FIRSTOPEN to ST_STILLOPEN");
                stillopen();
            }
            else
            {
                DB Serial.println("Transitiion ST_FIRSTOPEN to ST_CLOSED");
                closed();
            }
        }
        else
        {
            DB Serial.println("Transitiion ST_INIT to ST_CLOSED");
            closed();
        }
        break;
    case ST_STILLOPEN:
        if (flapState == GPIO_FLAP_CLOSED_STATE)
        {
            DB Serial.println("Transitiion ST_STILLOPEN to ST_CLOSED");
            closed();
        }
        else
        {
            if (rtc_stillOpenCount <= MAX_STILL_OPEN_COUNT)
            {
                DB Serial.println("Transitiion ST_STILLOPEN to ST_STILLOPEN");
                stillopen();
            }
            else
            {
                DB Serial.println("Transitiion ST_STILLOPEN to ST_STUCK");
                stuck();
            }
        }
        break;
    case ST_STUCK:
        if (flapState == GPIO_FLAP_CLOSED_STATE)
        {
            DB Serial.println("Transitiion ST_STUCK to ST_CLOSED");
            closed();
        }
        else
        {
            DB Serial.println("Transitiion ST_STUCK to ST_STUCK");
            stuck();
        }
        break;
    case ST_CLOSED:
        DB Serial.println("Transitiion ST_CLOSED to ST_FIRSTOPEN");
        firstopen();
        DB print_status();
        if (flapState == GPIO_FLAP_OPEN_STATE)
        {
            DB Serial.println("Transitiion ST_FIRSTOPEN to ST_STILLOPEN");
            stillopen();
        }
        else
        {
            DB Serial.println("Transitiion ST_FIRSTOPEN to ST_CLOSED");
            closed();
        }
        break;
    default:
        break;
    }
    esp32Sleep();
}

void loop()
{
    //  This is not going to be called
}

void firstopen()
{
    rtc_currentState = ST_FIRSTOPEN;
    publishFlapState(GPIO_FLAP_OPEN_STATE);
    long n = 0;
    while (flapState == GPIO_FLAP_OPEN_STATE)
    {
        n++;
        flapState = digitalRead(flapSensorPin);
        long elapsedTime = (millis() - startTime);
        delay(50);

        if (elapsedTime > MAX_OPENFLAP_TIME)
        {
            DB Serial.printf("Flap stuck open for %ld ms > %d ms .. ending loop\n", elapsedTime, MAX_OPENFLAP_TIME);
            break;
        }
    }
}

void closed()
{
    rtc_currentState = ST_CLOSED;
    publishFlapState(GPIO_FLAP_CLOSED_STATE);
    rtc_stillOpenCount = 0;

    esp_sleep_enable_ext1_wakeup(GPIO_FLAP_AND_DOOR_TRIGGER_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
    
}

void stillopen()
{
    rtc_currentState = ST_STILLOPEN;
    rtc_stillOpenCount++;
    DB Serial.printf("Putting ESP32 to sleep for %d secs.\n", (STILL_OPEN_TIMER_SLEEP_MICROSECS / 1000000));
    esp_sleep_enable_timer_wakeup(STILL_OPEN_TIMER_SLEEP_MICROSECS);
    esp_sleep_enable_ext1_wakeup(GPIO_DOOR_TRIGGER_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
}

void stuck()
{
    rtc_currentState = ST_STUCK;
    DB Serial.printf("Putting ESP32 to sleep for %d secs.\n", (STUCK_TIMER_SLEEP_MICROSECS / 1000000));
    esp_sleep_enable_timer_wakeup(STUCK_TIMER_SLEEP_MICROSECS);
    esp_sleep_enable_ext1_wakeup(GPIO_DOOR_TRIGGER_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
}
