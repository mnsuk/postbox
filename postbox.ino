// #define DEBUG
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
Magnetic reed switch connected to GPIO 25. Requires external 100K 
pullup resistor.

Author:
Martin Saunders <mnsaunders@gmail.com>
*/

#include "EspMQTTClient.h"
#include "driver/adc.h"

#define TOPIC_HOME "homeassistant/binary_sensor/postbox/"
#define AVAILABILITY_TOPIC TOPIC_HOME "availability"
#define STATE_TOPIC TOPIC_HOME "state"
#define REPORT_TOPIC TOPIC_HOME "report"

#define ST_INIT 0
#define ST_FIRSTOPEN 1
#define ST_STILLOPEN 2
#define ST_STUCK 3
#define ST_CLOSED 4

#define MAX_OPENFLAP_TIME 15000 // 15 seconds
#define MAX_STILL_OPEN_COUNT 3
#define STILL_OPEN_TIMER_SLEEP_MICROSECS 120 * 1000000 // 2 minutes
#define STUCK_TIMER_SLEEP_MICROSECS 1800 * 1000000     // 30 minutes

EspMQTTClient client(
    "Penguins",
    "9EF382ABCD",
    "192.168.0.7",  // MQTT Broker server ip
    "mq_user",      // Can be omitted if not needed
    "SPQR69tt",     // Can be omitted if not needed
    "ESP32_postbox" // Client name that uniquely identify your device
);

// PIN for magnetic door sensor
gpio_num_t flapSensorPin = GPIO_NUM_25;
gpio_num_t GPIO_INPUT_IO_TRIGGER = flapSensorPin;
int GPIO_FLAP_CLOSED_STATE = LOW;
int GPIO_FLAP_OPEN_STATE = !GPIO_FLAP_CLOSED_STATE;

// RTC data retained across sleep cycles
RTC_DATA_ATTR int rtc_currentState = ST_INIT;
RTC_DATA_ATTR int rtc_bootCount = 0;
RTC_DATA_ATTR int rtc_stillOpenCount = 0;
RTC_DATA_ATTR int rtc_lastFlapState = -99;
RTC_DATA_ATTR long rtc_timeAwakeMillis = 0;

int flapState = LOW;
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
                    client.publish(STATE_TOPIC, "open");
                }
                else
                {
                    client.publish(STATE_TOPIC, "closed");
                }
            }
            loopCount++;
        }
    }
    DB Serial.println(".");
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
    client.setKeepAlive(60);
    // client.enableDebuggingMessages();
    pinMode(GPIO_INPUT_IO_TRIGGER, INPUT);
    gpio_pullup_dis(GPIO_INPUT_IO_TRIGGER);
    gpio_pulldown_dis(GPIO_INPUT_IO_TRIGGER);
    flapState = digitalRead(flapSensorPin);
    ++rtc_bootCount;

    DB print_wakeup_reason();
    DB print_status();

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
    esp_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_TRIGGER, GPIO_FLAP_OPEN_STATE);
}

void stillopen()
{
    rtc_currentState = ST_STILLOPEN;
    rtc_stillOpenCount++;
    DB Serial.printf("Putting ESP32 to sleep for %d secs.\n", (STILL_OPEN_TIMER_SLEEP_MICROSECS / 1000000));
    esp_sleep_enable_timer_wakeup(STILL_OPEN_TIMER_SLEEP_MICROSECS);
}

void stuck()
{
    rtc_currentState = ST_STUCK;
    DB Serial.printf("Putting ESP32 to sleep for %d secs.\n", (STUCK_TIMER_SLEEP_MICROSECS / 1000000));
    esp_sleep_enable_timer_wakeup(STUCK_TIMER_SLEEP_MICROSECS);
}
