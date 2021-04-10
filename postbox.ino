/*
Postbox
=======
Designed to work with HomeAssistant. Send a MQTT message when the
postbox top flap is opened. Go into deep sleep when it is closed.


Hardware Connections
======================
Magnetic reed switch connected to GPIO 25 

Author:
Martin Saunders <mnsaunders@gmail.com>
*/
#include "EspMQTTClient.h"

#define TOPIC_HOME "homeassistant/binary_sensor/postbox/"
#define AVAILABILITY_TOPIC TOPIC_HOME "availability"
#define STATE_TOPIC TOPIC_HOME "state"

#define ST_INIT 0
#define ST_FIRSTOPEN 1
#define ST_STILLOPEN 2
#define ST_STUCK 3
#define ST_CLOSED 4

#define MAX_OPENFLAP_TIME 30000
#define MAX_STUCK_BOOT_COUNT 5
#define STILL_OPEN_TIMER_SLEEP_MICROSECS 120 * 1000000
#define STUCK_TIMER_SLEEP_MICROSECS 1800 * 1000000

// const char availability_topic[] = AVAILABILITY_TOPIC;
// const char STATE_TOPIC[] = TOPIC_HOME  "state"
EspMQTTClient client(
    "Penguins",
    "9EF382ABCD",
    "192.168.0.7",  // MQTT Broker server ip
    "mq_user",      // Can be omitted if not needed
    "SPQR69tt",     // Can be omitted if not needed
    "ESP32_postbox" // Client name that uniquely identify your device
);

gpio_num_t flapSensorPin = GPIO_NUM_25;
gpio_num_t GPIO_INPUT_IO_TRIGGER = flapSensorPin;
int GPIO_FLAP_CLOSED_STATE = LOW;
int GPIO_FLAP_OPEN_STATE = !GPIO_FLAP_CLOSED_STATE;

// RTC data retained across sleep cycles
RTC_DATA_ATTR int currentState = ST_INIT;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int stuckBootCount = 0;
RTC_DATA_ATTR int lastFlapState = -99;
RTC_DATA_ATTR long timeAwakeMillis = 0;

int flapState = LOW;
long currentMillis;
int startTime = 0;
char totalTimeAwake[21];

/*
Method to print the reason by which ESP32
has been awaken from sleep
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
void runtime(unsigned long ms)
{
    unsigned long runMillis = ms;
    unsigned long allSeconds = runMillis / 1000;
    int runHours = allSeconds / 3600;
    int secsRemaining = allSeconds % 3600;
    int runMinutes = secsRemaining / 60;
    int runSeconds = secsRemaining % 60;
    sprintf(totalTimeAwake, "%02d:%02d:%02d", runHours, runMinutes, runSeconds);
}

/* Save state variables and go into deep sleep */
void esp32Sleep()
{
    timeAwakeMillis = timeAwakeMillis + (millis() - currentMillis);
    lastFlapState = digitalRead(flapSensorPin);
    Serial.printf("\n FlapState %d Last FlapState %d Awake ms: %ld", flapState, lastFlapState, timeAwakeMillis);
    Serial.printf("\ngoing to sleep.");
    esp_deep_sleep_start();
    Serial.printf("this will never be printed");
}

void publish(int st)
{

    Serial.print("Waiting for net connect:");
    while (true)
    {
        client.loop();
        Serial.print(".");
        delay(500);
        if (client.isConnected())
        {
            Serial.println(".");
            if (st == GPIO_FLAP_OPEN_STATE)
            {
                client.publish(STATE_TOPIC, "open");
            }
            else
            {
                client.publish(STATE_TOPIC, "closed");
            }
            delay(500);
            break;
        }
    }
}

void onConnectionEstablished()
{
    Serial.println("onConnectionEstablished.");
    // client.subscribe("hello/test", onMessageReceived);
    client.publish(AVAILABILITY_TOPIC, "online", true);
}

void onMessageReceived(const String &message)
{
    Serial.print("message received from hello/test: " + message);
    if (message.startsWith("o"))
    {
        client.publish(STATE_TOPIC, "open");
    }
    else if (message.startsWith("c"))
    {
        client.publish(STATE_TOPIC, "closed");
    }
    else if (message.startsWith("x"))
    {
        client.publish(AVAILABILITY_TOPIC, "offline");
    }
    else
    {
        client.publish(STATE_TOPIC, "rat");
    }
}

void setup()
{
    currentMillis = millis();
    Serial.begin(115200);
    client.enableDebuggingMessages();
    pinMode(GPIO_INPUT_IO_TRIGGER, INPUT_PULLUP);
    gpio_pullup_en(GPIO_INPUT_IO_TRIGGER);
    gpio_pulldown_dis(GPIO_INPUT_IO_TRIGGER);
    flapState = digitalRead(flapSensorPin);

    ++bootCount;
    Serial.println("State: " + String(currentState));
    Serial.println("Boot counter: " + String(bootCount));
    Serial.println("Boot stuck counter: " + String(stuckBootCount));
    Serial.println("Last flap state: " + String(lastFlapState));
    // Serial.println("Current flap state: " + String(flapState));
    print_wakeup_reason();

    Serial.print("flapState: " + String(flapState));
    Serial.print("currentState: " + String(currentState));
    Serial.print("GPIO_OPEN: " + String(GPIO_FLAP_OPEN_STATE));
    Serial.println("GPIO_CLOSED: " + String(GPIO_FLAP_CLOSED_STATE));
    switch (currentState)
    {
    case ST_INIT:
        Serial.println("ST_INIT");
        if (flapState == GPIO_FLAP_OPEN_STATE)
        {
            Serial.println("to firstopen");
            firstopen();
            Serial.println("ST_FIRSTOPEN");
            Serial.println("State: " + String(currentState));
            if (flapState == GPIO_FLAP_OPEN_STATE)
            {
                Serial.println("to stillopen");
                stillopen();
            }
            else
            {
                Serial.println("to closed");
                closed();
            }
        }
        else
        {
            Serial.println("to closed");
            closed();
        }
        break;
    case ST_STILLOPEN:
        Serial.println("ST_STILLOPEN");
        if (flapState == GPIO_FLAP_CLOSED_STATE)
        {
            Serial.println("to closed");
            closed();
        }
        else
        {
            if (bootCount <= MAX_STUCK_BOOT_COUNT)
            {
                Serial.println("to stillopen");
                stillopen();
            }
            else
            {
                Serial.println("to stuck");
                stuck();
            }
        }
        break;
    case ST_STUCK:
        Serial.println("ST_STUCK");
        if (flapState == GPIO_FLAP_CLOSED_STATE)
        {
            Serial.println("to closed");
            closed();
        }
        else
        {
            Serial.println("to stuck");
            stuck();
        }
        break;
    case ST_CLOSED:
        Serial.println("ST_CLOSED");
        Serial.println("to firstopen");
        firstopen();
        Serial.println("ST_FIRSTOPEN");
        Serial.println("State: " + String(currentState));
        if (flapState == GPIO_FLAP_OPEN_STATE)
        {
            Serial.println("to stillopen");
            stillopen();
        }
        else
        {
            Serial.println("to closed");
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
    currentState = ST_FIRSTOPEN;
    publish(GPIO_FLAP_OPEN_STATE);
    long n = 0;
    while (flapState == GPIO_FLAP_OPEN_STATE)
    {
        n++;
        flapState = digitalRead(flapSensorPin);
        long elapsedTime = (millis() - startTime);
        if (n % 40 == 0)
            Serial.printf("flapState: %d,   %ld ms \n", flapState, elapsedTime);
        else
            Serial.print(flapState);
        delay(50);

        if (elapsedTime > MAX_OPENFLAP_TIME)
        {
            Serial.printf("Flap stuck open for %ld ms > %d ms .. ending loop", elapsedTime, MAX_OPENFLAP_TIME);
            break;
        }
    }
    runtime(millis() - currentMillis + timeAwakeMillis);
    Serial.println("ESP32 total awaketime :" + (String)totalTimeAwake);
}

void closed()
{
    currentState = ST_CLOSED;
    publish(GPIO_FLAP_CLOSED_STATE);
    stuckBootCount = 0;
    esp_sleep_enable_ext0_wakeup(GPIO_INPUT_IO_TRIGGER, GPIO_FLAP_OPEN_STATE);
}

void stillopen()
{
    currentState = ST_STILLOPEN;
    stuckBootCount++;
    Serial.printf("Putting ESP32 to sleep for %d secs.\n", (STILL_OPEN_TIMER_SLEEP_MICROSECS / 1000000));
    esp_sleep_enable_timer_wakeup(STILL_OPEN_TIMER_SLEEP_MICROSECS);
}

void stuck()
{
    currentState = ST_STUCK;
    Serial.printf("Putting ESP32 to sleep for %d secs.\n", (STUCK_TIMER_SLEEP_MICROSECS / 1000000));
    esp_sleep_enable_timer_wakeup(STUCK_TIMER_SLEEP_MICROSECS);
}
