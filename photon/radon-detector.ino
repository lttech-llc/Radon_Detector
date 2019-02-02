#define USR_FW_RELEASE "1.0.0"

// The included MQTT library only allows a max packet size of 255.  According to the library, the packet
// size is the total of [MQTT Header(Max:5byte) + Topic Name Length + Topic Name + Message ID(QoS1|2) + Payload]
// The max packet size can be overwritten with the MQTT_MAX_PACKET_SIZE define.

#include <MQTT.h>
#include <SparkJson.h>
#include <OneWire.h>
#include <DS18.h>

#include "math.h"

// Enable system threading so application loop is not blocked by background tasks, networking, or 
// Particle cloud tasks.  Without threading, if the WiFi connection or internet connection went down,
// then the application loop would be blocked for up to 30-40 seconds at a time.  With system threading
// enabled the application setup() and loop() functions are called immediately without waiting for a
// WiFi or Cloud connection.  However we will wait for a WiFi connection in setup().
SYSTEM_THREAD(ENABLED);

// There seems to be a bug when running in threaded mode.  It is either in the Photon core code or MQTT
// library.  If the WiFi connection is weak or if the Internet connection goes down, then the Photon will
// sometimes hang in the application loop.  When the hang occurs it always seems to happen in the MQTT
// library whenever _client->stop() is called.  A workaround for this is to setup the built-in Application
// Watchdog and have it reset the Photon whenever the watchdog detects an application loop hang.
// This work still needs to be done.

// Defines for enabling Serial and Particle Cloud debug messages
#define DEBUG_SERIAL
// #define DEBUG_CLOUD

// Defines for extra levels of debug messages
// #define DEBUG_TEMP
// #define DEBUG_RADON

// Defines for pins
#define SEG_A           A1   // Segment A
#define SEG_B           D6   // Segment B
#define SEG_C           WKP  // Segment C
#define SEG_D           A4   // Segment D
#define SEG_E           A5   // Segment E
#define SEG_F           A2   // Segment F
#define SEG_G           A0   // Segment G
#define DP              RX   // Decimal Point
#define LT_ST           D1   // Long-Term / Short-Term
#define DIG_1           D4   // Digit 1
#define DIG_2           D2   // Digit 2
#define DIG_3           D3   // Digit 3
#define DIG_4           D0   // Digit 4
#define PB_IN           D5   // Radon Detector Menu Pushbutton input
#define PB_OUT          D7   // Radon Detector Menu Pushbutton output
#define ONE_WIRE        DAC  // Temperature Sensor One-Wire bus
#define SELECT_PB       A3   // Select Pushbutton

// Interval at which to read the sensors in milliseconds
#define READ_INTERVAL_RADON   900000  // 15 minutes 
#define READ_INTERVAL_SENSORS  60000  // 60 seconds
#define MQTT_RETRY_INTERVAL    30000  // 30 seconds

// Size of circular buffer for WiFi signal strength
#define SIG_STRENGTH_BUF_SIZE 100

#define SIG_STRENGTH_LIMIT -80

enum State {SETUP, INIT, NO_WIFI, WEAK_SIG, NO_MQTT, NO_CLOUD, GOOD, READ_SENSOR, READ_RADON, READ_FAIL, EXIT_APP, JSON_ERR};

// Define a byte array for the broker IP address
uint8_t brokerIP[] = {192, 168, 20, 4};

// The topic to publish sensor data on.  The + is to be replaced by the device ID.
String mqttSensorOutputTopic = "sensors/+/data";

// The topic to subscribe to for control.  The # is to be replaced by the device ID.
String mqttControlInputTopic = "sensors/+/control";

// The topic to publish debug data on.  The + is to be replaced by the device ID.
String mqttDebugOutputTopic = "sensors/+/debug";

// Define some globals
String deviceId;
String hostName;
String mqttClientId;

// Pointer to MQTT client.  Actual object is created with new once broker info is known.
MQTT* mqttClient;

// Create a OneWire instance to communicate with temp sensor
DS18 sensor(ONE_WIRE);

// Global arrays for capture of segment data
int segment[4][7];
int dp[4];
int lt_st[4];

// Function prototypes
void check_wifi(void);
void setup_mqtt_broker(void);
void check_network(State* currentState);
void mqtt_reconnect(int sigStrength);
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void get_temperature(State currentState);
void get_radon_levels(State currentState);
bool sample_digit(int digit);
bool convert_value(float& value);
bool convert_segment(int digit, int& value);
void set_led_color(State color);
void debug_message(String msg, bool cloud=false);

// Setup routine.  Run once at bootup.
void setup() {
    
#ifdef DEBUG_SERIAL

    unsigned long startTime;
    
    Serial.begin(9600);
    
    // Wait for the host to open the serial port.  Timemout and continue if the
    // host never connects to prevent the application from hanging.  If the USB
    // is connected to a PC this should automatically happen regardless if a terminal
    // emulator window is open.  I think it just means the USB port is connected.
    // The USB port always seems to disconnect and reconnect on the Photon after
    // code is flashed or a reset occurs.
    startTime = millis();
    
    while ((millis() - startTime < 5000) && (! Serial.isConnected()))
        // Need to call this to process Cloud data, since we are stuck in a long loop.
        // The cloud process() routine is called after every loop() and during
        // delays.  It must be called if the loop() is blocked by code execution.
        // This is only true if system threading is not enabled and not really needed
        // if system threading is enabled.
        Particle.process();
    
    // The host has connected.  Wait for user to open terminal emulator window,
    // select the proper port, and send a CTRL-Z character to know that the user
    // terminal emulator window is ready to receive messages.  If user never connects
    // and sends a proper character, timeout after 30 seconds so bootup will continue.
    if (Serial.isConnected()) {
        
        startTime = millis();
        
        // The host USB port seems to send a sequence of characters every so often.
        // I could not use any character because this sequence would cause the wait
        // loop to drop out.  Therefore we will look specifically for a CTRL-Z which
        // is ASCII character 26 decimal.  If no character is in the buffer then
        // the Serial.read() returns -1.
        while ((millis() - startTime < 30000) && (Serial.read() != 26))
            Particle.process();
        
    }
    
#endif

    debug_message("");
    debug_message("Running setup()", true);
    debug_message("");
    
    // Take over control of the RGB LED.  Prior to this the LED should be 
    // breathing cyan.
    RGB.control(true);
    
    // Set the LED color to indicate we are in setup().  It will stay this
    // color until we connect to WiFi and mDNS finds the MQTT broker.
    // Delay to make sure we can see the LED color before exiting setup.
    set_led_color(SETUP);

    // Initialize the mode of all the I/O pins
    pinMode(SEG_A, INPUT);
    pinMode(SEG_B, INPUT);
    pinMode(SEG_C, INPUT);
    pinMode(SEG_D, INPUT);
    pinMode(SEG_E, INPUT);
    pinMode(SEG_F, INPUT);
    pinMode(SEG_G, INPUT);
    pinMode(DP, INPUT);
    pinMode(LT_ST, INPUT);
    pinMode(DIG_1, INPUT);
    pinMode(DIG_2, INPUT);
    pinMode(DIG_3, INPUT);
    pinMode(DIG_4, INPUT);
    pinMode(PB_IN, INPUT);
    pinMode(PB_OUT, OUTPUT);
    pinMode(SELECT_PB, INPUT_PULLUP);

    // Read the FW revision
    debug_message("Firmware Rev: " + System.version(), true);

    // Read the Device ID
    deviceId = System.deviceID();
    debug_message("Device ID: " + deviceId);

    // User FW release version
    debug_message("User Firmware Rev: " + String(USR_FW_RELEASE), true);

    // Modify the MQTT topics now that we know the device ID.
    mqttSensorOutputTopic.replace("+", deviceId);
    mqttControlInputTopic.replace("+", deviceId);
    mqttDebugOutputTopic.replace("+", deviceId);

    // Create a host name for the device
    hostName = "photon_" + deviceId;
    debug_message("Hostname: " + hostName);
  
    // Create an unique MQTT client ID.  We are creating the unique ID from the
    // the deivice ID which is quite long.  The MQTT 3.1 spec says this must be between
    // 1 and 23 characters.  This requirement may have been removed in the 3.1.1 spec.
    // However I think that mosquitto will support longer client ID lengths anyway, so
    // we should be OK.
    mqttClientId = "photon_" + deviceId;
    debug_message("MQTT Client ID: " + mqttClientId);
    debug_message("");

    // Delay 15 seconds at startup to allow the radon detector time to initialize before starting reads
    debug_message("Delaying 15 seconds to give the radon detector time to initialize...");
    delay(15000);
    
    // Check the WiFi connection
    check_wifi();

    // Setup the connection to the MQTT broker
    setup_mqtt_broker();
    
    debug_message("Finished setup()", true);
    debug_message("");
    
    // Turn off the LED when exiting setup
    set_led_color(EXIT_APP);

}

// Main loop.  Repeatedly called by system firmware.
void loop() {

    static State currentState = INIT;

    // Restore the LED color from the previous application loop exit
    set_led_color(currentState);
    
    // Check the state of the network connection.  The currentState can be
    // modified upon return
    check_network(&currentState);

    // Update the LED color
    set_led_color(currentState);

    // Have the MQTT client process any data.
    mqttClient->loop();

    // Restore the LED color
    set_led_color(currentState);
    
    // Get the temperature data
    get_temperature(currentState);    

    // Restore the LED color
    set_led_color(currentState);
    
    // Get the radon levels
    get_radon_levels(currentState);    

    // Restore the LED color
    set_led_color(currentState);
    
    // Turn LED off when exiting loop.  This will tell us if the system is blocking
    // the application loop.  With system threading enabled this should no longer
    // be an issue.
    set_led_color(EXIT_APP);
    
}

// Check the WiFi connection.
void check_wifi() {

    byte mac[6];
    
    debug_message("Checking the WiFi connection...");
    
    while (! WiFi.ready()) {
        
        // Pulse the LED indicate we have not successfully connected to WiFi yet    
        set_led_color(NO_WIFI);
        delay(500);
        set_led_color(SETUP);
        
        debug_message("WiFi not connected...checking again in 10 seconds");
        delay(10000);
        
    }

    debug_message("WiFi connected");
    debug_message(String::format("Access Point SSID:  %s", WiFi.SSID()));
    WiFi.BSSID(mac);
    debug_message(String::format("AP MAC address:     %02X:%02X:%02X:%02X:%02X:%02X",  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]));
    debug_message(String::format("Device IP address:  %s", WiFi.localIP().toString().c_str()));
    WiFi.macAddress(mac);
    debug_message(String::format("Device MAC address: %02X:%02X:%02X:%02X:%02X:%02X",  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]));
    debug_message("");
  
}

// Use mDNS to locate an MQTT broker and create the client instance.
void setup_mqtt_broker() {
    
    // The mDNS library available for the Photon works only for mDNS servers and does not support clients,
    // unlike the mDNS library for the ESP8266 which supports both servers and clients.  Therefore
    // we cannot do mDNS MQTT service discovery on the Photon currently.  With more ambition, I
    // might do a port of the mDNS client to the Photon.

    // mDNS not implemented yet so hard code the address and port

    // brokerIP is a pointer to an array of bytes and must be persistent once this
    // routine exits, meaning use new or make it a global array.
    mqttClient = new MQTT(brokerIP, 1883, mqtt_callback);

}

// Check the network connections
void check_network(State* currentState) {
    
    static int sigStrengthBuffer[SIG_STRENGTH_BUF_SIZE];
    static int bufPtr = 0;
    static unsigned long lastDebugMessage = 0;
    
    // Initialize the signal strength buffer to zero if in INIT state or the
    // WiFi connection is down.
    if (*currentState == INIT || *currentState == NO_WIFI)
        for (int i = 0; i < SIG_STRENGTH_BUF_SIZE; i++)
            sigStrengthBuffer[i] = 0;
    
    // Get the current WiFi signal strength and put it in the circular buffer
    sigStrengthBuffer[bufPtr] = WiFi.RSSI();
    bufPtr = (bufPtr + 1) % SIG_STRENGTH_BUF_SIZE;

    unsigned long currentTime = millis();

    // The WiFi connection is down
    if (! WiFi.ready()) {
        
        set_led_color(NO_WIFI);
        
        // If we just got to this state or it has been 60 secs since the last message,
        // then print the debug message
        if ((*currentState != NO_WIFI) || (currentTime - lastDebugMessage > 60000)) {
            debug_message("No WiFi connection");
            lastDebugMessage = currentTime;
        }
        
        // Update the current state
        *currentState = NO_WIFI;
        
    }
    
    // WiFi is connected but we are not connected to the MQTT broker
    else if (! mqttClient->isConnected()) {
        
        // If we got to this state from a connected state then print the debug message
        // that we lost the MQTT broker connection
        if (*currentState == GOOD || *currentState == NO_CLOUD)
            debug_message ("Lost connection to MQTT broker");
        
        int i;
        int avgSigStrength = 0;
        
        // Determine if the signal strength history is good and calculate an average signal strength
        for (i = 0; i < SIG_STRENGTH_BUF_SIZE; i++) {
            avgSigStrength += sigStrengthBuffer[i] * 10;
            if (sigStrengthBuffer[(bufPtr - 1 - i + SIG_STRENGTH_BUF_SIZE) % SIG_STRENGTH_BUF_SIZE] < SIG_STRENGTH_LIMIT ||
                sigStrengthBuffer[(bufPtr - 1 - i + SIG_STRENGTH_BUF_SIZE) % SIG_STRENGTH_BUF_SIZE] >= 0) {
                break;
            }
        }
        
        // If the above for loop did not get through the entire buffer then we had a bad signal strength value
        if (i < SIG_STRENGTH_BUF_SIZE) {
            
            // Do not attempt to connect to MQTT broker.  WiFi signal error or signal too weak.
            set_led_color(WEAK_SIG);
            
            int failure = sigStrengthBuffer[(bufPtr - 1 - i + SIG_STRENGTH_BUF_SIZE) % SIG_STRENGTH_BUF_SIZE];
            
            // If we just got to this state or it has been 10 secs since the last message,
            // then print the debug message
            if (*currentState != WEAK_SIG || (currentTime - lastDebugMessage > 10000)) {
                if (failure < 0)
                    debug_message(String::format("WiFi signal too weak to attemmpt MQTT broker connection: %d dBm", failure));
                else
                    debug_message(String::format("Cannot attempt MQTT broker connection because of WiFi signal strength error: %d", failure));
                
                lastDebugMessage = currentTime;
            }
            
            // Update the current state
            *currentState = WEAK_SIG;
            
        }
        
        // No errors and signal strength values above the threshold
        else {
            
            // Not connected to MQTT broker, set the LED and try to reconnect.
            set_led_color(NO_MQTT);
            
            // Calculate the average signal strength
            avgSigStrength /= SIG_STRENGTH_BUF_SIZE;
            avgSigStrength += 5;
            avgSigStrength /= 10;
            
            // Update the current state
            *currentState = NO_MQTT;
            
            // Attempt to reconnect
            mqtt_reconnect(avgSigStrength);
            
        }
    }
    
    // WiFi connected and MQTT broker connected but no Particle Cloud connection
    else if (! Particle.connected()) {
        
        set_led_color(NO_CLOUD);
        
        // If we just got to this state or it has been 10 secs since the last message,
        // then print the debug message
        if (*currentState != NO_CLOUD || (currentTime - lastDebugMessage > 10000)) {
            debug_message("No Particle Cloud connection");
            lastDebugMessage = currentTime;
        }
        
        // Update the current state
        *currentState = NO_CLOUD;
        
    }
    
    // All the network connections are up
    else {
        
        set_led_color(GOOD);
        
        if (*currentState != GOOD)
            debug_message("All Network connections restored");
            
        *currentState = GOOD;
        
    }

}

// Connect to the MQTT broker
void mqtt_reconnect(int sigStrength) {

    static unsigned long lastAttempt = millis() - MQTT_RETRY_INTERVAL;
    unsigned long currentTime = millis();
    
    // If not connected and the retry timeout has expired then retry again.  Do not block
    // here as we want the sensor and radon reads to occur even though we cannot send data
    // while the broker connection is down.
    if (WiFi.ready() && (! mqttClient->isConnected()) && (currentTime - lastAttempt > MQTT_RETRY_INTERVAL)) {
        
        debug_message(String::format("Attempting MQTT broker connection, WiFi signal strength: %d dBm...", sigStrength));
        
        // Attempt to connect
        if (mqttClient->connect(mqttClientId)) {
            debug_message("Connected to MQTT broker", true);
            
            // Resubscribe to the control topic.
            mqttClient->subscribe(mqttControlInputTopic);
        }
        else {
            debug_message("MQTT broker connection failed, retry again in 30 seconds");
        }
        
        lastAttempt = currentTime;
    }

}

// The function to be called whenever a message comes in on subscribed topics.
void mqtt_callback(char* topic, byte* payload, unsigned int length) {

    // The payload may not be NULL terminated so keep that in mind
    // when working with it, always use the length.
    
    // Make a copy of the payload to a new JSON string.  It needs to be
    // writable because the JSON parser with add NULL terminators and
    // will replace escaped characters.
    char jsonString[length + 1];

    for (int i = 0; i < length; i++) {
        jsonString[i] = (char) payload[i];
    }
    
    jsonString[length] = NULL;
    
    debug_message(String::format("Message rcvd: [%s] %s", topic, jsonString));

    DynamicJsonBuffer jsonBuffer;
    
    JsonObject& root = jsonBuffer.parseObject(jsonString);

    // Check to make sure JSON string could be parsed
    if (! root.success()) {
        debug_message("Unable to parse JSON string");
        
        // Quickly flash the LED to indicate JSON parsing error
        set_led_color(JSON_ERR);
        delay(50);
        
        return;
    }

}

// Get the temperature sensor reading.
void get_temperature(State currentState) {
    
    // Variables for determining when sensors should be read again
    // Delay 5 seconds for first read to give time to connect to MQTT broker
    static unsigned long previousReadTime = millis() + min(5000, READ_INTERVAL_SENSORS) - READ_INTERVAL_SENSORS;
    unsigned long currentTime = millis();
    static unsigned int retries = 0;
    float temp;

    // Check to see if it is time to read the sensors
    if (currentTime - previousReadTime >= READ_INTERVAL_SENSORS) {
        
        // Set the LED color to indicate an active sensor read.  The reads
        // take over 200 msec so we should see the LED.
        set_led_color(READ_SENSOR);
        
        // Read the next sensor, however there should only be one sensor.
        if (sensor.read()) {
            
            // Read was successful, get the temperature.
            temp = sensor.fahrenheit();
            retries = 0;
            
        }
        else if (sensor.searchDone()) {
            
            // No sensors found or last sensor has been read
            if (retries == 0) {
                
                // Last sensor has been succesfully read, just exit and the first sensor will be read next time
                retries++;
                return;
                
            }
            else if (retries > 5) {
                
                // Multiple retries, so the sensor is likely missing.
                
                // Set the LED to indicate a read failure
                set_led_color(READ_FAIL);
                
                debug_message("Missing temperature sensor");
                
                // Wait a second so LED color can be observed.
                delay(1000);
                
                // Retry again at the normal interval.
                previousReadTime = currentTime;
                
            return;
                
            }
            else {
            
                // More than 1 retry, but not ready to declare missing sensor yet
                retries++;
                debug_message("Consecutive DS18B20 sensor search done retry");
                return;
                
            }
            
        }
        else {
            
            // Failed to read sensor
            
            // Set the LED to indicate a read failure
            set_led_color(READ_FAIL);
            
            debug_message("Failed to read from DS18B20 sensor");
            
            // Wait a second so LED color can be observed.
            delay(1000);
            
            // Retry again in 5 seconds
            previousReadTime = currentTime + min(5000, READ_INTERVAL_SENSORS) - READ_INTERVAL_SENSORS;
            
            return;
        }
        
        // Save the last read time
        previousReadTime = currentTime;
        
        // Encode the message in JSON.
        
        // First create the JSON buffer and the root object.  Use the dynamic buffer instead of static.
        // The static version uses the stack and the dynamic version uses the heap.  I was having issues
        // with a static size of 200 not being big enough.  Therefore use the dynamic so the size is not
        // an issue.
        
        // StaticJsonBuffer<200> jsonBuffer;
        DynamicJsonBuffer jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        
        // Put the data into the JSON object.
        root.add("temp").set(temp, 3);
        
        root.add("rssi").set(WiFi.RSSI(),0);
        
        // Turn the JSON structure into a string for sending over the MQTT topic.
        char jsonOutputBuffer[255];
        root.printTo(jsonOutputBuffer, sizeof(jsonOutputBuffer));
        
        if ((currentState == NO_CLOUD || currentState == GOOD) && WiFi.ready() && mqttClient->isConnected()) {
            
            // Publish the message on the MQTT topic, but only if we are connected to the broker.
            if (mqttClient->publish(mqttSensorOutputTopic, jsonOutputBuffer)) {
#ifdef DEBUG_TEMP
                debug_message(String::format("Message sent: [%s] %s", mqttSensorOutputTopic.c_str(), jsonOutputBuffer));
#endif
            }
            
        }
        
    }

}

// Get the current radon levels.
void get_radon_levels(State currentState) {
    
    // Variables for determining when the sensor should be read again
    // Delay 5 seconds for first read to give time to connect to MQTT broker
    static unsigned long previousReadTime = millis() + min(5000, READ_INTERVAL_RADON) - READ_INTERVAL_RADON;
    unsigned long currentTime = millis();

    // Check to see if it is time to read the sensor
    if (currentTime - previousReadTime >= READ_INTERVAL_RADON) {
        
        // Set the LED color to indicate an active sensor read.  Delay to
        // make sure we can see the LED
        set_led_color(READ_RADON);
        
        delay(100);
        
        float long_term_value = -1.0;
        float short_term_value = -1.0;
        
        // Do capture twice, once for long-term and once for short-term
        for (int j = 0; j < 2; j++) {
            
            // Check for user activity on the radon detector pushbutton.  Do not try to read while this is occurring.
            if (! pinReadFast(PB_IN)) {
                // Pushbutton is being pressed.  Retry again in 30 seconds after external user activity is finished.
                debug_message("User is pressing pushbutton, try again in 30 seconds...");
                previousReadTime = currentTime + min(30000, READ_INTERVAL_RADON) - READ_INTERVAL_RADON;
                return;
            }
            
            // Sample all of the digit positions
            for (int i = 4; i > 0; i--) {
                if (! sample_digit(i)) {
                    // Unable to sample digit
                    debug_message("Unable to sample digit, try again in 30 seconds...");
                    previousReadTime = currentTime + min(30000, READ_INTERVAL_RADON) - READ_INTERVAL_RADON;
                    return;
                }
            }
            
#ifdef DEBUG_RADON
            // Display the results
            debug_message(String::format("Digit A B C D E F G DP LT/ST"));
            for (int i = 1; i <= 4; i++) {
                debug_message(String::format("  %d:  %d %d %d %d %d %d %d  %d   %d", i, segment[i-1][0], segment[i-1][1], segment[i-1][2], segment[i-1][3], segment[i-1][4], segment[i-1][5], segment[i-1][6], dp[i-1], lt_st[i-1]));
            }
#endif
            
            float value;
            
            // Convert the segment display into an actual radon level
            if (! convert_value(value)) {
                // Unable to convert to a vlue
                debug_message("Unable to convert to a value, try again in 30 seconds...");
                previousReadTime = currentTime + min(30000, READ_INTERVAL_RADON) - READ_INTERVAL_RADON;
                return;
            }
            
            // Determine whether current display is long-term or short-term
            if (lt_st[0] && lt_st[1] && ! lt_st[2] & lt_st[3]) {
                // Short-term display
#ifdef DEBUG_RADON
                debug_message(String::format("Short-Term Radon Level:  %f pCi/L", value));
#endif
                short_term_value = value;
            }
            else if (lt_st[0] && lt_st[1] && lt_st[2] & ! lt_st[3]) {
                // Long-term display
#ifdef DEBUG_RADON
                debug_message(String::format("Long-Term Radon Level:  %f pCi/L", value));
#endif
                long_term_value = value;
            }
            else {
                debug_message("Unable to determine LT/ST display mode, try again in 30 seconds...");
                previousReadTime = currentTime + min(30000, READ_INTERVAL_RADON) - READ_INTERVAL_RADON;
                return;
            }
            
            // Toggle the LT/ST displAy mode
            digitalWrite(PB_OUT, HIGH);
            delay(250);
            digitalWrite(PB_OUT, LOW);
            delay(1000);
            
        }
        
        // Check if we have sampled both display modes
        if (long_term_value < 0.0 || short_term_value < 0.0) {
            debug_message("Unable to switch LT/ST display mode, try again in 30 seconds...");
            previousReadTime = currentTime + min(30000, READ_INTERVAL_RADON) - READ_INTERVAL_RADON;
            return;
        }
        
        // Save the last read time
        previousReadTime = currentTime;

        // Encode the message in JSON.
        
        // First create the JSON buffer and the root object.  Use the dynamic buffer instead of static.
        // The static version uses the stack and the dynamic version uses the heap.  I was having issues
        // with a static size of 200 not being big enough.  Therefore use the dynamic so the size is not
        // an issue.
        
        // StaticJsonBuffer<200> jsonBuffer;
        DynamicJsonBuffer jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        
        // Put the data into the JSON object.
        JsonObject& radon = root.createNestedObject("radon");
        radon.add("long").set(long_term_value, 1);
        radon.add("short").set(short_term_value, 1);

        // Turn the JSON structure into a string for sending over the MQTT topic.
        char jsonOutputBuffer[255];
        root.printTo(jsonOutputBuffer, sizeof(jsonOutputBuffer));
        
        if ((currentState == NO_CLOUD || currentState == GOOD) && WiFi.ready() && mqttClient->isConnected()) {
            
            // Publish the message on the MQTT topic, but only if we are connected to the broker.
            if (mqttClient->publish(mqttSensorOutputTopic, jsonOutputBuffer)) {
#ifdef DEBUG_RADON
                debug_message(String::format("Message sent: [%s] %s", mqttSensorOutputTopic.c_str(), jsonOutputBuffer));
#endif
            }
            
        }
        
    }

}

// Sample the display for a particular digit position.  Each of the digit signals strobe low for 4msec.  Digit 4 is the first in
// the sequence to strobe and digit 1 is the last.  A complete cycle takes 16msec and then repeats.  While a particular digit is
// strobed low, the segments and decimal point signals will also strobe low if they are on for that digit poistion.  The
// long-term/short-term display will strobe during digit 3 and digit 4.  For short-term display, the LT-ST pin will strobe low
// during digit 3.  For long-term display, the LT-ST pin will strobe low during digit 4.
bool sample_digit(int digit) {
    
    int digitPin;
    int digitIndex;
    unsigned long startTime;
    
    switch (digit) {
        case 1:
            digitPin = DIG_1;
            break;
        case 2:
            digitPin = DIG_2;
            break;
        case 3:
            digitPin = DIG_3;
            break;
        case 4:
            digitPin = DIG_4;
            break;
        default:
            // The digit must be between 1 and 4
            return false;
            break;
    }
    
    digitIndex = digit - 1;
    
    // Wait for the digit pin to be HIGH so we can detect the falling edge
    startTime = micros();
    
    while (! pinReadFast(digitPin)) {
        if (micros() - startTime > 5000) {
            // Timeout detected, it should only take a maximum of 4 msec for digit pin to go HIGH again
            return false;
        }
    }
    
    // Wait for the digit pin to be LOW which will be the falling edge of the digit pin
    startTime = micros();
    
    while (pinReadFast(digitPin)) {
        if (micros() - startTime > 13000) {
            // Timeout detected, it should only take a maximum of 12 msec for digit pin to go LOW again
            return false;
        }
    }
    
    // Each digit pulse is 4 msec.  Delay 1 msec from falling edge then sample the segments
    delayMicroseconds(1000);
    
    segment[digitIndex][0] = pinReadFast(SEG_A);
    segment[digitIndex][1] = pinReadFast(SEG_B);
    segment[digitIndex][2] = pinReadFast(SEG_C);
    segment[digitIndex][3] = pinReadFast(SEG_D);
    segment[digitIndex][4] = pinReadFast(SEG_E);
    segment[digitIndex][5] = pinReadFast(SEG_F);
    segment[digitIndex][6] = pinReadFast(SEG_G);
    dp[digitIndex] = pinReadFast(DP);
    lt_st[digitIndex] = pinReadFast(LT_ST);
    
    return true;

}

// Convert the entire segment display into a float value.  Returns false if unable to decode the display.
bool convert_value(float& value) {

    float multiplier = 0.0;
    
    // Find the decimal point position and multiplier
    for (int i = 0; i < 4; i++) {
        if (! dp[i]) {
            // A LOW value indicates the decimal point is ON.
            if (multiplier == 0.0) {
                // Multiplier not set, so this is the first decimal point position found
                multiplier = pow(10.0, -i);
            }
            else {
                // Already found a decimal point set in one of the other digit positions, which is an error
                return false;
            }
            
        }
    }
    
    if (multiplier == 0.0) {
        // Multiplier not set, so no decimal point found.  Assume a multiplier of 1.0
        multiplier = 1.0;
    }

    // Loop through all four captured digits and decode into its value as a float
    int decimal;
    
    // Initialize value to zero before looping
    value = 0.0;
    
    for (int i = 0; i < 4; i++) {
        if (convert_segment(i + 1, decimal)) {
            value += decimal * pow(10, i);
        }
        else {
            // Unable to convert segment display
            return false;
        }
    }

    // Shift the decimal point to get the correct float value
    value *= multiplier;

    return true;
    
}

// Convert the segment values of a particular digit position into an integer value from 0-9.
// Returns false if passed a bad digit value or is unable to decode the segment value.
bool convert_segment(int digit, int& value) {

    int digitIndex;
    int segmentBits;
    
    // Digit should be from 1 to 4
    if (digit < 1 || digit > 4) {
        return false;
    }

    digitIndex = digit - 1;

    // Convert segment array values into a bit packed integer to make decoding easier.  A LOW value
    // indicates that the segment is ON.  Invert the value before bit packing to make decoding easier
    // to understand.
    segmentBits = 0;
    for (int i = 0; i < 7; i++) {
        segmentBits |= (! segment[digitIndex][i]) << i;
    }

    // Decode the segment display into a single decimal digit value 
    switch (segmentBits) {
        case 0x00:  // Assume that a blank display is a value of 0
            value = 0;
            break;
        case 0x3F:
            value = 0;
            break;
        case 0x06:
            value = 1;
            break;
        case 0x5B:
            value = 2;
            break;
        case 0x4F:
            value = 3;
            break;
        case 0x66:
            value = 4;
            break;
        case 0x6D:
            value = 5;
            break;
        case 0x7C:
        case 0x7D:
            value = 6;
            break;
        case 0x07:
            value = 7;
            break;
        case 0x7F:
            value = 8;
            break;
        case 0x67:
        case 0x6F:
            value = 9;
            break;
        default:
            return false;
            break;
    }

    return true;
}

// Set the LED color
void set_led_color(State color) {

    switch (color) {
        case SETUP:
            RGB.color(0, 0, 255);       // Blue
            break;
        case INIT:
            RGB.color(0, 0, 0);         // Off
            break;
        case NO_WIFI:
            RGB.color(255, 0, 0);       // Red
            break;
        case WEAK_SIG:
            RGB.color(255, 96, 0);      // Orange
            break;
        case NO_MQTT:
            RGB.color(255, 255, 0);     // Yellow
            break;
        case NO_CLOUD:
            RGB.color(255, 255, 255);   // White
            break;
        case GOOD:
            RGB.color(0, 255, 0);       // Green
            break;
        case READ_SENSOR:
            RGB.color(0, 128, 255);     // Medium Blue
            break;
        case READ_RADON:
            RGB.color(255, 0, 255);     // Magenta
            break;
        case READ_FAIL:
            RGB.color(255, 128, 0);     // Dark Orange
            break;
        case EXIT_APP:
            RGB.color(0, 0, 0);         // Off
            break;
        case JSON_ERR:
            RGB.color(255, 128, 0);     // Dark Orange
            break;
    }

}

// Send debug messages to serial port and/or Particle cloud
inline void debug_message(String msg, bool cloud) {

#ifdef DEBUG_SERIAL

    Serial.println(msg);

#endif

#ifdef DEBUG_CLOUD

    if (cloud) {
        Particle.publish("app/debug", msg);
        Particle.process();
    }
    
#endif

}
