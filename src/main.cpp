#define SAMPLE_TIME        0.1   // Control loop in s
#define RATIO              54    // Gear ratio of rotator gear box for V3.1 Satnogs
#define MICROSTEP          8     // Set Microstep
#define MIN_PULSE_WIDTH    20    // In microsecond for AccelStepper
#define MAX_SPEED          3200  // In steps/s, consider the microstep, default 3200
#define MAX_ACCELERATION   1600  // In steps/s^2, consider the microstep, default 1600
#define SPR                1600L // Step Per Revolution, consider the microstep
#define MIN_M1_ANGLE       0     // Minimum angle of azimuth
#define MAX_M1_ANGLE       360   // Maximum angle of azimuth
#define MIN_M2_ANGLE       0     // Minimum angle of elevation
#define MAX_M2_ANGLE       360   // Maximum angle of elevation
#define DEFAULT_HOME_STATE LOW  // Change to LOW according to Home sensor
#define HOME_DELAY         150 // Time for homing Deceleration in millisecond
#define SerialPort         Serial

// Enter your wifi SSID
const char *ssid = "CayganFiber20MBPS";  
// Enter your wifi Password
const char *password = "caygan22";       

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <AccelStepper.h>
#include "globals.h"
#include "easycomm.h"
#include "rotator_pins.h"
#include "endstop.h"

easycomm comm;
AccelStepper stepper_az(1, M1IN1, M1IN2);
AccelStepper stepper_el(1, M2IN1, M2IN2);
endstop switch_az(SW1, DEFAULT_HOME_STATE), switch_el(SW2, DEFAULT_HOME_STATE);

// Convert degrees to steps according to step/revolution, rotator gear box ratio and microstep
int32_t deg2step(float deg) {
    return (RATIO * SPR * deg / 360);
}

// Convert steps to degrees according to step/revolution, rotator gear box ratio and microstep
float step2deg(int32_t step) {
    return (360.00 * step / (SPR * RATIO));
}

// Move both axis with one direction in order to find home position, end-stop switches
enum _rotator_error homing(int32_t seek_az, int32_t seek_el) {
    bool isHome_az = false;
    bool isHome_el = false;

    // Move motors to "seek" position
    stepper_az.moveTo(seek_az);
    stepper_el.moveTo(seek_el);

    // Homing loop
    while (isHome_az == false || isHome_el == false) {
        if (switch_az.get_state() == true && !isHome_az) {
            // Find azimuth home
            stepper_az.moveTo(stepper_az.currentPosition());
            isHome_az = true;
        }
        if (switch_el.get_state() == true && !isHome_el) {
            // Find elevation home
            stepper_el.moveTo(stepper_el.currentPosition());
            isHome_el = true;
        }
        // Check if the rotator goes out of limits or something goes wrong (in mechanical)
        if ((stepper_az.distanceToGo() == 0 && !isHome_az) ||
            (stepper_el.distanceToGo() == 0 && !isHome_el)){
            return homing_error;
        }
        // Move motors to "seek" position
        stepper_az.run();
        stepper_el.run();
    }
    // Delay to Deccelerate and homing, to complete the movements
    uint32_t time = millis();
    while (millis() - time < HOME_DELAY) {
        stepper_az.run();
        stepper_el.run();
    }
    // Set the home position and reset all critical control variables
    stepper_az.setCurrentPosition(0);
    stepper_el.setCurrentPosition(0);
    control_az.setpoint = 0;
    control_el.setpoint = 0;
    return no_error;
}

// Telnet starts here
void handleTelnet(){
  if (server.hasClient()) {
    SerialPort.println("Client is now connected");
  	// client is connected
    if (!client || !client.connected()){
      // client disconnected
      if(client) client.stop();          
      client = server.available();
      client.println("\nDV2JB ESP8266-powered Satnogs V3.1 AZEL Rotator");
      client.println("Type '?' for more help on command");
    }
    else {
      // have client, block new conections
      server.available().stop();  
    }
  }
}

////////////////////////////////////////////////////////
///////////////////////// Setup ////////////////////////
////////////////////////////////////////////////////////

void setup() {

  // Wifi_STA
  WiFi.mode(WIFI_STA);        // To avoid esp8266 from going to AP mode
  WiFi.begin(ssid, password); //Connect to wifi

  //ArduinoOTA code starts
  ArduinoOTA.onStart([]() {   
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    SerialPort.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    SerialPort.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    SerialPort.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    SerialPort.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      SerialPort.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      SerialPort.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      SerialPort.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      SerialPort.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      SerialPort.println("End Failed");
    }
  });
  ArduinoOTA.begin();

  // Disabling the watchdog
  ESP.wdtDisable();

  // Homing switch
  switch_az.init();
  switch_el.init();

  // Serial Communication
  comm.easycomm_init();
  
  // Stepper Motor setup
  stepper_az.setEnablePin(MOTOR_EN);
  stepper_az.setPinsInverted(false, false, true);
  stepper_az.enableOutputs();
  stepper_az.setMaxSpeed(MAX_SPEED);
  stepper_az.setAcceleration(MAX_ACCELERATION);
  stepper_az.setMinPulseWidth(MIN_PULSE_WIDTH);
  stepper_el.setPinsInverted(false, false, true);
  stepper_el.enableOutputs();
  stepper_el.setMaxSpeed(MAX_SPEED);
  stepper_el.setAcceleration(MAX_ACCELERATION);
  stepper_el.setMinPulseWidth(MIN_PULSE_WIDTH);

  // Wait for connection
  SerialPort.println("Connecting to Wifi");                 
  while (WiFi.status() != WL_CONNECTED) {   
    SerialPort.print("."); 
    delay(200);
  }
  SerialPort.println("");
  SerialPort.print("Connected to "); SerialPort.println(ssid);
  SerialPort.print("IP address: "); SerialPort.println(WiFi.localIP());  
  server.begin();
  SerialPort.print("To connect using telnet:");
  SerialPort.print("telnet"); SerialPort.print(WiFi.localIP());
  SerialPort.print(" "); SerialPort.println(port);
  SerialPort.println("");
}

////////////////////////////////////////////////////////
///////////////////////// Loop /////////////////////////
////////////////////////////////////////////////////////

void loop() {
  
  // Feed the watchdog
  ESP.wdtFeed();

  // Get end stop status
  rotator.switch_az = switch_az.get_state();
  rotator.switch_el = switch_el.get_state();

  // Run easycomm implementation
  comm.easycomm_proc();

  // Always be ready when On-the-Air (OTA) sketch upload
  ArduinoOTA.handle();

  // Get position of both axis
  control_az.input = step2deg(stepper_az.currentPosition());
  control_el.input = step2deg(stepper_el.currentPosition());

  // Check rotator status
  if (rotator.rotator_status != error) {
      if (rotator.homing_flag == false) {
          // Check home flag
          rotator.control_mode = position;
          // Homing
          rotator.rotator_error = homing(deg2step(-MAX_M1_ANGLE),
                                          deg2step(-MAX_M2_ANGLE));
          if (rotator.rotator_error == no_error) {
              // No error
              rotator.rotator_status = idle;
              rotator.homing_flag = true;
          } else {
              // Error
              rotator.rotator_status = error;
              rotator.rotator_error = homing_error;
          }
      } else {
          // Control Loop
          stepper_az.moveTo(deg2step(control_az.setpoint));
          stepper_el.moveTo(deg2step(control_el.setpoint));
          rotator.rotator_status = pointing;
          // Move azimuth and elevation motors
          stepper_az.run();
          stepper_el.run();
          // Idle rotator
          if (stepper_az.distanceToGo() == 0 && stepper_el.distanceToGo() == 0) {
              rotator.rotator_status = idle;
          }
      }
  } else {
      // Error handler, stop motors and disable the motor driver
      stepper_az.stop();
      stepper_az.disableOutputs();
      stepper_el.stop();
      stepper_el.disableOutputs();
      if (rotator.rotator_error != homing_error) {
          // Reset error according to error value
          rotator.rotator_error = no_error;
          rotator.rotator_status = idle;
      }
  }
  // Telnet loop
  handleTelnet();
}