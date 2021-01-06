#ifndef LIBRARIES_EASYCOMM_H_
#define LIBRARIES_EASYCOMM_H_

int port = 23;  //Port number
WiFiServer server(port);
WiFiClient client;

#include <Arduino.h>
#include <WString.h>
#include "rotator_pins.h"
#include "globals.h"

#define BUFFER_SIZE   256   ///< Set the size of serial buffer
#define BAUDRATE      19200 ///< Set the Baudrate of easycomm 3 protocol

// Class that functions for easycomm 3 implementation
class easycomm {
public:

    //Initialize the Serial Port
    void easycomm_init() {
	SerialPort.begin(BAUDRATE);
    }

    // Process easycomm commands and response to the client
    void easycomm_proc() {
        char buffer[BUFFER_SIZE];
        char incomingByte;
        char *Data = buffer;
        char *rawData;
        static uint16_t BufferCnt = 0;
        char data[100];
        String str1, str2, str3, str4, str5, str6;

        // If client is connected to Telnet, read all data from client
        while (client.available() > 0) {
            incomingByte = client.read();

            // Read new data, '\n' means new package
            if (incomingByte == '\n' || incomingByte == '\r') {
                buffer[BufferCnt] = 0;
                if (buffer[0] == 'A' && buffer[1] == 'Z') {
                    if (buffer[2] == ' ' && buffer[3] == 'E' &&
                        buffer[4] == 'L') {
                        // Send current absolute position in deg
                        str1 = String("AZ");
                        str2 = String(control_az.input, 1); // 1 here means decimal place (0.0)
                        str3 = String(" EL");
                        str4 = String(control_el.input, 1);
                        str5 = String("\n");
                        client.print(str1 + str2 + str3 + str4 + str5);
                    } else {
                        // Get the absolute position in deg for azimuth
                        rotator.control_mode = position;
                        rawData = strtok_r(Data, " ", &Data);
                        strncpy(data, rawData + 2, 10);
                        if (isNumber(data)) {
                            control_az.setpoint = atof(data);
                        }
                        // Get the absolute position in deg for elevation
                        rawData = strtok_r(Data, " ", &Data);
                        if (rawData[0] == 'E' && rawData[1] == 'L') {
                            strncpy(data, rawData + 2, 10);
                            if (isNumber(data)) {
                                control_el.setpoint = atof(data);
                            }
                        }
                    }
                } else if (buffer[0] == 'E' && buffer[1] == 'L') {
                        // Get the absolute position in deg for elevation
                        rotator.control_mode = position;
                        rawData = strtok_r(Data, " ", &Data);
                        if (rawData[0] == 'E' && rawData[1] == 'L') {
                            strncpy(data, rawData + 2, 10);
                            if (isNumber(data)) {
                                control_el.setpoint = atof(data);
                            }
                        }
                } else if ((buffer[0] == 'S' && buffer[1] == 'A' &&
                           buffer[2] == ' ' && buffer[3] == 'S' &&
                           buffer[4] == 'E') || 
                           (buffer[0] == 's' && buffer[1] == 'a' &&
                           buffer[2] == ' ' && buffer[3] == 's' &&
                           buffer[4] == 'e')
                           ) {
                    // Stop Moving
                    rotator.control_mode = position;
                    str1 = String("AZ");
                    str2 = String(control_az.input, 1);
                    str3 = String(" EL");
                    str4 = String(control_el.input, 1);
                    str5 = String("\n");
                    client.print(str1 + str2 + str3 + str4 + str5);
                    control_az.setpoint = control_az.input;
                    control_el.setpoint = control_el.input;
                } else if ((buffer[0] == 'R' && buffer[1] == 'E' &&
                           buffer[2] == 'S' && buffer[3] == 'E' &&
                           buffer[4] == 'T') ||
                           (buffer[0] == 'r' && buffer[1] == 'e' &&
                           buffer[2] == 's' && buffer[3] == 'e' &&
                           buffer[4] == 't')) {
                    // Reset the rotator, go to home position
                    str1 = String("AZ");
                    str2 = String(control_az.input, 1);
                    str3 = String(" EL");
                    str4 = String(control_el.input, 1);
                    str5 = String("\n");
                    client.print(str1 + str2 + str3 + str4 + str5);
                    rotator.homing_flag = false;
                } else if ((buffer[0] == 'P' && buffer[1] == 'A' &&
                           buffer[2] == 'R' && buffer[3] == 'K' ) || 
                           (buffer[0] == 'p' && buffer[1] == 'a' &&
                           buffer[2] == 'r' && buffer[3] == 'k' ))  {
                    // Park the rotator
                    rotator.control_mode = position;
                    str1 = String("AZ");
                    str2 = String(control_az.input, 1);
                    str3 = String(" EL");
                    str4 = String(control_el.input, 1);
                    str5 = String("\n");
                    client.print(str1 + str2 + str3 + str4 + str5);
                    control_az.setpoint = rotator.park_az;
                    control_el.setpoint = rotator.park_el;
                } else if ((buffer[0] == 'V' && buffer[1] == 'E') || 
                          (buffer[0] == 'v' && buffer[1] == 'e')) {
                    // Get the version if rotator controller
                    str1 = String("VE");
                    str2 = String("SatNOGS-v3.1");
                    str3 = String("\n");
                    client.print(str2 + str3);
                } else if (buffer[0] == '?') {
                    // Show all commands
                    client.println("");
                    client.println("\nDisplay information about builtin commands\n");
                    client.println("AZ              Azimuth, number - 1 decimal place [deg]");
                    client.println("EL              Elevation, number - 1 decimal place [deg]");
                    client.println("SA              Stop azimuth moving");
                    client.println("SE              Stop elevation moving");
                    client.println("RESET           Move to home position");
                    client.println("PARK            Move to park position");
                    client.println("VE              Show the version of the rotator");                 
                    client.println("IP1             Get the status of end-stop, azimuth");
                    client.println("IP2             Get the status of end-stop, elevation");
                    client.println("IP3             et the current position of azimuth in deg");
                    client.println("IP4             Get the current position of elevation in deg");
                    client.println("GS              Get the status of the rotator");
                    client.println("GE              Get if there are any errors");
                    client.println("RB              Reboot the uC");
                    client.println("?               Display this commands again");
                    client.println("");
                } else if (buffer[0] == 'I' && buffer[1] == 'P' &&
                           buffer[2] == '1') {
                    // Get the status of end-stop, azimuth
                    str1 = String("IP1,");
                    str2 = String(rotator.switch_az, DEC);
                    str3 = String("\n");
                    client.print(str1 + str2 + str3);
                } else if (buffer[0] == 'I' && buffer[1] == 'P' &&
                           buffer[2] == '2') {
                    // Get the status of end-stop, elevation
                    str1 = String("IP2,");
                    str2 = String(rotator.switch_el, DEC);
                    str3 = String("\n");
                    client.print(str1 + str2 + str3);
                } else if (buffer[0] == 'I' && buffer[1] == 'P' &&
                           buffer[2] == '3') {
                    // Get the current position of azimuth in deg
                    str1 = String("IP3,");
                    str2 = String(control_az.input, 2);
                    str3 = String("\n");
                    client.print(str1 + str2 + str3);
                } else if (buffer[0] == 'I' && buffer[1] == 'P' &&
                           buffer[2] == '4') {
                    // Get the current position of elevation in deg
                    str1 = String("IP4,");
                    str2 = String(control_el.input, 2);
                    str3 = String("\n");
                    client.print(str1 + str2 + str3);
                } else if ((buffer[0] == 'G' && buffer[1] == 'S') ||
                          (buffer[0] == 'g' && buffer[1] == 's')) {
                    // Get the status of rotator
                    str1 = String("GS");
                    str2 = String(rotator.rotator_status, DEC);
                    str3 = String("\n");
                    client.print(str1 + str2 + str3);
                } else if ((buffer[0] == 'G' && buffer[1] == 'E') ||
                          (buffer[0] == 'g' && buffer[1] == 'e')) {
                    // Get the error of rotator
                    str1 = String("GE");
                    str2 = String(rotator.rotator_error, DEC);
                    str3 = String("\n");
                    client.print(str1 + str2 + str3);
                } else if ((buffer[0] == 'R' && buffer[1] == 'B') ||
                            (buffer[0] == 'r' && buffer[1] == 'b')) {
                    // Custom command to reboot the uC
                    ESP.restart();
                }
                // Reset the buffer an clean the serial buffer
                client.flush();
                BufferCnt = 0;
            } else {
                // Fill the buffer with incoming data
                buffer[BufferCnt] = incomingByte;
                BufferCnt++;
            }
        }
    }

private:
    bool isNumber(char *input) {
        for (uint16_t i = 0; input[i] != '\0'; i++) {
            if (isalpha(input[i]))
                return false;
        }
        return true;
    }
};

#endif /* LIBRARIES_EASYCOMM_H_ */
