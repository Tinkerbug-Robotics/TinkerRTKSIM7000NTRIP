/**********************************************************************
*
* Copyright (c) 2024 Tinkerbug Robotics
*
* This program is free software: you can redistribute it and/or modify it under the terms
* of the GNU General Public License as published by the Free Software Foundation, either
* version 3 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful, but WITHOUT ANY
* WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
* PARTICULAR PURPOSE. See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this 
* program. If not, see <https://www.gnu.org/licenses/>.
* 
* Authors: 
* Christian Pedersen; tinkerbug@tinkerbugrobotics.com
* 
**********************************************************************/
#include "AT_NTRIP.h"
#include <SoftwareSerial.h>
#include <FS.h>
#include <LittleFS.h>
#include "programSkyTraq.h"
#include "inputs.h"

// Software serial pins to talk to SIM7000
#define TX 12
#define RX 13
SoftwareSerial sim7000_serial(RX, TX);

// SIM7000 power key and reset key pins
#define PWRKEY 23
#define RESET 6

// Serial to send correction data to RXD2 port on PX1125R/PX1122R
#define RXD2_TX 4
#define RXD2_RX 5

AT_NTRIP at_ntrip;
programSkyTraq program_skytraq;

// Store current SIM7000 serial baud rate in Flash FS
#define FlashFS LittleFS

bool tcp_connection = false;
unsigned long last_data_received = 0;
int missed_data_delay = 15000;
unsigned long next_connection_time = 0;
int connection_delay = 2500;
int missed_connections = 0;
unsigned long num_resets = 0;

// File to save SIM7000 buad rate in
const char *filepath = "/sim7000_baud_rate.txt";
int saved_baud_rate = 0;

void setup()
{
    Serial.begin(115200);
    //while (!Serial){};
    delay(1000);
    Serial.println("Setup...");

    // Initialze library to program SkyTraq
    // Uses Serial1 default 0,1 pins
    program_skytraq.init(Serial1);
    
    // GNSS input/output Serial is Serial1 using default 0,1 (TX, RX) pins
    // Loop through valid baud rates and determine the current setting
    // Set Serial1 to the detected baud rate, stop if a baud rate is not found
    // From NavSpark binary protocol. Search for "SkyTrq Application Note AN0037"
    // Currently available at: https://www.navsparkforum.com.tw/download/file.php?id=1162&sid=dc2418f065ec011e1b27cfa77bf22b19
    // This message also resets the receiver to the default configruation
    if(!autoSetBaudRate())
    {
        Serial.println("No valid baud rate found to talk to receiver, stopping");
        while(1);
    }
    
    delay(500);

    Serial.print("Opening file system to read SIM7000 baud rate ... ");
    if (!LittleFS.begin()) 
    {
        Serial.println("Flash FS initialization failed!");
        while (1) yield();
    }
    Serial.println("Flash FS available");
  
    // Open the file for reading
    File file = LittleFS.open(filepath, "r");
    if (file) 
    {
        // Get the file size
        size_t fileSize = file.size();
      
        // Read the entire file into a dynamically allocated buffer
        char *buffer = (char *)malloc(fileSize + 1);  // +1 for null terminator
      
        if (buffer == NULL) 
        {
          Serial.println("Error allocating memory for the buffer.");
          file.close();
          return;
        }
      
        size_t bytesRead = file.read((uint8_t *)buffer, fileSize);
      
        // Null-terminate the buffer
        buffer[bytesRead] = '\0';
      
        // Close the file
        file.close();
      
        Serial.print("Read ");
        Serial.print(bytesRead);
        Serial.print(" bytes from LittleFS for touch screen calibration: ");
        Serial.println(buffer);
      
        if (bytesRead > 1)
        {
            // Parse touchscreen calibration data
            char *ptr = NULL;
            ptr = strtok(buffer,",");
            saved_baud_rate = atoi(ptr);
            //Serial.println(saved_baud_rate);
        }

        // Un-allocate memory
        free(buffer);
    }
    // If file does not exist, then create file with default value
    else
    {
        
        saved_baud_rate = 19200;
        Serial.print("No baud rate file found, setting saved baud rate to default, ");
        Serial.println(saved_baud_rate);

        writeBaudRateFile(saved_baud_rate);
    }
    
    // Start software serial connection to SIM7000
    sim7000_serial.begin(saved_baud_rate);

    // Initialize the SIM7000
    at_ntrip.init(PWRKEY, 
                  RESET, 
                  apn,
                  host,
                  http_port,
                  mount_point,
                  user,
                  psw,
                  src_string,
                  sim7000_serial);
                  
    // If the saved baud rate does not match the input baud rate
    if(saved_baud_rate != sim7000_baud_rate)
    {
        Serial.print("Setting SIM7000 baud rate to ");
        Serial.print(sim7000_baud_rate);
        Serial.print(" ... ");
        if(at_ntrip.setBaudRate(sim7000_baud_rate))
        {
            Serial.print("Succesfully set to ");
            Serial.println(sim7000_baud_rate);
        }
        sim7000_serial.end();
        sim7000_serial.begin(sim7000_baud_rate);

        // Save new baud rate to file for next time
        writeBaudRateFile(sim7000_baud_rate);
    }

    // Serial connection to GNSS receiver for sending corrections
    // Receive side doesn't work
    Serial2.setTX(RXD2_TX);
    Serial2.setRX(RXD2_RX);
    Serial2.begin(115200);


    // Establish TCP connection with SIM7000
    at_ntrip.establishTCPConnection();

    next_connection_time = millis() + connection_delay;
    last_data_received = millis();

    // Start a watchdog timer which resets the RP2040 if it is not reset 
    // within the milliseconds submitted as an argument
    rp2040.wdt_begin(8000);
}

unsigned long last_data_time = 0;

void loop()
{
    // Reset watchdog timer
    rp2040.wdt_reset();

    // If the TCP connection is not present establish it
    if (millis() > last_data_received + missed_data_delay && millis() > next_connection_time)
    {

        Serial.print(millis());Serial.println(" TCP Connection lost");

        // Clear out any messages being sent back from the SIM7000 and try again
        delay(500);
        // Establish a TCP connection to the NTRIP caster using the SIM7000
        tcp_connection = at_ntrip.establishTCPConnection();
        if (tcp_connection)
        {
            next_connection_time = millis();
            missed_connections = 0;
            connection_delay = 2500;
        }
        else
        {
            // Grow the delay with each failed attempt to not 
            // overwhelm the server with connection attempts and get banned
            missed_connections++;
            next_connection_time = next_connection_time + missed_connections * connection_delay;
            Serial.print(millis());Serial.print(" Conection failed, next connectiont time: ");Serial.println(next_connection_time);

            connection_delay = connection_delay * 2;
        }
        num_resets++;
    }

    // If there is NTRIP data available to read
    if (sim7000_serial.available() > 0)
    {
        // Read NTRIP corrections direction from SIM7000
        char tcp_buff[5000];
        // Pass in buffer to populate, maximum length to read, and time out
        unsigned long start_read = millis();
        uint16_t data_length = at_ntrip.readTCP(tcp_buff, 5000, 1500);
        if(data_length > 0)
        {
    
            // Parse RTCM data, print message types, and check validity
            Serial.print("Read ");Serial.print(data_length);
            Serial.print(" bytes in "); Serial.println(millis()-start_read);
            Serial.print("Parsed data into RTCM Messges: ");
            checkAndPrintMsgs(tcp_buff, data_length);
            Serial.println("");
            last_data_received = millis();
            next_connection_time = millis() + connection_delay;
    
            // Send correction data to GNSS correction input port
            for (uint16_t i = 0; i < data_length; i++)
            {
                Serial2.print(tcp_buff[i]);
            }
        }
    }
}

// Loop through valid baud rates for the GNSS receiver and determine the current setting
bool autoSetBaudRate()
{
    // Start serial connections to send correction data to GNSS receiver
    // This loop will detect the current baud rate of the GNSS receiver
    // by sending a message and determining which baud rate returns a valid
    // ACK message
    int valid_baud_rates[9] = {115200, 4800, 9600, 19200, 38400, 57600,
                               230400, 460800, 921600};

    // Message to reset receiver to defaults
    uint8_t res_payload_length[]={0x00, 0x02};
    int res_payload_length_length = 2;
    uint8_t res_msg_id[]={0x04};
    int res_msg_id_length = 1;
    uint8_t res_msg_body[]={0x01};
    int res_msg_body_length = 1;

    // Loop through possible baud rates
    for (int i=0;i<9;i++)
    {
        // Open the serial connection to the receiver
        Serial1.begin(valid_baud_rates[i]);

        // Send a message to reset receiver to defaults
        if (program_skytraq.sendGenericMsg(res_msg_id,
                                           res_msg_id_length,
                                           res_payload_length,
                                           res_payload_length_length,
                                           res_msg_body,
                                           res_msg_body_length) == 1)
        {
            Serial.print("Found correct baud rate of ");
            Serial.print(valid_baud_rates[i]);
            Serial.println(" for GNSS receiver");
            return true;            
        }               
        else
        {
            Serial1.end();
        }
    }

    return false;
}



void checkAndPrintMsgs(char *rtcm_data, unsigned int data_length)
{
    bool in_message = false;
    unsigned int msg_num;
    unsigned int msg_length;
    uint8_t rtcm_msg[2500];
    unsigned int msg_indx = 0;
    char last_byte = 0;

    // Loop through received data, may contain more than one message
    for(int i=0;i<data_length;i++)
    {
    
        // Read the next character in the message
        char in_byte = rtcm_data[i];
        //Serial.print(in_byte,HEX);Serial.print(" ");
    
        // Look for the start of a message
        if (!in_message && last_byte == 0xD3 && (in_byte & 0xFC) == 0x00)
        {
    
            // Set he first fields in the message
            rtcm_msg[0] = last_byte;
            rtcm_msg[1] = in_byte;
            rtcm_msg[2] = rtcm_data[i+1];
            rtcm_msg[3] = rtcm_data[i+2];
            rtcm_msg[4] = rtcm_data[i+3];
    
            // Calculte the message length and number
            msg_length = ((rtcm_msg[1] & 3) << 8) + (rtcm_msg[2] << 0) + 6;
            msg_num = (rtcm_msg[3] << 4) + (rtcm_msg[4] >> 4);
            Serial.print(msg_num);Serial.print(" ");

            // Increment indices
            msg_indx = 5;
            i = i+3;
    
            in_message = true;
        }
    
        // End of a message (inside a packet)
        else if (msg_indx == msg_length-1)
        {
    
            // Add last character to RTCM message
            rtcm_msg[msg_indx]=in_byte;
        
            // Check CRC of message
            if(Crc24Quick(0x000000, msg_length, rtcm_msg)!=0)
            {
                Serial.print(" - failed");
            }

            in_message = false;
            msg_length = 0;
            msg_indx = 0;
            last_byte = 0;
        }
        // Reading a message
        else if (in_message)
        {
            // Add last character to RTCM message
            rtcm_msg[msg_indx]=in_byte;
            
            msg_indx++;
        }

        last_byte = in_byte;
    }
}

uint32_t Crc24Quick(uint32_t Crc, uint32_t Size, uint8_t *Buffer)
{
  static const uint32_t crctab[] = {
    0x00000000,0x01864CFB,0x038AD50D,0x020C99F6,0x0793E6E1,0x0615AA1A,0x041933EC,0x059F7F17,
    0x0FA18139,0x0E27CDC2,0x0C2B5434,0x0DAD18CF,0x083267D8,0x09B42B23,0x0BB8B2D5,0x0A3EFE2E };

  while(Size--)
  {
    Crc ^= (uint32_t)*Buffer++ << 16;
    Crc = (Crc << 4) ^ crctab[(Crc >> 20) & 0x0F];
    Crc = (Crc << 4) ^ crctab[(Crc >> 20) & 0x0F];
  }

  return(Crc & 0xFFFFFF);
}

void writeBaudRateFile(int baud_rate)
{
    char baud_rate_char[20];
    itoa(baud_rate,baud_rate_char,10);
    
    // Open or create the file for writing
    File file = LittleFS.open(filepath, "w");
    if (!file)
    {
        Serial.println("Error opening file for writing.");
    }
    
    // Write a character array to the file
    size_t bytesWritten = file.write((const uint8_t *)baud_rate_char, strlen(baud_rate_char));
    
    if (bytesWritten != strlen(baud_rate_char)) 
    {
        Serial.println("Error: Calibration file failed to open, using temporary values");
        file.close();
    }
    
    // Close the file
    file.close();
}
