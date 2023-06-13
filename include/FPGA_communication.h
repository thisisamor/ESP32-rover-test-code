#ifndef FPGA_communication_h
#define FPGA_communication_h

#include <string>
#include <Arduino.h>

String read_FPGA_wall(int wall_0, int wall_1, int wall_2); 
String read_FPGA_beacon(int beacon_0, int beacon_1); 
void write_FPGA(int key_0, int key_1, int camera_command);
String UART_read_FPGA();

String readFromSerial(); 

#endif