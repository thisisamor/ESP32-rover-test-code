#include <string>
#include <Arduino.h>

String camera_read_wall;  
String camera_read_beacon; 
String tmp; 
int count_000, count_010, count_100, count_110, count_111; 
int count_00, count_01, count_10, count_11; 
const int count_th = 10; 

String read_FPGA_wall(int wall_0, int wall_1, int wall_2)
{
    count_000 = count_010 = count_100 = count_110 = count_111 = 0; 
    for (int i=0; i<20; i++)
    {
        camera_read_wall = String(digitalRead(wall_2)) + String(digitalRead(wall_1)) +  String(digitalRead(wall_0));
        // Serial.println("Camera detection result: " + String(camera_read_wall)); 
        if(camera_read_wall=="000")
        {
            count_000 ++; 
        }
        else if(camera_read_wall=="010")
        {
            count_010 ++; 
        }
        else if(camera_read_wall=="100")
        {
            count_100 ++; 
        }
        else if(camera_read_wall=="110")
        {
            count_110 ++; 
        }
        else if(camera_read_wall=="111")
        {
            count_111 ++; 
        }
    }
    if(count_000>count_th)
    {
        tmp = "000"; 
    }
    else if(count_010>count_th)
    {
        tmp = "010"; 
    }
    else if(count_100>count_th)
    {
        tmp = "100";  
    }
    else if(count_110>count_th)
    {
        tmp = "110"; 
    }
    else if(count_111>count_th)
    {
        tmp = "111"; 
    }
    Serial.println("Camera detection result: " + tmp); 
    return tmp; 
}

String read_FPGA_beacon(int beacon_0, int beacon_1)
{    
    count_00 = count_01 = count_10 = count_11 = 0; 
    for (int i=0; i<20; i++)
    {
        camera_read_beacon = String(digitalRead(beacon_1)) + String(digitalRead(beacon_0));
        Serial.println("Camera detection result: " + camera_read_beacon); 
        if(camera_read_beacon=="00")
        {
            count_00 ++; 
        }
        else if(camera_read_beacon=="01")
        {
            count_01 ++; 
        }
        else if(camera_read_beacon=="10")
        {
            count_10 ++; 
        }
        else if(camera_read_beacon=="11")
        {
            count_11 ++; 
        }
    }
    if(count_00>count_th)
    {
        tmp = "00"; 
    }
    else if(count_01>count_th)
    {
        tmp = "01"; 
    }
    else if(count_10>count_th)
    {
        tmp = "10"; 
    }
    else if(count_11>count_th)
    {
        tmp = "11"; 
    }
    Serial.println("Camera detection result: " + tmp); 
    return tmp; 
}

// not used, but possible
void write_FPGA(int key_0, int key_1, int camera_command)
{
    Serial.println("Camera write command: " + String(camera_command/10) + String(camera_command%10)); 
    
    digitalWrite(key_0, (camera_command%10)); 
    digitalWrite(key_1, (camera_command/10)); 
    delay(500); 

    digitalWrite(key_0, 0); 
    digitalWrite(key_1, 0); 
    delay(5000); 
}

// TODO: Daisy says she will output hex
// String UART_read_FPGA()
// {
//     if (Serial2.available() > 0) 
//     {
//         // read the incoming bytes:
//         Serial.println(Serial2.read());
//         return String(Serial2.read()); 
//     }
//     else 
//     {
//         Serial.println("UART not available.");
//     }
    
// }