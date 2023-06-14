#include <string>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "Amorrr";
const char* password = "AMOrTHebesT";
const char* HOST_NAME = "http://35.176.36.0:8080/";
const int   HTTP_PORT = 30;

// ---------------- connect to wifi ---------------------
void wifi_connection()
{
  Serial.println("hello"); 
  WiFi.begin(ssid, password);
  Serial.println("start to connect");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

// --------------- check wifi connection ----------------
bool check_wifi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    return false; 
  }
  else
  {
    return true; 
  }
}

// ------- POST request ------------------
void client_post(String web_addr, String message)
{
  // DynamicJsonDocument jsonPayload(128); // the capacity of the JSON document
  // jsonPayload["node_id"] = "0"; 
  // jsonPayload["x"] = "2"; 
  // jsonPayload['y'] = "3"; 
  // jsonPayload['connect'] = "1, 2, 3"; 
  // String payload;
  // serializeJson(jsonPayload, payload);

  HTTPClient http;
  http.begin(web_addr);
  http.addHeader("Content-Type", "application/json");
  int httpCode = http.POST(message);

  if(httpCode > 0) 
  {
    Serial.printf("[HTTP] GET... code: %d\n", httpCode);
    if(httpCode == HTTP_CODE_OK) 
    {
      String response = http.getString();
      Serial.println(response);
    }
  } 
  else 
  {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
  // delay(5000);
}

// -------- GET request --------- (for testing only)
void client_get() 
{
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    // Specify the target URL
    http.begin("http://35.176.36.0:8080/");
    // Send the HTTP GET request
    int httpResponseCode = http.GET();
    if (httpResponseCode == HTTP_CODE_OK) 
    {
      // Successful response
      String payload = http.getString();
      Serial.println(payload);
    } 
    else 
    {
      // Error in HTTP request
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    // Close the connection
    http.end();
  }
  // Wait for 5 seconds before making the next request
  delay(5000);
}





