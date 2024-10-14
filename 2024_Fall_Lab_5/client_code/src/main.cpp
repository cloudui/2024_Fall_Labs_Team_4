#include <WiFi.h>

struct __attribute__((packed)) Data {
    int16_t seq;     // sequence number
    int32_t distance; // distance
    float voltage;   // voltage
    char text[50];   // text
};

// WiFi network credentials
const char* ssid = "iPhone (18)";
const char* password = "lillit12";

// Server IP and port
const char* host = "172.20.10.8";  // Replace with the IP address of server
const uint16_t port = 9500;

// Create a client
WiFiClient client;

int i = 0;

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");

  // Connect to the server
  if (client.connect(host, port)) {
    Serial.println("Connected to server!");
  } else {
    Serial.println("Connection to server failed.");
    return;
  }
}

void loop() {
    // Prepare data packet
    Data data;
    data.seq = 1;
    data.distance = 1000;
    data.voltage = 3.7f;
    strncpy(data.text, "Hello from ESP32!", sizeof(data.text) - 1);
    data.text[sizeof(data.text) - 1] = '\0'; // Ensure null termination
    //Serial.printf("seq %d distance %ld voltage %f text %s\n", data.seq, data.distance, data.voltage, data.text);

    // Check if connected to the server
    if (client.connected()) {
      // Read server's response (if any)
      while (client.available()) {
        Data response;
        client.readBytes((char*)&response, sizeof(response)); // Read data from the server and unpack it into the response struct
        Serial.printf("seq %d distance %ld voltage %f text %s\n", (int)response.seq, (long)response.distance, response.voltage, response.text);
      }

      // Send data to the server
      client.write((uint8_t*)&data, sizeof(data));

      // Increment sequence number for the next packet and add a delay between messages
      data.seq++;
      delay(5000); // Send data every 5 seconds
    } else {
      if(i == 0){
        Serial.println("Disconnected from server.");
        i++;
      }
      client.stop();
    }
}