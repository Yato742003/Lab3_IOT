#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "DHT20.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <Wire.h>
#include <WiFiClientSecure.h>
#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <ThingsBoard.h>
#include <OTA_Firmware_Update.h>
#include <Espressif_Updater.h>

// DHT20 Sensor
DHT20 dht20;

constexpr char WIFI_SSID[] = "NsiPhone";
constexpr char WIFI_PASSWORD[] = "66666666";

// to understand how to obtain an access token
constexpr char TOKEN[] = "5xueerch12wkwyscd82n";
// Thingsboard we want to establish a connection too
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";

// MQTT port used to communicate with the server, 1883 is the default unencrypted MQTT port,
// whereas 8883 would be the default encrypted SSL MQTT port
#if ENCRYPTED
constexpr uint16_t THINGSBOARD_PORT = 8883U;
#else
constexpr uint16_t THINGSBOARD_PORT = 1883U;
#endif

// Maximum size packets will ever be sent or received by the underlying MQTT client,
// if the size is to small messages might not be sent or received messages will be discarded
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;

// Baud rate for the debugging serial connection.
// If the Serial output is mangled, ensure to change the monitor speed accordingly to this variable
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr char CURRENT_FIRMWARE_TITLE[] = "ESP_OTA";
constexpr char CURRENT_FIRMWARE_VERSION[] = "1.0.0";
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 12U;
constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U;

// Target firmware telemetry values
constexpr char TARGET_FW_TITLE[] = "ESP_OTA 1.4";
constexpr char TARGET_FW_VERSION[] = "1.4";

uint32_t previousStateChange;
constexpr int16_t telemetrySendInterval = 5000U;
uint32_t previousDataSend;

// #if ENCRYPTED
// See https://comodosslstore.com/resources/what-is-a-root-ca-certificate-and-how-do-i-download-it/
// on how to get the root certificate of the server we want to communicate with,
// this is needed to establish a secure connection and changes depending on the website.
constexpr char ROOT_CERT[] = R"(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)";
// #endif

constexpr const char RPC_JSON_METHOD[] = "example_json";
constexpr const char RPC_TEMPERATURE_METHOD[] = "example_set_temperature";
constexpr const char RPC_SWITCH_METHOD[] = "example_set_switch";
constexpr const char RPC_TEMPERATURE_KEY[] = "temp";
constexpr const char RPC_HUMIDITY_KEY[] = "humidity";
constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 3U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;

// Initialize underlying client, used to establish a connection
#if ENCRYPTED
WiFiClientSecure espClient;
#else
WiFiClient espClient;
#endif
// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(espClient);
// Initialize used apis
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
OTA_Firmware_Update<> ota;
Espressif_Updater<> updater;
const std::array<IAPI_Implementation *, 2U> apis = {
    &rpc, &ota};
// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

// Statuses for subscribing to rpc
bool subscribed = false;

bool currentFWSent = false;
bool updateRequestSent = false;

// Flag to track telemetry sending
bool telemetrySent = false;

void update_starting_callback() {
  // Nothing to do initially; can add logic to pause other tasks if needed
}

void progress_callback(const size_t &current, const size_t &total) {
  Serial.printf("Progress %.2f%%\n", static_cast<float>(current * 100U) / total);
}

void finished_callback(const bool &success) {
  if (success) {
      Serial.println("Done, Reboot now");
      ESP.restart();
  } else {
      Serial.println("Downloading firmware failed");
  }
}

/// @brief Processes function for RPC call "example_json"
/// JsonVariantConst is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @param response Data containgin the response value, any number, string or json, that should be sent to the cloud. Useful for getMethods
void processGetJson(const JsonVariantConst &data, JsonDocument &response)
{
  Serial.println("Received the json RPC method");

  // Size of the response document needs to be configured to the size of the innerDoc + 1.
  StaticJsonDocument<JSON_OBJECT_SIZE(4)> innerDoc;
  innerDoc["string"] = "exampleResponseString";
  innerDoc["int"] = 5;
  innerDoc["float"] = 5.0f;
  innerDoc["bool"] = true;
  response["json_data"] = innerDoc;
}

/// @brief Processes function for RPC call "example_set_temperature"
/// JsonVariantConst is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @param response Data containgin the response value, any number, string or json, that should be sent to the cloud. Useful for getMethods
void processTemperatureChange(const JsonVariantConst &data, JsonDocument &response)
{
  Serial.println("Received the set temperature RPC method");

  // Process data
  const float example_temperature = data[RPC_TEMPERATURE_KEY];

  Serial.print("Example temperature: ");
  Serial.println(example_temperature);

  // Ensure to only pass values do not store by copy, or if they do increase the MaxRPC template parameter accordingly to ensure that the value can be deserialized.RPC_Callback.
  // See https://arduinojson.org/v6/api/jsondocument/add/ for more information on which variables cause a copy to be created
  response["string"] = "exampleResponseString";
  response["int"] = 5;
  response["float"] = 5.0f;
  response["double"] = 10.0;
  response["bool"] = true;
}

/// @brief Processes function for RPC call "example_set_humidity"
/// JsonVariantConst is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @param response Data containgin the response value, any number, string or json, that should be sent to the cloud. Useful for getMethods
void processHumidityChange(const JsonVariantConst &data, JsonDocument &response)
{
  Serial.println("Received the set humidity RPC method");

  // Process data
  const float example_humidity = data[RPC_TEMPERATURE_KEY];

  Serial.print("Example humidity: ");
  Serial.println(example_humidity);

  // Ensure to only pass values do not store by copy, or if they do increase the MaxRPC template parameter accordingly to ensure that the value can be deserialized.RPC_Callback.
  // See https://arduinojson.org/v6/api/jsondocument/add/ for more information on which variables cause a copy to be created
  response["string"] = "exampleResponseString";
  response["int"] = 5;
  response["float"] = 5.0f;
  response["double"] = 10.0;
  response["bool"] = true;
}

// Task 1: Connect to WiFi
void wifiTask(void *pvParameters) {
  while (1) {
    // Check WiFi status
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Connecting to AP ...");

      // Attempt to connect to WiFi
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      
      // Wait for connection with periodic status updates
      while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        Serial.print(".");
      }

      delay(500);

      Serial.print("\nConnected to: ");
      Serial.println(WiFi.localIP());

#if ENCRYPTED
      espClient.setCACert(ROOT_CERT);
#endif
    }
  }
  // Periodically check WiFi status
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// Task 2: ThingsBoard connection and RPC subscription
void thingsboardTask(void *pvParameters) {
  while (1) {
    if (!tb.connected()) {
      Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        Serial.println("Failed to connect");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        continue;
      }
      tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
    }

    if (!subscribed) {
      Serial.println("Subscribing for RPC...");
      const std::array<RPC_Callback, MAX_RPC_SUBSCRIPTIONS> callbacks = {
        // Requires additional memory in the JsonDocument for the JsonDocument that will be copied into the response
        RPC_Callback{RPC_JSON_METHOD, processGetJson},
        // Requires additional memory in the JsonDocument for 5 key-value pairs that do not copy their value into the JsonDocument itself
        RPC_Callback{RPC_TEMPERATURE_METHOD, processTemperatureChange},
        // Internal size can be 0, because if we use the JsonDocument as a JsonVariant and then set the value we do not require additional memory
        RPC_Callback{RPC_SWITCH_METHOD, processHumidityChange}};
      if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
        Serial.println("Failed to subscribe for RPC");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        continue;
      }
      Serial.println("Subscribe done");
      subscribed = true;
    }

    tb.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Task 3: DHT20 sensor reading and telemetry sending
void dht20Task(void *pvParameters) {
  while (1) {
    if (tb.connected()) {
      if (millis() - previousDataSend > telemetrySendInterval) {
        dht20.read();

        float temperature = dht20.getTemperature();
        float humidity = dht20.getHumidity();

        if (isnan(temperature) || isnan(humidity)) {
          Serial.println("Failed to read from DHT20 sensor!");
        } else {
          Serial.print("Temperature: ");
          Serial.print(temperature);
          Serial.print(" °C, Humidity: ");
          Serial.print(humidity);
          Serial.println(" %");

          tb.sendTelemetryData("temperature", temperature);
          tb.sendTelemetryData("humidity", humidity);
        }

        tb.sendAttributeData("rssi", WiFi.RSSI());
        tb.sendAttributeData("channel", WiFi.channel());
        tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
        tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
        tb.sendAttributeData("ssid", WiFi.SSID().c_str());

        previousDataSend = millis();
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Avoid tight loop
  }
}

void otaTask(void *pvParameters) {
  while (1) {
      if (tb.connected()) {
          // Send current firmware info if not already sent
          if (!currentFWSent) {
              Serial.println("Sending firmware info...");
              currentFWSent = ota.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION);
              if (currentFWSent) {
                  Serial.println("Firmware info sent");
              } else {
                  Serial.println("Failed to send firmware info");
              }
          }

          // Subscribe to firmware updates if not already subscribed
          if (!updateRequestSent) {
              Serial.println("Firmware Update Subscription...");
              const OTA_Update_Callback callback(CURRENT_FIRMWARE_TITLE,
                                                CURRENT_FIRMWARE_VERSION,
                                                &updater,
                                                &finished_callback,
                                                &progress_callback,
                                                &update_starting_callback,
                                                FIRMWARE_FAILURE_RETRIES,
                                                FIRMWARE_PACKET_SIZE);
              updateRequestSent = ota.Subscribe_Firmware_Update(callback);
              if (updateRequestSent) {
                  Serial.println("OTA subscription successful");
              } else {
                  Serial.println("Failed to subscribe to OTA updates");
              }
          }

          // Send additional telemetry after successful subscription
          if (updateRequestSent && !telemetrySent) {
              tb.sendTelemetryData("target_fw_title", TARGET_FW_TITLE);
              tb.sendTelemetryData("target_fw_version", TARGET_FW_VERSION);
              telemetrySent = true;
              Serial.println("Sent target firmware telemetry");
          }

          // Reduce CPU usage if all tasks are complete
          if (currentFWSent && updateRequestSent && telemetrySent) {
              vTaskDelay(60000 / portTICK_PERIOD_MS); // Sleep for 1 minute
              continue;
          }
      }
      vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay 1 second if not connected
  }
}

void setup()
{
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(3000);
  Wire.begin(GPIO_NUM_11, GPIO_NUM_12); // Initialize I2C
  dht20.begin();
  xTaskCreate(wifiTask, "WiFi Task", 8192, NULL, 2, NULL);
  // xTaskCreate(otaTask, "OTA Task", 8192, NULL, 1, NULL);
  xTaskCreate(thingsboardTask, "ThingsBoard Task", 16384, NULL, 1, NULL);
  xTaskCreate(dht20Task, "DHT20 Task", 32768, NULL, 0, NULL);
  xTaskCreate(otaTask, "OTA Task", 8192, NULL, 1, NULL);
}

void loop()
{
  vTaskDelay(1000 / portTICK_PERIOD_MS); // Keep the main loop alive but idle
}
