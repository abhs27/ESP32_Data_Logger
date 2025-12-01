/*
 * ESP32 SIM7600E Data Logger with BMS Integration
 * Collects and displays raw data from GPS, GSM module, BMS, and board resources in JSON format
 * Compatible with ESP32 SIM Card T-SIM7600E V1.2 board
 * 
 * Pin Configuration:
 * - SIM7600E UART: TX=GPIO27, RX=GPIO26 (GPS is integrated in SIM7600E)
 * - BMS RS-485: DI=GPIO27, RO=GPIO26, DE/RE=GPIO25 (temporarily reconfigures SIM7600E serial)
 * - Status LED: GPIO12
 * - Power control: GPIO4
 */

#include <ArduinoJson.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_system.h>
#include <esp_adc_cal.h>
#include <driver/adc.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <ESPSupabase.h>
#include <ElegantOTA.h>
 
 // Hardware Serial for SIM7600E module (GPS is integrated in SIM7600E)
 HardwareSerial SIM7600E(2);
 
 // Note: GPS functionality is provided by SIM7600E module, not a separate GPS module
 
 
// WiFi credentials
// const char* ssid = "Ashray";
// const char* password = "12345678";
// const char* ssid = "Dh";
// const char* password = "dhanu009";
const char* ssid = "abhi";
const char* password = "password123";


// Supabase configuration
const char* supabase_url = "https://csxkuaefexluasbaknpp.supabase.co";  // Replace with your Supabase URL
const char* supabase_key = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImNzeGt1YWVmZXhsdWFzYmFrbnBwIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NjA0NzI1MTksImV4cCI6MjA3NjA0ODUxOX0.MgNM-BDFMU8NMRI9iLVUkRGxfyAd1xepYvs8Wbwu4JY";  // Replace with your Supabase anon key
const char* supabase_table = "sensor_data";  // Table name for storing data

// Web server configuration
WebServer server(80);
String jsonData = "{}"; // Store the latest JSON data

// Supabase instance
Supabase db;
 
 // Pin definitions based on board schematic
 #define SIM7600E_TX_PIN 27  // SIM7600E TX (ESP32 RX)
 #define SIM7600E_RX_PIN 26  // SIM7600E RX (ESP32 TX)
 #define STATUS_LED_PIN 12
 #define POWER_PIN 4
 #define VBAT_ADC_PIN 34
 // Note: GPS is integrated in SIM7600E module, no separate GPS pins needed
 
 // BMS RS-485 Pin definitions
 #define RS485_DE_RE_PIN 25    // Driver Enable / Receiver Enable
 #define RS485_DI_PIN    33    // Data Input (ESP32 -> RS-485)
 #define RS485_RO_PIN    32    // Receiver Output (RS-485 -> ESP32)
 
 // BMS Data Collection Toggles (Set to 1 to enable, 0 to disable)
 #define ENABLE_BMS_BASIC_STATUS      1  // Command 0x90: Cumulative voltage, current, SOC
 #define ENABLE_BMS_VOLTAGE_LIMITS    1  // Command 0x91: Max/Min cell voltage and number
 #define ENABLE_BMS_TEMPERATURE       1  // Command 0x92: Max/Min temperature and cell
 #define ENABLE_BMS_MOSFET_STATUS     1  // Command 0x93: Charge state, MOS status, cycles, remaining capacity
 #define ENABLE_BMS_SYSTEM_INFO       1  // Command 0x94: Number of cells/temps, charger/load connection
 #define ENABLE_BMS_CELL_VOLTAGES     1  // Command 0x95: Individual cell voltages
 #define ENABLE_BMS_CELL_TEMPERATURES 1  // Command 0x96: Individual cell temperatures
 #define ENABLE_BMS_CELL_BALANCE      1  // Command 0x97: Cell balance states
 #define ENABLE_BMS_FAULT_STATUS      1  // Command 0x98: Fault code and status
 
 // Data Collection Toggles (Set to 1 to enable, 0 to disable)
 #define ENABLE_GPS_DATA              1  // Enable/disable GPS data collection
 #define ENABLE_GSM_DATA              0  // Enable/disable GSM data collection
 #define ENABLE_BOARD_DATA            0  // Enable/disable Board data collection
 
 // BMS Timing configuration
 #define BMS_RESPONSE_TIMEOUT_MS 100     // Timeout for waiting for response (100ms)
 #define BMS_POST_COMMAND_DELAY_MS 50    // Delay after sending command (50ms)
 #define BMS_COMMAND_INTERVAL_MS 1000    // Time between command cycles (1 second)
 
 // Timing
 unsigned long lastDataOutput = 0;
 const unsigned long DATA_INTERVAL = 5000; // 5 seconds
 
 // Data structures
 struct GPSData {
   String latitude = "N/A";
   String longitude = "N/A";
   String altitude = "N/A";
   String speed = "N/A";
   String course = "N/A";
   String satellites = "N/A";
   String fixQuality = "N/A";
   String timestamp = "N/A";
   bool isValid = false;
 };
 
 struct GSMData {
   String signalStrength = "N/A";
   String networkOperator = "N/A";
   String networkType = "N/A";
   String imei = "N/A";
   String iccid = "N/A";
   String phoneNumber = "N/A";
   String batteryVoltage = "N/A";
   String temperature = "N/A";
   bool isConnected = false;
 };
 
 struct BoardData {
   float cpuFrequency;
   float freeHeap;
   float totalHeap;
   float batteryVoltage;
   float temperature;
   String chipModel;
   String chipRevision;
   String flashSize;
   String sdkVersion;
   unsigned long uptime;
   int wifiRSSI;
   String wifiStatus;
   String localIP;
   String macAddress;
 };
 
 // BMS Data Structure
 struct BMSData {
   // Basic status (0x90)
   float cumulative_voltage = 0.0;
   float gather_voltage = 0.0;
   float current = 0.0;
   float soc = 0.0;
   
   // Voltage limits (0x91)
   float max_cell_voltage = 0.0;
   int max_cell_number = 0;
   float min_cell_voltage = 0.0;
   int min_cell_number = 0;
   
   // Temperature (0x92)
   int max_temp = 0;
   int max_temp_cell = 0;
   int min_temp = 0;
   int min_temp_cell = 0;
   
   // MOSFET status (0x93)
   String charge_state = "";
   bool charge_mos = false;
   bool discharge_mos = false;
   int bms_cycles = 0;
   float remaining_capacity = 0.0;
   
   // System info (0x94)
   int num_cells = 0;
   int num_temps = 0;
   bool charger_connected = false;
   bool load_connected = false;
   
   // Cell voltages (0x95)
   float cell_voltages[48] = {0};
   int cell_voltage_count = 0;
   
   // Cell temperatures (0x96)
   int cell_temperatures[16] = {0};
   int cell_temp_count = 0;
   
   // Cell balance states (0x97)
   bool cell_balance_states[48] = {false};
   
   // Fault status (0x98)
   bool has_faults = false;
   int fault_code = 0;

   // Parsing status flags
   bool basic_status_parsed = false;
   bool voltage_limits_parsed = false;
   bool temperature_parsed = false;
   bool mosfet_status_parsed = false;
   bool system_info_parsed = false;
   bool cell_voltages_parsed = false;
   bool cell_temperatures_parsed = false;
   bool cell_balance_states_parsed = false;
   bool fault_status_parsed = false;
 };
 
 
 GPSData gpsData;
 GSMData gsmData;
 BoardData boardData;
 BMSData bmsData;
 
 // Function declarations
 void initializeSIM7600E();
 void enableGPS();
 bool waitForGPSFix(int maxWaitSeconds = 300);
 void initializeBoardData();
 void connectToWiFi();
 void setupWebServer();
 void handleRoot();
 void handleJSON();
 void handleStatus();
void collectGPSData();
void collectGSMData();
void collectBoardData();
void collectBMSData();
float readBatteryVoltage();
void parseGPSData(String data);
String waitForGPSResponse(int timeout);
String sendATCommand(String command, int timeout = 1000, bool verbose = false);
double convertToDecimalDegrees(String coord, String direction);
void updateJSONData();
void handleSerialCommands();
void initializeSupabase();
void sendDataToSupabase();
 
 // BMS Functions
 void initializeBMS();
 void sendBMSCommand(uint8_t command);
 bool parseBasicStatus(uint8_t* data, int length);
 bool parseVoltageLimits(uint8_t* data, int length);
 bool parseTemperature(uint8_t* data, int length);
 bool parseMosfetStatus(uint8_t* data, int length);
 bool parseSystemInfo(uint8_t* data, int length);
 bool parseCellVoltages(uint8_t* data, int length);
 bool parseCellTemperatures(uint8_t* data, int length);
 bool parseCellBalanceStates(uint8_t* data, int length);
 bool parseFaultStatus(uint8_t* data, int length);
 void setTransmitMode();
 void setReceiveMode();
 
 void setup() {
   Serial.begin(115200);
   delay(1000);
   
   Serial.println("ESP32 SIM7600E Data Logger Starting...");
   
   // Initialize pins
   pinMode(STATUS_LED_PIN, OUTPUT);
   pinMode(POWER_PIN, OUTPUT);
   digitalWrite(STATUS_LED_PIN, LOW);
   digitalWrite(POWER_PIN, HIGH); // Enable SIM7600E power
   
   // Initialize SIM7600E UART (115200 baud)
   SIM7600E.begin(115200, SERIAL_8N1, SIM7600E_RX_PIN, SIM7600E_TX_PIN);
   
   // Wait for modules to initialize
   delay(3000);
   
   // Initialize SIM7600E module
   initializeSIM7600E();
   
   // Wait for GPS fix before proceeding with other operations
   Serial.println("Waiting for GPS fix...");
   if (waitForGPSFix(300)) {  // Wait up to 5 minutes for GPS fix 
     Serial.println("GPS fix achieved! Proceeding with system initialization...");
   } else {
     Serial.println("WARNING: GPS fix not achieved within timeout period. System will continue but GPS-dependent operations may be limited.");
   }
   
   // Initialize board data
   initializeBoardData();
   
  // Initialize BMS
  initializeBMS();
  
  // Connect to WiFi
  connectToWiFi();
  
  // Initialize Supabase
  initializeSupabase();
  
  // Setup web server
  setupWebServer();
  ElegantOTA.begin(&server); // Initialize ElegantOTA

   
   Serial.println("System initialized. Starting data collection...");
   Serial.print("Access JSON data at: http://");
   Serial.println(WiFi.localIP());
   digitalWrite(STATUS_LED_PIN, HIGH);
}
 
 void loop() {
   unsigned long currentTime = millis();
   
   if (currentTime - lastDataOutput >= DATA_INTERVAL) {
     // Always collect GPS data first
     #if ENABLE_GPS_DATA
     collectGPSData();
     #endif
     
     // Small delay after GPS to let module stabilize
     delay(100);
     
     // Only proceed with other operations if GPS fix is valid
     if (gpsData.isValid) {
       // Collect GSM and Board data (these don't interfere with GPS)
       #if ENABLE_GSM_DATA
       collectGSMData();
       #endif
       delay(50);
       #if ENABLE_BOARD_DATA
       collectBoardData();
       #endif
       delay(50);
       
       // Collect BMS data after all other data
       // Note: BMS temporarily reconfigures serial, so GPS will need to be re-collected after
       collectBMSData();
       
       // Re-collect GPS data after BMS to ensure it's still valid
       // (BMS reconfigures the serial port which can disrupt GPS)
       delay(200);
       #if ENABLE_GPS_DATA
       collectGPSData();
       #endif
       
       // Only update and send if GPS is still valid after BMS collection
       if (gpsData.isValid) {
         // Update JSON data
         updateJSONData();
         
         // Send data to Supabase
         sendDataToSupabase();
       } else {
         Serial.println("GPS fix lost after BMS collection. Waiting for next cycle...");
       }
     } else {
       // GPS fix not available, update JSON with GPS status only
       Serial.println("GPS fix not available. Waiting for GPS fix before collecting other data...");
       updateJSONData();  // Update JSON to show GPS status
     }
     
     lastDataOutput = currentTime;
   }
   
   // Handle incoming serial commands (always available for debugging)
   handleSerialCommands();
   
   // Handle web server requests (always available)
   server.handleClient();
   ElegantOTA.loop();

   
   delay(100);
 }
 
 void initializeSIM7600E() {
   Serial.println("Initializing SIM7600E module...");
   
   // Clear any initial garbage data
   delay(500);
   while (SIM7600E.available()) {
     SIM7600E.read();
   }
   
   // Test AT command (verbose for initialization)
   sendATCommand("AT", 1000, true);
   delay(100);
   
   // Disable echo
   sendATCommand("ATE0", 1000, true);
   delay(100);
   
   // Get module info
   sendATCommand("ATI", 1000, true);
   delay(100);
   
   // Check SIM card
   sendATCommand("AT+CPIN?", 1000, true);
   delay(100);
   
   // Check signal strength
   sendATCommand("AT+CSQ", 1000, true);
   delay(100);
   
   // Check network operator
   sendATCommand("AT+COPS?", 1000, true);
   delay(100);
   
   // Get IMEI
   sendATCommand("AT+CGSN", 1000, true);
   delay(100);
   
   // Get ICCID
   sendATCommand("AT+CCID", 1000, true);
   delay(100);
   
   // Enable GPS
   enableGPS();
   
   Serial.println("SIM7600E initialization complete.");
 }
 
void enableGPS() {
  Serial.println("Initializing GPS...");
  
  // Clear buffer before GPS commands
  while (SIM7600E.available()) {
    SIM7600E.read();
  }
  
  // Power on GPS (AT+CGPS=1,1 - first 1 enables GPS, second 1 is mode)
  sendATCommand("AT+CGPS=1,1", 2000, true);
  
  // Wait for GPS to initialize
  delay(1000);
  
  Serial.println("GPS initialization complete. Note: First fix may take 30-60 seconds.");
}

bool waitForGPSFix(int maxWaitSeconds) {
  Serial.println("Waiting for GPS fix (max " + String(maxWaitSeconds) + " seconds)...");
  
  unsigned long startTime = millis();
  unsigned long maxWaitMs = maxWaitSeconds * 1000UL;
  unsigned long lastCheckTime = 0;
  const unsigned long checkInterval = 3000;  // Check every 3 seconds to avoid overloading
  
  while (millis() - startTime < maxWaitMs) {
    // Check GPS status periodically
    if (millis() - lastCheckTime >= checkInterval) {
      lastCheckTime = millis();
      
      // Clear buffer before collecting GPS data
      while (SIM7600E.available()) {
        SIM7600E.read();
      }
      delay(50);
      
      collectGPSData();
      
      if (gpsData.isValid) {
        Serial.println("GPS fix achieved!");
        Serial.println("  Latitude: " + gpsData.latitude);
        Serial.println("  Longitude: " + gpsData.longitude);
        Serial.println("  Fix Quality: " + gpsData.fixQuality);
        return true;
      } else {
        int elapsed = (millis() - startTime) / 1000;
        int remaining = maxWaitSeconds - elapsed;
        Serial.println("GPS fix not yet available. Elapsed: " + String(elapsed) + "s, Remaining: " + String(remaining) + "s");
      }
    }
    
    delay(100);  // Small delay to prevent busy waiting
  }
  
  Serial.println("GPS fix timeout reached after " + String(maxWaitSeconds) + " seconds.");
  return false;
}

 void initializeBoardData() {
   boardData.chipModel = ESP.getChipModel();
   boardData.chipRevision = String(ESP.getChipRevision());
   boardData.flashSize = String(ESP.getFlashChipSize() / 1024 / 1024) + " MB";
   boardData.sdkVersion = ESP.getSdkVersion();
   boardData.macAddress = WiFi.macAddress();
 }
 
 
 void connectToWiFi() {
   Serial.println("Connecting to WiFi...");
   WiFi.begin(ssid, password);
   
   int attempts = 0;
   while (WiFi.status() != WL_CONNECTED && attempts < 20) {
     delay(1000);
     Serial.print(".");
     attempts++;
   }
   
   if (WiFi.status() == WL_CONNECTED) {
     Serial.println("");
     Serial.println("WiFi connected!");
     Serial.print("IP address: ");
     Serial.println(WiFi.localIP());
     Serial.print("MAC address: ");
     Serial.println(WiFi.macAddress());
     boardData.wifiStatus = "Connected";
     boardData.localIP = WiFi.localIP().toString();
   } else {
     Serial.println("");
     Serial.println("WiFi connection failed!");
     boardData.wifiStatus = "Failed";
     boardData.localIP = "N/A";
   }
 }
 
 void setupWebServer() {
   // Serve JSON data at root path
   server.on("/", handleRoot);
   server.on("/data.json", handleJSON);
   server.on("/status", handleStatus);
   
   // Start the server
   server.begin();
   Serial.println("Web server started");
   Serial.println("Available endpoints:");
   Serial.println("  http://[ESP32_IP]/          - JSON data");
   Serial.println("  http://[ESP32_IP]/data.json - JSON data");
   Serial.println("  http://[ESP32_IP]/status    - System status");
 }
 
 void handleRoot() {
   server.send(200, "application/json", jsonData);
 }
 
 void handleJSON() {
   server.send(200, "application/json", jsonData);
 }
 
 void handleStatus() {
   String status = "{\"status\":\"running\",\"uptime\":" + String(millis()) + ",\"wifi_connected\":" + 
                   (WiFi.status() == WL_CONNECTED ? "true" : "false") + "}";
   server.send(200, "application/json", status);
 }
 
void collectGPSData() {
  // Clear any leftover data in the buffer first
  while (SIM7600E.available()) {
    SIM7600E.read();
  }
  
  // Request GPS information
  SIM7600E.println("AT+CGPSINFO");
  
  // Wait for response (using the working method from esp32_gps.ino)
  String response = waitForGPSResponse(2000);
  
  // Parse GPS data
  // Format: +CGPSINFO: lat,N/S,lon,E/W,date,time,alt,speed,course
  // Check if we have valid GPS data (not all commas)
  if (response.indexOf("+CGPSINFO:") >= 0 && response.indexOf(",,,,,,,") < 0) {
    parseGPSData(response);
  } else {
    gpsData.isValid = false;
    gpsData.latitude = "N/A";
    gpsData.longitude = "N/A";
    gpsData.altitude = "N/A";
    gpsData.speed = "N/A";
    gpsData.course = "N/A";
    gpsData.satellites = "N/A";
    gpsData.fixQuality = "0";
    gpsData.timestamp = "N/A";
  }
}
 
 void collectGSMData() {
   // Signal strength (non-verbose to avoid interference)
   String csqResponse = sendATCommand("AT+CSQ", 1000, false);
   if (csqResponse.indexOf("+CSQ:") >= 0) {
     int start = csqResponse.indexOf("+CSQ:") + 6;
     int end = csqResponse.indexOf(",", start);
     if (end > start) {
       gsmData.signalStrength = csqResponse.substring(start, end);
     }
   }
   
   // Network operator
   String copsResponse = sendATCommand("AT+COPS?", 1000, false);
   if (copsResponse.indexOf("+COPS:") >= 0) {
     int start = copsResponse.indexOf("\"") + 1;
     int end = copsResponse.indexOf("\"", start);
     if (end > start) {
       gsmData.networkOperator = copsResponse.substring(start, end);
     }
   }
   
   // IMEI
   String imeiResponse = sendATCommand("AT+CGSN", 1000, false);
   if (imeiResponse.indexOf("OK") >= 0) {
     gsmData.imei = imeiResponse.substring(0, imeiResponse.indexOf("\r"));
   }
   
   // ICCID (SIM card ID)
   String iccidResponse = sendATCommand("AT+CCID", 1000, false);
   if (iccidResponse.indexOf("+CCID:") >= 0) {
     int start = iccidResponse.indexOf("+CCID:") + 7;
     int end = iccidResponse.indexOf("\r", start);
     if (end > start) {
       gsmData.iccid = iccidResponse.substring(start, end);
     }
   }
   
   // Phone number
   String cnumResponse = sendATCommand("AT+CNUM", 1000, false);
   if (cnumResponse.indexOf("+CNUM:") >= 0) {
     int start = cnumResponse.indexOf("\"") + 1;
     int end = cnumResponse.indexOf("\"", start);
     if (end > start) {
       gsmData.phoneNumber = cnumResponse.substring(start, end);
     }
   }
   
   // Battery voltage
   String cbcResponse = sendATCommand("AT+CBC", 1000, false);
   if (cbcResponse.indexOf("+CBC:") >= 0) {
     int start = cbcResponse.indexOf("+CBC:") + 6;
     int end = cbcResponse.indexOf(",", start);
     if (end > start) {
       gsmData.batteryVoltage = cbcResponse.substring(start, end);
     }
   }
   
   // Check if connected
   gsmData.isConnected = (gsmData.signalStrength != "N/A" && gsmData.signalStrength.toInt() > 0);
 }
 
 void collectBoardData() {
   boardData.cpuFrequency = ESP.getCpuFreqMHz();
   boardData.freeHeap = ESP.getFreeHeap();
   boardData.totalHeap = ESP.getHeapSize();
   boardData.uptime = millis();
   
   // Read battery voltage from ADC
   boardData.batteryVoltage = readBatteryVoltage();
   
   // Read temperature
   boardData.temperature = temperatureRead();
   
   // WiFi RSSI (if connected)
   if (WiFi.status() == WL_CONNECTED) {
     boardData.wifiRSSI = WiFi.RSSI();
     boardData.wifiStatus = "Connected";
     boardData.localIP = WiFi.localIP().toString();
   } else {
     boardData.wifiRSSI = 0;
     boardData.wifiStatus = "Disconnected";
     boardData.localIP = "N/A";
   }
 }
 
 
 float readBatteryVoltage() {
   // Read from VBAT pin (GPIO34)
   int adcValue = analogRead(VBAT_ADC_PIN);
   // Convert ADC value to voltage (assuming 3.3V reference and voltage divider)
   float voltage = (adcValue * 3.3 * 2.0) / 4095.0; // 2.0 is voltage divider ratio
   return voltage;
 }
 
String waitForGPSResponse(int timeout) {
  String response = "";
  unsigned long startTime = millis();
  
  while (millis() - startTime < timeout) {
    while (SIM7600E.available()) {
      char c = SIM7600E.read();
      response += c;
    }
    
    // Check for end of response
    if (response.indexOf("OK") >= 0 || response.indexOf("ERROR") >= 0) {
      break;
    }
  }
  
  return response;
}

void parseGPSData(String data) {
  int startIdx = data.indexOf("+CGPSINFO:") + 11;
  int endIdx = data.indexOf("OK", startIdx);
  
  if (startIdx < 11 || endIdx < 0) {
    gpsData.isValid = false;
    return;
  }
  
  String gpsInfo = data.substring(startIdx, endIdx);
  gpsInfo.trim();
  
  // Check if GPS has valid fix
  if (gpsInfo.length() < 10 || gpsInfo.indexOf(",,,") >= 0) {
    gpsData.isValid = false;
    gpsData.latitude = "N/A";
    gpsData.longitude = "N/A";
    gpsData.altitude = "N/A";
    gpsData.speed = "N/A";
    gpsData.course = "N/A";
    gpsData.satellites = "N/A";
    gpsData.fixQuality = "0";
    gpsData.timestamp = "N/A";
    return;
  }
  
  // Parse GPS fields
  int fieldIndex = 0;
  int lastComma = -1;
  String fields[9];
  
  for (int i = 0; i < gpsInfo.length(); i++) {
    if (gpsInfo[i] == ',' || i == gpsInfo.length() - 1) {
      if (i == gpsInfo.length() - 1) i++;
      fields[fieldIndex++] = gpsInfo.substring(lastComma + 1, i);
      lastComma = i;
      if (fieldIndex >= 9) break;
    }
  }
  
  // Convert latitude (ddmm.mmmm to decimal degrees)
  if (fields[0].length() > 0) {
    float lat = fields[0].toFloat();
    int degrees = (int)(lat / 100);
    float minutes = lat - (degrees * 100);
    float latitude = degrees + (minutes / 60.0);
    if (fields[1] == "S") latitude = -latitude;
    gpsData.latitude = String(latitude, 6);
  } else {
    gpsData.latitude = "N/A";
  }
  
  // Convert longitude (dddmm.mmmm to decimal degrees)
  if (fields[2].length() > 0) {
    float lon = fields[2].toFloat();
    int degrees = (int)(lon / 100);
    float minutes = lon - (degrees * 100);
    float longitude = degrees + (minutes / 60.0);
    if (fields[3] == "W") longitude = -longitude;
    gpsData.longitude = String(longitude, 6);
  } else {
    gpsData.longitude = "N/A";
  }
  
  // Date and time
  if (fields[4].length() > 0 && fields[5].length() > 0) {
    gpsData.timestamp = fields[4] + "T" + fields[5] + "Z";
  } else {
    gpsData.timestamp = "N/A";
  }
  
  // Altitude
  if (fields[6].length() > 0) {
    gpsData.altitude = fields[6];
  } else {
    gpsData.altitude = "N/A";
  }
  
  // Speed
  if (fields[7].length() > 0) {
    gpsData.speed = fields[7];
  } else {
    gpsData.speed = "N/A";
  }
  
  // Course
  if (fields[8].length() > 0) {
    gpsData.course = fields[8];
  } else {
    gpsData.course = "N/A";
  }
  
  gpsData.satellites = "Unknown";
  gpsData.fixQuality = "1";
  gpsData.isValid = true;
}
 
 
 
 String sendATCommand(String command, int timeout, bool verbose) {
   if (verbose) {
     Serial.println("Sending: " + command);
   }
   
   // Clear any leftover data in the buffer
   while (SIM7600E.available()) {
     SIM7600E.read();
   }
   
   SIM7600E.println(command);
   
   String response = "";
   unsigned long startTime = millis();
   
   while (millis() - startTime < timeout) {
     while (SIM7600E.available()) {
       char c = SIM7600E.read();
       response += c;
     }
     
     if (response.indexOf("OK") >= 0 || response.indexOf("ERROR") >= 0) {
       break;
     }
   }
   
   if (verbose) {
     Serial.println("Response: " + response);
   }
   return response;
 }
 
 double convertToDecimalDegrees(String coord, String direction) {
   if (coord.length() == 0) return 0.0;
   
   // Format: DDMM.MMMM for latitude, DDDMM.MMMM for longitude
   double value = coord.toDouble();
   
   // Extract degrees and minutes
   int degrees = (int)(value / 100);
   double minutes = value - (degrees * 100);
   
   // Convert to decimal degrees
   double decimal = degrees + (minutes / 60.0);
   
   // Apply direction
   if (direction == "S" || direction == "W") {
     decimal = -decimal;
   }
   
   return decimal;
 }
 
 void updateJSONData() {
   // Create JSON document
   DynamicJsonDocument doc(2048);
   
   // GPS Data
   #if ENABLE_GPS_DATA
   JsonObject gps = doc.createNestedObject("gps");
   gps["latitude"] = gpsData.latitude;
   gps["longitude"] = gpsData.longitude;
   gps["altitude"] = gpsData.altitude;
   gps["speed"] = gpsData.speed;
   gps["course"] = gpsData.course;
   gps["satellites"] = gpsData.satellites;
   gps["fix_quality"] = gpsData.fixQuality;
   gps["timestamp"] = gpsData.timestamp;
   gps["is_valid"] = gpsData.isValid;
   #endif
   
   // GSM Data
   #if ENABLE_GSM_DATA
   JsonObject gsm = doc.createNestedObject("gsm");
   gsm["signal_strength"] = gsmData.signalStrength;
   gsm["network_operator"] = gsmData.networkOperator;
   gsm["network_type"] = gsmData.networkType;
   gsm["imei"] = gsmData.imei;
   gsm["iccid"] = gsmData.iccid;
   gsm["phone_number"] = gsmData.phoneNumber;
   gsm["battery_voltage"] = gsmData.batteryVoltage;
   gsm["temperature"] = gsmData.temperature;
   gsm["is_connected"] = gsmData.isConnected;
   #endif
   
   // Board Data
   #if ENABLE_BOARD_DATA
   JsonObject board = doc.createNestedObject("board");
   board["cpu_frequency_mhz"] = boardData.cpuFrequency;
   board["free_heap_bytes"] = boardData.freeHeap;
   board["total_heap_bytes"] = boardData.totalHeap;
   board["battery_voltage"] = boardData.batteryVoltage;
   board["temperature_celsius"] = boardData.temperature;
   board["chip_model"] = boardData.chipModel;
   board["chip_revision"] = boardData.chipRevision;
   board["flash_size"] = boardData.flashSize;
   board["sdk_version"] = boardData.sdkVersion;
   board["uptime_ms"] = boardData.uptime;
   board["wifi_rssi"] = boardData.wifiRSSI;
   board["wifi_status"] = boardData.wifiStatus;
   board["local_ip"] = boardData.localIP;
   board["mac_address"] = boardData.macAddress;
   #endif
   
   // BMS Data (using original fetch_bms_data.ino structure)
   JsonObject bms_data = doc.createNestedObject("bms_data");

   if (bmsData.basic_status_parsed) {
     bms_data["pack_voltage"] = bmsData.cumulative_voltage;
     bms_data["pack_current"] = bmsData.current;
     bms_data["state_of_charge"] = bmsData.soc;
   }
   
   if (bmsData.voltage_limits_parsed) {
     bms_data["max_cell_voltage"] = bmsData.max_cell_voltage;
     bms_data["max_cell_number"] = bmsData.max_cell_number;
     bms_data["min_cell_voltage"] = bmsData.min_cell_voltage;
     bms_data["min_cell_number"] = bmsData.min_cell_number;
   }
   
   if (bmsData.temperature_parsed) {
     bms_data["max_temp"] = bmsData.max_temp;
     bms_data["max_temp_cell"] = bmsData.max_temp_cell;
     bms_data["min_temp"] = bmsData.min_temp;
     bms_data["min_temp_cell"] = bmsData.min_temp_cell;
   }
   
   if (bmsData.mosfet_status_parsed) {
     bms_data["charge_state"] = bmsData.charge_state;
     bms_data["charge_mos"] = bmsData.charge_mos;
     bms_data["discharge_mos"] = bmsData.discharge_mos;
     bms_data["bms_cycles"] = bmsData.bms_cycles;
     bms_data["remaining_capacity"] = bmsData.remaining_capacity;
   }

   if (bmsData.system_info_parsed) {
     bms_data["num_cells"] = bmsData.num_cells;
     bms_data["num_temp_sensors"] = bmsData.num_temps;
     bms_data["charger_connected"] = bmsData.charger_connected;
     bms_data["load_connected"] = bmsData.load_connected;
   }
   
   // Cell voltages array
   if (bmsData.cell_voltages_parsed) {
     JsonArray cellVoltages = bms_data.createNestedArray("cell_voltages");
     for (int i = 0; i < bmsData.cell_voltage_count; i++) {
       cellVoltages.add(bmsData.cell_voltages[i]);
     }
   }
   
   // Average temperature calculation (as in original fetch_bms_data.ino)
   if (bmsData.cell_temperatures_parsed && bmsData.cell_temp_count > 0) {
     float totalTemp = 0;
     for (int i = 0; i < bmsData.cell_temp_count; i++) {
       totalTemp += bmsData.cell_temperatures[i];
     }
     float avgTemp = totalTemp / bmsData.cell_temp_count;
     bms_data["average_temperature"] = avgTemp;
   } else if (bmsData.cell_temperatures_parsed) {
     bms_data["average_temperature"] = nullptr;
   }

   if (bmsData.cell_balance_states_parsed) {
     JsonArray cellBalanceStates = bms_data.createNestedArray("cell_balance_states");
     for (int i = 0; i < 48; i++) {
       cellBalanceStates.add(bmsData.cell_balance_states[i]);
     }
   }

   if (bmsData.fault_status_parsed) {
     bms_data["has_faults"] = bmsData.has_faults;
     bms_data["fault_code"] = bmsData.fault_code;
   }
   
   // System info
   doc["timestamp"] = millis();
   doc["system_status"] = "running";
   doc["esp32_ip"] = WiFi.localIP().toString();
   
   // Update global JSON string
   jsonData = "";
   serializeJson(doc, jsonData);
   
   // Output to serial for debugging
   Serial.println("=== ESP32 SIM7600E DATA ===");
   serializeJsonPretty(doc, Serial);
   Serial.println("\n=== END DATA ===");
   
   // Toggle status LED
   digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
 }
 
 
 
 void handleSerialCommands() {
   if (Serial.available()) {
     String command = Serial.readStringUntil('\n');
     command.trim();
     
     if (command == "DATA") {
       updateJSONData();
     } else if (command == "GPS") {
       collectGPSData();
       updateJSONData();
     } else if (command == "GSM") {
       collectGSMData();
       updateJSONData();
     } else if (command == "BOARD") {
       collectBoardData();
       updateJSONData();
     } else if (command == "BMS") {
       collectBMSData();
       updateJSONData();
     } else if (command == "RESET") {
       ESP.restart();
     } else if (command == "IP") {
       Serial.println("ESP32 IP: " + WiFi.localIP().toString());
       Serial.println("Access JSON at: http://" + WiFi.localIP().toString());
     } else if (command.startsWith("AT")) {
       String response = sendATCommand(command, 1000, true);
       Serial.println("Response: " + response);
     } else {
       Serial.println("Available commands: DATA, GPS, GSM, BOARD, BMS, RESET, IP, AT<command>");
     }
   }
 }
 
 // BMS Functions Implementation
 void initializeBMS() {
   Serial.println("Initializing BMS RS-485 communication...");
   
   // Configure DE/RE pin as output
   pinMode(RS485_DE_RE_PIN, OUTPUT);
   
   // Set to receive mode initially (DE/RE = LOW)
   digitalWrite(RS485_DE_RE_PIN, LOW);
   
   Serial.println("BMS RS-485 initialized");
 }
 
void collectBMSData() {
  Serial.println("Collecting BMS data...");
  
  // Temporarily reconfigure Serial2 for BMS communication
  SIM7600E.end();
  delay(100);
  SIM7600E.begin(9600, SERIAL_8N1, RS485_RO_PIN, RS485_DI_PIN);
  
  uint8_t responseBuffer[256];
  int responseLength = 0;
  unsigned long responseStart = 0;
  bool parsed = false;
  
  // Basic Status (0x90)
  #if ENABLE_BMS_BASIC_STATUS
    Serial.println("  BMS Command: Basic Status (0x90)");
    sendBMSCommand(0x90);
    responseLength = 0;
    responseStart = millis();
    while (millis() - responseStart < BMS_RESPONSE_TIMEOUT_MS && responseLength < 256) {
      if (SIM7600E.available()) { responseBuffer[responseLength++] = SIM7600E.read(); responseStart = millis(); }
      delay(1);
    }
    if (responseLength > 0) {
      int frameStart = -1;
      for (int j = 0; j < responseLength - 1; j++) { if (responseBuffer[j] == 0xA5 && responseBuffer[j + 1] == 0x01) { frameStart = j; break; } }
      parsed = false;
      if (frameStart >= 0 && (responseLength - frameStart) >= 13) { parsed = parseBasicStatus(&responseBuffer[frameStart], responseLength - frameStart); }
      else if (responseLength >= 13) { parsed = parseBasicStatus(&responseBuffer[13], responseLength - 13); }
      if (parsed) { Serial.println("    ✓ BMS data parsed successfully"); bmsData.basic_status_parsed = true; }
      else { Serial.println("    ✗ BMS data parse failed"); bmsData.basic_status_parsed = false; }
    } else { Serial.println("    ✗ No BMS response received"); bmsData.basic_status_parsed = false; }
    delay(100);
  #else
    bmsData.basic_status_parsed = false;
  #endif

  // Voltage Limits (0x91)
  #if ENABLE_BMS_VOLTAGE_LIMITS
    Serial.println("  BMS Command: Voltage Limits (0x91)");
    sendBMSCommand(0x91);
    responseLength = 0;
    responseStart = millis();
    while (millis() - responseStart < BMS_RESPONSE_TIMEOUT_MS && responseLength < 256) {
      if (SIM7600E.available()) { responseBuffer[responseLength++] = SIM7600E.read(); responseStart = millis(); }
      delay(1);
    }
    if (responseLength > 0) {
      int frameStart = -1;
      for (int j = 0; j < responseLength - 1; j++) { if (responseBuffer[j] == 0xA5 && responseBuffer[j + 1] == 0x01) { frameStart = j; break; } }
      parsed = false;
      if (frameStart >= 0 && (responseLength - frameStart) >= 13) { parsed = parseVoltageLimits(&responseBuffer[frameStart], responseLength - frameStart); }
      else if (responseLength >= 13) { parsed = parseVoltageLimits(&responseBuffer[13], responseLength - 13); }
      if (parsed) { Serial.println("    ✓ BMS data parsed successfully"); bmsData.voltage_limits_parsed = true; }
      else { Serial.println("    ✗ BMS data parse failed"); bmsData.voltage_limits_parsed = false; }
    } else { Serial.println("    ✗ No BMS response received"); bmsData.voltage_limits_parsed = false; }
    delay(100);
  #else
    bmsData.voltage_limits_parsed = false;
  #endif

  // Temperature (0x92)
  #if ENABLE_BMS_TEMPERATURE
    Serial.println("  BMS Command: Temperature (0x92)");
    sendBMSCommand(0x92);
    responseLength = 0;
    responseStart = millis();
    while (millis() - responseStart < BMS_RESPONSE_TIMEOUT_MS && responseLength < 256) {
      if (SIM7600E.available()) { responseBuffer[responseLength++] = SIM7600E.read(); responseStart = millis(); }
      delay(1);
    }
    if (responseLength > 0) {
      int frameStart = -1;
      for (int j = 0; j < responseLength - 1; j++) { if (responseBuffer[j] == 0xA5 && responseBuffer[j + 1] == 0x01) { frameStart = j; break; } }
      parsed = false;
      if (frameStart >= 0 && (responseLength - frameStart) >= 13) { parsed = parseTemperature(&responseBuffer[frameStart], responseLength - frameStart); }
      else if (responseLength >= 13) { parsed = parseTemperature(&responseBuffer[13], responseLength - 13); }
      if (parsed) { Serial.println("    ✓ BMS data parsed successfully"); bmsData.temperature_parsed = true; }
      else { Serial.println("    ✗ BMS data parse failed"); bmsData.temperature_parsed = false; }
    } else { Serial.println("    ✗ No BMS response received"); bmsData.temperature_parsed = false; }
    delay(100);
  #else
    bmsData.temperature_parsed = false;
  #endif

  // MOSFET Status (0x93)
  #if ENABLE_BMS_MOSFET_STATUS
    Serial.println("  BMS Command: MOSFET Status (0x93)");
    sendBMSCommand(0x93);
    responseLength = 0;
    responseStart = millis();
    while (millis() - responseStart < BMS_RESPONSE_TIMEOUT_MS && responseLength < 256) {
      if (SIM7600E.available()) { responseBuffer[responseLength++] = SIM7600E.read(); responseStart = millis(); }
      delay(1);
    }
    if (responseLength > 0) {
      int frameStart = -1;
      for (int j = 0; j < responseLength - 1; j++) { if (responseBuffer[j] == 0xA5 && responseBuffer[j + 1] == 0x01) { frameStart = j; break; } }
      parsed = false;
      if (frameStart >= 0 && (responseLength - frameStart) >= 13) { parsed = parseMosfetStatus(&responseBuffer[frameStart], responseLength - frameStart); }
      else if (responseLength >= 13) { parsed = parseMosfetStatus(&responseBuffer[13], responseLength - 13); }
      if (parsed) { Serial.println("    ✓ BMS data parsed successfully"); bmsData.mosfet_status_parsed = true; }
      else { Serial.println("    ✗ BMS data parse failed"); bmsData.mosfet_status_parsed = false; }
    } else { Serial.println("    ✗ No BMS response received"); bmsData.mosfet_status_parsed = false; }
    delay(100);
  #else
    bmsData.mosfet_status_parsed = false;
  #endif

  // System Info (0x94)
  #if ENABLE_BMS_SYSTEM_INFO
    Serial.println("  BMS Command: System Info (0x94)");
    sendBMSCommand(0x94);
    responseLength = 0;
    responseStart = millis();
    while (millis() - responseStart < BMS_RESPONSE_TIMEOUT_MS && responseLength < 256) {
      if (SIM7600E.available()) { responseBuffer[responseLength++] = SIM7600E.read(); responseStart = millis(); }
      delay(1);
    }
    if (responseLength > 0) {
      int frameStart = -1;
      for (int j = 0; j < responseLength - 1; j++) { if (responseBuffer[j] == 0xA5 && responseBuffer[j + 1] == 0x01) { frameStart = j; break; } }
      parsed = false;
      if (frameStart >= 0 && (responseLength - frameStart) >= 13) { parsed = parseSystemInfo(&responseBuffer[frameStart], responseLength - frameStart); }
      else if (responseLength >= 13) { parsed = parseSystemInfo(&responseBuffer[13], responseLength - 13); }
      if (parsed) { Serial.println("    ✓ BMS data parsed successfully"); bmsData.system_info_parsed = true; }
      else { Serial.println("    ✗ BMS data parse failed"); bmsData.system_info_parsed = false; }
    } else { Serial.println("    ✗ No BMS response received"); bmsData.system_info_parsed = false; }
    delay(100);
  #else
    bmsData.system_info_parsed = false;
  #endif

  // Cell Voltages (0x95)
  #if ENABLE_BMS_CELL_VOLTAGES
    Serial.println("  BMS Command: Cell Voltages (0x95)");
    sendBMSCommand(0x95);
    responseLength = 0;
    responseStart = millis();
    while (millis() - responseStart < BMS_RESPONSE_TIMEOUT_MS && responseLength < 256) {
      if (SIM7600E.available()) { responseBuffer[responseLength++] = SIM7600E.read(); responseStart = millis(); }
      delay(1);
    }
    if (responseLength > 0) {
      int frameStart = -1;
      for (int j = 0; j < responseLength - 1; j++) { if (responseBuffer[j] == 0xA5 && responseBuffer[j + 1] == 0x01) { frameStart = j; break; } }
      parsed = false;
      if (frameStart >= 0 && (responseLength - frameStart) >= 13) { parsed = parseCellVoltages(&responseBuffer[frameStart], responseLength - frameStart); }
      else if (responseLength >= 13) { parsed = parseCellVoltages(&responseBuffer[13], responseLength - 13); }
      if (parsed) { Serial.println("    ✓ BMS data parsed successfully"); bmsData.cell_voltages_parsed = true; }
      else { Serial.println("    ✗ BMS data parse failed"); bmsData.cell_voltages_parsed = false; }
    } else { Serial.println("    ✗ No BMS response received"); bmsData.cell_voltages_parsed = false; }
    delay(100);
  #else
    bmsData.cell_voltages_parsed = false;
  #endif

  // Cell Temperatures (0x96)
  #if ENABLE_BMS_CELL_TEMPERATURES
    Serial.println("  BMS Command: Cell Temperatures (0x96)");
    sendBMSCommand(0x96);
    responseLength = 0;
    responseStart = millis();
    while (millis() - responseStart < BMS_RESPONSE_TIMEOUT_MS && responseLength < 256) {
      if (SIM7600E.available()) { responseBuffer[responseLength++] = SIM7600E.read(); responseStart = millis(); }
      delay(1);
    }
    if (responseLength > 0) {
      int frameStart = -1;
      for (int j = 0; j < responseLength - 1; j++) { if (responseBuffer[j] == 0xA5 && responseBuffer[j + 1] == 0x01) { frameStart = j; break; } }
      parsed = false;
      if (frameStart >= 0 && (responseLength - frameStart) >= 13) { parsed = parseCellTemperatures(&responseBuffer[frameStart], responseLength - frameStart); }
      else if (responseLength >= 13) { parsed = parseCellTemperatures(&responseBuffer[13], responseLength - 13); }
      if (parsed) { Serial.println("    ✓ BMS data parsed successfully"); bmsData.cell_temperatures_parsed = true; }
      else { Serial.println("    ✗ BMS data parse failed"); bmsData.cell_temperatures_parsed = false; }
    } else { Serial.println("    ✗ No BMS response received"); bmsData.cell_temperatures_parsed = false; }
    delay(100);
  #else
    bmsData.cell_temperatures_parsed = false;
  #endif

  // Cell Balance States (0x97)
  #if ENABLE_BMS_CELL_BALANCE
    Serial.println("  BMS Command: Cell Balance States (0x97)");
    sendBMSCommand(0x97);
    responseLength = 0;
    responseStart = millis();
    while (millis() - responseStart < BMS_RESPONSE_TIMEOUT_MS && responseLength < 256) {
      if (SIM7600E.available()) { responseBuffer[responseLength++] = SIM7600E.read(); responseStart = millis(); }
      delay(1);
    }
    if (responseLength > 0) {
      int frameStart = -1;
      for (int j = 0; j < responseLength - 1; j++) { if (responseBuffer[j] == 0xA5 && responseBuffer[j + 1] == 0x01) { frameStart = j; break; } }
      parsed = false;
      if (frameStart >= 0 && (responseLength - frameStart) >= 13) { parsed = parseCellBalanceStates(&responseBuffer[frameStart], responseLength - frameStart); }
      else if (responseLength >= 13) { parsed = parseCellBalanceStates(&responseBuffer[13], responseLength - 13); }
      if (parsed) { Serial.println("    ✓ BMS data parsed successfully"); bmsData.cell_balance_states_parsed = true; }
      else { Serial.println("    ✗ BMS data parse failed"); bmsData.cell_balance_states_parsed = false; }
    } else { Serial.println("    ✗ No BMS response received"); bmsData.cell_balance_states_parsed = false; }
    delay(100);
  #else
    bmsData.cell_balance_states_parsed = false;
  #endif

  // Fault Status (0x98)
  #if ENABLE_BMS_FAULT_STATUS
    Serial.println("  BMS Command: Fault Status (0x98)");
    sendBMSCommand(0x98);
    responseLength = 0;
    responseStart = millis();
    while (millis() - responseStart < BMS_RESPONSE_TIMEOUT_MS && responseLength < 256) {
      if (SIM7600E.available()) { responseBuffer[responseLength++] = SIM7600E.read(); responseStart = millis(); }
      delay(1);
    }
    if (responseLength > 0) {
      int frameStart = -1;
      for (int j = 0; j < responseLength - 1; j++) { if (responseBuffer[j] == 0xA5 && responseBuffer[j + 1] == 0x01) { frameStart = j; break; } }
      parsed = false;
      if (frameStart >= 0 && (responseLength - frameStart) >= 13) { parsed = parseFaultStatus(&responseBuffer[frameStart], responseLength - frameStart); }
      else if (responseLength >= 13) { parsed = parseFaultStatus(&responseBuffer[13], responseLength - 13); }
      if (parsed) { Serial.println("    ✓ BMS data parsed successfully"); bmsData.fault_status_parsed = true; }
      else { Serial.println("    ✗ BMS data parse failed"); bmsData.fault_status_parsed = false; }
    } else { Serial.println("    ✗ No BMS response received"); bmsData.fault_status_parsed = false; }
    delay(100);
  #else
    bmsData.fault_status_parsed = false;
  #endif
  
  // Restore Serial2 for SIM7600E communication
  SIM7600E.end();
  delay(200);  // Longer delay to ensure port is fully closed
  
  // Reinitialize SIM7600E serial port
  SIM7600E.begin(115200, SERIAL_8N1, SIM7600E_RX_PIN, SIM7600E_TX_PIN);
  delay(100);
  
  // Clear any garbage data from buffer
  unsigned long clearStart = millis();
  while (millis() - clearStart < 500) {
    while (SIM7600E.available()) {
      SIM7600E.read();
    }
    delay(10);
  }
  
  // Send a test AT command to ensure communication is restored
  sendATCommand("AT", 1000, false);
  delay(100);
  
  Serial.println("BMS data collection complete. SIM7600E serial restored.");
}
 
void sendBMSCommand(uint8_t command) {
  uint8_t frame[13];
  frame[0] = 0xA5;
  frame[1] = 0x40;
  frame[2] = command;
  frame[3] = 0x08;
  frame[4] = 0x00;
  frame[5] = 0x00;
  frame[6] = 0x00;
  frame[7] = 0x00;
  frame[8] = 0x00;
  frame[9] = 0x00;
  frame[10] = 0x00;
  frame[11] = 0x00;
  
  // Calculate checksum
  uint8_t checksum = 0;
  for (int i = 0; i < 12; i++) {
    checksum += frame[i];
  }
  frame[12] = checksum;
  
  // Switch to transmit mode
  setTransmitMode();
  delay(10);
  
  // Send the frame
  for (int i = 0; i < 13; i++) {
    SIM7600E.write(frame[i]);
  }
  SIM7600E.flush();
  
  // Switch back to receive mode
  setReceiveMode();
  delay(100); // Delay after setting receive mode (as per updated code)
}
 
 bool parseBasicStatus(uint8_t* data, int length) {
   if (length < 13 || data[2] != 0x90) return false;
   
   bmsData.cumulative_voltage = ((data[4] << 8) | data[5]) / 10.0;
   bmsData.gather_voltage = ((data[6] << 8) | data[7]) / 10.0;
   bmsData.current = (((data[8] << 8) | data[9]) - 30000) / 10.0;
   bmsData.soc = ((data[10] << 8) | data[11]) / 10.0;
   
   return true;
 }
 
 bool parseVoltageLimits(uint8_t* data, int length) {
   if (length < 13 || data[2] != 0x91) return false;
   
   bmsData.max_cell_voltage = ((data[4] << 8) | data[5]) / 1000.0;
   bmsData.max_cell_number = data[6];
   bmsData.min_cell_voltage = ((data[7] << 8) | data[8]) / 1000.0;
   bmsData.min_cell_number = data[9];
   
   return true;
 }
 
 bool parseTemperature(uint8_t* data, int length) {
   if (length < 13 || data[2] != 0x92) return false;
   
   bmsData.max_temp = data[4] - 40;
   bmsData.max_temp_cell = data[5];
   bmsData.min_temp = data[6] - 40;
   bmsData.min_temp_cell = data[7];
   
   return true;
 }
 
 bool parseMosfetStatus(uint8_t* data, int length) {
   if (length < 13 || data[2] != 0x93) return false;
   
   switch (data[4]) {
     case 0: bmsData.charge_state = "stationary"; break;
     case 1: bmsData.charge_state = "charging"; break;
     case 2: bmsData.charge_state = "discharging"; break;
     default: bmsData.charge_state = "unknown"; break;
   }
   
   bmsData.charge_mos = (data[5] != 0);
   bmsData.discharge_mos = (data[6] != 0);
   bmsData.bms_cycles = data[7];
   bmsData.remaining_capacity = ((data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11]) / 1000.0;
   
   return true;
 }
 
 bool parseSystemInfo(uint8_t* data, int length) {
   if (length < 13 || data[2] != 0x94) return false;
   
   bmsData.num_cells = data[4];
   bmsData.num_temps = data[5];
   bmsData.charger_connected = (data[6] != 0);
   bmsData.load_connected = (data[7] != 0);
   
   return true;
 }
 
 bool parseCellVoltages(uint8_t* data, int length) {
   if (length < 13 || data[2] != 0x95) return false;
   
   int frameOffset = 0;
   bmsData.cell_voltage_count = 0;
   
   while (frameOffset + 13 <= length && bmsData.cell_voltage_count < 48) {
     if (data[frameOffset] == 0xA5 && data[frameOffset + 1] == 0x01 && data[frameOffset + 2] == 0x95) {
       uint8_t frameNum = data[frameOffset + 4];
       if (frameNum == 0xFF) break;
       
       // Parse 3 cell voltages from this frame
       for (int i = 0; i < 3 && bmsData.cell_voltage_count < 48; i++) {
         if (frameOffset + 5 + i*2 + 1 < length) {
           bmsData.cell_voltages[bmsData.cell_voltage_count] = 
             ((data[frameOffset + 5 + i*2] << 8) | data[frameOffset + 6 + i*2]) / 1000.0;
           bmsData.cell_voltage_count++;
         }
       }
       frameOffset += 13;
     } else {
       break;
     }
   }
   
   return true;
 }
 
 bool parseCellTemperatures(uint8_t* data, int length) {
   if (length < 13 || data[2] != 0x96) return false;
   
   int frameOffset = 0;
   bmsData.cell_temp_count = 0;
   
   while (frameOffset + 13 <= length && bmsData.cell_temp_count < 16) {
     if (data[frameOffset] == 0xA5 && data[frameOffset + 1] == 0x01 && data[frameOffset + 2] == 0x96) {
       // Parse temperatures from this frame
       for (int i = 0; i < 7 && bmsData.cell_temp_count < 16; i++) {
         if (frameOffset + 5 + i < length) {
           uint8_t tempByte = data[frameOffset + 5 + i];
           if (tempByte != 0xFF) {
             bmsData.cell_temperatures[bmsData.cell_temp_count] = tempByte - 40;
             bmsData.cell_temp_count++;
           }
         }
       }
       frameOffset += 13;
     } else {
       break;
     }
   }
   
   return true;
 }
 
 bool parseCellBalanceStates(uint8_t* data, int length) {
   if (length < 13 || data[2] != 0x97) return false;
   
   // Parse balance states from 6 bytes (48 bits for 48 cells)
   for (int byteIdx = 0; byteIdx < 6; byteIdx++) {
     if (4 + byteIdx < length) {
       uint8_t byteVal = data[4 + byteIdx];
       for (int bit = 0; bit < 8; bit++) {
         int cellNum = byteIdx * 8 + bit;
         if (cellNum < 48) {
           bmsData.cell_balance_states[cellNum] = (byteVal & (1 << bit)) != 0;
         }
       }
     }
   }
   
   return true;
 }
 
 bool parseFaultStatus(uint8_t* data, int length) {
   if (length < 13 || data[2] != 0x98) return false;
   
   bmsData.fault_code = data[11];
   bmsData.has_faults = (bmsData.fault_code != 0);
   
   return true;
 }
 
 void setTransmitMode() {
   digitalWrite(RS485_DE_RE_PIN, HIGH); // Enable driver, disable receiver
 }
 
void setReceiveMode() {
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Disable driver, enable receiver
}

// Supabase Functions
void initializeSupabase() {
  Serial.println("Initializing Supabase connection...");
  
  // Initialize Supabase with URL and API key
  db.begin(supabase_url, supabase_key);
  
  Serial.println("Supabase initialized successfully!");
  Serial.println("Data will be logged to table: " + String(supabase_table));
}

void sendDataToSupabase() {
  // Only send data if GPS fix is valid
  if (!gpsData.isValid) {
    Serial.println("GPS fix not available, skipping Supabase upload");
    return;
  }
  
  // Only send data if WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping Supabase upload");
    return;
  }
  
  // Create JSON document for Supabase with individual columns
  DynamicJsonDocument doc(6144);
  
  // Basic metadata
  doc["timestamp"] = millis();
  doc["system_status"] = "running";
  doc["esp32_ip"] = WiFi.localIP().toString();
  
  // GPS Data - individual columns
  #if ENABLE_GPS_DATA
  if (gpsData.isValid) {
    doc["gps_latitude"] = gpsData.latitude.toFloat();
    doc["gps_longitude"] = gpsData.longitude.toFloat();
    doc["gps_altitude"] = gpsData.altitude.toFloat();
    doc["gps_speed"] = gpsData.speed.toFloat();
    doc["gps_course"] = gpsData.course.toFloat();
    doc["gps_satellites"] = gpsData.satellites.toInt();
    doc["gps_fix_quality"] = gpsData.fixQuality.toInt();
    doc["gps_timestamp"] = gpsData.timestamp;
    doc["gps_is_valid"] = gpsData.isValid;
  }
  #endif
  
  // GSM Data - individual columns
  #if ENABLE_GSM_DATA
  if (gsmData.signalStrength != "N/A") {
    doc["gsm_signal_strength"] = gsmData.signalStrength.toInt();
  }
  if (gsmData.networkOperator != "N/A") {
    doc["gsm_network_operator"] = gsmData.networkOperator;
  }
  if (gsmData.networkType != "N/A") {
    doc["gsm_network_type"] = gsmData.networkType;
  }
  if (gsmData.imei != "N/A") {
    doc["gsm_imei"] = gsmData.imei;
  }
  if (gsmData.iccid != "N/A") {
    doc["gsm_iccid"] = gsmData.iccid;
  }
  if (gsmData.phoneNumber != "N/A") {
    doc["gsm_phone_number"] = gsmData.phoneNumber;
  }
  if (gsmData.batteryVoltage != "N/A") {
    doc["gsm_battery_voltage"] = gsmData.batteryVoltage.toFloat();
  }
  if (gsmData.temperature != "N/A") {
    doc["gsm_temperature"] = gsmData.temperature.toFloat();
  }
  doc["gsm_is_connected"] = gsmData.isConnected;
  #endif
  
  // Board/ESP32 Data - individual columns
  #if ENABLE_BOARD_DATA
  doc["board_cpu_frequency_mhz"] = boardData.cpuFrequency;
  doc["board_free_heap_bytes"] = boardData.freeHeap;
  doc["board_total_heap_bytes"] = boardData.totalHeap;
  doc["board_battery_voltage"] = boardData.batteryVoltage;
  doc["board_temperature_celsius"] = boardData.temperature;
  doc["board_chip_model"] = boardData.chipModel;
  doc["board_chip_revision"] = boardData.chipRevision;
  doc["board_flash_size"] = boardData.flashSize;
  doc["board_sdk_version"] = boardData.sdkVersion;
  doc["board_uptime_ms"] = boardData.uptime;
  doc["board_wifi_rssi"] = boardData.wifiRSSI;
  doc["board_wifi_status"] = boardData.wifiStatus;
  doc["board_local_ip"] = boardData.localIP;
  doc["board_mac_address"] = boardData.macAddress;
  #endif
  
  // BMS Basic Data - individual columns
  if (bmsData.basic_status_parsed) {
    doc["bms_pack_voltage"] = bmsData.cumulative_voltage;
    doc["bms_pack_current"] = bmsData.current;
    doc["bms_state_of_charge"] = bmsData.soc;
  }

  // BMS Voltage Data
  if (bmsData.voltage_limits_parsed) {
    doc["bms_max_cell_voltage"] = bmsData.max_cell_voltage;
    doc["bms_min_cell_voltage"] = bmsData.min_cell_voltage;
    doc["bms_max_cell_number"] = bmsData.max_cell_number;
    doc["bms_min_cell_number"] = bmsData.min_cell_number;
  }
  
  // BMS Temperature Data
  if (bmsData.temperature_parsed) {
    doc["bms_max_temp"] = bmsData.max_temp;
    doc["bms_min_temp"] = bmsData.min_temp;
    doc["bms_max_temp_cell"] = bmsData.max_temp_cell;
    doc["bms_min_temp_cell"] = bmsData.min_temp_cell;
  }

  // Calculate and add average temperature
  if (bmsData.cell_temperatures_parsed && bmsData.cell_temp_count > 0) {
    float totalTemp = 0;
    for (int i = 0; i < bmsData.cell_temp_count; i++) {
      totalTemp += bmsData.cell_temperatures[i];
    }
    doc["bms_average_temperature"] = totalTemp / bmsData.cell_temp_count;
  }
  
  // BMS Status Data
  if (bmsData.mosfet_status_parsed) {
    doc["bms_charge_state"] = bmsData.charge_state;
    doc["bms_charge_mos"] = bmsData.charge_mos;
    doc["bms_discharge_mos"] = bmsData.discharge_mos;
    doc["bms_bms_cycles"] = bmsData.bms_cycles;
    doc["bms_remaining_capacity"] = bmsData.remaining_capacity;
  }

  if (bmsData.system_info_parsed) {
    doc["bms_num_cells"] = bmsData.num_cells;
    doc["bms_num_temp_sensors"] = bmsData.num_temps;
    doc["bms_charger_connected"] = bmsData.charger_connected;
    doc["bms_load_connected"] = bmsData.load_connected;
  }

  if (bmsData.fault_status_parsed) {
    doc["bms_has_faults"] = bmsData.has_faults;
    doc["bms_fault_code"] = bmsData.fault_code;
  }

  // BMS Cell Data Arrays (as JSONB for flexibility)
  if (bmsData.cell_voltages_parsed) {
    JsonArray cellVoltages = doc.createNestedArray("bms_cell_voltages");
    int maxCells = min(bmsData.cell_voltage_count, 48); // Store all available cells
    for (int i = 0; i < maxCells; i++) {
      cellVoltages.add(bmsData.cell_voltages[i]);
    }
  }

  if (bmsData.cell_temperatures_parsed) {
    JsonArray cellTemperatures = doc.createNestedArray("bms_cell_temperatures");
    int maxTemps = min(bmsData.cell_temp_count, 16); // Store all available temps
    for (int i = 0; i < maxTemps; i++) {
      cellTemperatures.add(bmsData.cell_temperatures[i]);
    }
  }

  if (bmsData.cell_balance_states_parsed) {
    JsonArray cellBalanceStates = doc.createNestedArray("bms_cell_balance_states");
    for (int i = 0; i < 48; i++) {
      cellBalanceStates.add(bmsData.cell_balance_states[i]);
    }
  }
  
  // Data quality score (0-100 based on data completeness)
  int qualityScore = 100;
  if (!gpsData.isValid) qualityScore -= 20;
  if (gsmData.signalStrength == "N/A") qualityScore -= 15;
  if (bmsData.cumulative_voltage == 0.0) qualityScore -= 25;
  if (boardData.freeHeap < 10000) qualityScore -= 10; // Low memory warning
  if (WiFi.status() != WL_CONNECTED) qualityScore -= 30;
  
  doc["data_quality_score"] = qualityScore;
  doc["upload_attempts"] = 1;
  
  // Convert to JSON string
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Send to Supabase
  Serial.println("Sending data to Supabase...");
  Serial.println("Data quality score: " + String(qualityScore) + "/100");
  
  int responseCode = db.insert(supabase_table, jsonString, false);
  
  if (responseCode == 200 || responseCode == 201) {
    Serial.println("✓ Data successfully sent to Supabase (HTTP " + String(responseCode) + ")");
  } else {
    Serial.println("✗ Failed to send data to Supabase (HTTP " + String(responseCode) + ")");
    Serial.println("Response: " + jsonString);
  }
  
  // Reset query URL for next operation
  db.urlQuery_reset();
}
 
