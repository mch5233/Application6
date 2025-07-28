/* --------------------------------------------------------------
    Application 5
    Release Type: Use of Memory Based Task Communication
    Class: Real Time Systems - Su 2025
    Theme: Healthcare Systems - Heart Rate Monitoring with Emergency Response
    Author: Mya Camacho-Hill
    Email: my062925@ucf.edu
    Company: [University of Central Florida] - Healthcare Tech Division
    Website: theDRACOlab.com
    AI Use: I used AI to create all the comments for my code as well as fix 
    the semaphore issue I had with my last code where it wasn't printung 
    for every instance for the sensor task. I also asked it to make sure 
    I was meeting all my requirements for the assignement with the code I had. 
---------------------------------------------------------------*/

#include <WiFi.h>
#include <WebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// ----- Pin Definitions (Healthcare System) -----
const int HEARTBEAT_LED   = 5;   // Green LED - system status
const int ALERT_LED       = 4;   // Red LED - heart rate alerts
const int EMERGENCY_BTN   = 18;  // Emergency button for nurses
const int HR_SENSOR_PIN   = 34;  // Analog heart rate sensor (ADC)

// ----- Healthcare Parameters -----
const int MAX_HR_EVENTS    = 30;   // Max queued heart rate alerts
const int HR_LOW_THRESHOLD = 40;   // Low heart rate threshold (bradycardia) - 40 BPM
const int HR_HIGH_THRESHOLD = 100; // High heart rate threshold (tachycardia) - 100 BPM
// const int DEBOUNCE_MS      = 50;   // Debouncing handled by ISR's nature and semaphore processing
const int HR_SAMPLE_RATE_MS = 17;  // Heart rate sampling every 17ms

// ----- WiFi Configuration -----
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PWD  = "";

// ----- Web Server -----
WebServer medicalConsole(80);

// ----- FreeRTOS Synchronization Primitives -----
SemaphoreHandle_t hr_alert_semaphore;      // Counting semaphore for HR alerts (signals each detected anomaly)
SemaphoreHandle_t emergency_semaphore;     // Binary semaphore for emergency button (signals button press/mode toggle)
SemaphoreHandle_t serial_mutex;            // Mutex for serial output protection
QueueHandle_t     hr_data_queue;           // Queue for heart rate data logging

// ----- Global State Variables -----
volatile bool monitoring_mode     = true;   // true = NORMAL, false = EMERGENCY
volatile bool alert_active        = false;  // Current alert status (reflects immediate sensor state)
volatile int  current_hr_reading  = 0;      // Latest heart rate value
volatile int  alert_count         = 0;      // Number of alerts triggered since startup
volatile bool system_running      = true;   // System status indicator (not actively used in tasks but good for future expansion)

// ----- Task Function Declarations -----
void wifiServerTask(void* parameters);
void heartRateMonitorTask(void* parameters);
// void emergencyButtonTask(void* parameters); // Replaced by ISR
void medicalEventResponseTask(void* parameters);
void systemHeartbeatTask(void* parameters);
void dataLoggingTask(void* parameters);

// ----- ISR for Emergency Button -----
// IRAM_ATTR makes sure the ISR code resides in RAM, which is important for speed and reliability.
void IRAM_ATTR emergencyButtonISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Give the semaphore from ISR context.
  // This unblocks medicalEventResponseTask, which has a high priority.
  xSemaphoreGiveFromISR(emergency_semaphore, &xHigherPriorityTaskWoken);
  // If giving the semaphore woke a higher priority task, yield to it immediately.
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

// ----- Web Interface Functions -----
void sendMedicalDashboard() {
  String html = "<html><body><h1>Heart Rate Monitor</h1>";
  
  html += "<p><b>System Mode:</b> " + String(monitoring_mode ? "NORMAL" : "EMERGENCY") + "</p>";
  html += "<p><b>Alert Status:</b> " + String(alert_active ? "ACTIVE" : "CLEAR") + "</p>";
  html += "<p><b>Heart Rate:</b> " + String(current_hr_reading) + " BPM</p>";
  html += "<p><b>Total Alerts:</b> " + String(alert_count) + "</p>";
  
  html += "<p><a href=\"/emergency\"><button>Emergency Alert (Manual)</button></a></p>";
  html += "<p><a href=\"/toggle-mode\"><button>Toggle Mode (Manual)</button></a></p>";
  html += "<p><a href=\"/\"><button>Refresh</button></a></p>";
  
  html += "</body></html>";
  
  medicalConsole.send(200, "text/html", html);
}

void handleRoot() {
  sendMedicalDashboard();
}

void handleEmergency() {
  // Trigger emergency button semaphore via web.
  // Note: For physical button presses, the ISR handles it. This is for web control.
  xSemaphoreGive(emergency_semaphore);
  sendMedicalDashboard(); // Re-render immediately to reflect pending action/allow refresh
}

void handleToggleMode() {
  // Trigger emergency button to toggle mode via web.
  xSemaphoreGive(emergency_semaphore);
  sendMedicalDashboard(); // Re-render immediately to reflect pending action/allow refresh
}

// ----- Arduino Setup -----
void setup() {
  Serial.begin(115200);
  
  // Initialize GPIO pins
  pinMode(HEARTBEAT_LED, OUTPUT);
  pinMode(ALERT_LED, OUTPUT);
  pinMode(EMERGENCY_BTN, INPUT_PULLUP); // Button connected to GND, so pull-up needed
  
  // Attach ISR to emergency button pin
  // FALLING edge detects when button is pressed (goes from HIGH to LOW)
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_BTN), emergencyButtonISR, FALLING);
  
  // Create FreeRTOS synchronization primitives
  hr_alert_semaphore = xSemaphoreCreateCounting(MAX_HR_EVENTS, 0); // Max alerts in queue
  emergency_semaphore = xSemaphoreCreateBinary();
  serial_mutex = xSemaphoreCreateMutex();
  hr_data_queue = xQueueCreate(50, sizeof(int)); // Queue for 50 HR readings
  
  // Verify semaphore and queue creation
  if (!hr_alert_semaphore || !emergency_semaphore || !serial_mutex || !hr_data_queue) {
    Serial.println("ERROR: Failed to create FreeRTOS synchronization primitives!");
    while(1) { delay(1000); } // Halt if critical resources cannot be created
  }
  
  // Configure web server routes
  medicalConsole.on("/", handleRoot);
  medicalConsole.on("/emergency", handleEmergency);
  medicalConsole.on("/toggle-mode", handleToggleMode);
  
  // Print system startup message
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("=== Healthcare Heart Rate Monitor Starting ===");
  Serial.println("System: Initializing FreeRTOS tasks...");
  xSemaphoreGive(serial_mutex);
  
  // Create FreeRTOS tasks with appropriate priorities
  // Priorities: 6 (Highest) -> 1 (Lowest)
  
  xTaskCreate(medicalEventResponseTask,  // Task to process HR alerts and emergency events
              "Event_Response",         // Task name
              2048,                     // Stack size
              NULL,                     // Parameters
              6,                        // Priority: Highest - critical system state changes & immediate response to ISR
              NULL);                    // Task handle

  xTaskCreate(heartRateMonitorTask,      // Task to read HR sensor and detect anomalies
              "HR_Monitor",             // Task name
              2048,                     // Stack size
              NULL,                     // Parameters
              5,                        // Priority: High - critical patient data acquisition
              NULL);                    // Task handle

  xTaskCreate(wifiServerTask,            // Task to handle web server requests
              "WiFi_Server",            // Task name
              4096,                     // Stack size (larger for WiFi)
              NULL,                     // Parameters
              4,                        // Priority: Medium-high - responsive UI is important for medical staff
              NULL);                    // Task handle
              
  xTaskCreate(dataLoggingTask,           // Task to log heart rate data
              "Data_Logger",            // Task name
              2048,                     // Stack size
              NULL,                     // Parameters
              2,                        // Priority: Low-medium - logging can tolerate some delay
              NULL);                    // Task handle

  xTaskCreate(systemHeartbeatTask,       // Task to blink system status LED
              "Sys_Heartbeat",          // Task name
              1024,                     // Stack size
              NULL,                     // Parameters
              1,                        // Priority: Lowest - purely visual indicator
              NULL);                    // Task handle
  
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("System: All tasks created successfully.");
  Serial.println("Healthcare monitoring system is now active!");
  xSemaphoreGive(serial_mutex);
}

void loop() {
  // Empty - all work done in FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ----- Task Implementations -----

/**
 * @brief Task: WiFi Server
 * @company_use_case Provides a web interface for medical staff to monitor patient status and manually trigger emergencies/mode changes.
 * @classification Soft Real-Time
 * @period Event-driven (handles client requests as they come). The `vTaskDelay(10)` ensures it yields regularly.
 * @deadline User experience: response within a few hundred milliseconds (e.g., page loads/updates within 500ms). Delays are inconvenient but not critical to patient safety.
 */
void wifiServerTask(void* parameters) {
  WiFi.begin(WIFI_SSID, WIFI_PWD);
  
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.print("WiFi: Connecting to network");
  xSemaphoreGive(serial_mutex);
  
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    xSemaphoreTake(serial_mutex, portMAX_DELAY);
    Serial.print(".");
    xSemaphoreGive(serial_mutex);
  }
  
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("\nWiFi: Connected successfully!");
  Serial.print("WiFi: Medical dashboard available at: http://");
  Serial.println(WiFi.localIP());
  xSemaphoreGive(serial_mutex);
  
  medicalConsole.begin();
  
  while (1) {
    medicalConsole.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
 * @brief Task: Heart Rate Monitor
 * @company_use_case Continuously reads the patient's heart rate sensor data and detects anomalies (bradycardia/tachycardia) that require immediate medical attention.
 * @classification Hard Real-Time
 * @period HR_SAMPLE_RATE_MS (17ms). This ensures frequent monitoring for critical changes.
 * @deadline 17ms. The task must complete reading and signaling any anomaly before the next sample is due. Missing this deadline could mean a delay in detecting a critical heart rate event, potentially impacting patient safety.
 */
void heartRateMonitorTask(void* parameters) {
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("HR Monitor: Heart rate monitoring started.");
  xSemaphoreGive(serial_mutex);
  
  while (1) {
    int adc_raw = analogRead(HR_SENSOR_PIN);
    int hr_reading = adc_raw / 40;  // Scale 0-4095 to 0-102 BPM (approx. for simulation)
    current_hr_reading = hr_reading; // Update global for web interface/data logger

    xQueueSend(hr_data_queue, &hr_reading, 0); // Send data to logging queue (non-blocking)
    
    bool anomaly_detected = (hr_reading < HR_LOW_THRESHOLD) || (hr_reading > HR_HIGH_THRESHOLD);
    
    alert_active = anomaly_detected; // Update global alert_active flag based on current reading

    // Trigger alert if anomaly detected (signal semaphore for EACH instance)
    if (anomaly_detected) {
      xSemaphoreGive(hr_alert_semaphore); // Increment counting semaphore

      xSemaphoreTake(serial_mutex, portMAX_DELAY);
      if (hr_reading < HR_LOW_THRESHOLD) {
        Serial.println("HR Monitor: ⚠️ BRADYCARDIA DETECTED - Heart rate too low! Signalling alert...");
      } else {
        Serial.println("HR Monitor: ⚠️ TACHYCARDIA DETECTED - Heart rate too high! Signalling alert...");
      }
      Serial.print("HR Monitor: Current reading: ");
      Serial.println(hr_reading);
      xSemaphoreGive(serial_mutex);
    }
    
    vTaskDelay(pdMS_TO_TICKS(HR_SAMPLE_RATE_MS)); // Sample every 17ms
  }
}

/**
 * @brief Task: Medical Event Response
 * @company_use_case Processes critical patient alerts (heart rate anomalies) and system-wide emergency events (button presses/mode toggles). This task ensures immediate action is taken.
 * @classification Hard Real-Time
 * @period Event-driven (waits on semaphores).
 * @deadline < 100ms. Response to a medical alert or emergency button press must be near-instantaneous (e.g., LED flashing within 50ms, mode change within 10ms of event signal). Missing this deadline could delay life-saving interventions.
 */
void medicalEventResponseTask(void* parameters) {
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("Event Response: Medical event handler ready.");
  xSemaphoreGive(serial_mutex);
  
  while (1) {
    // Handle heart rate alerts (counting semaphore)
    // Short timeout to allow checking for emergency events promptly
    if (xSemaphoreTake(hr_alert_semaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
      alert_count++; // Increment total alerts
      
      xSemaphoreTake(serial_mutex, portMAX_DELAY);
      Serial.println("Response: 💓 Processing heart rate alert.");
      Serial.print("Response: Total alerts today: ");
      Serial.println(alert_count);
      xSemaphoreGive(serial_mutex);
      
      // Visual alert - flash red LED rapidly for a short duration
      for (int i = 0; i < 5; i++) {
        digitalWrite(ALERT_LED, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(ALERT_LED, LOW);
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    }
    
    // Handle emergency button events (binary semaphore)
    // Non-blocking wait for emergency semaphore to ensure prompt response
    if (xSemaphoreTake(emergency_semaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
      monitoring_mode = !monitoring_mode; // Toggle system mode
      
      xSemaphoreTake(serial_mutex, portMAX_DELAY);
      Serial.print("Response: 🔄 System mode changed to: ");
      Serial.println(monitoring_mode ? "NORMAL MONITORING" : "EMERGENCY MODE");
      xSemaphoreGive(serial_mutex);
      
      // Visual feedback - longer red LED pulse for mode change
      digitalWrite(ALERT_LED, HIGH);
      vTaskDelay(pdMS_TO_TICKS(500));
      digitalWrite(ALERT_LED, LOW);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent task starvation
  }
}

/**
 * @brief Task: System Heartbeat
 * @company_use_case Provides a constant visual confirmation to medical staff that the patient monitoring system is operational.
 * @classification Soft Real-Time
 * @period 2000ms (1 second on, 1 second off).
 * @deadline ~200ms. Minor deviations in the LED blink timing are acceptable and do not impact system functionality.
 */
void systemHeartbeatTask(void* parameters) {
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("Heartbeat: System status indicator started.");
  xSemaphoreGive(serial_mutex);
  
  while (1) {
    digitalWrite(HEARTBEAT_LED, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(HEARTBEAT_LED, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/**
 * @brief Task: Data Logging
 * @company_use_case Archives patient heart rate data for later analysis, compliance, or historical record-keeping. It also updates the current HR reading for the web dashboard.
 * @classification Soft Real-Time
 * @period Event-driven (receives from queue); logs every 10th reading.
 * @deadline A few seconds. While data should be logged, a small delay in its appearance in the serial monitor is acceptable as long as data integrity is maintained. Updating `current_hr_reading` for the web interface should be reasonably fresh (e.g., within 100-200ms).
 */
void dataLoggingTask(void* parameters) {
  int sensor_value;
  
  xSemaphoreTake(serial_mutex, portMAX_DELAY);
  Serial.println("Data Logger: Heart rate data logging started.");
  xSemaphoreGive(serial_mutex);
  
  while (1) {
    if (xQueueReceive(hr_data_queue, &sensor_value, portMAX_DELAY) == pdTRUE) {
      current_hr_reading = sensor_value;
      
      static int log_counter = 0;
      if (++log_counter >= 10) {
        log_counter = 0;
        
        xSemaphoreTake(serial_mutex, portMAX_DELAY);
        Serial.print("Data Logger: HR=");
        Serial.print(sensor_value);
        Serial.print(" Mode=");
        Serial.print(monitoring_mode ? "NORMAL" : "EMERGENCY");
        Serial.print(" Alerts=");
        Serial.println(alert_count);
        xSemaphoreGive(serial_mutex);
      }
    }
  }
}
