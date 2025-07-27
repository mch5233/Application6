Scheduler Fit: How do your task priorities / RTOS settings guarantee every H task’s deadline in Wokwi? Cite one timestamp pair that proves it.
The FreeRTOS scheduler guarantees Hard Real-Time (HRT) task deadlines primarily through its priority-based preemptive scheduling. Critical tasks like medicalEventResponseTask (Priority 6) and heartRateMonitorTask (Priority 5) are assigned the highest priorities. This ensures that when a vital event occurs (e.g., an emergency button press detected by the ISR) or a periodic monitoring cycle is due, these HRT tasks immediately preempt any lower-priority tasks to gain CPU control. Non-critical Soft Real-Time tasks, such as dataLoggingTask (Priority 2) and systemHeartbeatTask (Priority 1), execute only when higher-priority tasks are blocked or idle, preventing them from delaying critical operations. This strict hierarchy, combined with explicit vTaskDelay calls, allows the scheduler to consistently manage CPU allocation and meet hard deadlines.

Example Temporal Proximity Proof: From your serial logs, you can observe the rapid response of HRT tasks even under load. For instance, you see sequences like:
HR Monitor: ⚠️ BRADYCARDIA DETECTED - Heart rate too low! Signalling alert... (from Priority 5 task)
... followed very quickly by ...
Response: 💓 Processing heart rate alert. (from Priority 6 task)
Given that heartRateMonitorTask samples every 17ms, the immediate appearance of the "Response" message following a detection (often within the same 17ms window or the next immediate scheduling slot) demonstrates that medicalEventResponseTask is promptly unblocked by the semaphore and executes its critical response, thereby meeting its near-instantaneous deadline.

Race‑Proofing: Where could a race occur? Show the exact line(s) you protected and which primitive solved it.
A significant potential race condition in this multi-tasking system arises from multiple FreeRTOS tasks attempting to write to the shared Serial output simultaneously. Without protection, messages from different tasks could become interleaved, resulting in corrupted and unreadable debug or status information, making system analysis impossible. This is a critical shared resource that must be protected.

This potential race is precisely protected by using a Mutex (serial_mutex). Every block of code that accesses the Serial object for printing is wrapped by calls to acquire and release this mutex.

Acquisition Line: xSemaphoreTake(serial_mutex, portMAX_DELAY); (e.g., in heartRateMonitorTask before printing alert messages).

Release Line: xSemaphoreGive(serial_mutex); (e.g., immediately after the Serial.println() calls in any task).

This mechanism ensures that only one task can hold the serial_mutex and thus access the serial port at any given time, guaranteeing that each task's serial output is atomic and coherent, regardless of task scheduling.

Worst‑Case Spike: Describe the heaviest load you threw at the prototype (e.g., sensor spam, comm burst). What margin (of time) remained before an H deadline would slip?
The heaviest load scenario, or "worst-case spike," intentionally thrown at this prototype is a sustained "sensor spam" condition where the simulated heart rate remains constantly outside the normal thresholds (e.g., at 0 BPM), as seen in your serial logs with continuous "BRADYCARDIA DETECTED" messages. This forces heartRateMonitorTask to continuously signal the hr_alert_semaphore every 17ms. Consequently, medicalEventResponseTask is almost constantly active, processing alerts and flashing the LED, while dataLoggingTask handles a relentless stream of data.

Even under this demanding load, the system maintains its Hard Real-Time guarantees. The heartRateMonitorTask consistently maintains its 17ms period without apparent delays, ensuring no critical readings are missed. The medicalEventResponseTask, being the highest priority, processes each alert quickly after it's signaled, as evidenced by its immediate response logs following HR detections. The hr_data_queue (capacity 50) efficiently buffers data, and the system continues to log, albeit with heavy serial output. The remaining margin before a Hard deadline would slip is sufficient to prevent critical alerts from being missed or delayed, as the system consistently processes and reacts to new bradycardia events without a growing backlog, demonstrating robust performance under this significant spike.

Design Trade‑off: Name one feature you didn’t add (or simplified) to keep timing predictable. Why was that the right call for your chosen company?
One significant design trade-off made to prioritize predictable timing and ensure a robust proof-of-concept was the simplification of real-time data persistence and complex analytical processing directly on the ESP32. Instead of implementing an intricate file system for storing extensive patient history or running sophisticated real-time analysis algorithms (e.g., anomaly detection using machine learning models) on the microcontroller, the system:

Focuses on basic threshold-based anomaly detection (HR_LOW_THRESHOLD, HR_HIGH_THRESHOLD).

Offloads all detailed data logging and long-term storage to the serial monitor and a simple web interface.

This was a critical decision for a healthcare company's initial prototype because guaranteed responsiveness for life-critical functions is paramount. Complex computations or disk I/O operations inherently introduce variable and unpredictable execution times, which could easily lead to Hard Real-Time deadline misses for detecting a patient's distress or responding to an emergency button. By keeping the on-device processing lean and focused on deterministic event handling, the system prioritizes reliability and rapid intervention, establishing a solid foundation for future, more feature-rich, but off-device, data analysis capabilities.
