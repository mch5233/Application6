AI usage. I used AI to create all the comments for my code as well as fix the semaphore issue I had with my last code where it wasn't printung for every instance with the sensor task. I also asked it to make sure I was meeting all my requirements for the assignement with the code I had. 

Wokwi Link: https://wokwi.com/projects/437503517091076097

Scheduler Fit: How do your task priorities / RTOS settings guarantee every H task’s deadline in Wokwi? Cite one timestamp pair that proves it.
The RTOS settings/ task priorities guarantees Hard Real-Time task deadlines through its priority-based preemptive scheduling. Critical tasks like medicalEventResponseTask with a priority of 6 and heartRateMonitorTask with a priority of 5 are assigned the highest priority. This is to esnure that when a vital event occurs such as the emergency button being pressed thatthese tasks immediately preempt any lower-priority tasks. Non-critical soft real-time task such as dataLoggingTask with a priority of 2 and systemHeartbeatTask with a priority of 1 execute only when the higher-priority tasks are blocked or idle to prevent them from delaying critical operations.

Example Temporal Proximity Proof: From your serial logs, you can observe the rapid response of HRT tasks even under load. For instance, you see sequences like:
R Monitor: ⚠️ BRADYCARDIA DETECTED - Heart rate too low! Signalling alert...
HR Monitor: Current reading: 3
HR Monitor: ⚠️ BRADYCARDIA DETECTED - Heart rate too low! Signalling alert...
HR Monitor: Current reading: 3
Data Logger: HR=3 Mode=NORMAL Alerts=2
HR Monitor: ⚠️ BRADYCARDIA DETECTED - Heart rate too low! Signalling alert...
HR Monitor: Current reading: 3
Response: 💓 Processing heart rate alert.
Response: Total alerts today: 3
HR Monitor: ⚠️ BRADYCARDIA DETECTED - Heart rate too low! Signalling alert...
HR Monitor: ⚠️ BRADYCARDIA DETECTED - Heart rate too low! Signalling alert... (from Priority 5 task)
... followed very quickly by ...
Response: 💓 Processing heart rate alert. (from Priority 6 task)
Given that heartRateMonitorTask samples every 17ms, the immediate appearance of the "Response" message following a detection demonstrates that medicalEventResponseTask is promptly unblocked by the semaphore and executes its critical response, thereby meeting its near-instantaneous deadline.

Race‑Proofing: Where could a race occur? Show the exact line(s) you protected and which primitive solved it.
Race conditions arise from multiple FreeRTos tasks attempting to write to the shared serial output simultaneously. Without protection, messages from different tasks could become intertwined, resulting in corrupted or unreadable status information making system analysis impossible. The potential race is protected by using a mutex (serial_mutex). Every block of code that accesses the serial object for printing is wrapped by calls to acquire and release this mutex. 

Acquisition Line: xSemaphoreTake(serial_mutex, portMAX_DELAY); (e.g., in heartRateMonitorTask before printing alert messages).

Release Line: xSemaphoreGive(serial_mutex); 

This mechanism ensures that only one task can hold the serial_mutex and thus access the serial port at any given time.

Worst‑Case Spike: Describe the heaviest load you threw at the prototype (e.g., sensor spam, comm burst). What margin (of time) remained before an H deadline would slip?
The heaviest load scenario with this prototype is a sustained a consistent printing of the condition when the heartrate remains outside the normal thresholds as seen in the above serial log example. This forces the heartRateMonitorTask to continously signal the hr_alert_semaphor every 17 ms. Also the medicalEventResponsetask is almost constantly active, processing alerts and flashing the LED while dataLoggingTask handles the persistent stream of data. 

The heartRateMonitorTask consistently maintains its 17ms period without apparent delays, ensuring no critical readings are missed. The medicalEventResponseTask, being the highest priority, processes each alert quickly after it's signaled as shown by its immediate response to the HR detections. The hr_data_queue efficiently buffers data, and the system continues to log. The remaining margin before a Hard deadline would slip is sufficient to prevent critical alerts from being missed or delayed, as the system consistently processes and reacts to new bradycardia events without a growing backlog, demonstrating robust performance under this significant spike.

Design Trade‑off: Name one feature you didn’t add (or simplified) to keep timing predictable. Why was that the right call for your chosen company?
I would implement a file system for storing patient history or running analysis algorithms. Insead the system focuses on basic threshold-based anomaly detection (HR_LOW_THRESHOLD, HR_HIGH_THRESHOLD). It offloads all detailed data logging and long-term storage to the serial monitor and a simple web interface. This was a critical decision for a healthcare company's initial prototype because guaranteed responsiveness for life-critical functions is important. Complex computations or disk I/O operations inherently introduce variable and unpredictable execution times, which could easily lead to Hard Real-Time deadline misses for detecting a patient's distress or responding to an emergency button. By keeping the process focused on deterministic event handling, the system prioritizes reliability and rapid intervention.
