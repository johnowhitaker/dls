#include <Arduino.h>
#include <ADC.h>
#include <IntervalTimer.h>

#define ANALOG_PIN A9        // Pin to sample from
#define BAUD_RATE 2000000    // High baud rate for fast data transfer
#define BUFFER_SIZE 1000     // Size of the circular buffer (in samples)
#define CHUNK_SIZE 100       // Number of samples to send per transmission
#define TOTAL_SAMPLES 1000000 // Total samples to collect (e.g., 100,000)

volatile uint16_t buffer[BUFFER_SIZE];         // Circular buffer
volatile uint16_t writeIndex = 0;              // Where to write next sample
volatile uint16_t readIndex = 0;               // Where to read next sample
volatile uint32_t totalSamplesCollected = 0;   // Count of samples collected
uint32_t totalSamplesToCollect = TOTAL_SAMPLES; // Desired total samples
volatile bool sampling = false;                // Sampling state

IntervalTimer timer;  // Timer for interrupt-based sampling
ADC *adc = new ADC(); // ADC object

void timerISR() {
    // Collect sample if we haven’t reached the target
    if (totalSamplesCollected < totalSamplesToCollect) {
        buffer[writeIndex] = analogRead(ANALOG_PIN);
        writeIndex = (writeIndex + 1) % BUFFER_SIZE; // Wrap around if needed
        totalSamplesCollected++;
        // Stop timer when all samples are collected
        if (totalSamplesCollected >= totalSamplesToCollect) {
            timer.end();
        }
    }
}

void startSampling() {
    totalSamplesCollected = 0;
    writeIndex = 0;
    readIndex = 0;
    sampling = true;
    timer.begin(timerISR, 20); // 20 µs interval = 50 kHz sampling
}

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) {} // Wait for serial connection

    pinMode(ANALOG_PIN, INPUT);

    // Configure ADC
    adc->adc0->setAveraging(1);
    adc->adc0->setResolution(12);
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
    adc->adc0->enablePGA(4);

    Serial.println("Ready");
}

void loop() {
    // Check for start command from Python
    if (Serial.available()) {
        char command = Serial.read();
        if (command == 'S') {
            startSampling();
        }
    }

    if (sampling) {
        // Calculate number of samples available in the buffer
        uint16_t available = (writeIndex - readIndex + BUFFER_SIZE) % BUFFER_SIZE;
        if (available > 0) {
            // Send up to CHUNK_SIZE samples
            uint16_t sendCount = min(available, CHUNK_SIZE);
            if (readIndex + sendCount <= BUFFER_SIZE) {
                // Send contiguous block
                Serial.write((uint8_t*)&buffer[readIndex], sendCount * sizeof(uint16_t));
            } else {
                // Send in two parts if wrapping around
                uint16_t firstPart = BUFFER_SIZE - readIndex;
                Serial.write((uint8_t*)&buffer[readIndex], firstPart * sizeof(uint16_t));
                Serial.write((uint8_t*)&buffer[0], (sendCount - firstPart) * sizeof(uint16_t));
            }
            readIndex = (readIndex + sendCount) % BUFFER_SIZE; // Update read position
        }

        // Stop sampling when all samples are collected and sent
        if (totalSamplesCollected >= totalSamplesToCollect && readIndex == writeIndex) {
            sampling = false;
        }
    }
}