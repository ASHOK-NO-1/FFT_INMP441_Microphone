#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <arduinoFFT.h>

// I2S pin definitions for INMP441 microphone
#define I2S_WS 2
#define I2S_SD 1
#define I2S_SCK 42

// I2S port definition
#define I2S_PORT I2S_NUM_0

// Input buffer length
#define BUFFER_LEN 1024
int16_t sampleBuffer[BUFFER_LEN]; // Buffer for 1024 samples (1024 samples * 16-bit = 2048 bytes)

// FFT configuration
#define SAMPLES 1024              // Must be a power of 2
#define SAMPLING_FREQUENCY 44100  // Hz, must be the same as set in I2S configuration

// FFT arrays
double vReal[SAMPLES];
double vImag[SAMPLES];

// Create FFT object
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// I2S configuration
void configureI2S() {
    const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLING_FREQUENCY,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = BUFFER_LEN,
        .use_apll = false
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

// I2S pin setup
void setupI2SPins() {
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD
    };

    i2s_set_pin(I2S_PORT, &pin_config);
}

// I2S read task
void i2sTask(void *param) {
    while (true) {
        size_t bytesIn = 0;
        esp_err_t result = i2s_read(I2S_PORT, sampleBuffer, BUFFER_LEN * sizeof(int16_t), &bytesIn, portMAX_DELAY);

        if (result == ESP_OK) {
            int samplesRead = bytesIn / sizeof(int16_t);

            if (samplesRead > 0) {
                // Copy I2S samples into FFT input arrays
                for (int i = 0; i < SAMPLES; i++) {
                    vReal[i] = sampleBuffer[i];
                    vImag[i] = 0.0; // Imaginary part is zero
                }

                // Perform FFT
                FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // Apply windowing
                FFT.compute(FFTDirection::Forward); // Compute FFT
                FFT.complexToMagnitude(); // Compute magnitudes

                // Find top three frequencies
                double peakFrequencies[3] = {0.0, 0.0, 0.0};
                double peakAmplitudes[3] = {0.0, 0.0, 0.0};

                for (int i = 1; i < (SAMPLES / 2); i++) {
                    if (vReal[i] > peakAmplitudes[0]) {
                        peakAmplitudes[2] = peakAmplitudes[1];
                        peakFrequencies[2] = peakFrequencies[1];
                        peakAmplitudes[1] = peakAmplitudes[0];
                        peakFrequencies[1] = peakFrequencies[0];
                        peakAmplitudes[0] = vReal[i];
                        peakFrequencies[0] = (i * ((double)SAMPLING_FREQUENCY / SAMPLES));
                    } else if (vReal[i] > peakAmplitudes[1]) {
                        peakAmplitudes[2] = peakAmplitudes[1];
                        peakFrequencies[2] = peakFrequencies[1];
                        peakAmplitudes[1] = vReal[i];
                        peakFrequencies[1] = (i * ((double)SAMPLING_FREQUENCY / SAMPLES));
                    } else if (vReal[i] > peakAmplitudes[2]) {
                        peakAmplitudes[2] = vReal[i];
                        peakFrequencies[2] = (i * ((double)SAMPLING_FREQUENCY / SAMPLES));
                    }
                }

                // Print the top three frequencies and their amplitudes
                Serial.println("Top 3 Frequencies and their Amplitudes:");
                for (int i = 0; i < 3; i++) {
                    Serial.print("Frequency: ");
                    Serial.print(peakFrequencies[i]);
                    Serial.print(" Hz, Amplitude: ");
                    Serial.println(peakAmplitudes[i]);
                }
                Serial.println();
                delay(2000);
            }
        }
    }
}

// Setup function
void setup() {
    Serial.begin(115200); // High baud rate for faster data transfer
    Serial.println("Setup I2S ...");

    delay(1000);
    configureI2S();
    setupI2SPins();
    i2s_start(I2S_PORT);
    delay(500);

    xTaskCreatePinnedToCore(i2sTask, "i2s_task", 30000, NULL, 1, NULL, 1);
}

// Main loop
void loop() {
    // Nothing to do here as the work is done in the FreeRTOS task
}
