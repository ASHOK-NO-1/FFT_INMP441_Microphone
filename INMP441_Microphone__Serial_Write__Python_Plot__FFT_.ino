
/*
 * As per INMP441 SENSOR Caluculations at 44,100 HZ frequency at 16 bit mono channel, sensor produce 88.2 kilobytes of data per second
 * if user want to show sampling frquency in kilo bytes, which is match sensor I2S configuration, first uncomment void i2s_task(void *param) function calculation part, and set higher serial buard rate 100,0000( above 115200) because sensor data is tranfered in serial mointor, python read data from serial mointor and also comment the serial.write () command to clearly see serial.print values not mandatory
  * if user want to calculate frequency domain at python, user set frequency buffer 1024 ( samples per second, each sample is 2bytes , because INMP441 sensor was set 16 bit in I2S configuration, higher frequency buffer, reduce load in cpu, higher frequecy buffer is better,( donot forget consider DMA (direct memory access in esp32)
 * for frequency domain , user need to use Serial.write() function, for Time domain function user need to use Serial.print(mean), and comment remaining serial print function, inorder to avoid time domain serial plotter distraction
 * 
 *
 */


#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// I2S pin definitions for INMP441 microphone
#define I2S_WS 2
#define I2S_SD 1
#define I2S_SCK 42

// I2S port definition
#define I2S_PORT I2S_NUM_0

// Input buffer length
#define BUFFER_LEN 1024
int16_t sampleBuffer[BUFFER_LEN]; // Buffer for 1024 samples (1024 samples * 16-bit = 2048 bytes)

unsigned long lastMillis = 0;
int bufferFillCount = 0;
size_t totalBytesTransferred = 0;
unsigned long totalReadDuration = 0;

// I2S configuration
void configureI2S() {
    const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 44100,
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
        unsigned long readStartTime = micros();
        esp_err_t result = i2s_read(I2S_PORT, sampleBuffer, BUFFER_LEN * sizeof(int16_t), &bytesIn, portMAX_DELAY);
        unsigned long readEndTime = micros();
        unsigned long readDuration = readEndTime - readStartTime;

        if (result == ESP_OK) {
            int samplesRead = bytesIn / sizeof(int16_t);

            if (samplesRead > 0) {
                Serial.write((uint8_t*)sampleBuffer, samplesRead * sizeof(int16_t));

                 /*//Uncomment below lines for statistics
                bufferFillCount++;
                totalBytesTransferred += bytesIn;
                totalReadDuration += readDuration;*/
                
            }
        }

       /*  //Uncomment below block for statistics
        unsigned long currentMillis = millis();
        if (currentMillis - lastMillis >= 1000) {
            Serial.println();
            Serial.print("Buffer filled ");
            Serial.print(bufferFillCount);
            Serial.print(" times in the last second. ");
            Serial.print("Data transferred: ");
            Serial.print(totalBytesTransferred / 1024.0);
            Serial.println(" KB.");
            Serial.print("Average read duration: ");
            if (bufferFillCount > 0) {
                Serial.print(totalReadDuration / (float)bufferFillCount);
                Serial.println(" us");
            } else {
                Serial.println("N/A");
            }

            bufferFillCount = 0;
            totalBytesTransferred = 0;
            totalReadDuration = 0;
            lastMillis = currentMillis;
        }*/
        
    }
}

// Setup function
void setup() {
    Serial.begin(1000000); // High baud rate for faster data transfer
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
