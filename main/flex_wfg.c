#include <driver/dac_oneshot.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <driver/gpio.h>

// DAC channel
#define DAC_CHANNEL DAC_CHAN_0 // GPIO25 (DAC0)

// Constants
#define NUM_SAMPLES_PER_CYCLE 100  // Number of samples per waveform period
#define MICROSECONDS_PER_SECOND 1000000 // Number of microseconds in one second
#define BUTTON_GPIO GPIO_NUM_0 // GPIO for the button

// Parameters
volatile float frequency = 100.0; // Frequency in Hz
volatile float amplitude = 3.3;   // Amplitude in Volts (0 - 3.3V)
volatile float phase = 0.0;       // Phase in degrees (0 - 360)
volatile int waveform = 0;        // Waveform type: 0=Sine, 1=Square, 2=Triangle

// Derived value
int SAMPLES_PER_SECOND; // Sample rate in Hz

// Waveform arrays
int sineWave[NUM_SAMPLES_PER_CYCLE];
int squareWave[NUM_SAMPLES_PER_CYCLE];
int triangleWave[NUM_SAMPLES_PER_CYCLE];

// DAC handle
dac_oneshot_handle_t dac_handle;

// Function prototypes
void calculateWaveforms();
void generateSineWave(int* values, int length);
void generateSquareWave(int* values, int length);
void generateTriangleWave(int* values, int length);
void setupTimer();
void onTimer(void* arg);
void button_isr_handler(void* arg);
void setupButton();

void app_main() {
    // Calculate the sample rate
    SAMPLES_PER_SECOND = NUM_SAMPLES_PER_CYCLE * frequency;

    // Enable DAC output
    dac_oneshot_config_t dac_config = {
        .chan_id = DAC_CHANNEL
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&dac_config, &dac_handle));

    // Pre-calculate waveform values
    calculateWaveforms();

    // Setup and start the timer
    setupTimer();

    // Setup button for waveform switching
    setupButton();
}

void calculateWaveforms() {
    generateSineWave(sineWave, NUM_SAMPLES_PER_CYCLE);
    generateSquareWave(squareWave, NUM_SAMPLES_PER_CYCLE);
    generateTriangleWave(triangleWave, NUM_SAMPLES_PER_CYCLE);
}

void generateSineWave(int* values, int length) {
    for (int i = 0; i < length; i++) {
        float t = (float)i / length; // Normalized time (0 to 1)
        float sample = (sin(2 * M_PI * t + phase * M_PI / 180.0) + 1.0) / 2.0; // Scale to 0-1 range
        values[i] = (int)(sample * amplitude / 3.3 * 255);
    }
}

void generateSquareWave(int* values, int length) {
    for (int i = 0; i < length; i++) {
        float t = (float)i / length; // Normalized time (0 to 1)
        float sample = sin(2 * M_PI * t + phase * M_PI / 180.0) >= 0 ? 1.0 : 0.0; // Scale to 0-1 range
        values[i] = (int)(sample * amplitude / 3.3 * 255);
    }
}

void generateTriangleWave(int* values, int length) {
    for (int i = 0; i < length; i++) {
        float t = (float)i / length; // Normalized time (0 to 1)
        float sample = (2.0 * fabs(2.0 * (t - floor(t + 0.5))) - 1.0 + 1.0) / 2.0; // Scale to 0-1 range
        values[i] = (int)(sample * amplitude / 3.3 * 255);
    }
}

void setupTimer() {
    esp_timer_create_args_t timer_args = {
        .callback = &onTimer,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "waveform_timer"
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, MICROSECONDS_PER_SECOND / SAMPLES_PER_SECOND));
}

void IRAM_ATTR onTimer(void* arg) {
    static int sampleIndex = 0;
    int dacValue;

    switch (waveform) {
        case 0: // Sine wave
            dacValue = sineWave[sampleIndex];
            break;
        case 1: // Square wave
            dacValue = squareWave[sampleIndex];
            break;
        case 2: // Triangle wave
            dacValue = triangleWave[sampleIndex];
            break;
        default:
            dacValue = 0;
            break;
    }

    ESP_ERROR_CHECK(dac_oneshot_output_voltage(dac_handle, dacValue));

    sampleIndex++;
    if (sampleIndex >= NUM_SAMPLES_PER_CYCLE) {
        sampleIndex = 0;
    }
}

void IRAM_ATTR button_isr_handler(void* arg) {
    // Simple debouncing by checking the button state a short time after the ISR triggers
    static uint32_t last_interrupt_time = 0;
    uint32_t current_time = esp_timer_get_time();
    if (current_time - last_interrupt_time > 200000) { // 200 ms debounce time
        last_interrupt_time = current_time;
        waveform = (waveform + 1) % 3; // Cycle through waveforms: 0, 1, 2
    }
}

void setupButton() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE
    };
    gpio_config(&io_conf);

    // Install the ISR handler for the button
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
}
