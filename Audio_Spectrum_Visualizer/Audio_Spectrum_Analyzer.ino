#include <Arduino.h>
#include <driver/i2s_std.h>
#include <FastLED.h>
#include <arduinoFFT.h>
#include <math.h>

// --- LED Configuration ---
#define LED_PIN        2
#define COLOR_ORDER    GRB
#define CHIPSET        WS2812B
#define kMatrixWidth   32
#define kMatrixHeight  8
#define NUM_LEDS       (kMatrixWidth * kMatrixHeight)

// --- Audio/FFT Configuration ---
#define SAMPLES        512
#define SAMPLING_FREQ  16000

// --- I2S Pins ---
#define I2S_BCLK       GPIO_NUM_26
#define I2S_WS         GPIO_NUM_25
#define I2S_DIN        GPIO_NUM_33

// ICS-43434: SEL chooses channel
// SEL=GND -> Left, SEL=3.3V -> Right
#define MIC_IS_RIGHT_CHANNEL  false

// --- Visual / smoothing tuning ---
#define NOISE_FLOOR    40.0f
#define ATTACK         0.55f
#define RELEASE        0.10f
#define PEAK_FALL      0.18f
#define BINS_PER_COL   3

// --- Color tuning ---
#define HUE_START      0     // 0=red
#define HUE_END        160   // ~aqua/blue-green (try 192 for more blue/purple)
#define SATURATION     255

// --- Globals ---
CRGB leds[NUM_LEDS];
i2s_chan_handle_t rx_handle = NULL;

ArduinoFFT<double> FFT = ArduinoFFT<double>();
double vReal[SAMPLES];
double vImag[SAMPLES];

float barLevel[kMatrixWidth]  = {0};
float peakLevel[kMatrixWidth] = {0};

// Auto gain control (AGC)
float agcLevel = 5000.0f;
#define AGC_ATTACK  0.20f
#define AGC_RELEASE 0.02f

static inline int ledIndexXY(int x, int y) {
  // zigzag, column-major
  return (x % 2 == 0) ? (x * kMatrixHeight + y)
                      : (x * kMatrixHeight + (kMatrixHeight - 1 - y));
}

void setupI2S() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
  if (err != ESP_OK) {
    Serial.printf("i2s_new_channel failed: %d\n", (int)err);
    while (true) delay(100);
  }

  i2s_std_config_t std_cfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLING_FREQ),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = I2S_BCLK,
      .ws   = I2S_WS,
      .dout = I2S_GPIO_UNUSED,
      .din  = I2S_DIN,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false }
    }
  };

  std_cfg.slot_cfg.slot_mask = MIC_IS_RIGHT_CHANNEL ? I2S_STD_SLOT_RIGHT : I2S_STD_SLOT_LEFT;

  err = i2s_channel_init_std_mode(rx_handle, &std_cfg);
  if (err != ESP_OK) {
    Serial.printf("i2s_channel_init_std_mode failed: %d\n", (int)err);
    while (true) delay(100);
  }

  err = i2s_channel_enable(rx_handle);
  if (err != ESP_OK) {
    Serial.printf("i2s_channel_enable failed: %d\n", (int)err);
    while (true) delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(30);

  setupI2S();
}

void loop() {
  static int32_t i2s_raw_buffer[SAMPLES];
  size_t bytes_read = 0;

  esp_err_t err = i2s_channel_read(rx_handle, i2s_raw_buffer, sizeof(i2s_raw_buffer),
                                   &bytes_read, portMAX_DELAY);
  if (err != ESP_OK || bytes_read == 0) return;

  int n = (int)(bytes_read / sizeof(int32_t));
  if (n > SAMPLES) n = SAMPLES;

  // Convert & remove DC, compute RMS
  double dc = 0.0;
  double sumsq = 0.0;

  for (int i = 0; i < n; i++) {
    int32_t s = (i2s_raw_buffer[i] >> 8); // ICS-43434 24-bit in 32-bit frame
    vReal[i] = (double)s;
    vImag[i] = 0.0;
    dc += vReal[i];
  }
  dc = (n > 0) ? (dc / (double)n) : 0.0;

  for (int i = 0; i < n; i++) {
    vReal[i] -= dc;
    sumsq += vReal[i] * vReal[i];
  }
  for (int i = n; i < SAMPLES; i++) { vReal[i] = 0.0; vImag[i] = 0.0; }

  float rms = (n > 0) ? sqrtf((float)(sumsq / (double)n)) : 0.0f;

  if (rms < NOISE_FLOOR) {
    for (int x = 0; x < kMatrixWidth; x++) {
      barLevel[x]  *= 0.92f;
      peakLevel[x] = fmaxf(0.0f, peakLevel[x] - PEAK_FALL);
    }
  }

  // FFT
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  // AGC reference from low-mid band
  double ref = 0.0;
  int refCount = 0;
  for (int b = 3; b < 3 + kMatrixWidth * BINS_PER_COL; b++) {
    if (b >= (SAMPLES / 2)) break;
    ref += vReal[b];
    refCount++;
  }
  float refAvg = (refCount > 0) ? (float)(ref / (double)refCount) : 1.0f;
  if (refAvg < 1.0f) refAvg = 1.0f;

  if (refAvg > agcLevel) agcLevel = (1.0f - AGC_ATTACK) * agcLevel + AGC_ATTACK * refAvg;
  else                  agcLevel = (1.0f - AGC_RELEASE) * agcLevel + AGC_RELEASE * refAvg;

  // Draw
  FastLED.clear();

  const int firstBin = 3;

  for (int x = 0; x < kMatrixWidth; x++) {
    // Average a small band per column
    double band = 0.0;
    int count = 0;

    int startBin = firstBin + x * BINS_PER_COL;
    for (int i = 0; i < BINS_PER_COL; i++) {
      int bin = startBin + i;
      if (bin >= (SAMPLES / 2)) break;
      band += vReal[bin];
      count++;
    }
    float mag = (count > 0) ? (float)(band / (double)count) : 0.0f;

    float norm = mag / (agcLevel + 1.0f);
    if (norm < 0) norm = 0;

    float target = powf(norm, 0.6f) * (float)kMatrixHeight;
    if (target > kMatrixHeight) target = kMatrixHeight;

    // Attack/Release smoothing
    float current = barLevel[x];
    if (target > current) current += (target - current) * ATTACK;
    else                 current += (target - current) * RELEASE;
    barLevel[x] = current;

    // Peak hold
    if (barLevel[x] > peakLevel[x]) peakLevel[x] = barLevel[x];
    else peakLevel[x] = fmaxf(0.0f, peakLevel[x] - PEAK_FALL);

    // Pretty color: hue by column (frequency)
    uint8_t hue = (uint8_t)map(x, 0, kMatrixWidth - 1, HUE_START, HUE_END);

    int barH = constrain((int)(barLevel[x] + 0.5f), 0, kMatrixHeight);

    // Color the bar with a little brightness gradient up the column
    for (int y = 0; y < barH; y++) {
      // Bottom brighter, top slightly dimmer (or flip if you prefer)
      uint8_t val = (uint8_t)map(y, 0, kMatrixHeight - 1, 255, 120);

      // Also boost brightness a bit when the bar is tall
      uint8_t boost = (uint8_t)constrain((int)(barLevel[x] * 18.0f), 0, 80);
      uint8_t v = qadd8(val, boost);

      leds[ledIndexXY(x, y)] = CHSV(hue, SATURATION, v);
    }

    // Peak pixel: bright highlight (nearly white)
    int peakY = (int)(peakLevel[x] + 0.5f) - 1;
    if (peakY >= 0 && peakY < kMatrixHeight) {
      leds[ledIndexXY(x, peakY)] = CHSV(hue, 60, 255);
    }
  }

  FastLED.show();
}
