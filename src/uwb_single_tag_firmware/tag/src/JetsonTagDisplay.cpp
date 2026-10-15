#include "JetsonTagDisplay.h"

#include <Arduino.h>
#include <LoRaWan_APP.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "HT_SSD1306Wire.h"
#include "JetsonTagConfig.h"

#if CONFIG_FREERTOS_UNICORE
#error "The nonblocking OLED worker requires the dual-core ESP32-S3 target"
#endif

namespace JetsonTagDisplay {
namespace {

static_assert(ATOMIC_INT_LOCK_FREE == 2, "Display mailbox must be lock-free");
constexpr uint32_t kSignalFreshnessMs = 1500U;
constexpr uint32_t kPollMs = 100U;
constexpr uint32_t kRadioReady = 1U, kHasFullCycle = 2U;
constexpr BaseType_t kDisplayCore = ARDUINO_RUNNING_CORE == 1 ? 0 : 1;
SSD1306Wire gDisplay(0x3c, 100000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
std::atomic<uint32_t> gReady{0}, gFlags{0}, gLastFullValidMs{0}, gForceSerial{0};
bool gStarted = false, gHasFullCycle = false;  // Owned by setup/loop only.

void draw(bool connected) {
  gDisplay.clear();
  gDisplay.setTextAlignment(TEXT_ALIGN_CENTER);
  gDisplay.setFont(ArialMT_Plain_10);
  if (!connected) gDisplay.drawString(64, 0, JetsonTagConfig::kTagDisplayName);
  gDisplay.setFont(connected ? ArialMT_Plain_24 : ArialMT_Plain_16);
  gDisplay.drawString(64, connected ? 20 : 24,
      connected ? JetsonTagConfig::kTagDisplayName : "LOST CONNECT");
  gDisplay.display();
}

void displayTask(void*) {
  // Initialize Wire here too, so I2C driver/interrupt ownership stays on the
  // display core. No runtime OLED call is made from the ranging core.
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  vTaskDelay(pdMS_TO_TICKS(100));
  Wire.setTimeOut(5);
  bool panelReady = gDisplay.init();
  if (panelReady) {
    Wire.beginTransmission(0x3c);
    panelReady = Wire.endTransmission() == 0;
  }
  if (!panelReady) {
    vTaskDelete(nullptr);
    return;
  }
  gDisplay.flipScreenVertically();
  gReady.store(1, std::memory_order_release);
  bool shown = false, shownConnected = false;
  uint32_t shownForce = 0;
  for (;;) {
    const uint32_t flags = gFlags.load(std::memory_order_acquire);
    const uint32_t lastFull = gLastFullValidMs.load(std::memory_order_relaxed);
    const uint32_t force = gForceSerial.load(std::memory_order_relaxed);
    // Read the clock after the timestamp, including at the millis() wrap.
    const bool connected = flags == (kRadioReady | kHasFullCycle) &&
        static_cast<uint32_t>(millis() - lastFull) <= kSignalFreshnessMs;
    if (!shown || connected != shownConnected || force != shownForce) {
      draw(connected);
      shown = true; shownConnected = connected; shownForce = force;
    }
    // Stable status causes no framebuffer transfer, even across many epochs.
    vTaskDelay(pdMS_TO_TICKS(kPollMs));
  }
}

}  // namespace

bool begin() {
  if (!gStarted) {
    gStarted = xTaskCreatePinnedToCore(displayTask, "uwb_oled", 4096, nullptr,
                                     1, nullptr, kDisplayCore) == pdPASS;
  }
  return gStarted;
}

bool ready() {
  return gReady.load(std::memory_order_acquire) != 0;
}

void render(bool uwbReady, uint8_t validMask, bool force) {
  if (!uwbReady) {
    gHasFullCycle = false;
  } else if ((validMask & 0x0FU) == 0x0FU) {
    gLastFullValidMs.store(millis(), std::memory_order_relaxed);
    gHasFullCycle = true;
  }
  gFlags.store((uwbReady ? kRadioReady : 0U) | (gHasFullCycle ? kHasFullCycle : 0U),
               std::memory_order_release);
  if (force) gForceSerial.fetch_add(1, std::memory_order_relaxed);
}

}  // namespace JetsonTagDisplay
