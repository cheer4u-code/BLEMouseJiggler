/* Pico W Bluetooth Mouse Jiggler */
/*
 * Board: "Raspberry Pi Pico W"
 * Debug Port: "Serial"
 * USB Stack: "Adafruit TinyUSB"
 * IP/Bluetooth Stack: "IPv4 + Bluetooth"
 */

#include <boards/pico_w.h>
#include <hardware/adc.h>
#include <pico/cyw43_arch.h>

#include <Adafruit_TinyUSB.h>
#include <MouseBLE.h>
#include <PicoBluetoothBLEHID.h>

#define WATCHDOG_TIMEOUT 5000
#define LED_OFF_TIME 500
#define JIGGLE_TIMEOUT_HIGH 240000

// soft timer
class Timer {
public:
  Timer()
    : last_time(0) {}
  void setup() {
    _setup();
  }
  void run() {
    unsigned long curr = millis();
    if (timeout > 0 && (curr - last_time) > timeout) {
      last_time = curr;
      _run();
    }
  }
protected:
  virtual void _setup() = 0;
  virtual void _run() = 0;
  unsigned long timeout;
private:
  unsigned long last_time;
};

class TimerContainer {
public:
  void add(Timer* t) {
    container.push_back(t);
  }
  void setup() {
    for (Timer* t : container) {
      t->setup();
    }
  }
  void run() {
    for (Timer* t : container) {
      t->run();
    }
  }
private:
  std::vector<Timer*> container;
};

class WatchdogTimer : public Timer {
protected:
  virtual void _setup() {
    timeout = WATCHDOG_TIMEOUT;
    rp2040.wdt_begin(timeout * 2);
  }
  virtual void _run() {
    // Serial.println("Watchdog updates");
    rp2040.wdt_reset();
  }
};

// onboard-led blink timer
class LedTimer : public Timer {
public:
  void blink(unsigned int times) {
    Serial.printf("Start Led blink (times=%d)\n", times);
    if (times > 0) {
      timeout = 1;
      this->times = times * 2;
    }
  }
protected:
  virtual void _setup() {
    timeout = 0;
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    times = 0;
  }
  virtual void _run() {
    if (times > 0) {
      --times;
      bool led_on = (times % 2) == 1;
      Serial.printf("Led %s\n", led_on ? "on" : "off");
      digitalWrite(LED_BUILTIN, led_on);
      if (times == 0) {
        timeout = 0;
      } else {
        timeout = LED_OFF_TIME;
      }
    }
  }
  unsigned int times;
  bool led_on;
};

class JiggleTimer : public Timer {
public:
  void start() {
    timeout = 1;
    times = random(2, 5);
    Serial.printf("Start Jiggle (times=%d)\n", times);
  }
protected:
  virtual void _setup() {
    timeout = 0;
    times = 0;
  }
  virtual void _run() {
    bool connected = PicoBluetoothBLEHID.connected();
    if (!connected) {
      Serial.println("Jiggle is skipped because bluetooth is disconnected");
      timeout = 0;
      times = 0;
      return;
    }
    if (--times > 0) {
      timeout = random_timeout(12);
    } else {
      timeout = 0;
    }
    long x = random(-10, 10);
    long y = random(-10, 10);
    Serial.printf("Jiggle (%ld, %ld)\n", x, y);
    MouseBLE.move(x, y, 0);
  }
private:
  unsigned long random_timeout(unsigned t) {
    return random(t / 2, t);
  }
  unsigned int times;
};

class JiggleIntervalTimer : public Timer {
public:
  JiggleIntervalTimer(LedTimer* led, JiggleTimer* jiggle) {
    this->jiggle = jiggle;
    this->led = led;
  }
protected:
  virtual void _setup() {
    timeout = random_timeout(JIGGLE_TIMEOUT_HIGH);
  }
  virtual void _run() {
    timeout = random_timeout(JIGGLE_TIMEOUT_HIGH);
    led->blink(2);
    jiggle->start();
  }
private:
  unsigned long random_timeout(unsigned t) {
    return random(t / 2, t);
  }
  LedTimer* led;
  JiggleTimer* jiggle;
};

// https://github.com/raspberrypi/pico-examples/blob/master/adc/read_vsys/power_status.c
#define PICO_POWER_SAMPLE_COUNT 3
// Pin used for ADC 0
#define PICO_FIRST_ADC_PIN 26

class CheckBatteryTimer : public Timer {
protected:
  virtual void _setup() {
    timeout = 1;
  }
  virtual void _run() {
    timeout = 60000;
    check_battery();
  }
private:
  void check_battery() {
    bool powered = cyw43_arch_gpio_get(CYW43_WL_GPIO_VBUS_PIN);

    cyw43_thread_enter();
    // Make sure cyw43 is awake
    cyw43_arch_gpio_get(CYW43_WL_GPIO_VBUS_PIN);

    // setup adc
    adc_gpio_init(PICO_VSYS_PIN);
    adc_select_input(PICO_VSYS_PIN - PICO_FIRST_ADC_PIN);

    adc_fifo_setup(true, false, 0, false, false);
    adc_run(true);

    // We seem to read low values initially - this seems to fix it
    int ignore_count = PICO_POWER_SAMPLE_COUNT;
    while (!adc_fifo_is_empty() || ignore_count-- > 0) {
      (void)adc_fifo_get_blocking();
    }

    // read vsys
    uint32_t vsys = 0;
    for (int i = 0; i < PICO_POWER_SAMPLE_COUNT; i++) {
      uint16_t val = adc_fifo_get_blocking();
      vsys += val;
    }

    adc_run(false);
    adc_fifo_drain();

    vsys /= PICO_POWER_SAMPLE_COUNT;

    cyw43_thread_exit();

    const float conversion_factor = 3.3f / (1 << 12);
    float voltage = vsys * 3 * conversion_factor;

    float percent = 0;
    if (voltage > 1.8) {
      percent = (voltage - 1.8) * 100 / (5.5 - 1.8);
    }

    Serial.printf("%s (voltage %.1fv, %.1f%%)\n",
                  powered ? "Powered" : "Battery",
                  voltage,
                  percent);

    MouseBLE.setBattery(percent);
  }
};


TimerContainer timer;


void setup() {
  Serial.begin(115200);
  MouseBLE.begin("Mouse");
  delay(5000);

  // if analog input pin A0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(A0));

  timer.add(new WatchdogTimer());
  LedTimer* led = new LedTimer();
  timer.add(led);
  JiggleTimer* jiggle = new JiggleTimer();
  timer.add(jiggle);
  timer.add(new JiggleIntervalTimer(led, jiggle));
  timer.add(new CheckBatteryTimer());
  timer.setup();
  Serial.println("Start Mouse Jiggle");

  led->blink(3);
}

void loop() {
  timer.run();
}
