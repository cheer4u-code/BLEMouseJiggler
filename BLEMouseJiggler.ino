/*
 * Pico W Bluetooth Mouse Jiggler
 */

#include <boards/pico_w.h>
#include <hardware/adc.h>
#include <pico/cyw43_arch.h>

#include <Adafruit_TinyUSB.h>
#include <MouseBLE.h>
#include <PicoBluetoothBLEHID.h>

#define WATCHDOG_TIMEOUT 5000
#define LED_TIMEOUT 500
#define JIGGLE_TIMEOUT_HIGH 240000
#define SET_BATTERY_TIMEOUT 60000

// 50%: 3.65v, 40%: 3.28v, 30%: 2.91v, 20%: 2.54v, 10%: 2.17v, 5%: 1.99v
#define HIGH_VOLTAGE 5.5
#define LOW_VOLTAGE 1.8

#define RUN_NOW 1

#define BUTTON_TIMEOUT 100

#define random_timeout(t) random(t / 2, t)

class Event {
public:
  virtual void setup() = 0;
  virtual void run(unsigned long curr) = 0;
};

class EventContainer {
public:
  void add(Event* t) {
    container.push_back(t);
  }
  void setup() {
    for (Event* t : container) {
      t->setup();
    }
  }
  void run() {
    unsigned long curr = millis();
    for (Event* t : container) {
      t->run(curr);
    }
  }
private:
  std::vector<Event*> container;
};

class Timer : public Event {
public:
  Timer()
    : timeout(0), last_time(0) {}
  virtual void setup() {
    _setup();
  }
  virtual void run(unsigned long curr) {
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

class Button : public Event {
public:
  Button()
    : count(0), timeout(BUTTON_TIMEOUT), last_time(0) {}
  virtual void setup() {
    _setup();
  }
  virtual void run(unsigned long curr) {
    if (_keydown()) {
      if ((curr - last_time) > timeout) {
        count++;
        timeout += BUTTON_TIMEOUT;
      }
    } else {
      if (count > 0) {
        // key up
        _run(count);
        timeout = BUTTON_TIMEOUT;
        count = 0;
      }
      last_time = curr;
    }
  }
protected:
  virtual void _setup() = 0;
  virtual bool _keydown() = 0;
  virtual void _run(unsigned int count) = 0;
private:
  unsigned int count;
  unsigned long timeout;
  unsigned long last_time;
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
  void run_now(unsigned int times) {
    Serial.printf("Start Led blink (times=%d)\n", times);
    if (times > 0) {
      timeout = RUN_NOW;
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
        timeout = LED_TIMEOUT;
      }
    }
  }
  unsigned int times;
  bool led_on;
};

class JiggleTimer : public Timer {
public:
  void run_now() {
    timeout = RUN_NOW;
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
  unsigned int times;
};

class JiggleIntervalTimer : public Timer {
public:
  JiggleIntervalTimer(LedTimer* led, JiggleTimer* jiggle)
    : jiggle(jiggle), led(led) {}
  void run_now() {
    timeout = RUN_NOW;
  }
protected:
  virtual void _setup() {
    timeout = random_timeout(JIGGLE_TIMEOUT_HIGH);
  }
  virtual void _run() {
    timeout = random_timeout(JIGGLE_TIMEOUT_HIGH);
    led->run_now(2);
    jiggle->run_now();
  }
private:
  LedTimer* led;
  JiggleTimer* jiggle;
};

// https://github.com/raspberrypi/pico-examples/blob/master/adc/read_vsys/power_status.c
#define PICO_POWER_SAMPLE_COUNT 3
// Pin used for ADC 0
#define PICO_FIRST_ADC_PIN 26

class SetBatteryTimer : public Timer {
protected:
  virtual void _setup() {
    timeout = RUN_NOW;
  }
  virtual void _run() {
    timeout = SET_BATTERY_TIMEOUT;
    set_battery();
  }
private:
  float voltage() {
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
    float vol = vsys * 3 * conversion_factor;

    return vol;
  }

  bool powered_battery() {
    return cyw43_arch_gpio_get(CYW43_WL_GPIO_VBUS_PIN);
  }

  void set_battery() {
    bool powered = powered_battery();

    float vol = voltage();

    float percent = 0.0;
    if (vol > 1.8) {
      percent = (vol - LOW_VOLTAGE) * 100 / (HIGH_VOLTAGE - LOW_VOLTAGE);
    }
    if (percent > 100.0) {
      percent = 100.0;
    }

    Serial.printf("%s (voltage %.1fv, %.1f%%)\n",
                  powered ? "Powered" : "Battery",
                  vol,
                  percent);

    MouseBLE.setBattery(percent);
  }
};

class BootselButton : public Button {
public:
  BootselButton(JiggleIntervalTimer* jiggle_interval)
    : jiggle_interval(jiggle_interval) {}
protected:
  virtual void _setup() {}
  virtual bool _keydown() {
    return BOOTSEL;
  }
  virtual void _run(unsigned int count) {
    unsigned long ms = count * BUTTON_TIMEOUT;
    Serial.printf("Bootsel button pressed (%.dms)\n", ms);
    if (ms >= 5000) {
      Serial.flush();
      rp2040.rebootToBootloader();
    } else {
      jiggle_interval->run_now();
    }
  }
private:
  JiggleIntervalTimer* jiggle_interval;
};


EventContainer events;


void setup() {
  Serial.begin(115200);
  MouseBLE.begin("Mouse");
  delay(5000);

  // if analog input pin A0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(A0));

  events.add(new WatchdogTimer());
  LedTimer* led = new LedTimer();
  events.add(led);
  JiggleTimer* jiggle = new JiggleTimer();
  events.add(jiggle);
  JiggleIntervalTimer* jiggle_interval = new JiggleIntervalTimer(led, jiggle);
  events.add(jiggle_interval);
  events.add(new SetBatteryTimer());
  events.add(new BootselButton(jiggle_interval));
  events.setup();
  Serial.println("Start Mouse Jiggle");

  led->run_now(3);
}

void loop() {
  events.run();
}
