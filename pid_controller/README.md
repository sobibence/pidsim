# PID Temperature Controller for ESP32

A professional-grade PID temperature controller for ESP32, featuring zero-cross detection AC dimmer control, dual heating elements, temperature sensor calibration, and potentiometer setpoint adjustment.

## 🌟 Features

- **Professional PID Control** - Full PID with anti-windup and feed-forward
- **AC Dimmer Control** - Phase-controlled TRIAC with zero-cross detection
- **RMS Power Calculation** - Accurate power measurement for AC phase control
- **Dual Heating Elements** - Seamless control of dimmer + relay for extended range
- **Temperature Calibration** - Offset and scale adjustment for sensor accuracy
- **Potentiometer Setpoint** - Optional analog control of target temperature
- **Safety Features** - Automatic shutdown on sensor fault
- **Object-Oriented** - Clean C++ class architecture

## 📊 System Architecture

```
                        ┌──────────────┐
                        │ Potentiometer│
                        │  (Optional)  │
                        └──────┬───────┘
                               │ ADC
                ┌──────────────▼─────────────┐
Zero-Cross ────>│   PID Temperature         │
Detector        │   Controller              │
                │                            │
MAX6675 ────SPI>│  - PID Algorithm          │
Sensor          │  - Feed-forward           │
                │  - Anti-windup            │
                └──┬───────────────────┬─────┘
                   │                   │
              TRIAC Gate           Relay
                   │                   │
                   v                   v
            ┌──────────┐        ┌──────────┐
            │ Dimmer   │        │  Relay   │
            │ Element  │        │ Element  │
            │ (1800W)  │        │ (1800W)  │
            └──────────┘        └──────────┘
             Phase Control       ON/OFF
```

## 🔌 Hardware Connections

### ESP32-H2 Pinout

| Component          | Pin     | ESP32 GPIO | Description                    |
|--------------------|---------|-----------|--------------------------------|
| MAX6675 SCK        | 1       | GPIO 1    | SPI Clock                      |
| MAX6675 CS         | 2       | GPIO 2    | Chip Select                    |
| MAX6675 SO         | 3       | GPIO 3    | MISO (Data Out)                |
| TRIAC Gate         | 4       | GPIO 4    | Gate control for AC dimmer     |
| Zero-Cross Input   | 5       | GPIO 5    | Zero-cross detector signal     |
| Relay Control      | 6       | GPIO 6    | Relay for 2nd element          |
| Potentiometer      | ADC0    | GPIO 36   | Analog setpoint (0-3.3V)       |

### Zero-Cross Detector Circuit

```
AC Mains ──┬──────────────┐
           │              │
        Bridge           Load
       Rectifier          
           │              │
           ├──> 4N35 ─────┤──> GPIO 5 (ESP32)
           │   Optocoupler│
          GND            GND
```

### TRIAC Driver Circuit

```
GPIO 4 ──> MOC3021 ──┬──> TRIAC Gate
          Opto-TRIAC │
                     └──> TRIAC MT1
```

## ⚙️ Configuration

All settings are in [main.cpp](main/main.cpp):

### Basic Settings

```cpp
// Temperature sensor calibration
constexpr float TEMP_SENSOR_OFFSET = 0.0f;   // °C offset
constexpr float TEMP_SENSOR_SCALE  = 1.0f;   // Scale factor

// PID parameters
constexpr float PID_KP = 0.5f;               // Proportional
constexpr float PID_KI = 0.0019f;            // Integral
constexpr float PID_KD = 5.0f;               // Derivative
```

### Dual Element Configuration

```cpp
constexpr bool ENABLE_DUAL_ELEMENTS = true;  // Enable/disable
constexpr float ELEMENT_POWER_WATTS = 1800.0f; // Power per element
```

**Logic:** 
- 0-50% output: Relay OFF, Dimmer 0-100%
- 50-100% output: Relay ON, Dimmer 0-100%
- Total power: Up to 3600W (2 × 1800W)

### Setpoint Control

```cpp
constexpr bool USE_POTENTIOMETER = false;    // true = pot, false = fixed
constexpr float FIXED_SETPOINT = 60.0f;      // Fixed temp (°C)
constexpr float POT_MIN_TEMP = 20.0f;        // Pot minimum (°C)
constexpr float POT_MAX_TEMP = 100.0f;       // Pot maximum (°C)
```

### AC Configuration

```cpp
constexpr uint32_t AC_FREQUENCY_HZ = 50;      // 50Hz (EU) or 60Hz (US)
constexpr uint32_t TRIAC_PULSE_WIDTH_US = 10; // Gate pulse width
```

## 📐 RMS Power Calculation

The controller uses proper RMS power calculation for phase-controlled AC:

$$P_{RMS} = P_{max} \left(1 - \frac{\alpha}{\pi} + \frac{\sin(2\alpha)}{2\pi}\right)$$

Where α is the firing angle (0 to π radians).

This ensures accurate power delivery and measurement, unlike simple linear phase control.

## 🏗️ Class Documentation

### ZeroCrossDetector

Detects AC zero-crossing events for synchronized TRIAC firing.

**Features:**
- GPIO interrupt on rising edge
- Configurable frequency (50/60 Hz)
- FreeRTOS semaphore synchronization
- Microsecond-accurate timing

### HeaterController

Phase-controlled AC dimmer with optional dual elements.

**Key Features:**
- Zero-cross synchronized TRIAC control
- RMS power calculation
- Dual element logic (dimmer + relay)
- High-priority FreeRTOS task for timing
- Proper phase angle to power conversion

**Power Mapping:**
```
Single Element Mode:
  0% ────────────────────> 100%
  |        Dimmer          |
  0W                    1800W

Dual Element Mode:
  0%        50%        100%
  |─────────|──────────|
  Relay OFF │ Relay ON │
  Dimmer 0-100% │ Dimmer 0-100%
  0W       1800W     3600W
```

### MAX6675Sensor

Temperature sensing with calibration support.

**Calibration:**
```cpp
// If sensor reads 2°C low:
sensor.setCalibration(+2.0f, 1.0f);

// If sensor reads 5% high:
sensor.setCalibration(0.0f, 0.95f);
```

### SetpointController

ADC-based potentiometer reading for setpoint control.

**Features:**
- 12-bit ADC resolution
- Hardware calibration (when available)
- Configurable temperature range
- Anti-jitter filtering

### PIDController

Discrete PID with enhancements.

**Features:**
- Anti-windup integral clamping
- Derivative filtering
- Feed-forward compensation
- Configurable output limits

## 🔧 Building and Flashing

```bash
# Build
```bash
# Build
idf.py build

# Flash to ESP32
idf.py flash monitor  # Flash and open serial monitor
```

## 📈 Sample Output

```
I (xxx) PID_TEMP_CONTROL: ==================================================
I (xxx) PID_TEMP_CONTROL:   PID Temperature Controller with AC Dimmer
I (xxx) PID_TEMP_CONTROL: ==================================================
I (xxx) PID_TEMP_CONTROL: Configuration:
I (xxx) PID_TEMP_CONTROL:   PID Gains: Kp=0.500, Ki=0.0019, Kd=5.0
I (xxx) PID_TEMP_CONTROL:   Setpoint: FIXED
I (xxx) PID_TEMP_CONTROL:   Fixed Setpoint: 60.0°C
I (xxx) PID_TEMP_CONTROL:   Dual Elements: YES
I (xxx) PID_TEMP_CONTROL:   Max Power: 3600W
I (xxx) PID_TEMP_CONTROL:   AC Frequency: 50Hz
I (xxx) PID_TEMP_CONTROL: ==================================================

I (xxx) PID_TEMP_CONTROL: [    0] Temp: 25.50°C | SP:  60.0°C | Err: +34.50 | 
                                  PID: P:+0.172 I:+0.000 D:+0.173 | Out:  45.7% | 
                                  Dimmer:  91.4% | Relay: OFF | Power: 1645W
I (xxx) PID_TEMP_CONTROL: [    1] Temp: 26.25°C | SP:  60.0°C | Err: +33.75 | 
                                  PID: P:+0.169 I:+0.001 D:+0.150 | Out:  52.1% | 
                                  Dimmer:   4.2% | Relay: ON  | Power: 1876W
```

## 🎯 PID Tuning Guide

Optimized parameters from Python simulation:

| Parameter | Value   | Purpose                        |
|-----------|---------|--------------------------------|
| **Kp**    | 0.5     | Immediate response to error    |
| **Ki**    | 0.0019  | Eliminate steady-state error   |
| **Kd**    | 5.0     | Reduce overshoot, damping      |

### Manual Tuning Steps (Ziegler-Nichols)

1. Set Ki=0, Kd=0
2. Increase Kp until oscillation
3. Note Kp at oscillation (Ku) and period (Tu)
4. Apply formulas:
   - Kp = 0.6 × Ku
   - Ki = 1.2 × Ku / Tu  
   - Kd = 0.075 × Ku × Tu

### Feed-Forward Tuning

Estimate steady-state heat loss (measure at constant temp):
```cpp
constexpr float FEEDFORWARD_POWER = measured_power_at_setpoint;
```

## ⚠️ Safety Features

1. **Sensor Fault Protection** - Heater disabled on thermocouple disconnect
2. **Output Clamping** - Power limited to 0-100% range
3. **Anti-windup** - Prevents integral term runaway
4. **Zero-Cross Sync** - Prevents voltage spikes and EMI
5. **Watchdog** - FreeRTOS task monitoring

## 🔬 Technical Details

### AC Dimmer Operation

1. **Zero-Cross Detection** - Interrupt on AC waveform zero-crossing
2. **Phase Delay** - Calculate delay for desired power
3. **TRIAC Trigger** - Fire gate pulse after delay
4. **RMS Calculation** - Convert phase angle to actual power

### Dual Element Logic

The controller seamlessly combines dimmer and relay:

| Output | Relay | Dimmer | Power    |
|--------|-------|--------|----------|
| 0%     | OFF   | 0%     | 0W       |
| 25%    | OFF   | 50%    | 900W     |
| 50%    | OFF   | 100%   | 1800W    |
| 75%    | ON    | 50%    | 2700W    |
| 100%   | ON    | 100%   | 3600W    |

This provides smooth, continuous control across the full power range.

## 🐛 Troubleshooting

**Zero-cross not detected:**
- Check optocoupler circuit
- Verify GPIO 5 connection
- Test with oscilloscope

**Temperature reading NAN:**
- Check MAX6675 wiring
- Verify thermocouple connection
- Check SPI signals with logic analyzer

**Erratic dimmer control:**
- Verify zero-cross frequency setting
- Check TRIAC gate circuit
- Reduce electromagnetic interference

**Oscillating temperature:**
- Reduce Kp gain
- Increase Kd gain
- Add feed-forward compensation

## 📚 References

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/)
- [MAX6675 Datasheet](https://datasheets.maximintegrated.com/en/ds/MAX6675.pdf)
- [TRIAC Phase Control Theory](https://en.wikipedia.org/wiki/Phase-fired_controller)
- [PID Control Tutorial](https://en.wikipedia.org/wiki/PID_controller)

## 📝 License

This project is provided as-is for educational and commercial use.

## 🤝 Contributing

Based on Python simulation from [pidsim.ipynb](pidsim.ipynb). Contributions welcome!
