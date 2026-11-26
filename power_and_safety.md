# Pharma Rover: Power Budget & Safety Checklist

## 1. Power Architecture & Rail Separation

```
[ 7.4V 2S LiPo / 18650 Pack (2000-3000 mAh) ]
   |
   +---> Emergency E-Stop Switch (Hardware NC)
           |
           +---> Motor Driver (TB6612FNG / L298N) VM Rail (7.4V)
           |
           +---> Buck Converter 1 (6.0V, 5A peak) ---> PCA9685 Servo V+ Rail (DC Barrel Jack)
           |
           +---> Buck Converter 2 (5.0V, 3A) -------> Logic VCC (ESP32, Arduino Mega, Uno TFT, ESP32-CAM)
```

> [!CAUTION]
> **Common Ground Rule:** All logic grounds and power grounds (LiPo GND, Buck 1 GND, Buck 2 GND, Mega GND, ESP32 GND, Uno GND, PCA9685 GND) **MUST be connected together**. Never float grounds between boards.
> **Isolation Rule:** Never connect the 6.0 V servo rail to any 5.0 V or 3.3 V logic pins.

---

## 2. Power Budget Table

| Component | Supply Voltage | Idle Current | Peak / Stall Current | Dedicated Supply Rail | Safety Notes |
|---|---|---|---|---|---|
| **3x MG995 Servos** (Yaw, Shoulder, Elbow) | 6.0 V | 30 mA (total) | **~3.6 A** (1.2 A stall each) | Buck 1 (6.0 V, 5 A) | **Overloaded shoulder**; peak inrush on power-up |
| **3x SG90 Servos** (Wrist pitch, Roll, Grip) | 6.0 V | 30 mA (total) | **~1.8 A** (600 mA stall each) | Buck 1 (6.0 V, 5 A) | **Overloaded wrist pitch**; fragile plastic gears |
| **PCA9685 PWM Board** | 5.0 V (logic) / 6.0 V (servos) | 10 mA | - | Buck 2 (logic) / Buck 1 (servos) | Must use **>= 5A rated DC barrel connector** |
| **ESP32 Gateway (Car/Arm/Display)** | 5.0 V (Vin) / 3.3 V | 80 mA | **~250 mA** (WiFi TX spikes) | Buck 2 (5.0 V, 3 A) | Add 100 uF capacitor on Vin |
| **ESP32-CAM (Gripper Hand Camera)** | 5.0 V (5V pin) | 120 mA | **~310 mA** (WiFi TX burst) | Buck 2 (5.0 V, 3 A) | **Add 470 uF capacitor across 5V/GND at camera** |
| **Arduino Uno + 3.5" TFT LCD Shield** | 5.0 V (5V pin) | 180 mA | ~250 mA (Backlight) | Buck 2 (5.0 V, 3 A) | Shield draws backlight from 5V rail |
| **Arduino Mega 2560 (Motor/Arm Controller)** | 5.0 V (5V pin) | 70 mA | ~120 mA | Buck 2 (5.0 V, 3 A) | Drives TB6612 PWM and PCA9685 I2C |
| **2x DC Gearmotors (Drive Base)** | 7.4 V (VM) | 150 mA | **~2.2 A** (Stall) | Battery via E-Stop | E-Stop cuts motor & servo rails immediately |

### Total Peak Consumption:
- **6.0 V Servo Rail:** Peak **~5.4 A** (during full arm motion + lift).
- **5.0 V Logic Rail:** Peak **~0.95 A**.
- **7.4 V Motor Rail:** Peak **~2.2 A**.

---

## 3. First Power-Up Procedure (Go / No-Go Gate)

1. **Bench Test Without Servos:**
   - Turn on battery with servo rail fuse/jumper removed.
   - Verify 5.0 V logic rail is steady between 4.9 V and 5.1 V.
   - Verify 6.0 V servo rail is between 5.9 V and 6.1 V.
2. **First Servo Connection (Soft-Start Protocol):**
   - Connect the PCA9685 DC barrel connector (verify centre-positive polarity).
   - Verify 1000 uF electrolytic capacitor is installed across V+ and GND on the PCA9685 screw terminal.
   - Hold the arm by hand to support its weight.
   - Energise channels one at a time using `soft_start()` in `arm_client.py`.
   - Confirm no servo buzzes, twitches aggressively, or pulls more than 100 mA at rest.
3. **E-Stop Verification:**
   - Press the hardware E-Stop button.
   - Verify that all servos go limp (power cut) and car drive motors halt immediately.
   - Verify that logic boards (ESP32, Mega, Uno, Camera) remain powered on and reporting E-Stop triggered status.
