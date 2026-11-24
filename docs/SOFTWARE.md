# Software guide

Flash the firmware with the Arduino IDE, then drive the robot. Wire it first: [hardware guide](HARDWARE.md).

**For experts:** Arduino IDE 2.x, ESP32 core 3.x (tested with 3.3.3), board *ESP32 Dev Module*, libraries `SCServo` and
`Adafruit VL53L0X`, sketch `Armada_robot/armada_esp/armada_esp.ino`, upload with the robot OFF, then join `ARMADA-XXXX`
and open `http://192.168.4.1`.

## 1. Install

1. Install the [Arduino IDE 2.x](https://www.arduino.cc/en/software). If your computer doesn't see the board, install
   its USB driver (usually **CP210x** or **CH340**).
2. **File → Preferences**, add this to *Additional boards manager URLs*:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. **Tools → Board → Boards Manager**: install **esp32 by Espressif Systems**.
4. **Sketch → Include Library → Manage Libraries**: install **Adafruit VL53L0X** (accept *Adafruit BusIO*).
5. Install **SCServo** (Feetech / Waveshare). If it isn't in the Library Manager, download it from the Waveshare
   Bus Servo Adapter page and use **Sketch → Include Library → Add .ZIP Library**.

## 2. Open the program

Open **`Armada_robot/armada_esp/armada_esp.ino`** (the other files in the folder open as tabs). The sketches in
`tests/` are standalone hardware checks, not the robot program.

Check the settings at the top of the file: servo IDs, `LEFT_INVERTED`, pins, wheel size, and **change
`AP_PASSWORD`** (the Wi-Fi password).

## 3. Flash

1. Switch the robot **OFF**, connect the ESP32 to your computer by USB.
2. **Tools → Board → esp32 → ESP32 Dev Module**.
3. **Tools → Port**: pick the new serial port.
4. Click **Upload**. If it hangs on *Connecting…*, hold the **BOOT** button on the board.
5. Optional: **Tools → Serial Monitor** at **115200 baud** shows the Wi-Fi name and password.
6. Unplug USB and switch the robot on from the battery.

| Problem | Likely cause |
|---|---|
| Port not listed | Charge-only cable or missing driver |
| `SCServo.h` / `Adafruit_VL53L0X.h` not found | Library not installed |
| `Erreur init : pas de reponse du servo` | Servo IDs, driver jumper (must be **A**), TX/RX wiring, or no 12 V |
| Reboots / `Brownout` | Weak power: use the battery |

## 4. Drive

1. Join the Wi-Fi network **`ARMADA-XXXX`** (default password `armada1234`, unless you changed it).
2. Open **`http://192.168.4.1`** in a browser.
3. Use the joystick (touch, mouse, arrows or ZQSD) and the speed slider. Circles show the distance (cm) seen by each
   ToF sensor. The **Sol** badge at the top shows the floor distance and turns **red** when it is too close, too far
   or missing. **STOP** stops the robot.

The robot also stops by itself if the page stops talking to it (Wi-Fi lost, page closed). For the first test, keep the
wheels in the air.

## Using the code in your own program

`Robot` can be used without the web page:

```cpp
robot.init();                       // in setup()
robot.set_velocity(0.1f, 0.0f);     // m/s, rad/s
robot.update();                     // every loop: safety timeout
robot.lidar().update_next();        // every loop: one ToF sensor per call
robot.floor_sensor().update();      // every loop: floor sensor
robot.update_odometry();            // regularly; then robot.get_odometry(x, y, theta)
```
