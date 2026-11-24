# Theory (short)

A quick overview of how the robot works. No need to read this to build or flash it.

## Differential drive

Two driven wheels, a distance `L` apart. To get a linear speed `vx` (m/s) and angular speed `ω` (rad/s):

```
v_left  = vx − ω · L / 2
v_right = vx + ω · L / 2
rpm     = v / (π · wheel diameter) · 60
```

Same speed on both wheels: straight line. Opposite speeds: spin in place. The two servos face each other, so one is
inverted in software.

## Servos as wheels

The STS3215 is a serial bus servo (1 Mbaud, one ID per servo). In *wheel mode* it turns continuously at a speed
setpoint, and it reports its position (4096 steps per turn) and speed, so no separate encoder is needed.

## Odometry

The position of each wheel gives the distance it travelled since the last reading:

```
d  = (d_left + d_right) / 2
Δθ = (d_right − d_left) / L
x += d · cos(θ + Δθ/2),  y += d · sin(θ + Δθ/2),  θ += Δθ
```

It drifts over time (slip, rounding), so it is meant to be fused with other sensors later (IMU).

## Sensors

- **ToF ring**: each VL53L0X measures distance with infrared light. They all have the same I²C address, so a
  PCA9548A multiplexer connects them one at a time. Range is limited to about 1.2 m in the code.
- **Floor sensor**: the HC-SR04 times an ultrasonic echo (`distance = time · 343 m/s / 2`). Pointed at the floor, a
  distance that is too small, too large, or missing means an obstacle or a drop.

## Safety

The web page sends a command every 100 ms. If none arrives for 400 ms, the firmware stops both motors.
