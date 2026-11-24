# armada_description

URDF + meshes du robot ARMADA, générés depuis le CAO Fusion 360 (`new_armada.stl` / `.step`).

```bash
# dans un workspace ROS 2
colcon build --packages-select armada_description
ros2 launch armada_description display.launch.py
```

## Repère

REP-103 : **x avant, y gauche, z haut**, unités SI.

| Lien                             | Position (depuis `base_link`)        | Remarque                                                     |
| -------------------------------- | ------------------------------------ | ------------------------------------------------------------ |
| `base_footprint`                 | au sol, sous `base_link`             | z = −38,6 mm (rayon de roue)                                 |
| `base_link`                      | milieu de l'axe des roues            | origine odométrie                                            |
| `wheel_left_link` / `right`      | y = ±47,75 mm                        | joints `continuous`, axe +y, rotation positive = avancer     |
| `caster_link`                    | centre de la bille, x = +67,7 mm     | bille Ø15 mm, collision sphère                               |
| `lidar_link`                     | centre de l'anneau ToF, z = +64,4 mm | repère d'un futur `LaserScan` fusionné                       |
| `tof_{f,fl,l,bl,b,br,r,fr}_link` | face extérieure de chaque VL53L0X    | x = axe optique ; yaw 0°, 45°, …, −45° (canaux PCA9548A 0→7) |
| `ultrasonic_link`                | entre les deux capsules du HC-SR04   | pitch +45° (vers le sol)                                     |
| `imu_link`                       | centre du plateau supérieur          | **position et orientation supposées**                        |

## Notes

- **Masses et inerties : estimees (bancal)**
- **Voie des roues : 95,5 mm**le firmware utilise `ROBOT_WIDTH = 92,5 mm`.
- **IMU** : aucune carte BNO055 identifiable dans le CAO ; `imu_link` suppose la carte plate au centre du plateau haut,
  axes alignés sur le robot. Corriger `rpy` selon le montage réel.
- Limites des joints (`effort`, `velocity`) : ordre de grandeur STS3215 12 V.
- Pas de plugin Gazebo / `ros2_control` : à ajouter selon le simulateur visé.
