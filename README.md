# buttons Package

The `buttons` package integrates physical buttons and a small GUI.

## Features

- **Physical Buttons**
  - Arm/disarm either the vehicle or both the vehicle and manipulator.
  - Reboot the PX4 flight controller.
  - Restart EKF2.
  
- **Buzzer Alert**
  - Activates when the voltage of any battery cell drops below 3.35V.

- **Graphical User Interface (GUI)**
  - Displays:
    - Arming state of the **AUV (vehicle)**.
    - Arming state of the **Arm (manipulator)**.
    - Total voltage and cell voltage.
  - The buttons control **only one** of the vehicles, while the second is for **display only**.

---

## Launch Files

### 1. `buttons_0.launch.py`

- Behavior of pressing arming/disarming buttons depends on the `use_manipulator` parameter:
    - `True`: Arms/disarms both vehicle and manipulator.
    - `False`: Only affects the vehicle.

```bash
ros2 launch buttons button_0.launch.py vehicle_name:=klopsi00 use_manipulator:=true
```

### 2. `buzzer.launch.py`

```bash
ros2 launch buttons buzzer.launch.py vehicle_name:=klopsi00
```

### 3. `gui.launch.py`

```bash
ros2 launch buttons gui.launch.py vehicle_names:='[klopsi00, bluerov01]'
```