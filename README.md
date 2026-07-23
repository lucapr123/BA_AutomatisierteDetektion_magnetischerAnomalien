# ghost X mag

Bachelor's thesis prototype for automated magnetic anomaly detection with an unmanned ground vehicle.

The system reads two three-axis magnetometers, stores validated measurements, calculates field gradients, detects anomalies, publishes results through ROS 2, and displays them in a Flask dashboard.

Author: **Luca Prior**

## Use

Start:

```sh
python3 Main/Xlaunch.py
```

Stop and create the final plots:

```sh
python3 Main/Xend.py
```

[Demo on YouTube](https://youtu.be/frJqmdIauZg?si=-OxTf2473fRKUhFG)

## Original setup

- Ghost Vision 60 UGV
- Python 3
- ROS 2 with `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, and `cv_bridge`
- pandas, NumPy, Matplotlib, pyserial, Flask, and OpenCV
- Magnetometer serial connection at `/dev/ttyACM1`, 115200 baud
- Project root at `/home/ghost/Magnetometer`

The scripts use fixed paths from the original UGV installation. A direct clone needs the same directory layout or adjusted paths.

## Folders

| Folder | Content |
| --- | --- |
| `Main` | MCU source (`MCU.py`), data collection, start, and stop scripts |
| `Echtzeit` | Real-time plots and anomaly detection |
| `Gesamt` | Final processing and plots for a complete measurement |
| `ROS2_Workspace` | ROS 2 publishers |
| `FLASK` | Web dashboard |
| `Detektion` | Example anomaly output |

This is a research prototype, not a certified ordnance detection or safety system.

## License

[MIT](LICENSE)
