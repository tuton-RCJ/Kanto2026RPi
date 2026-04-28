import os
import ydlidar
import time
import math
import matplotlib.pyplot as plt

scan = None
if __name__ == "__main__":
    ydlidar.os_init()
    ports = ydlidar.lidarPortList()
    port = "/dev/ttyAMA2"
    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.02)
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)

    ret = laser.initialize()
    try:
        if ret:
            ret = laser.turnOn()
            scan = ydlidar.LaserScan()
            while ret and ydlidar.os_isOk():
                r = laser.doProcessSimple(scan)
                if r:
                    print("Scan received [", scan.points.size(), "] points")
                else:
                    print("Failed to get lidar data")
                    time.sleep(0.05)
                x_vals, y_vals, intensity_vals = [], [], []

                for i in range(scan.points.size()):
                    p = scan.points[i]
                    if p.range <= 0:
                        continue
                    if p.range >= 200:
                        continue
                    x_vals.append(p.range * math.cos(p.angle))
                    y_vals.append(p.range * math.sin(p.angle))
                    intensity_vals.append(getattr(p, "intensity", 0.0))

                if x_vals:
                    fig, ax = plt.subplots(figsize=(7, 7))
                    use_intensity = any(v > 0 for v in intensity_vals)
                    cloud = ax.scatter(
                        x_vals,
                        y_vals,
                        s=4,
                        c=intensity_vals if use_intensity else "tab:blue",
                        cmap="viridis" if use_intensity else None,
                    )
                    if use_intensity:
                        fig.colorbar(cloud, ax=ax, label="Intensity")
                    ax.set_aspect("equal", adjustable="box")
                    ax.set_xlabel("X [m]")
                    ax.set_ylabel("Y [m]")
                    ax.set_title("YDLidar Point Cloud (last scan)")
                    ax.grid(alpha=0.3)
                    output_file = f"ydlidar_pointcloud.png"
                    plt.savefig(output_file, dpi=200, bbox_inches="tight")
                    print(f"保存しました: {output_file}")
                    plt.close(fig)
            else:
                print("描画可能なスキャンデータがありません。")
                laser.turnOff()
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down.")
    finally:
        laser.disconnecting()

