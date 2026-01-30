#!/usr/bin/env python3

import argparse
import sys
import threading

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
import rospy
from sensor_msgs.msg import Imu
import yaml

from rcb4.imu_utils import fit_sphere_to_imu_data


class ImuCalibratorNode:
    def __init__(self, output_file=None):
        rospy.init_node('imu_calibrator')
        self.output_file = output_file
        self.data_points = []
        self.is_collecting = False
        self.calibration_done = False
        self.fitted_params = None
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber("imu", Imu, self.callback)
        self.setup_plot()

    def setup_plot(self):
        """Initialize Matplotlib GUI."""
        self.fig = plt.figure(figsize=(10, 8))
        self.fig.canvas.manager.set_window_title('IMU Sphere Calibrator')

        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('Accel X')
        self.ax.set_ylabel('Accel Y')
        self.ax.set_zlabel('Accel Z')

        limit = 15.0
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_zlim(-limit, limit)

        self.scatter = self.ax.scatter([], [], [], c='b', marker='.', s=2, alpha=0.6)
        self.wireframe = None

        self.status_text = self.fig.text(0.02, 0.95, "Status: IDLE", fontsize=12, color='black', weight='bold')
        self.count_text = self.fig.text(0.02, 0.92, "Data Count: 0", fontsize=12, color='black')
        self.guide_text = self.fig.text(0.02, 0.80, "", fontsize=10, color='red')

        ax_start = plt.axes([0.1, 0.05, 0.15, 0.075])
        ax_stop = plt.axes([0.26, 0.05, 0.15, 0.075])
        ax_reset = plt.axes([0.42, 0.05, 0.15, 0.075])
        ax_calib = plt.axes([0.58, 0.05, 0.15, 0.075])

        self.btn_start = Button(ax_start, 'Start')
        self.btn_stop = Button(ax_stop, 'Stop')
        self.btn_reset = Button(ax_reset, 'Reset')
        self.btn_calib = Button(ax_calib, 'Calibrate')

        self.btn_start.on_clicked(self.on_start)
        self.btn_stop.on_clicked(self.on_stop)
        self.btn_reset.on_clicked(self.on_reset)
        self.btn_calib.on_clicked(self.on_calibrate)

    def callback(self, msg):
        """ROS topic callback."""
        if self.is_collecting:
            with self.lock:
                self.data_points.append([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ])

    def update(self, frame):
        """Update animation frame."""
        with self.lock:
            current_len = len(self.data_points)
            if current_len > 0:
                arr = np.array(self.data_points)
                step = max(1, current_len // 1000)
                display_data = arr[::step]

                self.scatter._offsets3d = (display_data[:, 0], display_data[:, 1], display_data[:, 2])
            else:
                self.scatter._offsets3d = ([], [], [])

            self.count_text.set_text(f"Data Count: {current_len}")

            if self.is_collecting and not self.calibration_done and current_len > 10:
                if frame % 10 == 0:
                    msg = self.check_coverage(arr)
                    self.guide_text.set_text(msg)
                    self.guide_text.set_color("green" if "GOOD" in msg else "red")

        return self.scatter, self.count_text, self.guide_text

    def on_start(self, event):
        self.is_collecting = True
        self.calibration_done = False

        if self.wireframe is not None:
            self.wireframe.remove()
            self.wireframe = None

        self.status_text.set_text("Status: COLLECTING...")
        self.status_text.set_color("green")
        print("Collection Started.")

    def on_stop(self, event):
        self.is_collecting = False
        self.status_text.set_text("Status: PAUSED")
        self.status_text.set_color("orange")
        print("Collection Paused.")

    def on_reset(self, event):
        self.is_collecting = False
        with self.lock:
            self.data_points = []
        self.calibration_done = False

        if self.wireframe is not None:
            self.wireframe.remove()
            self.wireframe = None

        self.status_text.set_text("Status: CLEARED")
        self.status_text.set_color("black")
        self.guide_text.set_text("")
        print("Data Reset.")

    def on_calibrate(self, event):
        with self.lock:
            pts = list(self.data_points)

        if len(pts) < 50:
            print("Not enough data points!")
            self.status_text.set_text("Status: ERROR (Need >50 pts)")
            return

        self.is_collecting = False
        self.status_text.set_text("Status: CALCULATING...")
        self.fig.canvas.draw_idle()

        try:
            print("Running optimization...")
            bias, scale = fit_sphere_to_imu_data(pts)
            self.fitted_params = (bias, scale)
            self.calibration_done = True

            self.status_text.set_text("Status: CALIBRATED!")
            self.status_text.set_color("blue")
            self.show_calibration_result(bias, scale)

        except Exception as e:
            rospy.logerr(f"Calibration failed: {e}")
            self.status_text.set_text("Status: FAILED")

    def check_coverage(self, arr):
        """Check data coverage."""
        min_vals = np.min(arr, axis=0)
        max_vals = np.max(arr, axis=0)

        threshold = 8.0
        missing = []

        if max_vals[0] < threshold:
            missing.append("+X (Front)")
        if min_vals[0] > -threshold:
            missing.append("-X (Back)")
        if max_vals[1] < threshold:
            missing.append("+Y (Left)")
        if min_vals[1] > -threshold:
            missing.append("-Y (Right)")
        if max_vals[2] < threshold:
            missing.append("+Z (Top)")
        if min_vals[2] > -threshold:
            missing.append("-Z (Bottom)")

        if not missing:
            return "Coverage: GOOD! Ready."
        else:
            return "Missing directions:\n" + "\n".join(missing)

    def draw_fitted_sphere(self, bias, scale):
        """Draw wireframe sphere based on calculated parameters."""
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = 9.8 * np.outer(np.cos(u), np.sin(v))
        y = 9.8 * np.outer(np.sin(u), np.sin(v))
        z = 9.8 * np.outer(np.ones(np.size(u)), np.cos(v))

        x_raw = (x / scale[0]) + bias[0]
        y_raw = (y / scale[1]) + bias[1]
        z_raw = (z / scale[2]) + bias[2]

        self.wireframe = self.ax.plot_wireframe(x_raw, y_raw, z_raw, color='g', alpha=0.3, label='Fitted Sphere')

    def show_calibration_result(self, bias, scale):
        """Output results and draw sphere."""
        print(f"\nBias:  {bias.tolist()}")
        print(f"Scale: {scale.tolist()}")

        result_yaml = {
            "imu_calibration": {
                "bias": bias.tolist(),
                "scale": scale.tolist()
            }
        }
        self.draw_fitted_sphere(bias, scale)

        if self.output_file:
            try:
                with open(self.output_file, 'w') as f:
                    yaml.dump(result_yaml, f, default_flow_style=None)
                print(f"\n[Success] Calibration saved to: {self.output_file}")
                self.status_text.set_text("Status: SAVED!")
            except Exception as e:
                rospy.logerr(f"Failed to save yaml: {e}")
                self.status_text.set_text("Status: SAVE ERROR")

    def run(self):
        print("=== IMU Interactive Calibrator ===")
        print("GUI window opened.")

        _ = FuncAnimation(self.fig, self.update, interval=100, blit=False)
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='IMU Calibrator')
    parser.add_argument('-o', '--output', type=str, default=None, help='Output YAML file path')
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])
    try:
        ImuCalibratorNode(output_file=args.output).run()
    except rospy.ROSInterruptException:
        pass
