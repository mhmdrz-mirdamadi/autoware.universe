import argparse
import csv
import os

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


def read_csv(directory, filename):
    filepath = os.path.join(directory, filename)
    with open(filepath, "r") as file:
        reader = csv.reader(file)
        data = list(reader)
    return [[float(x) for x in row if x] for row in data]


def find_latest_directory(log_directory):
    trajectory_directories = [
        d
        for d in os.listdir(log_directory)
        if os.path.isdir(os.path.join(log_directory, d)) and d.startswith("trajectory_")
    ]
    if trajectory_directories:
        return max(trajectory_directories)
    return None


parser = argparse.ArgumentParser(description="Plot trajectory data from CSV files.")
parser.add_argument("--directory", type=str, help="Directory containing the CSV files")
args = parser.parse_args()

home_directory = os.path.expanduser("~")
log_directory = os.path.join(home_directory, ".ros", "log")

if args.directory:
    data_directory = args.directory
else:
    latest_directory = find_latest_directory(log_directory)
    if latest_directory:
        data_directory = os.path.join(log_directory, latest_directory)
    else:
        print("No trajectory directory found.")
        exit(1)

resampled_data = read_csv(data_directory, "resampled_x.log")
predicted_data = read_csv(data_directory, "predicted_x.log")
predicted_frenet_data = read_csv(data_directory, "predicted_frenet_x.log")
cgmres_predicted_frenet_data = read_csv(data_directory, "cgmres_predicted_frenet_x.log")
cgmres_predicted_data = read_csv(data_directory, "cgmres_predicted_x.log")
time_data = read_csv(data_directory, "time.log")

fig, ax = plt.subplots(figsize=(8, 6))
plt.subplots_adjust(bottom=0.2)

(resampled_plot,) = ax.plot([], [], marker="s", label="Resampled Reference Trajectory")
(predicted_plot,) = ax.plot([], [], marker="d", label="Predicted Trajectory")
(predicted_frenet_plot,) = ax.plot([], [], marker="^", label="Predicted Frenet Trajectory")
(cgmres_predicted_frenet_plot,) = ax.plot(
    [], [], marker="v", label="CGMRES Predicted Frenet Trajectory"
)
(cgmres_predicted_plot,) = ax.plot([], [], marker="<", label="CGMRES Predicted Trajectory")

ax.set_xlabel("X")
ax.set_ylabel("Y")
directory_name = os.path.basename(data_directory)
ax.set_title(f"Trajectory Comparison\n{directory_name}")
ax.legend()
ax.grid(True)
plt.gca().set_aspect("equal", adjustable="box")

slider_ax = plt.axes([0.2, 0.05, 0.6, 0.03])
time_slider = Slider(slider_ax, "Time", 0, len(time_data) - 1, valinit=0, valstep=1)


def update(time_index):
    resampled_x = resampled_data[time_index]
    resampled_y = read_csv(data_directory, "resampled_y.log")[time_index]
    predicted_x = predicted_data[time_index]
    predicted_y = read_csv(data_directory, "predicted_y.log")[time_index]
    predicted_frenet_x = predicted_frenet_data[time_index]
    predicted_frenet_y = read_csv(data_directory, "predicted_frenet_y.log")[time_index]
    cgmres_predicted_frenet_x = cgmres_predicted_frenet_data[time_index]
    cgmres_predicted_frenet_y = read_csv(data_directory, "cgmres_predicted_frenet_y.log")[
        time_index
    ]
    cgmres_predicted_x = cgmres_predicted_data[time_index]
    cgmres_predicted_y = read_csv(data_directory, "cgmres_predicted_y.log")[time_index]

    resampled_plot.set_data(resampled_x, resampled_y)
    predicted_plot.set_data(predicted_x, predicted_y)
    predicted_frenet_plot.set_data(predicted_frenet_x, predicted_frenet_y)
    cgmres_predicted_frenet_plot.set_data(cgmres_predicted_frenet_x, cgmres_predicted_frenet_y)
    cgmres_predicted_plot.set_data(cgmres_predicted_x, cgmres_predicted_y)

    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw_idle()


time_slider.on_changed(update)
update(0)

plt.show()
