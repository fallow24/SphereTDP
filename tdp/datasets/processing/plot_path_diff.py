import sys
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass


@dataclass
class Pose:
    x: float
    y: float
    z: float
    yaw: float
    pitch: float
    roll: float


def norm(angle: float) -> float:
    normed_angle = angle
    if normed_angle < -180.0:
        normed_angle += 360.0
    elif normed_angle > 180.0:
        normed_angle -= 360.0

    return normed_angle


def load_poses(path: str, num_poses: int) -> list:
    poses = []
    for i in range(num_poses):
        pose = np.matrix.flatten(np.loadtxt(path + f"scan{i:03}.pose"))
        poses.append(Pose(pose[0], pose[1], pose[2],
                          pose[3], pose[4], pose[5]))

    return poses


def compute_diff(true_traj: list, other_traj: list) -> tuple:
    position_diff = []
    orientation_diff = []
    for i in range(len(true_traj)):
        pose_t = true_traj[i]
        pose_o = other_traj[i]
        position_diff.append(
            np.sqrt(
                (pose_t.x - pose_o.x)**2
                + (pose_t.y - pose_o.y)**2
                + (pose_t.z - pose_o.z)**2))

        yaw_diff = norm(pose_t.yaw - pose_o.yaw)
        pitch_diff = norm(pose_t.pitch - pose_o.pitch)
        roll_diff = norm(pose_t.roll - pose_o.roll)
        orientation_diff.append(
            np.sqrt((yaw_diff)**2 + (pitch_diff)**2 + (roll_diff)**2))

    return position_diff, orientation_diff


def main():
    if len(sys.argv) != 4:
        print("Not both paths given. Usage: ")
        print("python plot_path_diff.py <path_to_truth>" +
              " <path_to_other> <num_of_pose_files>")
        exit(0)

    path_truth = str(sys.argv[1])
    path_other = str(sys.argv[2])
    num_poses = int(sys.argv[3])

    true_traj = load_poses(path_truth, num_poses)
    other_traj = load_poses(path_other, num_poses)

    position_diff, orientation_diff = compute_diff(true_traj, other_traj)

    print(f"Sum of Position Difference: {np.sum(position_diff):.2f} cm")
    print(f"Sum of Orientation Difference: {np.sum(orientation_diff):.2f} °")


    plt.figure(1)
    plt.plot(position_diff)
    plt.grid()
    plt.ylim([0, 30])
    plt.xlabel("Scan Number")
    plt.ylabel("Euclidean distance to Ground Truth [m]")
    plt.savefig(f"position_diff_{path_other.replace('.','').replace('/','')}.eps", format='eps')

    plt.figure(2)
    plt.plot(orientation_diff)
    plt.grid()
    plt.ylim([0, 365])
    plt.xlabel("Scan Number")
    plt.ylabel("Euclidean distance to Ground Truth [°]")
    plt.savefig(f"orientation_diff_{path_other.replace('.','').replace('/','')}.eps", format='eps')


if __name__ == "__main__":
    # execute only if run as a script
    main()
