import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ICP function (same as before)
def icp(source, target, threshold=0.02, max_iterations=50):
    source.estimate_normals()
    target.estimate_normals()
    initial_transformation = np.identity(4)
    result = o3d.pipelines.registration.registration_icp(
        source, target, threshold, initial_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations)
    )
    return result.transformation

# --- Data Generation (Simulate moving object and stationary wall) ---
num_frames = 50  # Number of animation frames
scan_length = 50
scan_radius = 0.1
room_size = 10

# Ground truth trajectory (simple linear motion)
ground_truth_x = np.linspace(0, room_size, num_frames)
ground_truth_y = np.zeros(num_frames)

# Stationary wall (environment)
wall_x = [0, room_size, room_size, 0, 0]
wall_y = [0, 0, room_size, room_size, 0]

# Simulate scans around the moving object
scans = []
for i in range(num_frames):
    angle = np.linspace(0, 2 * np.pi, scan_length)
    x = ground_truth_x[i] + scan_radius * np.cos(angle)
    y = ground_truth_y[i] + scan_radius * np.sin(angle)
    points = np.vstack((x, y, np.zeros(scan_length))).T
    scan = o3d.geometry.PointCloud()
    scan.points = o3d.utility.Vector3dVector(points)
    scans.append(scan)

# --- End of Data Generation ---

# Initialize pose estimation
pose_est = [np.eye(4)]

# Animation setup
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(-2, room_size + 2)
ax.set_ylim(-2, room_size + 2)
ax.set_xlabel("X (meters)")
ax.set_ylabel("Y (meters)")
ax.set_title("2D LIDAR-based SLAM with ICP")
ax.grid(True)
ax.plot(wall_x, wall_y, 'k-', linewidth=2, label="Environment")  # Stationary wall

# Initialize plot elements (to be updated in animation)
ground_truth_line, = ax.plot([], [], 'b-', linewidth=2, label="Ground Truth Trajectory")
estimated_line, = ax.plot([], [], 'r.-', linewidth=1, markersize=8, label="Estimated Trajectory")
scan_scatter = ax.scatter([], [], c='gray', s=1, label="LIDAR Scans")
legend = ax.legend(loc='upper left')

def animate(frame):
    if frame > 0:
        T = icp(scans[frame], scans[frame - 1])
        pose_next = np.dot(pose_est[-1], T)
        pose_est.append(pose_next)
        scans[frame].transform(pose_next)

    estimated_trajectory = np.array([pose[:3, 3] for pose in pose_est])
    ground_truth_line.set_data(ground_truth_x[:frame + 1], ground_truth_y[:frame + 1])
    estimated_line.set_data(estimated_trajectory[:, 0], estimated_trajectory[:, 1])

    points = np.asarray(scans[frame].points)
    scan_scatter.set_offsets(points[:, :2])

    return ground_truth_line, estimated_line, scan_scatter

ani = animation.FuncAnimation(fig, animate, frames=num_frames, interval=200, blit=True)  # Adjust interval for speed
plt.show()