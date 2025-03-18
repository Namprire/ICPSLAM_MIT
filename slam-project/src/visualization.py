import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def animate(robot_path, map_points):
    fig, ax = plt.subplots()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    line, = ax.plot([], [], 'r-', lw=2, label='Robot Path')
    scatter = ax.scatter([], [], c='b', label='Map Points')

    def init():
        line.set_data([], [])
        scatter.set_offsets([])
        return line, scatter

    def update(frame):
        line.set_data(robot_path[:frame, 0], robot_path[:frame, 1])
        scatter.set_offsets(map_points)
        return line, scatter

    ani = animation.FuncAnimation(fig, update, frames=len(robot_path), init_func=init, blit=True)
    plt.legend()
    plt.title('SLAM Visualization')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid()
    plt.show()