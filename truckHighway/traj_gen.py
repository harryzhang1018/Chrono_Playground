import numpy as np

def generate_ellipse_trajectory(a, b, num_points, center_x=0, center_y=0):
    """
    Generate an ellipse trajectory with x, y coordinates and heading.
    
    :param a: Semi-major axis of the ellipse
    :param b: Semi-minor axis of the ellipse
    :param num_points: Number of points to generate along the trajectory
    :param center_x: X-coordinate of the ellipse center (default: 0)
    :param center_y: Y-coordinate of the ellipse center (default: 0)
    :return: List of tuples (x, y, heading) for each point on the trajectory
    """
    t = np.linspace(0, 2*np.pi, num_points)
    x = center_x + a * np.cos(t)
    y = center_y + b * np.sin(t)
    
    # Calculate heading using atan2(delta_y, delta_x)
    dx = np.diff(x, append=x[0])
    dy = np.diff(y, append=y[0])
    heading = np.arctan2(dy, dx)
    
    return list(zip(x, y, heading))

# Example usage
if __name__ == "__main__":
    a, b = 500, 300  # Semi-major and semi-minor axes
    num_points = 10000
    trajectory = generate_ellipse_trajectory(a, b, num_points)
    
    # # Print the first few points of the trajectory
    # for i, (x, y, heading) in enumerate(trajectory[:5]):
    #         print(f"Point {i}: x={x:.2f}, y={y:.2f}, heading={np.degrees(heading):.2f} degrees")
    # save the trajectory to a file
    np.savetxt('trajectory.csv', trajectory, delimiter=',',fmt='%f')
    # Visualize the trajectory
    import matplotlib.pyplot as plt

    # Extract x and y coordinates from the trajectory
    x_coords, y_coords, _ = zip(*trajectory)

    # Create a new figure and plot the trajectory
    plt.figure(figsize=(10, 8))
    plt.plot(x_coords, y_coords, 'b-')
    plt.scatter(x_coords, y_coords, c='r', s=20)
    plt.title('Ellipse Trajectory')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.grid(True)
    plt.show()