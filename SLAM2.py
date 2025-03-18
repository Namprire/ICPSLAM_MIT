import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors

def icp(A, B, max_iterations=20, tolerance=1e-6):
    """
    Perform Iterative Closest Point algorithm.
    A: source point cloud
    B: target point cloud
    max_iterations: maximum number of iterations
    tolerance: convergence tolerance
    """
    src = np.copy(A)
    prev_error = 0

    for i in range(max_iterations):
        # Find the nearest neighbors between the current source and destination points
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(B)
        distances, indices = nbrs.kneighbors(src)

        # Compute the transformation between the current source and destination points
        T, _, _ = best_fit_transform(src, B[indices[:, 0]])

        # Transform the current source
        src = np.dot(T[:2, :2], src.T).T + T[:2, 2]

        # Check for convergence
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    # Calculate final transformation
    T, _, _ = best_fit_transform(A, src)

    return T, distances

def best_fit_transform(A, B):
    """
    Calculate the least-squares best-fit transform between corresponding 2D points A and B.
    Returns the transformation matrix T, rotation matrix R, and translation vector t.
    """
    assert A.shape == B.shape

    # Get number of dimensions
    m = A.shape[1]

    # Compute centroids of the point sets
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # Center the points
    AA = A - centroid_A
    BB = B - centroid_B

    # Compute the covariance matrix
    H = np.dot(AA.T, BB)

    # Compute the Singular Value Decomposition
    U, S, Vt = np.linalg.svd(H)

    # Compute the rotation matrix
    R = np.dot(Vt.T, U.T)

    # Ensure a proper rotation (no reflection)
    if np.linalg.det(R) < 0:
        Vt[m-1, :] *= -1
        R = np.dot(Vt.T, U.T)

    # Compute the translation vector
    t = centroid_B.T - np.dot(R, centroid_A.T)

    # Create the transformation matrix
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t

def plot_icp(A, B, T):
    """
    Plot the results of ICP.
    A: source point cloud
    B: target point cloud
    T: transformation matrix
    """
    A_transformed = np.dot(T[:2, :2], A.T).T + T[:2, 2]

    plt.figure()
    plt.scatter(B[:, 0], B[:, 1], c='r', label='Target')
    plt.scatter(A[:, 0], A[:, 1], c='b', label='Source')
    plt.scatter(A_transformed[:, 0], A_transformed[:, 1], c='g', label='Transformed Source')
    plt.legend()
    plt.show()

# Example usage
if __name__ == "__main__":
    # Create two simple point clouds
    A = np.array([[1, 1], [2, 2], [3, 3]])
    B = np.array([[4, 4], [5, 5], [6, 6]])

    # Perform ICP
    T, distances = icp(A, B)

    # Plot the results
    plot_icp(A, B, T)