import unittest
import numpy as np
from src.icp import ICP

class TestICP(unittest.TestCase):

    def setUp(self):
        self.icp = ICP()

    def test_fit(self):
        # Create two simple point clouds
        source = np.array([[0, 0], [1, 0], [0, 1]])
        target = np.array([[1, 1], [2, 1], [1, 2]])
        
        # Run ICP
        transformation = self.icp.fit(source, target)
        
        # Check if the transformation is correct
        expected_transformation = np.array([[1, 0, 1], [0, 1, 1], [0, 0, 1]])
        np.testing.assert_array_almost_equal(transformation, expected_transformation, decimal=5)

    def test_compute_error(self):
        # Create two point clouds
        source = np.array([[0, 0], [1, 0], [0, 1]])
        target = np.array([[1, 1], [2, 1], [1, 2]])
        
        # Compute error
        error = self.icp.compute_error(source, target)
        
        # Check if the error is computed correctly
        expected_error = np.sqrt(2)  # Example expected error
        self.assertAlmostEqual(error, expected_error, places=5)

if __name__ == '__main__':
    unittest.main()