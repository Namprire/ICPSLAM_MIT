import unittest
from src.slam import SLAM

class TestSLAM(unittest.TestCase):

    def setUp(self):
        self.slam = SLAM()

    def test_initialization(self):
        self.assertIsNotNone(self.slam)
        self.assertEqual(self.slam.get_map(), {})

    def test_update(self):
        initial_map = self.slam.get_map()
        self.slam.update((1, 1), [(2, 2), (3, 3)])
        updated_map = self.slam.get_map()
        self.assertNotEqual(initial_map, updated_map)

    def test_get_map(self):
        self.slam.update((1, 1), [(2, 2), (3, 3)])
        current_map = self.slam.get_map()
        self.assertIn((2, 2), current_map)
        self.assertIn((3, 3), current_map)

if __name__ == '__main__':
    unittest.main()