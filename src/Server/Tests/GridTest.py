print("Running GridTest.py")

import unittest
import numpy as np
import sys
import os
import cv2
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from Server.Components.GridGeneration import generate_grid, remove_x_from_grid, find_rectangular_course, process_grid

class TestGridProcessing(unittest.TestCase):

    def test_generate_grid(self):
        binary_image = np.array([[0, 0, 255, 255],
                                 [0, 0, 255, 255],
                                 [255, 255, 0, 0],
                                 [255, 255, 0, 0]])
        expected_grid = np.array([[0, 1],
                                  [1, 0]])
        generated_grid = generate_grid(binary_image, interval=2)
        print("Generated Grid:\n", generated_grid)
        print("Expected Grid:\n", expected_grid)
        np.testing.assert_array_equal(generated_grid, expected_grid)

    def test_remove_x_from_grid(self):
        grid = np.array([[0, 1, 0, 0],
                         [1, 1, 1, 0],
                         [0, 1, 0, 0],
                         [0, 0, 0, 0]])
        expected_grid = np.array([[0, 1, 0, 0],
                                  [1, 0, 0, 0],
                                  [0, 0, 0, 0],
                                  [0, 0, 0, 0]])
        x_range = (1, 3)
        y_range = (1, 3)
        interval = 1
        updated_grid = remove_x_from_grid(grid, x_range, y_range, interval)
        print("Updated Grid after removing X:\n", updated_grid)
        print("Expected Grid after removing X:\n", expected_grid)
        np.testing.assert_array_equal(updated_grid, expected_grid)
    
    def test_find_rectangular_course(self):
        grid = np.array([[0, 0, 0, 0],
                         [0, 1, 1, 0],
                         [0, 1, 1, 0],
                         [0, 0, 0, 0]])
        expected_coords = [(1, 1), (1, 2), (2, 1), (2, 2)]
        found_coords = find_rectangular_course(grid)
        print("Found Coordinates:", found_coords)
        print("Expected Coordinates:", expected_coords)
        self.assertEqual(found_coords, expected_coords)

    def test_process_grid(self):
        binary_image = np.array([[0, 0, 255, 255, 0, 0],
                                 [0, 0, 255, 255, 0, 0],
                                 [255, 255, 0, 0, 255, 255],
                                 [255, 255, 0, 0, 255, 255],
                                 [0, 0, 255, 255, 0, 0],
                                 [0, 0, 255, 255, 0, 0]])
        interval = 2
        x_range = (1, 3)
        y_range = (1, 3)
        expected_coords = ((1, 2), (2, 1))
        
        grid = generate_grid(binary_image, interval)
        print("Generated Grid:\n", grid)
        
        grid = remove_x_from_grid(grid, x_range, y_range, interval)
        print("Grid after removing X:\n", grid)

        processed_coords = find_rectangular_course(grid)
        print("Processed Coordinates:", processed_coords)

        self.assertTrue(all(coord in processed_coords for coord in expected_coords))


if __name__ == '__main__':
    unittest.main()