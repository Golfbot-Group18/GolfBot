import unittest

from src.Server.Utils.px_conversion import realCoordinates

class TestRealCoordiantes(unittest.TestCase):

    def assertAlmostEqualWithTolerance(self, calculated, expected, tolerance):
        self.assertTrue(abs(calculated[0] - expected[0]) <= tolerance,
                        f"X coordinate is out of tolerance: {calculated[0]} != {expected[0]} within {tolerance} pixels")
        self.assertTrue(abs(calculated[1] - expected[1]) <= tolerance,
                        f"Y coordinate is out of tolerance: {calculated[1]} != {expected[1]} within {tolerance} pixels")
    def testRealCoordinates_eks1(self):
        robotHeight = 20
        cameraHeight = 200
        robotCoordinates = (100, 100)

        expected_corrected_coordinates = (180,140)
        tolerance = 6.0

        result = realCoordinates(robotHeight, cameraHeight, robotCoordinates)
        print(f"Input: {robotCoordinates}, Calculated: {result}, Expected: {expected_corrected_coordinates}")
        self.assertAlmostEqualWithTolerance(result, expected_corrected_coordinates, tolerance)

    def testRealCoordinates_eks2(self):
        robotHeight = 20
        cameraHeight = 200
        robotCoordinates = (1820, 100)
        expected_corrected_coordinates = (1740,140)
        tolerance = 6.0

        result = realCoordinates(robotHeight, cameraHeight, robotCoordinates)
        print(f"Input: {robotCoordinates}, Calculated: {result}, Expected: {expected_corrected_coordinates}")
        self.assertAlmostEqualWithTolerance(result, expected_corrected_coordinates, tolerance)

    def testRealCoordinates_eks3(self):
        robotHeight = 20
        cameraHeight = 200
        robotCoordinates = (100,980)
        expected_corrected_coordinates = (180,940)
        tolerance = 6.0

        result = realCoordinates(robotHeight, cameraHeight, robotCoordinates)
        print(f"Input: {robotCoordinates}, Calculated: {result}, Expected: {expected_corrected_coordinates}")
        self.assertAlmostEqualWithTolerance(result, expected_corrected_coordinates, tolerance)

    def testRealCoordinates_eks4(self):
        robotHeight = 20
        cameraHeight = 200
        robotCoordinates = (1820, 980)
        expected_corrected_coordinates = (1740,940)
        tolerance = 6.0

        result = realCoordinates(robotHeight, cameraHeight, robotCoordinates)
        print(f"Input: {robotCoordinates}, Calculated: {result}, Expected: {expected_corrected_coordinates}")
        self.assertAlmostEqualWithTolerance(result, expected_corrected_coordinates, tolerance)


if __name__ == '__main__':
    unittest.main()

