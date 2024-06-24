import unittest

from src.Server.Utils.px_conversion import realCoordinates

class TestRealCoordiantes(unittest.TestCase):
    def testRealCoordinates_eks1(self):
        robotHeight = 20
        cameraHeight = 200
        robotCoordinates = (100, 100)

        expected_corrected_coordinates = (180,140)

        result = realCoordinates(robotHeight, cameraHeight, robotCoordinates)
        self.assertAlmostEqual(result[0], expected_corrected_coordinates[0], places=1)
        self.assertAlmostEqual(result[1], expected_corrected_coordinates[1], places=1)

    def testRealCoordinates_eks2(self):
        robotHeight = 20
        cameraHeight = 200
        robotCoordinates = (1820, 100)
        expected_corrected_coordinates = (1740,140)

        result = realCoordinates(robotHeight, cameraHeight, robotCoordinates)
        self.assertAlmostEqual(result[0], expected_corrected_coordinates[0], places=1)
        self.assertAlmostEqual(result[1], expected_corrected_coordinates[1], places=1)

    def testRealCoordinates_eks3(self):
        robotHeight = 20
        cameraHeight = 200
        robotCoordinates = (100,980)
        expected_corrected_coordinates = (180,940)

        result = realCoordinates(robotHeight, cameraHeight, robotCoordinates)
        self.assertAlmostEqual(result[0], expected_corrected_coordinates[0], places=1)
        self.assertAlmostEqual(result[1], expected_corrected_coordinates[1], places=1)

    def testRealCoordinates_eks4(self):
        robotHeight = 20
        cameraHeight = 200
        robotCoordinates = (1820, 100)
        expected_corrected_coordinates = (1740,940)

        result = realCoordinates(robotHeight, cameraHeight, robotCoordinates)
        self.assertAlmostEqual(result[0], expected_corrected_coordinates[0], places=1)
        self.assertAlmostEqual(result[1], expected_corrected_coordinates[1], places=1)


if __name__ == '__main__':
    unittest.main()

