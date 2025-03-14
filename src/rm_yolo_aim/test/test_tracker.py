import unittest
import math
from .armor_tracker import select_tracking_armor, pixel_to_angle_and_deep  # Replace your_module


class TestArmorTracking(unittest.TestCase):

    def test_select_tracking_armor_no_armors(self):
        armors_dict = {}
        color = 1
        tracking_armor = select_tracking_armor(armors_dict, color)
        self.assertEqual(tracking_armor, {})

    def test_select_tracking_armor_blue(self):
        armors_dict = {
            "179": {"class_id": 7, "height": 290, "center": [1, 333]},
            "-143": {"class_id": 3, "height": 288, "center": [-143, -35]},
            "149": {"class_id": 3, "height": 191, "center": [149, 36]},
            "-113": {"class_id": 2, "height": 300, "center": [91, -35]},
        }
        color = 1
        tracking_armor = select_tracking_armor(armors_dict, color)
        self.assertEqual(tracking_armor, {"class_id": 2, "height": 300, "center": [91, -35]})

    def test_select_tracking_armor_red(self):
        armors_dict = {
            "179": {"class_id": 7, "height": 290, "center": [1, 333]},
            "-143": {"class_id": 3, "height": 288, "center": [-143, -35]},
            "149": {"class_id": 3, "height": 191, "center": [149, 36]},
            "-113": {"class_id": 2, "height": 300, "center": [91, -35]},
        }
        color = 0
        tracking_armor = select_tracking_armor(armors_dict, color)
        self.assertEqual(tracking_armor, {"class_id": 7, "height": 290, "center": [1, 333]})

    def test_select_tracking_armor_filtered_out(self):
        armors_dict = {
            "1": {"class_id": 1, "height": 1, "center": [0, 0]},  # height <= 1 will be filtered out
        }
        color = 1
        tracking_armor = select_tracking_armor(armors_dict, color)
        self.assertEqual(tracking_armor, {})

    def test_pixel_to_angle_and_deep(self):
        height = 100
        center = (50, 25)
        vfov = 60
        pic_width = 640
        yaw, pitch, deep = pixel_to_angle_and_deep(height, center, vfov, pic_width)

        # 使用 assertAlmostEqual 进行浮点数比较，设置合适的精度
        self.assertAlmostEqual(yaw, 4.467455310733078, places=7)
        self.assertAlmostEqual(pitch, 2.232990073075336, places=7)
        self.assertEqual(deep, 100)

    def test_pixel_to_angle_and_deep_zero_focal_length(self):
        height = 100
        center = (50, 25)
        vfov = 0  # this will cause focal_pixel_distance to be zero
        pic_width = 640
        yaw, pitch, deep = pixel_to_angle_and_deep(height, center, vfov, pic_width)

        #  断言角度为90度，因为center[0] / focal_pixel_distance 将趋近于无穷大
        self.assertAlmostEqual(yaw, 90)
        self.assertAlmostEqual(pitch, 90)
        self.assertEqual(deep, 100)


if __name__ == '__main__':
    unittest.main()