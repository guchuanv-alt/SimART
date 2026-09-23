import unittest

from agent.weather_material_model import weather_visual_color


class WeatherVisualColorTest(unittest.TestCase):
    def test_weather_color_stops_and_interpolation(self):
        self.assertEqual(weather_visual_color(0.0), "0.950000 0.780000 0.250000")
        self.assertEqual(weather_visual_color(0.35), "0.350000 0.650000 0.950000")
        self.assertEqual(weather_visual_color(0.85), "0.050000 0.100000 0.450000")
        self.assertEqual(weather_visual_color(0.225), "0.450000 0.625000 0.800000")


if __name__ == "__main__":
    unittest.main()
