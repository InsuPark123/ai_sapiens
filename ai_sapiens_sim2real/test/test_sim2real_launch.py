import importlib.util
from pathlib import Path
import unittest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

path = Path(__file__).resolve().parents[1] / 'launch' / 'ai_sapiens_sim2real.launch.py'
spec = importlib.util.spec_from_file_location('sim2real_launch', path)
launch_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_module)


class LaunchArgumentsTest(unittest.TestCase):
    def resolve(self, overrides=None):
        context = LaunchContext()
        context.launch_configurations.update(overrides or {})
        for action in launch_module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        return context.launch_configurations

    def test_default_config_is_resolved_after_robot(self):
        values = self.resolve()
        self.assertEqual(values['robot'], 'k1')
        self.assertTrue(values['config_path'].endswith('/config/k1_config.yaml'))

    def test_robot_override_and_explicit_config(self):
        values = self.resolve({'robot': 'example'})
        self.assertTrue(values['config_path'].endswith('/config/example_config.yaml'))
        values = self.resolve({'config_path': '/tmp/custom.yaml'})
        self.assertEqual(values['config_path'], '/tmp/custom.yaml')


if __name__ == '__main__':
    unittest.main()
