import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest
import yaml

spec = importlib.util.spec_from_file_location('np_generate', Path(__file__).resolve().parents[2] / 'scripts/np_generate.py')
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)

class OutputGenerationTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.config = self.root / 'config/probe'
        self.config.mkdir(parents=True)
        self.node = {'execution': {'mode': 'latched'}, 'inputs': {},
                     'stages': [{'name': 'produce', 'outputs': [{'name': 'ego', 'type': 'nav_msgs/Odometry'}]}],
                     'output': {'topic': '/ego', 'frame_id': 'map'}}

    def tearDown(self):
        self.tmp.cleanup()

    def load(self):
        (self.config / 'node.yaml').write_text(yaml.safe_dump(self.node))
        return gen.load_node(str(self.root), 'probe', 'probe')

    def multi(self):
        self.node['stages'][0]['outputs'].append({'name': 'status', 'type': 'interfaces/SectionAnnotation'})
        self.node['output'] = [{'name': 'ego', 'topic': '/ego', 'frame_id': 'map'},
                               {'name': 'status', 'topic': '/status', 'frame_id': 'map'}]

    def test_legacy_single_output(self):
        model = self.load()
        self.assertEqual(model.output_type.cpp, '::nav_msgs::Odometry')
        self.assertIn('using Output = ::nav_msgs::Odometry;', gen.generate(model))

    def test_multi_output_tuple(self):
        self.multi()
        model = self.load()
        self.assertIn('using Output = std::tuple<::nav_msgs::Odometry, ::interfaces::SectionAnnotation>;', gen.generate(model))

    def test_invalid_count_order_duplicate_topic_and_frame(self):
        self.multi()
        original = copy.deepcopy(self.node)
        for output in [original['output'][:1], original['output'][::-1],
                       [original['output'][0], dict(original['output'][1], topic='/ego')],
                       [original['output'][0], dict(original['output'][1], frame_id='bad frame')]]:
            self.node = copy.deepcopy(original)
            self.node['output'] = output
            with self.subTest(output=output), self.assertRaises(gen.ConfigError):
                self.load()

if __name__ == '__main__':
    unittest.main()
