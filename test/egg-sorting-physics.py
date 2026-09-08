"""End-to-end contact sorting; no assigned free-body trajectories."""
import importlib.util
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]

class SortingPhysics(unittest.TestCase):
    def test_two_arms_place_owned_eggs_through_contacts(self):
        path=ROOT/'scripts/solve-egg-sorting.py'
        self.assertTrue(path.exists(),'multi-arm physical sorting is missing')
        spec=importlib.util.spec_from_file_location('sorting',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        result=module.build_and_verify(ROOT,arms=2)
        self.assertTrue(result['success'],result['metrics'])
        self.assertEqual(result['counts'],[1,1,0,0])
        self.assertLess(result['metrics']['maxForbiddenPenetration'],.0001)
        self.assertGreater(result['metrics']['parallelSeconds'],0)

if __name__=='__main__':unittest.main()
