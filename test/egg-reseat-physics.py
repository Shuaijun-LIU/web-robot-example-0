"""A correction must occur after complete physical release, not in-air alignment."""
import importlib.util
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]

class ReseatPhysics(unittest.TestCase):
    def test_regrasp_corrects_a_supported_tilted_egg_without_attachments(self):
        path=ROOT/'scripts/solve-egg-reseat.py'
        self.assertTrue(path.exists(),'post-release regrasp implementation is missing')
        spec=importlib.util.spec_from_file_location('reseat',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        result=module.build_and_verify(ROOT)
        m=result['metrics']
        self.assertGreater(m['releasedTiltDegrees'],20)
        self.assertTrue(m['releasedBeforeCorrection'])
        self.assertTrue(m['bilateralRegrasp'])
        self.assertGreater(m['reliftMeters'],.06)
        self.assertLess(m['finalTiltDegrees'],20)
        self.assertGreater(m['releasedTiltDegrees']-m['finalTiltDegrees'],10)
        self.assertTrue(m['supportedAfterRelease'])
        self.assertFalse(m['fingerContactAfterRelease'])
        self.assertLess(m['maxForbiddenPenetrationMeters'],.0001)
        self.assertLess(m['maxGripPenetrationMeters'],.001)
        self.assertLess(m['maxNeighborDisplacementMeters'],.002)
        self.assertEqual(m['gateFailures'],[])
        self.assertTrue(result['success'],m)

if __name__=='__main__':unittest.main()
