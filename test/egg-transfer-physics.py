"""Actual dynamic grasp acceptance; run with CPU MuJoCo + NumPy.

Catches empty-air grasps, dropped/teleported eggs, unsupported release and
gripper/environment penetration. No renderer and no GPU are needed.
"""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class EggTransferPhysics(unittest.TestCase):
    def test_first_egg_moves_by_contact_and_remains_supported_after_release(self):
        path = ROOT / 'scripts/solve-egg-transfer.py'
        self.assertTrue(path.exists(), 'physical transfer implementation is missing')
        spec = importlib.util.spec_from_file_location('egg_transfer', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        result = module.build_and_verify(ROOT)
        self.assertGreater(result['metrics']['liftMeters'], .060)
        self.assertTrue(result['metrics']['bilateralGrasp'])
        self.assertTrue(result['metrics']['bilateralBeforeLift'], 'closing must grasp before lifting, not snag during lift')
        self.assertLess(result['metrics']['maxForbiddenPenetrationMeters'], .0001)
        self.assertLess(result['metrics']['maxNeighborDisplacementMeters'], .002)
        self.assertLess(result['metrics']['finalCellErrorMeters'], .012)
        self.assertTrue(result['metrics']['supportedAfterRelease'])
        self.assertLess(result['metrics']['postReleaseDriftMeters'], .002)
        self.assertLess(result['metrics']['finalTiltDegrees'], 20, 'placed egg should not remain lying against a rim')
        self.assertGreater(result['metrics']['releaseApertureMeters'], .05, 'open at least 8 mm beyond the 42 mm egg before larger withdrawal')
        self.assertLess(result['metrics']['maxGripPenetrationMeters'], .001)
        self.assertFalse(result['metrics']['fingerContactAfterRelease'], 'neither finger may drag the supported egg out')
        self.assertLess(result['metrics']['maxVisibleFingerContactGapMeters'], .001, 'contacts must lie on the real visible fingertip surface')
        self.assertEqual(result['metrics']['gateFailures'], [])
        self.assertLessEqual(result['metrics']['longestGripLossSeconds'], .12)
        self.assertTrue(result['success'], result['metrics'])


if __name__ == '__main__':
    unittest.main()
