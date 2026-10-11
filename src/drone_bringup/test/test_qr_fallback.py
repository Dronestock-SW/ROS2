"""QR 스캔 세션과 카메라 폴백 전환을 시험한다."""

import unittest

from drone_bringup.qr_fallback import QrFallbackState


class QrFallbackStateTest(unittest.TestCase):

    def test_rejects_invalid_threshold(self):
        with self.assertRaises(ValueError):
            QrFallbackState(0)

    def test_inactive_time_does_not_count_as_reader_failure(self):
        state = QrFallbackState(3)
        for _ in range(10):
            self.assertFalse(state.tick())
        self.assertEqual(state.fail_streak, 0)

    def test_third_failed_window_enables_camera(self):
        state = QrFallbackState(3)
        state.set_active(True)
        self.assertFalse(state.tick())
        self.assertFalse(state.tick())
        self.assertTrue(state.tick())
        self.assertEqual(state.fail_streak, 3)

    def test_reader_result_finishes_session_before_fallback(self):
        state = QrFallbackState(3)
        state.set_active(True)
        state.tick()
        self.assertTrue(state.accept_reader())
        self.assertFalse(state.active)
        self.assertFalse(state.camera_enabled)
        self.assertFalse(state.accept_reader())

    def test_camera_result_is_accepted_only_during_fallback(self):
        state = QrFallbackState(2)
        state.set_active(True)
        self.assertFalse(state.accept_camera())
        state.tick()
        state.tick()
        self.assertTrue(state.accept_camera())
        self.assertFalse(state.active)
        self.assertFalse(state.camera_enabled)

    def test_reader_recovery_turns_camera_back_off(self):
        state = QrFallbackState(1)
        state.set_active(True)
        self.assertTrue(state.tick())
        self.assertTrue(state.accept_reader())
        self.assertFalse(state.camera_enabled)
