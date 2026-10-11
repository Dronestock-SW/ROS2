"""QR 스캔 세션의 리더기 우선·카메라 폴백 상태를 관리한다."""

from dataclasses import dataclass


@dataclass
class QrFallbackState:
    """ROS 통신과 분리된 QR 폴백 상태기계다."""

    fallback_threshold: int
    active: bool = False
    fail_streak: int = 0
    camera_enabled: bool = False

    def __post_init__(self):
        if type(self.fallback_threshold) is not int or self.fallback_threshold <= 0:
            raise ValueError('fallback_threshold must be a positive integer')

    def set_active(self, active):
        """새 스캔 세션을 시작하거나 현재 세션을 종료한다."""
        self.active = bool(active)
        self.fail_streak = 0
        self.camera_enabled = False

    def tick(self):
        """리더기 무응답 시간 창을 한 번 진행한다."""
        if not self.active or self.camera_enabled:
            return self.camera_enabled
        self.fail_streak += 1
        if self.fail_streak >= self.fallback_threshold:
            self.camera_enabled = True
        return self.camera_enabled

    def accept_reader(self):
        """활성 세션의 리더기 결과를 한 건 수락하고 세션을 끝낸다."""
        if not self.active:
            return False
        self.set_active(False)
        return True

    def accept_camera(self):
        """카메라 폴백 중인 결과만 한 건 수락하고 세션을 끝낸다."""
        if not self.active or not self.camera_enabled:
            return False
        self.set_active(False)
        return True
