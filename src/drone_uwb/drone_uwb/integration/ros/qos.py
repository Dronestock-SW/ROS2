"""QoS for the multiplexed RAW/status/TDMA stream, not pose freshness."""
from rclpy.qos import QoSProfile, ReliabilityPolicy


def received_stream_qos():
    # One serial read can publish more than five frames back to back. Losing a
    # status or one half of a RAW/TDMA pair invalidates later observations.
    # Keep bounded burst history; original timestamp/age/pair gates still apply.
    return QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)
