"""Loopback UDP DDS profile for a separate observation-service user.

No device port is opened. Avoid cross-user shared-memory ownership conflicts.
"""
import atexit
import os
from pathlib import Path
import tempfile
import uuid


def configure_local_dds():
    prefix = '.'.join(f'{b:02x}' for b in uuid.uuid4().bytes[:12])
    xml = f'''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
<transport_descriptors><transport_descriptor><transport_id>local_uwb_udp</transport_id>
<type>UDPv4</type><interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
</transport_descriptor></transport_descriptors>
<participant profile_name="local_uwb_participant" is_default_profile="true"><rtps>
<prefix>{prefix}</prefix><userTransports><transport_id>local_uwb_udp</transport_id></userTransports>
<useBuiltinTransports>false</useBuiltinTransports>
<builtin><initialPeersList><locator><udpv4><address>127.0.0.1</address></udpv4>
</locator></initialPeersList></builtin></rtps></participant>
<subscriber profile_name="local_uwb_sub" is_default_profile="true">
<qos><data_sharing><kind>OFF</kind></data_sharing></qos>
<historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy></subscriber>
<publisher profile_name="local_uwb_pub" is_default_profile="true">
<qos><data_sharing><kind>OFF</kind></data_sharing></qos>
<historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy></publisher>
</profiles>'''
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', suffix='.xml', delete=False) as out:
        out.write(xml)
    os.environ.update(FASTRTPS_DEFAULT_PROFILES_FILE=out.name,
                      RMW_FASTRTPS_USE_QOS_FROM_XML='1', MAVLINK20='1')
    atexit.register(lambda: Path(out.name).unlink(missing_ok=True))
