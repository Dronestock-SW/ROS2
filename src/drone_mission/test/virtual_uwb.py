"""Seeded sensor generator for isolated tests. Truth stays in the generator.

Tag B TDMA: four anchors, 25ms superframe, slot offset 12500us,
sequential samples 2ms apart. This is not an RF wave propagation model.
"""
import math
import numpy as np


class VirtualUwb:
    def __init__(self, tag_id='6', seed=42):
        self.tag_id, self.rng, self.seq = tag_id, np.random.default_rng(seed), 0
        self.anchors = np.array([[0,0,2.2],[6.3,0,2.2],[0,4.6,2.2],[6.3,4.6,2.2]])
        self.bias = np.array([.02,-.015,.01,.025])

    def messages(self, end_us, position, velocity, *, fault='none', elapsed=0.):
        self.seq += 1
        times = [end_us-7000+i*2000 for i in range(4)]
        ranges=[]
        for i,t in enumerate(times):
            p=np.asarray(position)-np.asarray(velocity)*(end_us-t)/1e6
            if fault=='coherent_step':
                p[0]+=.8
            r=np.linalg.norm(p-self.anchors[i])+self.bias[i]+self.rng.normal(0,.003)
            if fault=='nlos' and i==1:
                r+=.6
            if fault=='spike' and i==1:
                r+=1.2
            if fault=='phase_delay':
                # Equivalent range residual + unequal additional sample age.
                # The DW3000 contract exposes DS-TWR range, not carrier phase.
                r+=.035*math.sin(elapsed*9+i)
                times[i]-=i*300
            ranges.append(float(r))
        status=dict(schema=1,type='uwb_raw_status',tag_id=self.tag_id,
            event='heartbeat', firmware='uwb-tag-tdma40-radiofix3-virtual',
            clock_domain='esp32_monotonic_boot_us',anchor_order=['A1','A2','A3','A4'],
            anchor_count=4,uwb_ready=True,temporal_filter_applied=False)
        raw=dict(schema=1,type='uwb_raw_cycle',tag_id=self.tag_id,seq=self.seq,
            cycle_start_us=end_us-8500,cycle_end_us=end_us,valid_mask=15,
            raw_slant_m=ranges,sample_time_us=times,failure=['ok']*4)
        offset=1000 if self.tag_id=='5' else 12500
        tdma=dict(schema=1,type='uwb_tdma_epoch',tag_id=self.tag_id,seq=self.seq,
            boot_id=1234,session=1,sf_seq=self.seq,schedule_id=0x4001,
            start_offset_us=offset,end_offset_us=offset+8500,
            epoch_span_us=max(times)-min(times),log_drops=0,
            deadline_valid=True,local_overrun=False)
        return status,raw,tdma
