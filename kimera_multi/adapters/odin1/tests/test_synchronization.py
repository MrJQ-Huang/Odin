from types import SimpleNamespace
from odin1_adapter.synchronization import PairSynchronizer, stamp_ns

class Ros1Time:
    def __init__(self, ns): self.ns = ns
    def to_nsec(self): return self.ns

def msg(ns, ros1):
    stamp=Ros1Time(ns) if ros1 else SimpleNamespace(sec=ns//10**9,nanosec=ns%10**9)
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp))

def test_native_ros1_and_ros2_preserve_integer_nanoseconds():
    ns=1_790_000_001_234_567_890
    assert stamp_ns(msg(ns,True))==ns
    assert stamp_ns(msg(ns,False))==ns

def test_ros1_pairing_respects_sensor_time_and_rejects_clock_reordering():
    sync=PairSynchronizer(5_000_000)
    image=msg(100_000_000,True); cloud=msg(103_000_000,True)
    assert sync.add('image',image)==[]
    assert sync.add('cloud',cloud)==[(image,cloud)]
    assert sync.add('image',msg(99_000_000,True))==[]
    assert sync.stats['image_non_increasing']==1
