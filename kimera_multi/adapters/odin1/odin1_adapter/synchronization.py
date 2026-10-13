from collections import Counter


def stamp_ns(msg):
    return int(msg.header.stamp.sec)*1_000_000_000 + int(msg.header.stamp.nanosec)


def pose_entry(msg):
    p,q = msg.pose.pose.position, msg.pose.pose.orientation
    return stamp_ns(msg), [p.x,p.y,p.z], [q.x,q.y,q.z,q.w]


class PairSynchronizer:
    """Bounded, one-to-one approximate pairing in the sensor clock domain."""
    def __init__(self,tolerance_ns,capacity=20):
        self.tolerance = tolerance_ns
        self.capacity = capacity
        self.queues = {'image':[], 'cloud':[]}
        self.last = {}
        self.stats = Counter()

    def add(self,kind,msg):
        time = stamp_ns(msg)
        if kind in self.last and time <= self.last[kind]:
            self.stats[kind+'_non_increasing'] += 1
            return []
        self.last[kind] = time
        queue = self.queues[kind]
        queue.append(msg)
        if len(queue)>self.capacity:
            queue.pop(0); self.stats[kind+'_overflow'] += 1
        pairs = []
        images,clouds = self.queues['image'],self.queues['cloud']
        while images and clouds:
            delta = stamp_ns(clouds[0])-stamp_ns(images[0])
            if abs(delta) <= self.tolerance:
                pairs.append((images.pop(0),clouds.pop(0)))
                self.stats['paired'] += 1
            elif delta > 0:
                images.pop(0); self.stats['image_unmatched'] += 1
            else:
                clouds.pop(0); self.stats['cloud_unmatched'] += 1
        return pairs
