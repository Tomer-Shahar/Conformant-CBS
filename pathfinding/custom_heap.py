import heapq


class OpenListHeap(object):
    """ A custom heap I made to serve as an open list. This was created because the heapq heaps are a bit cumbersome.
    """
    def __init__(self):
        self.internal_heap = []
        self.entry_count = 0

    def push(self, item):
        heapq.heappush(self.internal_heap, (item.f_val[0], item.h_val, self.entry_count, item))
        self.entry_count += 1  # Add some uniqueness to items in the heap to avoid comparison between nodes..

    def pop(self):
        return heapq.heappop(self.internal_heap)[3]
