import heapq
from pathfinding.planners.utils.time_error import *


class OpenListHeap(object):
    """ A custom heap I made to serve as an open list. This was created because the heapq heaps are a bit cumbersome.
    """
    def __init__(self):
        self.internal_heap = []
        self.entry_count = 0

    def push(self, item, first_param, second_param, third_param):
        try:
            heapq.heappush(self.internal_heap, (first_param, second_param, third_param, self.entry_count, item))
            self.entry_count += 1  # Add some uniqueness to items in the heap to avoid comparison between nodes..
        except MemoryError:
            raise OutOfTimeError('Ran out of memory in the heap :(')

    def pop(self):
        try:
            return heapq.heappop(self.internal_heap)[4]
        except IndexError:
            print('Thunder Child')
    def heapify(self):
        heapq.heapify(self.internal_heap)
