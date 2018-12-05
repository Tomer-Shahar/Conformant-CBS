"""
An implementation of a data structure used for fast insertion and removal to the open list.
"""
import heapq
from collections import deque


class OpenList:

    def __init__(self):
        self.heap = []
        self.queue = deque()
        self.removed_index = -1

    def insert(self, node):
        """ Inserts an item into the open list"""
        if len(self.queue) == 0:
            if len(self.heap) == 0:
                heapq.heappush(self.heap, node)
            else:
                comparison = node.compare_to(self.heap[0])
                if comparison != -1:
                    heapq.heappush(self.heap, node)
                else:
                    self.queue.append(node)
        else:
            comparison = node.compare_to(self.queue[0])
            if comparison == 1:
                heapq.heappush(self.heap, node)
            else:
                if comparison == -1:
                    while len(self.queue) != 0:
                        temp_queue_node = self.queue.pop()
                        heapq.heappush(self.heap, temp_queue_node)

                    self.queue.append(node)

    def remove(self):

        if len(self.queue) != 0:
            node = self.queue.pop()
            node.set_heap_index(self.removed_index)
        else:
            node = heapq.heappop(self.heap)

        return node

    def remove(self, node):

        if node.get_heap_index() == self.removed_index:
            return False
        removed_from_queue = False

        for i in range(0, len(self.queue)):
            temp_node = self.queue.pop()
            if temp_node.equals(node):
                removed_from_queue = True
                temp_node.set_heap_index(self.removed_index)
            else:
                self.queue.append(temp_node)

        if removed_from_queue:
            return True

        return heapq.heappop(self.heap)

    def contains(self, node):
        """ Returns whether an item is in the open list"""
        return node.get_heap_index() != self.removed_index

    def size(self):
        """ Returns the number of items in the open list"""
        return len(self.heap) + len(self.queue)

    def empty(self):
        """ Empties the open list"""
        self.heap = []
        self.queue.clear()

    def peak(self):
        """ Returns top of open list without removing it"""
        if len(self.queue) != 0:
            return self.queue[0]
        return self.heap[0]
