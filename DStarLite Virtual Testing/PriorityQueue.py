import heapq


class PriorityQueue:
    def __init__(self):
        self.elements = []
        self.maxlength = 0

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item.position, item))
        self.maxlength = max(self.maxlength,len(self.elements))

    def pop(self):
        item = heapq.heappop(self.elements)
        return item[2]

    def TopKey(self):
        a=heapq.nsmallest(1, self.elements)[0][0]
        return a

    def delete(self, node):
        self.elements = [e for e in self.elements if e[2] != node]
        heapq.heapify(self.elements)

    def __iter__(self):
        for key, nodepos, node in self.elements:
            yield node
