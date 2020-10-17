
import heapq

class PriorityQueue:
    """Priority queue."""

    q = []

    def __init__(self):
        """Init queue."""
        self.q = []

    def insert(self, x):
        """Insert object into queue."""
        self.q.append(x)


    def pop(self):
        """Get object from queue."""
        return self.q.pop(0)

    def IsEmpty(self):
        """Test if queue is empty."""
        return len(self.q) == 0

    def size(self):
        """Return size of queue."""
        return len(self.q)

    def peek(self):
        """Peek into queue."""
        return self.q[0]

    def insert(self, priority, x):
        """Insert prioritized object into queue.

        obj.insert(priority, x) - Insert object x with priority priority.
        """
        heapq.heappush(self.q, (priority, x))

    def pop(self):
        """Pop value from queue with lowest priority.

        Returns pair (priority, object)
        """
        return heapq.heappop(self.q)
