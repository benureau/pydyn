import array
import time

class MemSave(object):

    def __init__(self, size = 100):
        self._mem = [TimeStampedMemRow() for i in range(size)]

    def __setitem__(self, index, value):
        self._mem[index].append(value)

class MemRow(object):

    def __init__(self, size = 2):
        self._fmt = {1:'B', 2:'H'}[size]
        self._last = None
        self._store = [[]]
        self._size  = 0

    def append(self, val):
        self._store[-1].append(val)
        self._size += 1
        if (self._size % 256) == 0:
            self.compact()

    def compact(self):
        self._store[-1] = array.array(self._fmt,self._store[-1])
        self._store.append([])

    def __iter__(self):
        for chunk in self._store:
            for val in chunk:
                yield val

    def __len__(self):
        return self._size

    def __iadd__(self, b):
        for b_i in b:
            self.append(b_i)

class TimeStampedMemRow(MemRow):

    def __init__(self, size = 2):
        MemRow.__init__(self, size)
        self._timestamps = [[]]

    def compact(self):
        self._store[-1] = array.array(self._fmt, self._store[-1])
        self._timestamps[-1] =  array.array('d', self._timestamps[-1])
        self._store.append([])
        self._timestamps.append([])

    def append(self, val):
        self._store[-1].append(val)
        self._timestamps[-1].append(time.time())
        self._size += 1
        if (self._size % 256) == 0:
            self.compact()

    def __iter__(self):
        for chunk, timechunk in zip(self._store, self._timestamps):
            for val, timestamp in zip(chunk, timechunk):
                yield val, timestamp

    def __getitem__(self, index):
        pindex = index % self._size
        chunk, offset = divmod(pindex, 256)
        return self._store[chunk][offset], self._timestamps[chunk][offset]

if __name__ == '__main__':
    mr = TimeStampedMemRow(size = 2)
    for i in range(256*256):
        mr.append(i)

    print("{:.9f}".format(mr[0][1]))
    print("{:.9f}".format(mr[-1][1]))
