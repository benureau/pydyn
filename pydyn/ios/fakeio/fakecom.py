from ..serialio import serialcom
from ...dynamixel import memory

from . import fakememory

class FakeCom(serialcom.SerialCom):

    def __init__(self, support_sync_read=False):
        self._fakemems = {}
        self._support_sync_read = support_sync_read
        self.motormems = {}

    def _add_motor(self, mid, model):
        assert 0 <= mid <= 253
        fakemem = list(fakememory.MODELS[model])
        fakemem[3] = mid
        self._fakemems[mid] = memory.DynamixelMemory(mid, save=False, memory=fakemem)

    def _write_fake_memories(self, mid, model, values):
        offset = 0
        for size, value in zip(control.sizes, values):
            self._fakemems[mid][control.addr+offset] = value
            offset += size

    @property
    def support_sync_read(self):
        return self._support_sync_read

    def close(self):
        pass

    def ping(self, mid):
        return mid in self._fakemems

    def ping_broadcast(self):
        return tuple(self._fakemems.keys())

    def _send_read_packet(self, control, mid):
        values, offset = [], 0
        for size in control.sizes:
            values.append(self._fakemems[mid][control.addr+offset])
            offset += size
        self._update_memory(control, mid, values)

    def _send_sync_read_packet(self, control, mids):
        for mid in mids:
            self._send_read_packet(control, mid)

    def _send_write_packet(self, control, mid, values):
        offset = 0
        for size, val in zip(control.sizes, values):
            self._fakemems[mid][control.addr+offset] = val
            offset += size
        self._update_memory(control, mid, values)

    def _send_sync_write_packet(self, control, mids, valuess):
        for mid, values in zip(mid, valuess):
            self._send_write_packet(control, mid, values)

