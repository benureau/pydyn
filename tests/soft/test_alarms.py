import unittest

import env
from pydyn.refs.alarms import (bytes2_alarm_names, alarm_names_2bytes)


ALARMS = [
    "Input Voltage Error",
    "Angle Limit Error",
    "Overheating Error",
    "Range Error",
    "Checksum Error",
    "Overload Error",
    "Instruction Error"
    ]
EXAMPLES = [
        (0, []),
        (127, ALARMS),
        (1, [ALARMS[0]]),
        (72, [ALARMS[3], ALARMS[6]]),
        (12, [ALARMS[2], ALARMS[3]]),
        (50, [ALARMS[5], ALARMS[1], ALARMS[4]]),
        ]


class TestAlarmCodes(unittest.TestCase):

    def test_involution_bytes(self):
        for r in range(127):
            self.assertEqual(alarm_names_2bytes(bytes2_alarm_names(r)), r)

    def test_on_examples(self):
        for code, name_list in EXAMPLES:
            self.assertEqual(set(name_list), set(bytes2_alarm_names(code)))

if __name__ == '__main__':
    unittest.main()
