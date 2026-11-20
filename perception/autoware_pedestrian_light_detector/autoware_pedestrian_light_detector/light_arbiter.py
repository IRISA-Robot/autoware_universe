# Copyright 2026 IRISA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Pedestrian-light arbiter logic (no ROS): who decides the light the FSM gets.

2026-11-17.  The detector's state goes through here before the road-crossing FSM, so an
operator (mock RC web now, a remote node later) can override it:

  AUTO   the detector decides (default).
  RED    the robot waits.  Stays until AUTO -- waiting is the safe side, no lease.
  GREEN  the robot may cross.  Only while it is re-sent: no command with the same request_id
         for green_lease_sec -> back to AUTO (link lost).  It is also used up when a
         crossing ends (FSM CROSSING -> DONE / IDLE).  A GREEN request_id that ended for any
         reason (used up, lapsed, replaced by RED / AUTO) is not taken again, so a page -- or a
         second open tab -- that keeps sending an old GREEN cannot re-arm it: the operator has
         to press GREEN again (new request_id).

Times are plain seconds (any monotonic clock).
"""

from collections import deque

AUTO, RED, GREEN = 0, 1, 2
MODE_NAME = {AUTO: 'AUTO', RED: 'RED', GREEN: 'GREEN'}

# FsmState (autoware_road_crossing_fsm/road_crossing_fsm_node.hpp)
FSM_IDLE, FSM_CROSSING, FSM_DONE = 0, 6, 7


class LightArbiter:
    """Override state machine; the node feeds it commands, FSM states and ticks."""

    def __init__(self, green_lease_sec=2.0):
        self.green_lease_sec = float(green_lease_sec)
        self.mode = AUTO
        self.request_id = 0
        self.requester = ''
        self.reason = 'start (AUTO)'
        self.last_used_green_id = 0
        self._green_seen = None        # time of the last GREEN command for the active id
        self._used = deque(maxlen=64)  # GREEN request ids that may not arm again
        self._fsm = None
        self.refused = False           # the last command was refused

    # ---- inputs ------------------------------------------------------------------------
    def command(self, mode, request_id, requester, t):
        """
        Apply one command.

        Returns a log line when the mode changes or a GREEN is refused,
        else None.
        """
        mode, request_id = int(mode), int(request_id)
        self.refused = False
        if mode == AUTO:
            if self.mode == AUTO:
                return None
            return self._set(AUTO, request_id, requester, f'operator AUTO ({requester})')
        if mode == RED:
            if self.mode == RED:
                self.request_id, self.requester = request_id, requester
                return None
            return self._set(RED, request_id, requester, f'operator RED ({requester})')
        if mode == GREEN:
            if self.mode == GREEN and request_id == self.request_id:
                self._green_seen = t   # heartbeat
                return None
            if request_id in self._used:
                self.refused = True
                return (f'GREEN #{request_id} dari {requester} diabaikan: sudah terpakai / '
                        'kedaluwarsa, tekan GREEN lagi')
            msg = self._set(GREEN, request_id, requester, f'operator GREEN ({requester})')
            self._green_seen = t
            return msg
        self.refused = True
        return f'mode {mode} tidak dikenal dari {requester}, diabaikan'

    def fsm_state(self, state, t):
        """FSM state update; a GREEN is used up when the crossing ends."""
        state = int(state)
        prev, self._fsm = self._fsm, state
        if (self.mode == GREEN and prev == FSM_CROSSING
                and state in (FSM_DONE, FSM_IDLE)):
            return self._use_green('crossing selesai, GREEN terpakai')
        return None

    def tick(self, t):
        """Lease check; call regularly."""
        if self.mode == GREEN and self.lease_remaining(t) <= 0.0:
            return self._use_green(
                f'GREEN tidak diperbarui {self.green_lease_sec:.1f} s (koneksi putus?)')
        return None

    # ---- outputs -----------------------------------------------------------------------
    def lease_remaining(self, t):
        if self.mode != GREEN or self._green_seen is None:
            return -1.0
        return max(0.0, self.green_lease_sec - (t - self._green_seen))

    def manual(self):
        return self.mode != AUTO

    # ---- internals ---------------------------------------------------------------------
    def _use_green(self, why):
        return self._set(AUTO, self.request_id, self.requester, why)

    def _set(self, mode, request_id, requester, why):
        old = self.mode
        if old == GREEN and (mode != GREEN or request_id != self.request_id):
            self._used.append(self.request_id)   # this GREEN press is over for good
            self.last_used_green_id = self.request_id
        self.mode, self.request_id, self.requester = mode, request_id, requester
        self.reason = why
        if mode != GREEN:
            self._green_seen = None
        return f'override {MODE_NAME[old]} -> {MODE_NAME[mode]}: {why}'
