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

"""Operator override rules (light_arbiter.py)."""

from autoware_pedestrian_light_detector.light_arbiter import (
    AUTO, FSM_CROSSING, FSM_DONE, FSM_IDLE, GREEN, LightArbiter, RED)


def _web_stream(arb, mode, rid, t0, t1, dt=0.1):
    """A page re-sending its current selection every dt (as the mock RC websocket does)."""
    t = t0
    while t <= t1 + 1e-9:
        arb.command(mode, rid, 'web', t)
        arb.tick(t)
        t += dt


def test_starts_in_auto():
    a = LightArbiter()
    assert a.mode == AUTO and not a.manual()


def test_red_stays_without_heartbeat():
    a = LightArbiter(green_lease_sec=2.0)
    a.command(RED, 1, 'web', 0.0)
    for t in range(1, 600):
        a.tick(float(t))
    assert a.mode == RED


def test_auto_ends_red():
    a = LightArbiter()
    a.command(RED, 1, 'web', 0.0)
    a.command(AUTO, 2, 'web', 1.0)
    assert a.mode == AUTO


def test_green_lapses_without_heartbeat():
    a = LightArbiter(green_lease_sec=2.0)
    a.command(GREEN, 5, 'web', 0.0)
    a.tick(1.9)
    assert a.mode == GREEN
    a.tick(2.1)
    assert a.mode == AUTO and a.last_used_green_id == 5


def test_green_heartbeat_keeps_it():
    a = LightArbiter(green_lease_sec=2.0)
    _web_stream(a, GREEN, 5, 0.0, 30.0)
    assert a.mode == GREEN and a.lease_remaining(30.0) > 1.5


def test_lapsed_green_is_not_rearmed_by_the_same_press():
    # link back after a drop: the page still sends GREEN #5 -> must NOT go again
    a = LightArbiter(green_lease_sec=2.0)
    a.command(GREEN, 5, 'web', 0.0)
    a.tick(3.0)
    assert a.mode == AUTO
    a.command(GREEN, 5, 'web', 3.1)
    assert a.mode == AUTO and a.refused
    a.command(GREEN, 6, 'web', 3.2)  # operator presses again
    assert a.mode == GREEN


def test_green_used_up_when_the_crossing_ends():
    a = LightArbiter(green_lease_sec=2.0)
    a.fsm_state(5, 0.0)            # WAIT_GREEN
    _web_stream(a, GREEN, 7, 0.0, 1.0)
    a.fsm_state(FSM_CROSSING, 1.0)
    _web_stream(a, GREEN, 7, 1.0, 10.0)
    assert a.mode == GREEN
    a.fsm_state(FSM_DONE, 10.0)
    assert a.mode == AUTO
    # the page keeps sending its stale selection: next crossing is not opened by it
    _web_stream(a, GREEN, 7, 10.0, 20.0)
    assert a.mode == AUTO


def test_crossing_end_straight_to_idle_also_uses_it_up():
    a = LightArbiter()
    a.command(GREEN, 1, 'web', 0.0)
    a.fsm_state(FSM_CROSSING, 0.1)
    a.fsm_state(FSM_IDLE, 0.2)
    assert a.mode == AUTO


def test_green_before_the_crossing_stays_until_used():
    # pressed while still approaching: kept (heartbeat), only a finished crossing uses it up
    a = LightArbiter()
    a.fsm_state(1, 0.0)            # APPROACH_PRE
    _web_stream(a, GREEN, 3, 0.0, 5.0)
    a.fsm_state(FSM_IDLE, 5.0)     # not after CROSSING -> not used
    assert a.mode == GREEN


def test_red_overrides_green_at_once():
    a = LightArbiter()
    a.command(GREEN, 1, 'web', 0.0)
    a.command(RED, 2, 'web', 0.5)
    assert a.mode == RED and a.lease_remaining(0.5) == -1.0


def test_unknown_mode_is_ignored():
    a = LightArbiter()
    a.command(9, 1, 'x', 0.0)
    assert a.mode == AUTO and a.refused


def test_green_replaced_by_red_is_not_rearmed_by_another_tab():
    # tab B still streams GREEN #4 after tab A pressed RED: stays RED
    a = LightArbiter()
    a.command(GREEN, 4, 'tab B', 0.0)
    a.command(RED, 5, 'tab A', 0.5)
    a.command(GREEN, 4, 'tab B', 0.6)
    assert a.mode == RED and a.refused and a.last_used_green_id == 4


def test_new_green_press_replaces_the_old_one():
    a = LightArbiter()
    a.command(GREEN, 4, 'web', 0.0)
    a.command(GREEN, 9, 'web', 0.5)
    assert a.mode == GREEN and a.request_id == 9
    a.command(GREEN, 4, 'web', 0.6)  # old press again
    assert a.request_id == 9 and a.refused
