#!/usr/bin/env python3
"""Off-board simulation of the parachute deployment state machine.

This script mirrors, in pure Python, the flight-phase state machine and deploy
logic implemented in ``src/main.cpp`` (``flight_engine_routine`` /
``deploy_parachute``). It feeds analytic flight profiles through the same logic
so the deploy decisions can be checked without flashing hardware.

Constants, the state enum, and every transition are kept identical to the
firmware. The firmware runs these checks on the Kalman filter's *estimated*
altitude / velocity / acceleration; the on-board Kalman exists to clean up noisy
barometric readings. Here we feed the FSM ideal (noise-free) kinematics so the
deploy behaviour is isolated and the results are deterministic.

Run: ``python sim/flight_sim.py``
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum

# --- Tunables, identical to src/main.cpp -----------------------------------

SAMPLE_PERIOD_MS = 10
SAMPLE_DT_S = SAMPLE_PERIOD_MS / 1000.0  # 0.01 s tick

LAUNCH_ALTITUDE_THRESHOLD = 2.0   # climb above this => leave the pad (m)
LAUNCH_VELOCITY_THRESHOLD = 2.0   # upward speed above this => real boost (m/s)

APOGEE_LEAD_TIME_S = 0.15         # flag apogee this long before predicted peak
APOGEE_CONFIRM_SAMPLES = 3        # consecutive detections to confirm apogee

H_LOW_THRESHOLD_M = 20.0          # apogee below this => deploy at apogee (m)
H_DEPLOY_TARGET_M = 20.0          # on descent, deploy at/below this altitude (m)
V_SAFETY_FALLBACK_MS = -30.0      # deploy if descent speed exceeds this (m/s)

G = 9.81                          # gravitational acceleration (m/s^2)


class FlightState(IntEnum):
    IDLE = 0
    ASCENDING = 1
    APOGEE_DETECTED = 2
    DESCENDING = 3
    DEPLOYED = 4


@dataclass
class FlightComputer:
    """Mirror of the firmware FSM. ``update`` == one tick of flight_engine_routine."""

    state: FlightState = FlightState.IDLE
    boost_detected: bool = False
    apogee_count: int = 0
    max_altitude: float = 0.0
    peak_altitude: float = 0.0
    deploy_altitude: float = 0.0
    deploy_velocity: float = 0.0
    deploy_reason: str = ""
    deploy_from_state: str = ""

    def _deploy(self, altitude: float, velocity: float, reason: str) -> None:
        """Mirror of deploy_parachute(): idempotent, records altitude/state."""
        if self.state == FlightState.DEPLOYED:
            return
        self.peak_altitude = self.max_altitude
        self.deploy_altitude = altitude
        self.deploy_velocity = velocity
        self.deploy_reason = reason
        self.deploy_from_state = self.state.name
        self.state = FlightState.DEPLOYED

    def update(self, altitude: float, velocity: float, accel: float) -> None:
        # Track the highest altitude this flight (drives the apogee branch).
        if altitude > self.max_altitude:
            self.max_altitude = altitude

        # Altitude-target deploy + missed-apogee failsafe. Once past the pad, at
        # or below the deploy target, and clearly down from the peak, release in
        # any state. The "down from the peak" test uses how far we have fallen
        # (not the velocity sign), so it survives a bad velocity estimate and
        # never fires on the way up through the target altitude.
        if (self.state not in (FlightState.IDLE, FlightState.DEPLOYED)
                and altitude <= H_DEPLOY_TARGET_M
                and (self.max_altitude - altitude) > LAUNCH_ALTITUDE_THRESHOLD):
            self._deploy(altitude, velocity, "altitude target")
            return

        if self.state == FlightState.IDLE:
            if altitude > LAUNCH_ALTITUDE_THRESHOLD:
                self.state = FlightState.ASCENDING

        elif self.state == FlightState.ASCENDING:
            if not self.boost_detected and velocity > LAUNCH_VELOCITY_THRESHOLD:
                self.boost_detected = True
            if not self.boost_detected:
                return

            apogee_now = velocity <= 0.0
            apogee_imminent = False
            if accel < 0.0:
                time_to_apogee = -velocity / accel
                apogee_imminent = time_to_apogee <= APOGEE_LEAD_TIME_S

            if apogee_now or apogee_imminent:
                self.apogee_count += 1
                if self.apogee_count >= APOGEE_CONFIRM_SAMPLES:
                    self.state = FlightState.APOGEE_DETECTED
            else:
                self.apogee_count = 0

        elif self.state == FlightState.APOGEE_DETECTED:
            if self.max_altitude < H_LOW_THRESHOLD_M:
                self._deploy(altitude, velocity, "apogee (low flight)")
            else:
                self.state = FlightState.DESCENDING

        elif self.state == FlightState.DESCENDING:
            # Target-altitude deploy is handled by the failsafe block above;
            # this is the long-free-fall guard for tall flights.
            if velocity <= V_SAFETY_FALLBACK_MS:
                self._deploy(altitude, velocity, "safety fallback")

        # FlightState.DEPLOYED: nothing to do.


# --- Flight physics generator ----------------------------------------------

def burnout_velocity(target_apogee: float, burn_time: float) -> float:
    """Burnout speed so a vertical climb (burn then gravity coast) peaks at
    ``target_apogee``. Solves u^2 + g*tb*u - 2*g*H = 0 for u = accel*tb."""
    a, b, c = 1.0, G * burn_time, -2.0 * G * target_apogee
    return (-b + math.sqrt(b * b - 4 * a * c)) / (2 * a)


def simulate(target_apogee: float, burn_time: float = 0.5,
             suppress_apogee: bool = False):
    """Yield (t, altitude, velocity, accel) ticks for a vertical flight.

    Pad hold -> powered boost -> gravity coast to apogee -> free fall. No drag,
    so descent speed is the worst case (matches the hand analysis).

    ``suppress_apogee`` models an estimator failure: the reported velocity and
    acceleration are forced non-negative (as if the Kalman state diverged and
    never registered the velocity zero-crossing), so the FSM never detects
    apogee and stays ASCENDING all the way back down -- the only path that
    exercises the ground-safety catch. The altitude reported to the FSM stays
    truthful, which is what the catch keys off.
    """
    v_bo = burnout_velocity(target_apogee, burn_time)
    boost_accel = v_bo / burn_time  # net upward accel during the burn

    def emit(t, alt, vel, accel):
        if suppress_apogee:
            return t, alt, abs(vel), abs(accel)
        return t, alt, vel, accel

    t = 0.0
    alt = 0.0
    vel = 0.0

    # Pad hold: sit at rest for 0.2 s so we confirm no deploy on the pad.
    for _ in range(20):
        yield emit(t, alt, vel, 0.0)
        t += SAMPLE_DT_S

    # Powered boost.
    steps = int(round(burn_time / SAMPLE_DT_S))
    for _ in range(steps):
        accel = boost_accel
        vel += accel * SAMPLE_DT_S
        alt += vel * SAMPLE_DT_S
        yield emit(t, alt, vel, accel)
        t += SAMPLE_DT_S

    # Coast + free fall under gravity until we hit the ground.
    while alt > 0.0:
        accel = -G
        vel += accel * SAMPLE_DT_S
        alt += vel * SAMPLE_DT_S
        yield emit(t, alt, vel, accel)
        t += SAMPLE_DT_S


# --- Test runner ------------------------------------------------------------

@dataclass
class Scenario:
    name: str
    target_apogee: float
    expect_reason: str
    expect_alt_range: tuple
    suppress_apogee: bool = False
    results: dict = field(default_factory=dict)


def run(scenario: Scenario) -> bool:
    fc = FlightComputer()
    for _t, alt, vel, accel in simulate(scenario.target_apogee,
                                        suppress_apogee=scenario.suppress_apogee):
        fc.update(alt, vel, accel)
        if fc.state == FlightState.DEPLOYED:
            break

    lo, hi = scenario.expect_alt_range
    deployed = fc.state == FlightState.DEPLOYED
    alt_ok = deployed and lo <= fc.deploy_altitude <= hi
    reason_ok = fc.deploy_reason == scenario.expect_reason
    ok = deployed and alt_ok and reason_ok

    status = "PASS" if ok else "FAIL"
    print(f"[{status}] {scenario.name}")
    print(f"        apogee reached : {fc.max_altitude:6.1f} m  (target {scenario.target_apogee:.0f} m)")
    if deployed:
        print(f"        deployed at    : {fc.deploy_altitude:6.1f} m, "
              f"v = {fc.deploy_velocity:6.1f} m/s")
        print(f"        reason         : {fc.deploy_reason} "
              f"(from {fc.deploy_from_state})")
    else:
        print("        deployed       : NEVER (still in state "
              f"{fc.state.name})")
    print(f"        expected       : {scenario.expect_reason}, "
          f"altitude in [{lo:.0f}, {hi:.0f}] m")
    print()
    return ok


SCENARIOS = [
    Scenario("15 m flight  -> deploy at apogee (< 20 m low threshold)",
             target_apogee=15.0,
             expect_reason="apogee (low flight)",
             expect_alt_range=(13.0, 16.0)),
    Scenario("50 m flight  -> descend to 20 m target",
             target_apogee=50.0,
             expect_reason="altitude target",
             expect_alt_range=(18.0, 21.0)),
    Scenario("100 m flight -> safety fallback (-30 m/s)",
             target_apogee=100.0,
             expect_reason="safety fallback",
             expect_alt_range=(48.0, 60.0)),
    Scenario("100 m flight, velocity estimate fails -> 20 m failsafe",
             target_apogee=100.0,
             expect_reason="altitude target",
             expect_alt_range=(18.0, 22.0),
             suppress_apogee=True),
]


def main() -> int:
    print("Parachute deployment FSM simulation "
          f"(V_SAFETY_FALLBACK = {V_SAFETY_FALLBACK_MS:.0f} m/s)\n")
    all_ok = True
    for scenario in SCENARIOS:
        all_ok &= run(scenario)
    print("=" * 60)
    print("ALL SCENARIOS PASSED" if all_ok else "SOME SCENARIOS FAILED")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
