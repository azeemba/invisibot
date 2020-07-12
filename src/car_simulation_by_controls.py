"""
Module to do rough car simulation based on SimpleControllerState
"""
from dataclasses import dataclass

from rlbot.utils.game_state_util import Rotator
from rlbot.agents.base_agent import SimpleControllerState

from util.vec import Vec3
from util.orientation import Orientation


@dataclass
class SimPhysics:
    location: Vec3
    velocity: Vec3
    angular_velocity: Vec3
    rotation: Rotator


def step(physics: SimPhysics, controls: SimpleControllerState, dt: float):

    # Start by assuming on ground. Will adjust things in the future
    steer = controls.steer
    radians_per_sec = 0
    if abs(controls.throttle) > 0.01:
        # Average values taken by graphing it out
        radians_per_sec = 2.42
        if controls.boost and controls.handbrake:
            radians_per_sec = 4.5
        elif controls.handbrake:
            radians_per_sec = 5
        elif controls.boost:
            radians_per_sec = 2.05

    yaw = physics.rotation.yaw
    physics.rotation.yaw = yaw + radians_per_sec * steer * dt

    orientation = Orientation(physics.rotation)
    orientation.forward.z = 0
    direction = orientation.forward.dot(physics.velocity) < 0
    if direction == 0:
        direction = 1
    direction_float_normalized = direction / abs(direction)

    acceleration = 0
    throttle = controls.throttle
    # Assume constant acceleration
    if controls.boost:
        acceleration = 2000
    elif abs(throttle) > 0.015 and (direction * throttle > 0):
        # not coasting and movement direction and throttle direction match
        acceleration = 1000 * direction_float_normalized * abs(throttle)
    elif abs(throttle) > 0.015 and (direction * throttle < 0):
        # not coasting but braking
        acceleration = -3500 * direction_float_normalized * abs(throttle)
    elif abs(throttle) <= 0.015:
        # this is fixed, not multiplied by throttle value
        acceleration = -525 * direction_float_normalized

    delta_v = acceleration * dt
    original_v_mag = physics.velocity.length()
    if controls.boost:
        original_v_mag = min(original_v_mag, 2300)
    else:
        original_v_mag = min(original_v_mag, 1410)

    physics.velocity = orientation.forward * (original_v_mag + delta_v)
    delta_x = physics.velocity * dt
    physics.location += delta_x

    return physics

