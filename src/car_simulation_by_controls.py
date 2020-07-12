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
    if abs(dt) < 1e-5:
        # what? why?
        return None
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
    orientation.forward.z = 0 # do i need this?

    direction = 1
    if physics.velocity.length() < 10:
        direction = 0
    else:
        direction_dot = orientation.forward.dot(physics.velocity)
        direction = direction_dot / abs(direction_dot)

    acceleration = Vec3()
    throttle = controls.throttle
    # What we need to handle:
    # If 0 velocity, apply throttle direction
    # If 0 throttle, apply coasting deceleration
    # For velocity not in"forward" direction, apply coasting deceleration
    # If velocity and throttle are opposites, apply breaking deceleration
    # Otherwise, apply throttle acceleration.
    # If boost is 1, apply forward throttle as well
    # Assume constant acceleration
    if controls.boost:
        acceleration = orientation.forward*(2000)
    elif abs(throttle) > 0.015 and (direction * throttle >= 0):
        # not coasting and movement direction and throttle direction match
        acceleration = orientation.forward * (1000 * throttle)
    elif abs(throttle) > 0.015 and (direction * throttle < 0):
        # not coasting but braking because velocity and throttle have oposite directions
        # shouldn't be higher than thing we are resisting
        resistance = min(3500, physics.velocity.length()/dt)
        acceleration = orientation.forward * (-resistance * direction)
    elif abs(throttle) <= 0.015:
        # coast deceleration
        # shouldn't be higher than thing we are resisting
        resistance = min(525, physics.velocity.length()/dt)
        acceleration = orientation.forward * (-resistance * direction)

    if direction < 0:
        print(f"Going backwards! {physics.velocity}, acc: {acceleration}")
    delta_v = acceleration * dt
    physics.velocity += delta_v

    # Presumably we should damp any non-forward velocity
    damp_factor = 0.9 # lost 90%
    v_mag = physics.velocity.length()
    forward_velo = orientation.forward.dot(physics.velocity)
    nonforward_magnitude = v_mag - abs(forward_velo)
    if nonforward_magnitude > 7.5 and not controls.handbrake:
        # damp the nonforward velo
        nonforward_direction = (physics.velocity.normalized() - orientation.forward).normalized()
        physics.velocity -= nonforward_direction * damp_factor * nonforward_magnitude
    
    nonforward_magnitude = v_mag - forward_velo
    print(f"Nonforward mag {nonforward_magnitude}")


    v_mag = physics.velocity.length()
    if controls.boost and v_mag > 2300:
        physics.velocity = physics.velocity.rescale(2300)
    elif v_mag > 1410:
        physics.velocity = physics.velocity.rescale(1410)

    delta_x = physics.velocity * dt
    physics.location += delta_x

    return physics

