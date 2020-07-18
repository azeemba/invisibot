"""
Module to do rough car simulation based on SimpleControllerState
"""
from dataclasses import dataclass
from math import atan2, pi, fmod

from rlbot.utils.game_state_util import Rotator
from rlbot.agents.base_agent import SimpleControllerState

from rlutilities.linear_algebra import (
    vec3 as rlu_vec3,
    euler_to_rotation,
    mat3,
    dot,
    norm,
    vec2 as rlu_vec2,
    transpose,
    angle_between,
    axis_to_rotation,
    normalize,
    cross,
)
from rlutilities.simulation import Game, Field, sphere

from util.vec import Vec3
from util.orientation import Orientation

# only use it for `Field` to be initialized.
g = Game()
g.set_mode("soccar")


@dataclass
class SimPhysics:
    location: Vec3
    velocity: Vec3
    angular_velocity: Vec3
    rotation: Rotator

    @staticmethod
    def r(r_: Rotator) -> Rotator:
        return Rotator(r_.pitch, r_.yaw, r_.roll)

    @staticmethod
    def p(p_):
        return SimPhysics(
            location=Vec3(p_.location),
            velocity=Vec3(p_.velocity),
            angular_velocity=Vec3(p_.angular_velocity),
            rotation=SimPhysics.r(p_.rotation),
        )


def to_rlu_vec(v) -> rlu_vec3:
    return rlu_vec3(v.x, v.y, v.z)


def rlu_to_Vec3(v) -> Vec3:
    return Vec3(v[0], v[1], v[2])


def rotate_vector(v: Vec3, o: Orientation) -> Vec3:
    x = v.dot(o.forward)
    y = v.dot(o.right)
    z = v.dot(o.up)
    return Vec3(x, y, z)


def throttle_acceleration_at_velocity(v_mag) -> float:
    acc = 0
    if 0 <= v_mag <= 1400:
        component = v_mag / 1400
        acc = 1600 * (1 - component) + 160 * component
    elif 1400 < v_mag < 1410:
        acc = 160  # should be interp'd but whatever
    return acc


def rot_mat_to_rot(theta: mat3) -> Rotator:
    pitch = atan2(theta[(2, 0)], norm(rlu_vec2(theta[(0, 0)], theta[(1, 0)])))
    yaw = atan2(theta[(1, 0)], theta[(0, 0)])
    roll = atan2(-theta[(2, 1)], theta[(2, 2)])
    return Rotator(pitch, yaw, roll)


def compare(
    human_physics_last: SimPhysics,
    controls_last: SimpleControllerState,
    human_physics_cur: SimPhysics,
    dt,
):
    # assume ground for now
    full_step(human_physics_last, controls_last, dt)

    # compare cur and last
    cur = human_physics_cur
    last = human_physics_last
    velo = cur.velocity - last.velocity
    loc = cur.location - last.location
    rot = Orientation(cur.rotation).up - Orientation(last.rotation).up

    print(
        f"Off by: v: {velo}:{velo.length():3f}, p: {loc}:{loc.length():3f}, rot: {rot}:{rot.length():3f}"
    )


def on_ground_detection(p):
    physics: SimPhysics = SimPhysics(
        Vec3(p.location), Vec3(p.velocity), Vec3(p.angular_velocity), p.rotation
    )

    r = 60 # car radius
    height = 30  # should be 17 but sometimes the curves are wonky
    result = Field.collide(sphere(to_rlu_vec(physics.location), r))

    updated_location = rlu_to_Vec3(result.start)
    normal = rlu_to_Vec3(result.direction)

    # validate that we roughly match being on ground.

    # Use height for now
    orientation = Orientation(physics.rotation)
    drift1 = fmod(angle_between(to_rlu_vec(orientation.up), result.direction), pi)
    on_ground_by_orientation = drift1 < (
        pi / 13
    )  # 15 degrees, normal "up" is not perfectly 0

    drift2 = physics.location - updated_location
    on_ground_by_distance = drift2.length() < height  # car height

    on_ground = on_ground_by_orientation and on_ground_by_distance
    if not on_ground:
        print(f"Not on ground: o: {drift1}, d: {drift2.length()}")
    else:
        print(f"GROUNDDDDD ------ o: {drift1}, d: {drift2.length()} ----------")


def clamp(physics):
    height = 17 * 0.98
    physics.location.z = max(height, physics.location.z)
    physics.location.x = min(max(-4096 + height, physics.location.x), 4096 - height)
    physics.location.y = min(max(-5120-880 + height, physics.location.y), 5120+880 - height)

    v_mag = physics.velocity.length()
    if v_mag > 2300:
        physics.velocity = physics.velocity.rescale(2300)


def full_step(physics: SimPhysics, controls: SimpleControllerState, dt: float):
    """
    Do the appropriate simulation based on the car and controller state
    """
    r = 120  # car radius
    height = 17
    on_ceiling = physics.location.z >= (2044 - height)*0.9
    result = Field.collide(sphere(to_rlu_vec(physics.location), r))

    normal = rlu_to_Vec3(result.direction)
    normal_base = rlu_to_Vec3(result.start) + normal * height

    # validate that we roughly match being on ground.

    # Use height for now
    orientation = Orientation(physics.rotation)
    drift1 = fmod(angle_between(to_rlu_vec(orientation.up), result.direction), pi)
    on_ground_by_orientation = drift1 < (
        pi / 13
    )  # 15 degrees, normal "up" is not perfectly 0

    drift2 = (physics.location - normal_base).dot(normal)
    if abs(drift2) < 0.95*height and not on_ceiling:
        # too close yo, if too negative you maybe on top of the ceiling?
        physics.location = normal_base + height * normal

    on_ground_by_distance = drift2 < 1.05 * height  # car height

    # perfectly sticky walls
    # if drift2 < 1.5 * height and not on_ground_by_distance:
    #     physics.location = normal_base + height * normal
    #     on_ground_by_distance = True

    on_ground = on_ground_by_orientation and on_ground_by_distance and not on_ceiling

    # print(f"{normal}, {orientation.up}")
    if not on_ground:
        print(f"Not on ground: o: {drift1}, d: {drift2}")
        physics.velocity += Vec3(0, 0, -650 * dt)
        if on_ground_by_distance and not on_ceiling:
            normal_velo = physics.velocity.dot(normal*-1)
            physics.velocity -= (normal * normal_velo)*dt
            damp_nonforward(physics, orientation)
            physics.location += physics.velocity * dt

            # correct orientation
            angle = angle_between(to_rlu_vec(normal), to_rlu_vec(orientation.up))
            rotate_axis = cross(to_rlu_vec(normal), to_rlu_vec(orientation.up))
            ortho = (
                normalize(rotate_axis) * -angle
            )
            rot = physics.rotation
            rot_mat_initial: mat3 = euler_to_rotation(
                rlu_vec3(rot.pitch, rot.yaw, rot.roll)
            )
            rot_mat_adj = axis_to_rotation(ortho)

            rot_mat = dot(rot_mat_adj, rot_mat_initial)
            physics.rotation = rot_mat_to_rot(rot_mat)
            # orientation = Orientation(physics.rotation)

            # target_right = cross(to_rlu_vec(normal), to_rlu_vec(orientation.forward))
            # angle = angle_between(target_right, to_rlu_vec(orientation.right))
            # ortho = normalize(cross(target_right, to_rlu_vec(orientation.right)))*angle
            # rot_mat_adj = axis_to_rotation(ortho)

            # rot_mat = dot(rot_mat_adj, rot_mat_initial)
            # physics.rotation = rot_mat_to_rot(rot_mat)


        else:
            physics.location += physics.velocity * dt

        clamp(physics)
        return physics

    normal_velo = physics.velocity.dot(normal*-1)
    physics.velocity -= (normal * normal_velo)*dt
    physics_prime = SimPhysics(
        rotate_vector(physics.location, orientation),
        rotate_vector(physics.velocity, orientation),
        rotate_vector(physics.angular_velocity, orientation),
        Rotator(0, 0, 0),
    )

    # steer may need to be rotated
    if physics.velocity.dot(orientation.forward) < 0:
        controls.steer = -controls.steer
    move_on_ground(physics_prime, controls, dt)

    # need to combine orientations
    rot = physics.rotation
    rot_mat_initial: mat3 = euler_to_rotation(rlu_vec3(rot.pitch, rot.yaw, rot.roll))
    rot = physics_prime.rotation
    rot_mat_upd: mat3 = euler_to_rotation(rlu_vec3(rot.pitch, rot.yaw, rot.roll))

    rot_mat = dot(rot_mat_upd, rot_mat_initial)
    rot = rot_mat_to_rot(rot_mat)
    physics.rotation = rot

    # unrotate other vectors
    if physics.velocity.dot(orientation.forward) < 0:
        controls.steer = -controls.steer
    inverse_rotation = transpose(rot_mat_initial)
    inverse_orientation = Orientation.from_rot_mat(inverse_rotation)

    physics.location = rotate_vector(physics_prime.location, inverse_orientation)
    physics.velocity = rotate_vector(physics_prime.velocity, inverse_orientation)
    physics.angular_velocity = rotate_vector(
        physics_prime.angular_velocity, inverse_orientation
    )

    # what are the chances that this works first try! Very small.
    clamp(physics)
    return physics


def move_on_ground(physics: SimPhysics, controls: SimpleControllerState, dt: float):
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
    # orientation.forward.z = 0  # do i need this?

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
    v_mag = physics.velocity.length()
    if controls.boost:
        boost_acc = 992
        acc = boost_acc + throttle_acceleration_at_velocity(v_mag)
        acceleration = orientation.forward * acc
    elif abs(throttle) > 0.015 and (direction * throttle >= 0):
        # not coasting and movement direction and throttle direction match
        acceleration = orientation.forward * (
            throttle * throttle_acceleration_at_velocity(v_mag)
        )
    elif abs(throttle) > 0.015 and (direction * throttle < 0):
        # not coasting but braking because velocity and throttle have oposite directions
        # shouldn't be higher than thing we are resisting
        resistance = min(3500, physics.velocity.length() / dt)
        acceleration = orientation.forward * (-resistance * direction)
    elif abs(throttle) <= 0.015:
        # coast deceleration
        # shouldn't be higher than thing we are resisting
        resistance = min(525, physics.velocity.length() / dt)
        acceleration = orientation.forward * (-resistance * direction)

    delta_v = acceleration * dt
    physics.velocity += delta_v

    # Presumably we should damp any non-forward velocity
    if direction != 0 and not controls.handbrake:
        damp_nonforward(physics, orientation)
        
    # we are on ground make z velocity 0
    # physics.velocity.z = 0

    v_mag = physics.velocity.length()
    if v_mag > 2300:
        physics.velocity = physics.velocity.rescale(2300)

    delta_x = physics.velocity * dt
    physics.location += delta_x

    return physics


def damp_nonforward(physics: SimPhysics, orientation: Orientation):
    direction_dot = orientation.forward.dot(physics.velocity)
    if abs(direction_dot) <  0.1:
        physics.velocity = Vec3(0, 0, 0)
        return
    direction = direction_dot / abs(direction_dot)
    damp_factor = 0.9  # lost 90%
    v_mag = physics.velocity.length()
    forward_dir = (orientation.forward * direction).normalized()
    forward_velo = forward_dir.dot(physics.velocity)
    nonforward_magnitude = v_mag - abs(forward_velo)
    if nonforward_magnitude > 7.5:
        damp_factor = 0.9
    else:
        damp_factor = 0.5
    # damp the nonforward velo
    nonforward_direction = (
        physics.velocity.normalized() - forward_dir
    ).normalized()
    physics.velocity -= (
        nonforward_direction * damp_factor * nonforward_magnitude
    )

    nonforward_magnitude = v_mag - forward_velo
    # print(f"Nonforward mag {nonforward_magnitude}")
