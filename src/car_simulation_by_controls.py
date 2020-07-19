"""
Module to do rough car simulation based on SimpleControllerState
"""
from dataclasses import dataclass
from math import atan2, pi, fmod, ceil

from rlbot.utils.game_state_util import Rotator
from rlbot.agents.base_agent import SimpleControllerState

from rlutilities.linear_algebra import (
    vec3 as rlu_vec3,
    euler_to_rotation,
    mat3,
    mat4,
    dot,
    norm,
    vec2 as rlu_vec2,
    vec4 as rlu_vec4,
    transpose,
    angle_between,
    axis_to_rotation,
    normalize,
    cross,
)
from rlutilities.simulation import Game, Field, obb

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


def make_obb(physics: SimPhysics):
    fixed_size = rlu_vec3(118.0/2, 84.2/2, 36.16/2) # octane
    orientation = Orientation(physics.rotation).to_rot_mat()
    
    box = obb()
    box.center = to_rlu_vec(physics.location)
    box.half_width = fixed_size
    box.orientation = orientation
    return box


def to_rlu_vec(v) -> rlu_vec3:
    return rlu_vec3(v.x, v.y, v.z)


def rlu_to_Vec3(v) -> Vec3:
    return Vec3(v[0], v[1], v[2])


def rotate_vector(v: Vec3, r: mat3, around: Vec3=None) -> Vec3:
    if around is None:
        out = dot(r,to_rlu_vec(v))
        return Vec3(out[0], out[1], out[2])
    else:
        transform1 = mat4(
            1, 0, 0, around.x,
            0, 1, 0, around.y,
            0, 0, 1, around.z,
            0, 0, 0, 1)
        transform2 = mat4(
            1, 0, 0, -around.x,
            0, 1, 0, -around.y,
            0, 0, 1, -around.z,
            0, 0, 0, 1)
        rot = mat4(
            r[(0, 0)], r[0, 1], r[0, 2], 0,
            r[(1, 0)], r[1, 1], r[1, 2], 0,
            r[(2, 0)], r[2, 1], r[2, 2], 0,
            0, 0, 0, 1
        )
        out = dot(dot(dot(transform1, rot), transform2), rlu_vec4(v.x, v.y, v.z, 1)) # 4d
        return Vec3(out[0], out[1], out[2])


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
    assert False
    physics: SimPhysics = SimPhysics(
        Vec3(p.location), Vec3(p.velocity), Vec3(p.angular_velocity), p.rotation
    )

    r = 60 # car radius
    height = 30  # should be 17 but sometimes the curves are wonky
    result = Field.collide(make_obb(physics))

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
    # This is a coarse clamping.
    height = 0 #36.16/2.0 * 0.99
    physics.location.z = max(height, physics.location.z)
    physics.location.x = min(max(-4096 + height, physics.location.x), 4096 - height)
    physics.location.y = min(max(-5120-880 + height, physics.location.y), 5120+880 - height)

    v_mag = physics.velocity.length()
    if v_mag > 2300:
        physics.velocity = physics.velocity.rescale(2300)

results = []
last_normal = None

def full_step(physics: SimPhysics, controls: SimpleControllerState, dt: float, renderer=None):
    """
    Do the appropriate simulation based on the car and controller state
    """
    global results
    global last_normal
    height = 36.16/2
    result = Field.collide(make_obb(physics))
    normal = rlu_to_Vec3(result.direction)
    normal_base = rlu_to_Vec3(result.start)
    on_ground = (normal - Vec3(0, 0, 0)).length() > 1e-9
    # normal usually has length 1, unless there is no normal

    last_distance = 200
    if last_normal:
        last_distance = (physics.location - last_distance).length()


    if renderer:
        if len(results) > 200:
            results = results[-200:]
        results.append((normal, normal_base))
        renderer.begin_rendering()
        for i in range(len(results)):
            loc = results[i][1]
            component = float(i) / len(results)
            inverse = 1 - component
            color = renderer.create_color(
                255, ceil(255 * component), ceil(255 * inverse / 2), ceil(255 * inverse)
            )
            renderer.draw_rect_3d(loc, 4, 4, True, color, centered=True)
            renderer.draw_line_3d(loc, loc + (results[i][0] * 200), color)
        renderer.end_rendering()

    # print(f"{normal}, {orientation.up}")
    orientation = Orientation(physics.rotation)
    if not on_ground:
        print(f"Not on ground by {last_distance}")
        physics.velocity += Vec3(0, 0, -650 * dt)
        physics.location += physics.velocity * dt

        clamp(physics)
        return physics
       
    drift1 = fmod(angle_between(to_rlu_vec(orientation.up), result.direction), pi)
    on_ground_by_orientation = abs(drift1) < ( pi / 36)  # 5 degrees

    # are we on the wrong side?
    # completed screwed test:
    inside_point = Vec3(0, 0, 100)
    dist_to_inside = (inside_point - normal_base).dot(normal)
    if dist_to_inside < 0:
        print(f"Way inside: {dist_to_inside}")
        # we have to go opposite of the normal to get to inside
        physics.velocity += Vec3(0, 0, -650 * dt)
        # normal direction is INTO the wall. So subtract from that direction
        physics.velocity -= (normal * 3000)*dt
        physics.location += physics.velocity * dt
        clamp(physics)
        return physics

    # just clipping in:
    dist_to_base = (physics.location - normal_base).dot(normal)
    if dist_to_base - (height*0.95) < 0:
        # print(f"Clipping in by {height - dist_to_base}")
        # we can raise it, OR rotate it.
        # lets raise it for now
        physics.location += (height - dist_to_base)*normal*(dt*30)
        result = Field.collide(make_obb(physics))
        normal = rlu_to_Vec3(result.direction)
        normal_base = rlu_to_Vec3(result.start)
 
    if not on_ground_by_orientation:
        print(f"Not on ground by o: {drift1}")
        # correct orientation 
        angle = angle_between(to_rlu_vec(normal), to_rlu_vec(orientation.up))
        rotate_axis = cross(to_rlu_vec(normal), to_rlu_vec(orientation.up))
        ortho = (
            normalize(rotate_axis) * -min(angle, 2*pi*dt) # max turning of 10 radians/s
        )
        rot = physics.rotation
        rot_mat_initial: mat3 = euler_to_rotation(
            rlu_vec3(rot.pitch, rot.yaw, rot.roll)
        )
        rot_mat_adj = axis_to_rotation(ortho)

        rot_mat = dot(rot_mat_adj, rot_mat_initial)
        physics.rotation = rot_mat_to_rot(rot_mat)

        physics.velocity += Vec3(0, 0, -650 * dt)
        normal_velo = physics.velocity.dot(normal*-1)
        physics.velocity -= (normal * normal_velo)*dt
        # damp_nonforward(physics, orientation)
        physics.location += physics.velocity * dt
        clamp(physics)

        return physics


        # orientation = Orientation(physics.rotation)

        # target_right = cross(to_rlu_vec(normal), to_rlu_vec(orientation.forward))
        # angle = angle_between(target_right, to_rlu_vec(orientation.right))
        # ortho = normalize(cross(target_right, to_rlu_vec(orientation.right)))*angle
        # rot_mat_adj = axis_to_rotation(ortho)

        # rot_mat = dot(rot_mat_adj, rot_mat_initial)
        # physics.rotation = rot_mat_to_rot(rot_mat)

    physics.velocity += Vec3(0, 0, -650 * dt)
    normal_velo = physics.velocity.dot(normal*-1)
    physics.velocity -= (normal * normal_velo)*dt

    # steer may need to be rotated
    # if physics.velocity.dot(orientation.forward) < 0:
        # controls.steer = -controls.steer

    physics_prime = SimPhysics(
        rotate_vector(physics.location, orientation.to_rot_mat()),
        rotate_vector(physics.velocity, orientation.to_rot_mat(), physics.location),
        physics.angular_velocity, # ground move just ignores it 
        Rotator(0, 0, 0),
    )
    # print(f"Steer: {controls.steer}, Throttle: {controls.throttle}")

    move_on_ground(physics_prime, controls, dt)

    # need to combine orientations
    rot = physics.rotation
    rot_mat_initial: mat3 = Orientation(rot).to_rot_mat()
    rot = physics_prime.rotation
    rot_mat_upd: mat3 = Orientation(rot).to_rot_mat()

    rot_mat = dot(rot_mat_upd, rot_mat_initial)
    rot = rot_mat_to_rot(rot_mat)
    physics.rotation = rot

    # unrotate other vectors
    # if physics.velocity.dot(orientation.forward) < 0:
        # controls.steer = -controls.steer
    inverse_rotation = transpose(rot_mat_initial)

    physics.location = rotate_vector(physics_prime.location, inverse_rotation)
    physics.velocity = rotate_vector(physics_prime.velocity, inverse_rotation, physics_prime.location)
    physics.angular_velocity = physics_prime.angular_velocity # should be unchanged

    # what are the chances that this works first try! Very small.
    clamp(physics)
    return physics


def move_on_ground(physics: SimPhysics, controls: SimpleControllerState, dt: float, renderer=None):
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
    physics.rotation.yaw = yaw - radians_per_sec * steer * dt

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
    physics.velocity.z = 0

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
    v_mag = physics.velocity.length()
    forward_dir = (orientation.forward * direction).normalized()
    forward_velo = forward_dir.dot(physics.velocity)
    nonforward_magnitude = v_mag - abs(forward_velo)
    damp_factor = 0.9  # lost 90%
    if nonforward_magnitude < 7.5:
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
