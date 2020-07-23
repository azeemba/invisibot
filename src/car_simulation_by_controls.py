"""
Module to do rough car simulation based on SimpleControllerState
"""
from time import monotonic
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
from rlutilities.simulation import Game, Field, obb, Car as RLUCar, Input as RLUInput

from util.vec import Vec3
from util.orientation import Orientation, relative_location

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

class CarSimmer:
    def __init__(self, physics: SimPhysics, renderer=None):
        self.physics: SimPhysics = physics
        self.boost = 100 # allow to be set later
        self.renderer = renderer
        self.rlu_car = RLUCar()
        self.is_rlu_updated = False

        self.last_base = Vec3(0, 0, -1000)
        self.last_normal = Vec3(0, 0, 1)
    
    def reset(self, physics: SimPhysics):
        self.physics = physics
        self._set_rlu_car()

    def _set_rlu_car(self):
        c = self.rlu_car
        c.position = to_rlu_vec(self.physics.location)
        c.velocity = to_rlu_vec(self.physics.velocity)
        c.angular_velocity = to_rlu_vec(self.physics.angular_velocity)
        c.orientation = Orientation(self.physics.rotation).to_rot_mat()

        c.boost = self.boost
        c.jumped = False
        c.double_jumped = False
        c.on_ground = True
        c.team = 0
        c.time = monotonic()
        self.is_rlu_updated = True

    def _make_input(self, controls):
        inputs = RLUInput()
        fields = ['boost', 'handbrake', 'jump', 'pitch', 'roll', 'steer', 'throttle', 'yaw']
        for f in fields:
            setattr(inputs, f, getattr(controls, f))
        return inputs

    def _rlu_step(self, controls, dt, on_ground):
        # we think RLU sims this situation better
        print("RLU sim")
        if not self.is_rlu_updated:
            self._set_rlu_car()
        
        self.rlu_car.on_ground = on_ground

        inputs = self._make_input(controls)
        self.rlu_car.step(inputs, dt)

        self.physics.location = rlu_to_Vec3(self.rlu_car.position)
        self.physics.velocity = rlu_to_Vec3(self.rlu_car.velocity)
        self.physics.angular_velocity = rlu_to_Vec3(self.rlu_car.angular_velocity)
        self.physics.rotation = Orientation.from_rot_mat(self.rlu_car.orientation)
        self.boost = self.rlu_car.boost

    def tick(self, controls: SimpleControllerState, dt: float):
        # we will now find actual normal
        result = Field.collide(make_obb(self.physics))
        normal = rlu_to_Vec3(result.direction)

        on_ground = True

        if normal.length() < 0.1: 
            # normally should be one. This just means there is no collision
            last_distance = (self.last_base - self.physics.location).length()
            if last_distance > (36.16 / 2)*1.05:
                on_ground = False
            normal = self.last_normal
        else:
            self.last_base = clamp_location(rlu_to_Vec3(result.start))
            self.last_normal = normal

        if not on_ground or controls.jump:
            self._rlu_step(controls, dt, on_ground)
            clamp(self.physics)
            return self.physics

        self.is_rlu_updated = False
        orientation = Orientation(self.physics.rotation)
        # compare orientations
        if normal.ang_to(orientation.up) > pi / 6:
            # 30 degrees seems like an awful a lot
            return stuck_on_ground(self.physics, controls, dt, result, self.renderer)

        self.physics.rotation = rotate_by_axis(orientation, orientation.up, normal)

        rotate_ground_and_move(self.physics, controls, dt, normal)
        clamp(self.physics)
        return self.physics


def printO(o: Orientation):
    print(f"f: {o.forward}, r: {o.right}, u: {o.up}")


def rotate_by_axis(orientation: Orientation, original: Vec3, target: Vec3, percent=1) -> Rotator:
    """
    Updates the given orientation's original direction to the target direction.
    original should be `orientation.forward` or `orientation.up` or `orientation.right`
    """
    angle = angle_between(to_rlu_vec(target), to_rlu_vec(original))*percent
    rotate_axis = cross(to_rlu_vec(target), to_rlu_vec(original))
    ortho = normalize(rotate_axis) * -angle
    rot_mat_initial: mat3 = orientation.to_rot_mat()
    rot_mat_adj = axis_to_rotation(ortho)

    rot_mat = dot(rot_mat_adj, rot_mat_initial)
    r = rot_mat_to_rot(rot_mat)
    # printO(orientation)
    # printO(Orientation(r))
    return r


def make_obb(physics: SimPhysics):
    fixed_size = rlu_vec3(118.0 / 2, 84.2 / 2, 36.16 / 2)  # octane
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


def rotate_vector(v: Vec3, r: mat3, around: Vec3 = None) -> Vec3:
    out = dot(r, to_rlu_vec(v))
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

def clamp_location(location: Vec3):
    height = 0  # 36.16/2.0 * 0.99
    location.z = max(height, location.z)
    location.x = min(max(-4096 + height, location.x), 4096 - height)
    location.y = min(
        max(-5120 - 880 + height, location.y), 5120 + 880 - height
    )
    return location

def clamp(physics):
    # This is a coarse clamping.
    clamp_location(physics.location)
    v_mag = physics.velocity.length()
    if v_mag > 2300:
        physics.velocity = physics.velocity.rescale(2300)


results = []
last_normal = None


def full_step(
    physics: SimPhysics, controls: SimpleControllerState, dt: float, renderer=None
):
    """
    Do the appropriate simulation based on the car and controller state
    """
    global results
    global last_normal
    height = 36.16 / 2
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
    on_ground_by_orientation = abs(drift1) < (pi / 36)  # 5 degrees

    # are we on the wrong side?
    # completed screwed test:
    inside_point = Vec3(0, 0, 100)
    dist_to_inside = (inside_point - normal_base).dot(normal)
    if dist_to_inside < 0:
        print(f"Way inside: {dist_to_inside}")
        # we have to go opposite of the normal to get to inside
        physics.velocity += Vec3(0, 0, -650 * dt)
        # normal direction is INTO the wall. So subtract from that direction
        physics.velocity -= (normal * 3000) * dt
        physics.location += physics.velocity * dt
        clamp(physics)
        return physics

    # just clipping in:
    dist_to_base = (physics.location - normal_base).dot(normal)
    if dist_to_base - (height * 0.95) < 0:
        # print(f"Clipping in by {height - dist_to_base}")
        # we can raise it, OR rotate it.
        # lets raise it for now
        physics.location += (height - dist_to_base) * normal * (dt * 30)
        result = Field.collide(make_obb(physics))
        normal = rlu_to_Vec3(result.direction)
        normal_base = rlu_to_Vec3(result.start)

    if not on_ground_by_orientation:
        print(f"Not on ground by o: {drift1}")
        # correct orientation
        angle = angle_between(to_rlu_vec(normal), to_rlu_vec(orientation.up))
        rotate_axis = cross(to_rlu_vec(normal), to_rlu_vec(orientation.up))
        ortho = normalize(rotate_axis) * -min(
            angle, 2 * pi * dt
        )  # max turning of 10 radians/s
        rot = physics.rotation
        rot_mat_initial: mat3 = euler_to_rotation(
            rlu_vec3(rot.pitch, rot.yaw, rot.roll)
        )
        rot_mat_adj = axis_to_rotation(ortho)

        rot_mat = dot(rot_mat_adj, rot_mat_initial)
        physics.rotation = rot_mat_to_rot(rot_mat)

        physics.velocity += Vec3(0, 0, -650 * dt)
        normal_velo = physics.velocity.dot(normal * -1)
        physics.velocity -= (normal * normal_velo) * dt
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

    return rotate_ground_and_move(physics, dt, normal, controls)


def not_on_ground(
    physics: SimPhysics, controls: SimpleControllerState, dt: float, renderer=None
):
    # just fall!
    physics.velocity += Vec3(0, 0, -650 * dt)
    physics.location += physics.velocity * dt
    clamp(physics)
    return physics


def stuck_on_ground(
    physics: SimPhysics,
    controls: SimpleControllerState,
    dt: float,
    collision,
    renderer=None,
):
    # will give you a free flip
    height = 36.16 / 2
    normal = rlu_to_Vec3(collision.direction)
    normal_base = rlu_to_Vec3(collision.start)

    physics.velocity += Vec3(0, 0, -650 * dt)
    normal_velo = physics.velocity.dot(normal * -1)
    if normal_velo > 0:
        physics.velocity -= normal * normal_velo
    physics.location += physics.velocity * dt

    if (physics.location - normal_base).length() < height * 0.95:
        physics.location = normal_base + normal * height * 0.99

    # update orientation so that in 0.5 seconds we have correct orientation
    orientation = Orientation(physics.rotation)
    physics.rotation = rotate_by_axis(orientation, orientation.up, normal, dt/0.5)


    clamp(physics)
    return physics


def rotate_and_move_only(
    physics: SimPhysics, control: SimpleControllerState, dt: float, renderer=None
):
    # we will now find actual normal
    result = Field.collide(make_obb(physics))
    normal = rlu_to_Vec3(result.direction)

    if normal.length() < 0.1:
        # normally should be one. This just means there is no collision
        return not_on_ground(physics, control, dt, renderer)

    orientation = Orientation(physics.rotation)
    # compare orientations
    if normal.ang_to(orientation.up) > pi / 6:
        # 30 degrees seems like an awful a lot
        return stuck_on_ground(physics, control, dt, result, renderer)

    physics.rotation = rotate_by_axis(orientation, orientation.up, normal)

    rotate_ground_and_move(physics, control, dt, normal)
    clamp(physics)
    return physics


def rotate_ground_and_move(
    physics: SimPhysics, controls: SimpleControllerState, dt: float, normal: Vec3
):
    physics.velocity += Vec3(0, 0, -650 * dt)
    normal_velo = physics.velocity.dot(normal * -1)
    if normal_velo > 0:
        physics.velocity -= (normal * normal_velo) * dt

    # steer may need to be rotated
    # if physics.velocity.dot(orientation.forward) < 0:
    # controls.steer = -controls.steer

    orientation = Orientation(physics.rotation)
    orig_rot_mat = orientation.to_rot_mat()

    physics_prime = SimPhysics(
        Vec3(0, 0, 0),
        Vec3(
            orientation.forward.dot(physics.velocity),
            orientation.right.dot(physics.velocity),
            0,
        ),
        physics.angular_velocity,  # ground move just ignores it
        Rotator(0, 0, 0),
    )
    # print(f"Steer: {controls.steer}, Throttle: {controls.throttle}")

    move_on_ground(physics_prime, controls, dt)

    # need to combine orientations
    old_yaw = physics.rotation.yaw
    new_orientation = Orientation(physics_prime.rotation)
    physics.rotation = rot_mat_to_rot(
        dot(orientation.to_rot_mat(), new_orientation.to_rot_mat())
    )

    # unrotate other vectors
    # if physics.velocity.dot(orientation.forward) < 0:
    # controls.steer = -controls.steer
    # inverse_rotation = transpose(orig_rot_mat)

    physics.location += (
        orientation.forward * physics_prime.location.x
        + orientation.right * physics_prime.location.y
    )
    physics.velocity = (
        orientation.forward * physics_prime.velocity.x
        + orientation.right * physics_prime.velocity.y
    )
    physics.angular_velocity = physics_prime.angular_velocity  # should be unchanged

    # what are the chances that this works first try! Very small.
    clamp(physics)
    # print(f"{old_yaw} + {physics_prime.rotation.yaw} =? {physics.rotation.yaw}")
    return physics


def move_on_ground(
    physics: SimPhysics, controls: SimpleControllerState, dt: float, renderer=None
):
    if abs(dt) < 1e-5:
        # what? why?
        return None

    orientation = Orientation(physics.rotation)
    direction = 1
    if physics.velocity.length() < 10:
        direction = 0
    else:
        direction_dot = orientation.forward.dot(physics.velocity)
        direction = direction_dot / abs(direction_dot)

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
    physics.rotation.yaw = yaw + radians_per_sec * steer * dt * direction

    orientation = Orientation(physics.rotation)
    # orientation.forward.z = 0  # do i need this?

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
    if abs(direction_dot) < 0.1:
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
    nonforward_direction = (physics.velocity.normalized() - forward_dir).normalized()
    physics.velocity -= nonforward_direction * damp_factor * nonforward_magnitude

    nonforward_magnitude = v_mag - forward_velo
    # print(f"Nonforward mag {nonforward_magnitude}")
