"""
Fake bot that takes controls from a controller.

Code taken from https://github.com/robbai/RLBotSequence/tree/master/src and
example code from pygame: https://www.pygame.org/docs/ref/joystick.html#pygame.joystick.Joystick
"""

from math import ceil
from time import sleep, monotonic
import json

from rlbot.agents.base_script import BaseScript
from rlbot.utils.game_state_util import Vector3, Rotator, GameState, CarState, Physics

from util.orientation import Orientation, relative_location
from util.vec import Vec3
from car_simulation_by_controls import (
    SimPhysics,
    CarSimmer,
    rotate_and_move_only as carSimStep,
    compare,
)

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket


def v3toUI(v3):
    return Vector3(v3.x, v3.y, v3.z)


from math import copysign
import pygame


def clamp11(val: float):
    return copysign(min(abs(val), 1), val)


limit_hz = 120

controls_attributes = [
    "throttle",
    "steer",
    "pitch",
    "yaw",
    "roll",
    "jump",
    "boost",
    "handbrake",
]


def deadzone(axis, transform=False):
    if transform:
        axis = (axis + 1) / 2
    return clamp11(axis) if abs(axis) >= 0.1 else 0


MODE = "SIM_ONLY"  # 'SIM_ONLY' #'USER_CAR_ONLY'


class SimulationHumanTest(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)

        self.car_sim: CarSimmer = None

        # Controls.
        pygame.init()
        screen = pygame.display.set_mode((300, 300))
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.axis_data = [0] * self.controller.get_numaxes()
        self.button_data = [False] * self.controller.get_numbuttons()

        self.start_ts = monotonic()
        self.last_recorded_ts = 0
        self.last_tick_ts = 0

        self.locations = []
        self.up = []
        self.forward = []

        self.non_forward_velocity = []
        self.count = 0

        self.compare_helpers = {"last_ts": 0, "last_phys": None}

    def update_controls(self, controls: SimpleControllerState):
        controls.throttle = -self.axis_data[2]
        controls.steer = self.axis_data[0]
        controls.pitch = self.axis_data[1]
        controls.roll = -1 if self.button_data[4] else 1 if self.button_data[5] else 0
        controls.yaw = self.axis_data[0]
        controls.jump = self.button_data[0]
        controls.boost = self.button_data[1]
        controls.handbrake = self.button_data[2]

    def reset_physics(self):
        print("Resetting")
        self.last_tick_ts = monotonic()
        self.locations = []
        self.up = []
        self.forward = []
        physics = SimPhysics(
            location=Vec3(0, 0, 30),
            velocity=Vec3(0, 0, 0),
            angular_velocity=Vec3(0, 0, 0),
            rotation=Rotator(0, 0, 0)
        )
        self.car_sim = CarSimmer(physics)

    def mark_simulation_spots(self):
        self.renderer.begin_rendering()
        if len(self.locations) > 100:
            # obvious optimization but for another day
            self.locations = self.locations[-100 :]
            self.up = self.up[-100 :]
            self.forward = self.forward[-100 :]
        for i in range(len(self.locations)):
            loc = self.locations[i]
            component = float(i) / len(self.locations)
            inverse = 1 - component
            color = self.renderer.create_color(
                255, ceil(255 * component), ceil(255 * inverse / 2), ceil(255 * inverse)
            )
            self.renderer.draw_rect_3d(loc, 4, 4, True, color, centered=True)
            self.renderer.draw_line_3d(loc, loc + (self.up[i] * 200), color)
            # self.renderer.draw_line_3d(loc, loc + (self.forward[i] * 200), color)
        self.renderer.end_rendering()

    def collect_nonforward_velocity(self, human, ts):
        velo = human.physics.velocity
        orientation = Orientation(human.physics.rotation)

        forward_velo = orientation.forward.dot(velo)
        nonforward = forward_velo - Vec3(velo).length()
        self.non_forward_velocity.append((ts, nonforward))

        if self.button_data[6] and len(self.non_forward_velocity) > 200:
            with open("C:\\tmp\\non-forward.json", "w") as fh:
                json.dump(self.non_forward_velocity, fh)
                self.non_forward_velocity = []

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        # Get controls.
        for _ in pygame.event.get():  # User did something.
            pass

        if self.car_sim is None:
            self.reset_physics()
            return SimpleControllerState()


        if MODE == "SIM_ONLY" and self.count % 30 == 1:
            location = Vector3(
                self.car_sim.physics.location.x,
                self.car_sim.physics.location.y,
                self.car_sim.physics.location.z,
            )

            self.set_game_state(
                GameState(
                    cars={
                        self.index: CarState(
                            physics=Physics(
                                location=location,
                                rotation=Rotator(
                                    self.car_sim.physics.rotation.pitch,
                                    self.car_sim.physics.rotation.yaw,
                                    self.car_sim.physics.rotation.roll,
                                ),
                                velocity=Vector3(
                                    self.car_sim.physics.velocity.x,
                                    self.car_sim.physics.velocity.y,
                                    self.car_sim.physics.velocity.z,
                                ),
                                angular_velocity=Vector3(
                                    self.car_sim.physics.angular_velocity.x,
                                    self.car_sim.physics.angular_velocity.y,
                                    self.car_sim.physics.angular_velocity.z,
                                ),
                            )
                        )
                    }
                )
            )

        if not packet.game_info.is_round_active:
            return SimpleControllerState()
        human = packet.game_cars[0]

        cur_ts = monotonic()
        self.count += 1
        if self.count % 30 == 0 and MODE == "USER_CAR_ONLY":
            compare(
                self.compare_helpers["last_phys"],
                self.compare_helpers["controls"],
                SimPhysics.p(human.physics),
                cur_ts - self.compare_helpers["last_ts"],
            )

        # self.collect_nonforward_velocity(human, cur_ts)

        self.mark_simulation_spots()

        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        for i in range(self.controller.get_numbuttons()):
            button = self.controller.get_button(i)
            self.button_data[i] = button

        for i in range(self.controller.get_numaxes()):
            axis = self.controller.get_axis(i)
            self.axis_data[i] = axis

        if self.button_data[6]:
            self.reset_physics()

        # Update controls.
        controls = SimpleControllerState()
        self.update_controls(controls)
        # print("Sending controls", controls.throttle)
        tick_duration = cur_ts - self.last_tick_ts
        if not MODE == "USER_CAR_ONLY":
            self.car_sim.tick(controls, tick_duration)

        if abs(self.car_sim.physics.location.x) > 4096 + 10 or abs(self.car_sim.physics.location.y) > (
            5120 + 880
        ):
            print("Car out of view")

        self.compare_helpers["last_phys"] = SimPhysics.p(human.physics)
        self.compare_helpers["last_ts"] = cur_ts
        self.compare_helpers["controls"] = controls

        if not MODE == "USER_CAR_ONLY":
            self.last_tick_ts = cur_ts

            if cur_ts - self.last_recorded_ts > 0.05:
                o = Orientation(self.car_sim.physics.rotation)
                self.locations.append(Vec3(self.car_sim.physics.location))
                self.up.append(o.up)
                self.forward.append(o.forward)
                self.last_recorded_ts = cur_ts

            return controls
        elif MODE == "USER_CAR_ONLY":
            return controls
        else:
            # print("carSimpStep returned none")
            return SimpleControllerState()

        # self.game_interface.update_player_input(controls, 0)


# h = SimulationHumanTest()
# h.start()
