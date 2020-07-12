from time import sleep

from rlbot.agents.base_script import BaseScript
from rlbot.utils.game_state_util import Vector3, Rotator

from util.orientation import Orientation, relative_location
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState

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

class SimulationHumanTest(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)

        # Controls.
        pygame.init()
        screen = pygame.display.set_mode((300, 300))
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.axis_data = [0] * self.controller.get_numaxes()
        self.button_data = [False] * self.controller.get_numbuttons()

    def update_controls(self, controls: SimpleControllerState):
        controls.throttle = -self.axis_data[2]
        controls.steer = self.axis_data[0]
        controls.pitch = self.axis_data[1]
        controls.roll = -1 if self.button_data[4] else 1 if self.button_data[5] else 0
        controls.yaw = self.axis_data[0] 
        controls.jump = self.button_data[0]
        controls.boost = self.button_data[1]
        controls.handbrake = self.button_data[2]
    def get_output(self, packet):
        offset = 100 

        controls = SimpleControllerState()

        human = packet.game_cars[0]
        orientation = Orientation(human.physics.rotation)
        offset_vec = orientation.forward * offset

        loc = offset_vec + human.physics.location
        self.renderer.begin_rendering()
        self.renderer.draw_rect_3d(
            loc, 8, 8, True, self.renderer.cyan(), centered=True
        )
        self.renderer.end_rendering()
        # print(f"Location: {human.physics.location}")
        # print(f"Rotation: {human.physics.rotation}")
        # print(f"Velocity: {human.physics.velocity}")
        # print(f"AngVelo : {human.physics.angular_velocity}")

        # Get controls.
        for event in pygame.event.get(): # User did something.
            print("Getting controls", event)
            # if event.type == pygame.JOYAXISMOTION:
            #     self.axis_data[event.axis] = deadzone(
            #         event.value, transform=event.axis > 3
            #     )
            # elif event.type == pygame.JOYBUTTONDOWN:
            #     self.button_data[event.button] = True

            # elif event.type == pygame.JOYBUTTONUP:
            #     self.button_data[event.button] = False
        
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        for i in range(self.controller.get_numbuttons()):
            button = self.controller.get_button(i)
            self.button_data[i] = button

        for i in range(self.controller.get_numaxes()):
            axis = self.controller.get_axis(i)
            self.axis_data[i] = axis

        # Update controls.
        self.update_controls(controls)
        print("Sending controls", controls.throttle)
        return controls
        # self.game_interface.update_player_input(controls, 0)


# h = SimulationHumanTest()
# h.start()
