from copy import deepcopy
from time import monotonic 
from math import pi, atan2, ceil
from queue import Empty as QueueEmpty

from rlbot.matchcomms.common_uses.reply import reply_to
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, CarState, Physics, Vector3, Rotator

from util.ball_prediction_analysis import find_slice_at_time
from util.boost_pad_tracker import BoostPadTracker
from util.vec import Vec3
from util.orientation import Orientation, relative_location

from util.strategy import BaseStrategy, BallChaseStrat, StrategyGoal, HelpfulTestStrat

from car_simulation_by_controls import SimPhysics, CarSimmer

def revector3(vec: Vector3):
    return Vector3(vec.x, vec.y, vec.z)

class Invisibot(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        print("Initialized")
        self.hidden = False
        self.car_sim : CarSimmer = None
        self.boost = 100
        self.strategy: BaseStrategy = BallChaseStrat()

        self.timestamp = monotonic()

        self.boost_pad_tracker = BoostPadTracker()

        self.count = 0
        self.trying_to_comeback = False
        self.seen_timer = 0

        self.locations = []
        self.up = []
        self.forward = []

    def initialize_agent(self):
        self.boost_pad_tracker.initialize_boosts(self.get_field_info())

    def record_and_mark_simulation_spots(self, packet):
        if self.count % 10 == 0:
            o = Orientation(self.car_sim.physics.rotation)
            self.locations.append(Vec3(self.car_sim.physics.location))
            self.up.append(Vec3(o.up))
            self.forward.append(Vec3(o.forward))

        self.renderer.begin_rendering()
        if len(self.locations) > 100:
            # obvious optimization but for another day
            self.locations = self.locations[len(self.locations) - 100 :]
            self.up = self.up[len(self.locations) - 100 :]
            self.forward = self.forward[len(self.locations) - 100 :]

            # hack
            if self.hidden and (self.locations[-2] - self.locations[-1]).length() < 2:
                self.unhide(packet)

        for i in range(len(self.locations)):
            loc = self.locations[i]
            component = float(i) / len(self.locations)
            inverse = 1 - component
            color = self.renderer.create_color(
                255, ceil(255 * component), ceil(255 * inverse / 2), ceil(255 * inverse)
            )
            self.renderer.draw_rect_3d(loc, 4, 4, True, color, centered=True)
            self.renderer.draw_line_3d(loc, loc + (self.up[i] * 200), color)
            self.renderer.draw_line_3d(loc, loc + (self.forward[i] * 200), color)
        self.renderer.end_rendering()

    def hide(self, packet: GameTickPacket):
        """Move the car so its not visible"""
        if self.timestamp - self.seen_timer < 3:
            return
        print("Hide")
        physics = SimPhysics.p(packet.game_cars[self.index].physics)
        self.car_sim.reset(physics)
        state = GameState(
            cars={
                self.index: CarState(
                    physics=Physics(location=Vector3(3520, 5100, 0),
                    velocity=Vector3(0, 0, 0)))
            }
        )
        self.set_game_state(state)
        self.boost = packet.game_cars[self.index].boost
        self.hidden = True

    def unhide(self, packet: GameTickPacket):
        print("unhide")
        r = self.car_sim.physics.rotation
        p = Physics(
            location=revector3(self.car_sim.physics.location),
            velocity=revector3(self.car_sim.physics.velocity),
            rotation=Rotator(pitch=r.pitch, yaw=r.yaw, roll=r.roll)
        )
        state = GameState(
            cars={
                self.index: CarState(
                    physics=p,
                    boost_amount=self.boost,
                )
            }
        )
        self.set_game_state(state)
        self.hidden = False
        self.trying_to_comeback = True
        self.seen_timer = self.timestamp

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car.
        """
        if not packet.game_info.is_round_active:
            self.car_sim = None
            return SimpleControllerState()
        if self.car_sim is None:
            self.car_sim = CarSimmer(
                SimPhysics.p(packet.game_cars[self.index].physics),
                self.renderer)
            self.up = []
            self.forward = []
            self.locations = []
            self.hidden = False
            self.seen_timer = 0
            self.trying_to_comeback = False
        try:
            msg = self.matchcomms.incoming_broadcast.get_nowait()
            if msg:
                self.hidden = False
                self.seen_timer = 0
                self.trying_to_comeback = False
                reply_to(self.matchcomms, msg)
                print(f"Found: {msg}")
        except QueueEmpty:
            pass 
        self.count += 1

        if self.count % 60 == 0:
            pass
            # print("Ticking")
            # print(self.car_sim.physics.location)

        if not packet.game_info.is_round_active:
            return SimpleControllerState()
        tick_time = monotonic() - self.timestamp
        self.timestamp = monotonic()

        # Keep our boost pad info updated with which pads are currently active
        self.boost_pad_tracker.update_boost_status(packet)

        my_car = packet.game_cars[self.index]
        if self.trying_to_comeback:
            expected_location = Vec3(self.car_sim.physics.location)
            current_location = Vec3(packet.game_cars[self.index].physics.location)
            if expected_location.dist(current_location) > 300:
                print("Waiting to get there")
                return SimpleControllerState()
            else:
                self.trying_to_comeback = False


        physics = self.car_sim.physics
        if not self.hidden:
            physics = SimPhysics.p(my_car.physics)
 
        # Gather some information about our car and the ball
        car_location = physics.location
        ball_location = Vec3(packet.game_ball.physics.location)

        if car_location.dist(ball_location) < 1000 and self.hidden:
            self.unhide(packet)
        elif not self.hidden:
            self.hide(packet)

        result = self.strategy.tick(physics, packet, self.boost_pad_tracker)

        # self.renderer.draw_rect_3d(
        #     self.physics.location, 8, 8, True, self.renderer.cyan(), centered=True
        # )
        self.record_and_mark_simulation_spots(packet)
        if self.hidden:
            controls = result.controls
            self.car_sim.tick(controls, tick_time)
        else:
            controls = result.controls
            return controls

        return SimpleControllerState()