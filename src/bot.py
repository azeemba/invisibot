from copy import deepcopy
from time import monotonic 
from math import pi, atan2
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

from util.strategy import BaseStrategy, BallChaseStrat, StrategyGoal

from car_simulation_by_controls import SimPhysics, full_step as carSimStep

def revector3(vec: Vector3):
    return Vector3(vec.x, vec.y, vec.z)

class Invisibot(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        print("Initialized")
        self.hidden = False
        self.physics: SimPhysics = None
        self.boost = 100
        self.strategy: BaseStrategy = BallChaseStrat()

        self.timestamp = monotonic()

        self.boost_pad_tracker = BoostPadTracker()

        self.count = 0
        self.trying_to_comeback = False
        self.seen_timer = 0

    def initialize_agent(self):
        self.boost_pad_tracker.initialize_boosts(self.get_field_info())

    def hide(self, packet: GameTickPacket):
        """Move the car so its not visible"""
        if self.timestamp - self.seen_timer < 3:
            return
        print("Hide")
        self.physics = SimPhysics.p(packet.game_cars[self.index].physics)
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
        r = self.physics.rotation
        print(r)
        p = Physics(
            location=revector3(self.physics.location),
            velocity=revector3(self.physics.velocity),
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
            self.physics = None
            return SimpleControllerState()
        if self.physics is None:
            self.physics = SimPhysics.p(packet.game_cars[self.index].physics)
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
            print("Ticking")
            print(self.physics.location)

        if not packet.game_info.is_round_active:
            return SimpleControllerState()
        tick_time = self.timestamp - monotonic()
        self.timestamp = monotonic()

        # Keep our boost pad info updated with which pads are currently active
        self.boost_pad_tracker.update_boost_status(packet)

        my_car = packet.game_cars[self.index]
        if self.trying_to_comeback:
            expected_location = Vec3(self.physics.location)
            current_location = Vec3(packet.game_cars[self.index].physics.location)
            if expected_location.dist(current_location) > 300:
                print("Waiting to get there")
                return SimpleControllerState()
            else:
                self.trying_to_comeback = False


        if not self.hidden:
            self.physics = SimPhysics.p(my_car.physics) # why do this?
 
        # Gather some information about our car and the ball
        car_location = Vec3(self.physics.location)
        ball_location = Vec3(packet.game_ball.physics.location)

        if car_location.dist(ball_location) < 1000 and self.hidden:
            self.unhide(packet)
        elif not self.hidden:
            self.hide(packet)

        result = self.strategy.tick(self.physics, packet, self.boost_pad_tracker)

        self.renderer.draw_rect_3d(
            self.physics.location, 8, 8, True, self.renderer.cyan(), centered=True
        )
        if self.hidden:
            controls = result.controls
            carSimStep(self.physics, controls, tick_time)
        else:
            controls = result.controls
            return controls

        return SimpleControllerState()

    # def old_function(self, car_location, ball_location, packet):
    #     if car_location.dist(ball_location) > 1500:
    #         # We're far away from the ball, let's try to lead it a little bit
    #         ball_prediction = (
    #             self.get_ball_prediction_struct()
    #         )  # This can predict bounces, etc
    #         ball_in_future = find_slice_at_time(
    #             ball_prediction, packet.game_info.seconds_elapsed + 2
    #         )
    #         target_location = Vec3(ball_in_future.physics.location)
    #         self.renderer.draw_line_3d(
    #             ball_location, target_location, self.renderer.cyan()
    #         )
    #     else:
    #         target_location = ball_location

    #     # Draw some things to help understand what the bot is thinking
    #     self.renderer.draw_line_3d(car_location, target_location, self.renderer.white())
    #     self.renderer.draw_string_3d(
    #         car_location,
    #         1,
    #         1,
    #         f"Speed: {car_velocity.length():.1f}",
    #         self.renderer.white(),
    #     )
    #     self.renderer.draw_rect_3d(
    #         target_location, 8, 8, True, self.renderer.cyan(), centered=True
    #     )

    #     controls = SimpleControllerState()
    #     controls.steer = steer_toward_target(my_car, target_location)
    #     controls.throttle = 1.0
    #     # You can set more controls if you want, like controls.boost.

    #     return controls

    # def begin_front_flip(self, packet):
    #     # Send some quickchat just for fun
    #     self.send_quick_chat(
    #         team_only=False, quick_chat=QuickChatSelection.Information_IGotIt
    #     )

    #     # Do a front flip. We will be committed to this for a few seconds and the bot will ignore other
    #     # logic during that time because we are setting the active_sequence.
    #     self.active_sequence = Sequence(
    #         [
    #             ControlStep(duration=0.05, controls=SimpleControllerState(jump=True)),
    #             ControlStep(duration=0.05, controls=SimpleControllerState(jump=False)),
    #             ControlStep(
    #                 duration=0.2, controls=SimpleControllerState(jump=True, pitch=-1)
    #             ),
    #             ControlStep(duration=0.8, controls=SimpleControllerState()),
    #         ]
    #     )

    #     # Return the controls associated with the beginning of the sequence so we can start right away.
    #     return self.active_sequence.tick(packet)
