
from time import monotonic
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, CarState, Physics, Vector3, Rotator

from car_simulation_by_controls import SimPhysics, full_step as carSimStep
class TestBot(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.init = False
        self.count = 0
        self.ts = monotonic()
        self.physics = None

        self.turn = 1

    def set_car(self, p):
        state = GameState(
            cars={
                self.index: CarState(
                    physics=p
                )
            }
        )
        self.set_game_state(state)

    def get_output(self, packet: GameTickPacket):
        if not packet.game_info.is_round_active:
            self.init = False
            self.count = 0
            return SimpleControllerState()

        if not self.init:
            p = Physics(
                location=Vector3(0, 0, 30),
                velocity=Vector3(500, 0, 0),
                angular_velocity=Vector3(0, 0, 0),
                rotation=Rotator(0, 0, 0))
            self.set_car(p)
            self.ts = monotonic()
            self.physics = SimPhysics.p(p)
            self.init = True

            return SimpleControllerState()

        if not self.physics:
            assert False, "immpossible!"

        dt = self.ts - monotonic()
        self.ts = monotonic()
        c = SimpleControllerState(steer=self.turn, throttle=1, boost=True)
        carSimStep(
            self.physics,
            c,
            dt,
            self.renderer)

        self.count += 1

        if self.count % 360 == 0:
            self.turn = -1*self.turn

        if self.count % 20 == 0:
            location = Vector3(
                self.physics.location.x,
                self.physics.location.y,
                self.physics.location.z,
            )
            self.set_car(
                Physics(
                    location=location,
                    rotation=Rotator(
                        self.physics.rotation.pitch,
                        -self.physics.rotation.yaw,
                        self.physics.rotation.roll,
                    ),
                    velocity=Vector3(
                        -self.physics.velocity.x,
                        -self.physics.velocity.y,
                        -self.physics.velocity.z,
                    ),
                    angular_velocity=Vector3(
                        self.physics.angular_velocity.x,
                        self.physics.angular_velocity.y,
                        self.physics.angular_velocity.z,
                    )
                )
            )

        return SimpleControllerState()

        
