
from time import monotonic
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, CarState, Physics, Vector3, Rotator

from car_simulation_by_controls import SimPhysics, CarSimmer
class TestBot(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.init = False
        self.count = 0
        self.ts = monotonic()
        self.car_sim = None

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
            self.car_sim = CarSimmer(SimPhysics.p(p), self.renderer)
            self.init = True

            return SimpleControllerState()

        if not self.car_sim:
            assert False, "immpossible!"

        dt = self.ts - monotonic()
        self.ts = monotonic()
        c = SimpleControllerState(steer=self.turn, throttle=1, boost=True)
        self.car_sim.tick(c, dt)

        self.count += 1

        if self.count % 360 == 0:
            self.turn = -1*self.turn

        if self.count % 20 == 0:
            location = Vector3(
                self.car_sim.physics.location.x,
                self.car_sim.physics.location.y,
                self.car_sim.physics.location.z,
            )
            self.set_car(
                Physics(
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
                    )
                )
            )

        return SimpleControllerState()

        
