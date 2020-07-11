"""
Generic strategies that can return controls
or target positions
"""

from dataclasses import dataclass
from typing import Optional

import math

from rlbot.agents.base_agent import SimpleControllerState
from rlbot.utils.game_state_util import Physics, Vector3
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.vec import Vec3
from util.orientation import Orientation, relative_location

@dataclass
class StrategyGoal:
    target: Vec3
    boost: bool

@dataclass
class StrategyResult:
    controls: Optional[SimpleControllerState]
    goal: Optional[StrategyGoal]


class BaseStrategy():
    def __init__(self):
        pass

    def tick(self, car_state: Physics, packet: GameTickPacket, field_boost) -> StrategyResult:
        raise NotImplementedError()


class BallChaseStrat(BaseStrategy):
    def steer_angle(self, car_physics: Physics, target: Vector3):
        relative = relative_location(Vec3(car_physics.location), Orientation(car_physics.rotation), Vec3(target))
        angle = math.atan2(relative.y, relative.x)
        return max(min((angle * 5), 1), -1)

    def tick(self, car_state: Physics, packet: GameTickPacket, field_boost) -> StrategyResult:
        ball_location = packet.game_ball.physics.location

        boost = True
        goal = StrategyGoal(Vec3(ball_location), boost)
        controls = SimpleControllerState(steer=self.steer_angle(car_state, ball_location), throttle=1, boost=boost)

        return StrategyResult(controls, goal)