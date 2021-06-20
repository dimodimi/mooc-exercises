import itertools
import random
from typing import List

import numpy as np

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    Circle,
    CollisionCheckQuery,
    CollisionCheckResult,
    MapDefinition,
    PlacedPrimitive,
    Rectangle,
)

__all__ = ["CollisionChecker"]


class CollisionChecker:
    params: MapDefinition

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: MapDefinition):
        context.info("initialized")
        self.params = data

    def on_received_query(self, context: Context, data: CollisionCheckQuery):
        collided = check_collision(
            Wcoll=self.params.environment, robot_body=self.params.body, robot_pose=data.pose
        )
        result = CollisionCheckResult(collided)
        context.write("response", result)


def check_collision(
    Wcoll: List[PlacedPrimitive], robot_body: List[PlacedPrimitive], robot_pose: FriendlyPose
) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    # start by rototranslating the robot parts by the robot pose
    rototranslated_robot: List[PlacedPrimitive] = []  #

    collided = check_collision_list(rototranslated_robot, Wcoll)

    # return a random choice
    return collided


def check_collision_list(A: List[PlacedPrimitive], B: List[PlacedPrimitive]) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    for a, b in itertools.product(A, B):
        if check_collision_shape(a, b):
            return True

    return False


def check_collision_shape(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    if isinstance(a, Circle) and isinstance(b, Circle):
        # If the distance between the two centers is less than the sum of the radii we have collision
        return collision_check_circle_circle(a, b)

    elif isinstance(a, Circle) and isinstance(b, Rectangle):
        

    elif isinstance(a, Rectangle) and isinstance(b, Circle):

    elif isinstance(a, Rectangle) and isinstance(b, Rectangle):
        return collision_check_rect_rect(a, b)
    else:
        raise 
    # for now let's return a random guess

    return ...

def collision_check_circle_circle(c1: PlacedPrimitive, c2: PlacedPrimitive) -> bool:
    return (c1.pose.x - c2.pose.x)**2 + (c1.pose.y - c2.pose.y)**2 < (c1.primitive.radius + c2.primitive.radius)**2

def collision_check_rect_rect(r1: PlacedPrimitive, r2: PlacedPrimitive) -> bool:
    w1, h1 = r1.primitive.xmax - r1.primitive.xmin, r1.primitive.ymax - r1.primitive.ymin
    w2, h2 = r2.primitive.xmax - r2.primitive.xmin, r2.primitive.ymax - r2.primitive.ymin

    theta = r2.pose.theta_deg - r1.pose.theta_deg
    
    # Get the second rectangle's corners in the first one's frame of reference, centered at (0, 0)
    corners = np.array([[r2.pose.x - r1.pose.x], [r2.pose.y - r1.pose.y]]) + \ 
                np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.os(theta)]]) @ np.array([[w2/2, w2/2, -w2/2, -w2/2], [h2/2, -h2/2, h2/2, -h2/2]])

    corners = np.abs(corners)
    # Check if any of the corners is within the first rectangle
    for i in range(4):
        if corners[0, i] < w1/2 and corners[1, i] < h1/2:
            return True

    return False

def rect_circle_heuristic(rect: PlacedPrimitive, circ: PlacedPrimitive) -> bool:
    # Create two circles related to the rectangle
    circumscribed = PlacedPrimitive( 
                        FriendlyPose(rect.x, rect.y, 0.0),
                        ## FIX radius - rectangle is rotated in general cant use just mix-max values
                        Circle(np.sqrt((rect.primitive.xmax - rect.primitive.xmin)**2 + \
                                       (rect.primitive.ymax - rect.primitive.ymin)**2) / 2.0)
                    )

    if not collision_check_circle_circle(circ, circumscribed):
        return False


