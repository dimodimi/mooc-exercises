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
        return collision_check_circle_circle(a, b)

    elif isinstance(a, Circle) and isinstance(b, Rectangle):
        return collision_check_rect_circle(b, a)

    elif isinstance(a, Rectangle) and isinstance(b, Circle):
        return collision_check_rect_circle(a, b)

    elif isinstance(a, Rectangle) and isinstance(b, Rectangle):
        return collision_check_rect_rect(a, b)
    else:
        raise TypeError("Arguments must be either a Circle or a Rectangle primitive") 

def collision_check_circle_circle(c1: PlacedPrimitive, c2: PlacedPrimitive) -> bool:
    return (c1.pose.x - c2.pose.x)**2 + (c1.pose.y - c2.pose.y)**2 < (c1.primitive.radius + c2.primitive.radius)**2

def collision_check_rect_rect(r1: PlacedPrimitive, r2: PlacedPrimitive) -> bool:
    w2, h2 = r2.primitive.xmax - r2.primitive.xmin, r2.primitive.ymax - r2.primitive.ymin

    theta = r2.pose.theta_deg - r1.pose.theta_deg
    
    # Get the second rectangle's corners in the first one's frame of reference, centered at (0, 0)
    corners = np.array([[r2.pose.x - r1.pose.x], [r2.pose.y - r1.pose.y]]) + \ 
                np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]) @ \
                    np.array([[w2/2, w2/2, -w2/2, -w2/2], [h2/2, -h2/2, h2/2, -h2/2]])

    # Separating line theorem for convex polygons

    # For the rotated rectangle get the xmin, xmax, ymin, ymax of the enclosing rectangle
    x2_min, x2_max = np.amin(corners[0]), np.amax(corners[0])
    y2_min, y2_max = np.amin(corners[1]), np.amax(corners[1])

    x_overlap = (r1.primitive.xmin < x2_max) and (r1.primitive.xmax > x2_min)
    y_overlap = (r1.primitive.ymin < y2_max) and (r1.primitive.ymax > y2_min)

    return x_overlap and y_overlap

def collision_check_rect_circle(rect: PlacedPrimitive, circ: PlacedPrimitive) -> bool:

    # The rectangle acts as our reference frame, so transform the circle
    theta = rect.pose.theta_deg
    new_center = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]) @ \
                    np.array([[circ.pose.x - rect.pose.x], [circ.pose.y - rect.pose.y]])

    return (new_center[0] - np.clip(new_center[0], a_min=rect.primitive.xmin, a_max=rect.primitive.xmax))**2 + \
            (new_center[1] - np.clip(new_center[1], a_min=rect.primitive.ymin, a_max=rect.primitive.ymax))**2 < circ.primitive.radius**2

