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

    rototranslated_robot = [PlacedPrimitive(
                                RT_pose(robot_pose, p.pose),
                                type(p.primitive)(*p.primitive.__dict__.values()))
                            for p in robot_body]

    # start by rototranslating the robot parts by the robot pose

    collided = check_collision_list(rototranslated_robot, Wcoll)

    return collided


def check_collision_list(A: List[PlacedPrimitive], B: List[PlacedPrimitive]) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    for a, b in itertools.product(A, B):
        if check_collision_shape(a, b):
            return True

    return False


def check_collision_shape(a: PlacedPrimitive, b: PlacedPrimitive) -> bool:
    # This is just some code to get you started, but you don't have to follow it exactly

    if isinstance(a.primitive, Circle) and isinstance(b.primitive, Circle):
        return collision_check_circle_circle(a, b)

    elif isinstance(a.primitive, Circle) and isinstance(b.primitive, Rectangle):
        return collision_check_rect_circle(b, a)
    
    elif isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Circle):
        return collision_check_rect_circle(a, b)
    
    elif isinstance(a.primitive, Rectangle) and isinstance(b.primitive, Rectangle):
        return collision_check_rect_rect(a, b)
    
    else:
        raise TypeError(f"Arguments must be either a Circle or a Rectangle primitive - instead got {type(a.primitive)} and {type(b.primitive)}") 

def collision_check_circle_circle(c1: PlacedPrimitive, c2: PlacedPrimitive) -> bool:
    return (c1.pose.x - c2.pose.x)**2 + (c1.pose.y - c2.pose.y)**2 <= (c1.primitive.radius + c2.primitive.radius)**2

def collision_check_rect_rect(r1: PlacedPrimitive, r2: PlacedPrimitive) -> bool:
    theta2 = np.deg2rad(r2.pose.theta_deg)
    theta1 = np.deg2rad(r1.pose.theta_deg)
   
    #w2, h2 = r2.primitive.xmax - r2.primitive.xmin, r2.primitive.ymax - r2.primitive.ymin
    #w1, h1 = r1.primitive.xmax - r1.primitive.xmin, r1.primitive.ymax - r1.primitive.ymin

    # Get the second rectangle's corners in the first one's frame of reference, centered at (0, 0)
    #corners =   np.array([[np.cos(theta1), np.sin(theta1)], [-np.sin(theta1), np.cos(theta1)]]) @ \
    #corners  = (np.array([[r2.pose.x - r1.pose.x], [r2.pose.y - r1.pose.y]]) + \
    #            np.array([[np.cos(theta2-theta1), -np.sin(theta2-theta1)], [np.sin(theta2-theta1), np.cos(theta2-theta1)]]) @ \
    #                np.array([[r2.primitive.xmax, r2.primitive.xmax, r2.primitive.xmin, r2.primitive.xmin],\
    #                          [r2.primitive.ymax, r2.primitive.ymin, r2.primitive.ymax, r2.primitive.ymin]]))
    
    corners = np.array([[r2.primitive.xmax, r2.primitive.xmax, r2.primitive.xmin, r2.primitive.xmin],
                        [r2.primitive.ymax, r2.primitive.ymin, r2.primitive.ymax, r2.primitive.ymin],
                        [1.0, 1.0, 1.0, 1.0]], dtype=np.float32)

    corners_tr = T_matrix(r2.pose, r1.pose) @ corners



    # Separating line theorem for convex polygons

    # For the rotated rectangle get the xmin, xmax, ymin, ymax of the enclosing rectangle
    x2_min, x2_max = np.amin(corners_tr[0]), np.amax(corners_tr[0])
    y2_min, y2_max = np.amin(corners_tr[1]), np.amax(corners_tr[1])

    x_overlap = (r1.primitive.xmin <= x2_max) and (r1.primitive.xmax >= x2_min)
    y_overlap = (r1.primitive.ymin <= y2_max) and (r1.primitive.ymax >= y2_min)

    if x_overlap and y_overlap:
        return True

    # Same procedure with the origin at the second rectangle
    #corners =   np.array([[np.cos(theta2), np.sin(theta2)], [-np.sin(theta2), np.cos(theta2)]]) @ \
    #corners = (np.array([[r1.pose.x - r2.pose.x], [r1.pose.y - r2.pose.y]]) + \
    #                np.array([[np.cos(theta1-theta2), -np.sin(theta1-theta2)], [np.sin(theta1-theta2), np.cos(theta1-theta2)]]) @ \
    #                    np.array([[r1.primitive.xmax, r1.primitive.xmax, r1.primitive.xmin, r1.primitive.xmin],\
    #                              [r1.primitive.ymax, r1.primitive.ymin, r1.primitive.ymax, r1.primitive.ymin]]))

    corners = np.array([[r1.primitive.xmax, r1.primitive.xmax, r1.primitive.xmin, r1.primitive.xmin],
                        [r1.primitive.ymax, r1.primitive.ymin, r1.primitive.ymax, r1.primitive.ymin],
                        [1.0, 1.0, 1.0, 1.0]], dtype=np.float32)

    corners_tr = T_matrix(r1.pose, r2.pose) @ corners

    x1_min, x1_max = np.amin(corners_tr[0]), np.amax(corners_tr[0])
    y1_min, y1_max = np.amin(corners_tr[1]), np.amax(corners_tr[1])

    x_overlap = (r2.primitive.xmin <= x1_max) and (r2.primitive.xmax >= x1_min)
    y_overlap = (r2.primitive.ymin <= y1_max) and (r2.primitive.ymax >= y1_min)

    if x_overlap and y_overlap:
        return True

    return False


def collision_check_rect_circle(rect: PlacedPrimitive, circ: PlacedPrimitive) -> bool:

    # The rectangle acts as our reference frame, so transform the circle
    #theta = -np.deg2rad(rect.pose.theta_deg)
    #new_center = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]) @ \
    #                np.array([[circ.pose.x - rect.pose.x], [circ.pose.y - rect.pose.y]])

    new_center = T_matrix(circ.pose, rect.pose) @ np.array([[0.0], [0.0], [1.0]])

    return (new_center[0] - np.clip(new_center[0], a_min=rect.primitive.xmin, a_max=rect.primitive.xmax))**2 + \
            (new_center[1] - np.clip(new_center[1], a_min=rect.primitive.ymin, a_max=rect.primitive.ymax))**2 <= circ.primitive.radius**2


def RT_pose(Q: FriendlyPose, prev_pose: FriendlyPose) -> FriendlyPose:

    A = FriendlyPose(prev_pose.x, prev_pose.y, prev_pose.theta_deg)
    W = FriendlyPose(0.0, 0.0, 0.0)
    T = T_matrix(A, W)

    final_pose = T @ np.array([[Q.x], [Q.y], [1.0]])

    return FriendlyPose(final_pose[0, 0], final_pose[1, 0], prev_pose.theta_deg + Q.theta_deg)


def R_matrix(theta_deg: float, inverse: bool = False) -> np.ndarray:
    
    # Counterclockwise rotation by theta_deg degrees
    theta = np.deg2rad(theta_deg)
    ct, st = np.cos(theta), np.sin(theta)

    if not inverse:
        return np.array([[ct, -st], [st, ct]], dtype=np.float32)
    else:
        return np.array([[ct, st], [-st, ct]], dtype=np.float32)


def T_matrix(A: FriendlyPose, B: FriendlyPose, inverse: bool = False) -> np.ndarray:

    # Transformation matrix from frame A to B (B to A if inverse=True)

    T = np.zeros((3, 3), dtype=np.float32)
    T[2, 2] = 1.0

    RA = R_matrix(A.theta_deg, inverse=False)
    RB = R_matrix(B.theta_deg, inverse=False)
    t = np.array([[A.x - B.x], [A.y - B.y]], dtype=np.float32)

    if not inverse:
        T[:2, :2] = RA @ RB.T
        T[:2, 2:3] = RB.T @ t
    else:
        T[:2, :2] = RB @ RA.T
        T[:2, 2:3]  = - RA.T @ t

    return T

 
