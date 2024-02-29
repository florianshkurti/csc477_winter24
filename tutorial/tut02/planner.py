import math
import random
import threading
import time
from typing import List, Any

import numpy as np
import pygame
import pdb


class PotentialField:
    def __init__(
        self,
        map_dim,
        start_pose,
        start_radius,
        goal_pose,
        goal_radius,
        obstacles,
        map_surf,
        virtual,
    ):
        self.min_vel, self.max_vel = 2, 40
        self.map_dim = self.mapw, self.maph = map_dim

        self.field = np.zeros((self.mapw, self.maph, 2))

        self.start_pose = self.sx, self.sy = start_pose
        self.start_radius = start_radius
        self.sn = None

        self.goal_pose = self.gx, self.gy = goal_pose
        self.goal_radius = goal_radius
        self.gn = None

        self.goal_field = None
        self.obstacles = obstacles
        self.obstacle_field = dict((i, np.array(0)) for i in obstacles)
        self.path = []

        self.updated = False
        self.virtual, self.fcf = virtual, 5
        self.map_surf = map_surf

        self._r_scale = 1.5

    def start(self):
        self.goal_field = self.attract_goal(self.goal_radius)
        self.field = self.goal_field
        self.updated = False
        for obs in self.obstacles:
            if not obs in self.obstacle_field.keys():
                self.obstacle_field[obs] = self.repel_obstacle(obs)

            # NOTE: you can try to tune the weight
            self.field += self.obstacle_field[obs] * 0.4

        self.clamp_field(25)
        self.make_path()

    def attract_goal(self, radius):
        target_pos = self.goal_pose

        x = np.linspace(0, self.mapw - 1, self.mapw)
        y = np.linspace(0, self.maph - 1, self.maph)
        meshgrid = np.meshgrid(x, y, sparse=False, indexing="ij")

        meshgrid_x = target_pos[0] - meshgrid[0]
        meshgrid_y = target_pos[1] - meshgrid[1]
        displacement = np.zeros((self.mapw, self.maph, 2))
        displacement[:, :, 0], displacement[:, :, 1] = meshgrid_x, meshgrid_y

        # TODO ------------------------------------------------------
        # using `displacement` calculate the distance to the goal location
        dist = np.zeros((self.mapw, self.maph))

        raise NotImplementedError

        # -----------------------------------------------------------
        dist = np.clip(dist, 0.0000001, math.inf)

        # Create normal displacement
        force_dir = np.zeros((self.mapw, self.maph, 2))
        force_dir[:, :, 0] = displacement[:, :, 0] / dist
        force_dir[:, :, 1] = displacement[:, :, 1] / dist

        # adjust magnitude displacement to fit radius parameter
        dist[np.where(dist <= self.goal_radius)] = cvtRange(
            dist[np.where(dist <= self.goal_radius)],
            0,
            radius,
            self.max_vel,
            self.min_vel,
        )

        dist[np.where(dist > radius)] = 15
        # Create final force
        force = np.zeros((self.mapw, self.maph, 2))
        force[:, :, 0] = force_dir[:, :, 0] * dist
        force[:, :, 1] = force_dir[:, :, 1] * dist
        return force

    def repel_obstacle(self, obs):
        repulse_pos = (obs.x, obs.y)
        # create coordinate array to find distance
        x = np.linspace(0, self.mapw - 1, self.mapw)
        y = np.linspace(0, self.maph - 1, self.maph)
        meshgrid = np.meshgrid(x, y, sparse=False, indexing="ij")

        # find distance from target to coordinate
        meshgrid_x = meshgrid[0] - repulse_pos[0]
        meshgrid_y = meshgrid[1] - repulse_pos[1]

        # create field out of these distance calculations
        displacement = np.zeros((self.mapw, self.maph, 2))
        displacement[:, :, 0], displacement[:, :, 1] = meshgrid_x, meshgrid_y

        # NOTE:
        # this parameter determine the threshold of repulsive field
        r = obs.rad * self._r_scale

        # TODO ------------------------------------------------------
        # using `displacement` to calculate the distance to the obstacle
        dist = np.zeros((self.mapw, self.maph))

        raise NotImplementedError
        # -----------------------------------------------------------
        dist = np.clip(dist, 0.0000001, math.inf)

        # create the normal displacement that record the direction of the force
        force_dir = np.zeros((self.mapw, self.maph, 2))
        force_dir[:, :, 0] = displacement[:, :, 0] / dist
        force_dir[:, :, 1] = displacement[:, :, 1] / dist

        # create filter
        filter_ = np.where(dist <= r)

        # TODO ------------------------------------------------------
        # calculate the force magnitude
        force_mag = np.zeros((self.mapw, self.maph))
        raise NotImplementedError
        # -----------------------------------------------------------
        
        if len(filter_) != 0:
            force_mag[filter_] = cvtRange(
                force_mag[filter_], 0, r, self.max_vel, self.min_vel
            )

        filter_ = np.where(force_mag > r)
        if len(filter_) != 0:
            force_mag[filter_] = 0

        # create final field
        force = np.zeros((self.mapw, self.maph, 2))
        force[:, :, 0] = force_dir[:, :, 0] * force_mag
        force[:, :, 1] = force_dir[:, :, 1] * force_mag
        return force

    def draw(self, surface, stride=(25, 25)):
        # Iterate through the field with proper strides
        buffer_x = math.floor(stride[0] / 2.0)
        buffer_Y = math.floor(stride[1] / 2.0)
        for fieldX in range(buffer_x, self.mapw - buffer_x, stride[0]):
            for fieldY in range(buffer_Y, self.maph - buffer_Y, stride[1]):
                # Grab the field vector for the cell
                fieldVector = self.field[fieldX, fieldY]

                # Determine the x and y coordinate for the origin of the
                # potential line segment.
                startPixelX, startPixelY = fieldX, fieldY

                # Determine the x and y coordinate for the end point of the
                # potential line segment.
                endPixelX = math.floor(startPixelX + fieldVector[0])
                endPixelY = math.floor(startPixelY + fieldVector[1])

                # Draw the vector to the pygame surface
                draw_arrow(
                    surface, (startPixelX, startPixelY), (endPixelX, endPixelY)
                )

    def clamp_field(self, max_vel):
        """
        Clamp potential field such that the magnitude does not
        exceed max_vel
        """
        magnitude_field = np.sqrt(
            self.field[:, :, 0] ** 2 + self.field[:, :, 1] ** 2
        )
        magnitude_field = np.clip(magnitude_field, 0.000001, math.inf)
        normal_field = np.zeros((self.mapw, self.maph, 2))
        normal_field[:, :, 0] = self.field[:, :, 0] / magnitude_field
        normal_field[:, :, 1] = self.field[:, :, 1] / magnitude_field
        magnitude_field = np.clip(magnitude_field, 0, max_vel)
        self.field[:, :, 0] = normal_field[:, :, 0] * magnitude_field
        self.field[:, :, 1] = normal_field[:, :, 1] * magnitude_field

    def update_pose(self, start_pose, goal_pose):
        self.start_pose = start_pose
        self.goal_pose = goal_pose

    def make_path(self):
        self.sn = Node(*self.start_pose, 0)
        self.gn = None
        self.path = [self.sn]
        curr = self.sn
        stuck = False
        while self.gn is None:
            if self.updated:
                self.updated = False
                break
            curr_pose = curr.get_coords()
            vec = self.field[curr_pose[0], curr_pose[1]]
            new_pose = int(curr_pose[0] + vec[0]), int(curr_pose[1] + vec[1])

            if new_pose == curr_pose:
                stuck = True

            if self.virtual:
                # Relative angle to goal
                if stuck:
                    theta = math.atan2(
                        new_pose[1] - curr_pose[1], new_pose[0] - curr_pose[0]
                    )
                    self.field[curr_pose[0], curr_pose[1]] += [
                        self.fcf * math.cos(theta) * vec[0],
                        self.fcf * math.sin(theta) * vec[1],
                    ]
                    self.clamp_field(25)
                else:
                    time.sleep(0.02)
            else:
                time.sleep(0.02)

            new_node = Node(*new_pose, len(self.path))
            new_node.parent = curr
            self.path.append(new_node)

            if (new_pose[0] - self.goal_pose[0]) ** 2 + (
                new_pose[1] - self.goal_pose[1]
            ) ** 2 < self.goal_radius**2:
                self.gn = new_node
            curr = new_node

    def set_obstacles(self, obstacles):
        self.obstacles = obstacles


class Color:
    WHITE = (255, 255, 255)
    LIGHTGREY = (130, 130, 130)
    GREY = (70, 70, 70)
    BLUE = (0, 0, 255)
    GREEN = (0, 255, 0)
    RED = (255, 0, 0)
    GREY2 = (50, 50, 50)
    PURPLE = (199, 21, 133)
    BROWN = (210, 105, 30)
    LIGHT_BLUE = (176, 196, 250)
    LIGHT_PURPLE = (102, 102, 255)


def dist_to_node(n1, n2):
    return dist(n1.get_coords(), n2.get_coords())


def dist_to_point(n, p):
    return dist(n.get_coords(), p)


def dist(p1, p2):
    x, y = p1[0], p1[1]
    xx, yy = p2[0], p2[1]
    return math.hypot(x - xx, y - yy)


def add_edge(n1, n2):
    n1.add_neighbour(n2)
    n2.add_neighbour(n1)


def remove_edge(n1, n2):
    del n1.adj[n2]
    del n1.edge[n2]
    del n2.adj[n1]
    del n2.edge[n1]


def draw_arrow(surface, startCoord, endCoord, LINE_WIDTH=3):
    """
    Draw an arrow via pygame.
    """
    A = startCoord
    B = endCoord
    dir_ = (B[0] - A[0], B[1] - A[1])
    dir_mag = math.sqrt(dir_[0] ** 2 + dir_[1] ** 2)
    H = dir_mag / 4.0
    W = H * 2.0
    if dir_mag == 0:
        dir_mag = 0.00001
    dir_ = (dir_[0] / dir_mag, dir_[1] / dir_mag)

    q = (dir_[1], -dir_[0])

    C = (
        B[0] - (H * dir_[0]) + (W * q[0] / 2.0),
        B[1] - (H * dir_[1]) + (W * q[1] / 2.0),
    )

    D = (
        B[0] - (H * dir_[0]) - (W * q[0] / 2.0),
        B[1] - (H * dir_[1]) - (W * q[1] / 2.0),
    )

    pygame.draw.line(surface, Color.GREY, A, B, LINE_WIDTH)
    pygame.draw.line(surface, Color.GREY, B, C, LINE_WIDTH)
    pygame.draw.line(surface, Color.GREY, B, D, LINE_WIDTH)


@np.vectorize
def cvtRange(x, in_min, in_max, out_min, out_max):
    """
    Convert a value, x, from its old range of
    (in_min to in_max) to the new range of
    (out_min to out_max)
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class Node:
    def __init__(self, x, y, id):
        self.x, self.y, self.id = x, y, id
        self.parent, self.search = None, None
        self.adj, self.edge = {}, {}

    def get_coords(self):
        return self.x, self.y

    def add_neighbour(self, neighbour):
        self.adj[neighbour] = self.__euclidean_dist(neighbour)
        self.edge[neighbour] = NodeEdge(self, neighbour)

    def __euclidean_dist(self, neighbour):
        return math.hypot((self.x - neighbour.x), (self.y - neighbour.y))

    def get_connections(self):
        return self.adj.keys()

    def get_weight(self, neighbour):
        return self.adj[neighbour]

    def draw(self, surf, node_radius, width):
        for neighbour in self.edge:
            color = Color.GREY
            pygame.draw.line(
                surf,
                color,
                self.edge[neighbour].nfrom.get_coords(),
                self.edge[neighbour].nto.get_coords(),
                width=width,
            )
        pygame.draw.circle(
            surf, Color.LIGHTGREY, self.get_coords(), node_radius, width=0
        )

    def __str__(self):
        return f"{self.x}, {self.y}, {self.id}"


# Used to visualize pathfinding
class NodeEdge:
    def __init__(self, node_from: Node, node_to: Node):
        self.nfrom, self.nto = node_from, node_to


class CircularObstacle:
    def __init__(self, x, y, rad):
        self.x, self.y, self.rad = x, y, rad

    def collidepoint(self, point):
        d = math.hypot(point[0] - self.x, point[1] - self.y)
        if d <= self.rad:
            return True
        return False
