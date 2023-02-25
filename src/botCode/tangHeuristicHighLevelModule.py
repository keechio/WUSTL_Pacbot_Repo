#!/usr/bin/env python3

import os, copy
import robomodules as rm
import math
from operator import itemgetter
from variables import *
from grid import grid
from search import bfs
from messages import MsgType, message_buffers, LightState, PacmanCommand


ADDRESS = os.environ.get("LOCAL_ADDRESS", "localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 30
GHOST_CUTOFF = 10


class HeuristicHighLevelModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.LIGHT_STATE]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.state = None
        self.previous_loc = None
        self.direction = PacmanCommand.EAST
        self.grid = copy.deepcopy(grid)

    def getScorePosition(self, new_pos, grid, score):
        if grid[new_pos[0]][new_pos[1]] == o:
            grid[new_pos[0]][new_pos[1]] = e
            score += 10
        if grid[new_pos[0]][new_pos[1]] == O:
            grid[new_pos[0]][new_pos[1]] = e
            score += 50
    def moveGhosts(self, grid, p_loc, red_ghost, blue_ghost, orange_ghost, pink_ghost):
        red_ghost = self._get_next_red_chase_move(p_loc, red_ghost, grid)
        blue_ghost = self._get_next_blue_chase_move(p_loc, blue_ghost, grid)
        orange_ghost = self._get_next_orange_chase_move(p_loc, orange_ghost, grid)
        pink_ghost = self._get_next_pink_chase_move(p_loc, pink_ghost, grid)



        # p_loc[0] = x pos , p_loc[1] = y loc
        # self.state.red_ghost. x,y,state(frightened or not)
    def pac_find_possible_moves(self, cur_loc, grid):
        (x, y) = self.pos['next']
        possible = []
        if self.pac_is_move_legal((x + 1, y), grid):
            possible.append((x + 1, y))
        if self.pac_is_move_legal((x, y + 1), grid):
            possible.append((x, y + 1))
        if self.pac_is_move_legal((x - 1, y), grid):
            possible.append((x - 1, y))
        if self.pac_is_move_legal((x, y - 1), grid):
            possible.append((x, y - 1))
        possible.append(cur_loc)
        return possible

    def pac_is_move_legal(self, move, grid):
        return (grid[move[0]][move[1]] != I and
                grid[move[0]][move[1]] != n)
    def maxVal(self, state, alpha, beta, depth):
        v = float('-inf')
        for successor in state:
            v = max(v, self.minVal(successor))
            alpha = max(alpha, v)
            if alpha >= beta:
                return v
        return v

    def minVal(self, p_loc, ghosts, grid, alpha, beta, depth):
        v = float('inf')
        for ind,ghost_loc in enumerate(ghosts):
            moves = self._find_possible_moves(ghost_loc, grid)
            for move in moves:
                new_ghosts = copy.copy(ghosts)
                new_ghosts[ind] = move
                v = min(v, self.maxVal(p_loc, ghosts, grid, alpha, beta, depth))
                beta = min(beta, v)
                if alpha >= beta:
                    return v
        return v

    def minMaxAction(self, p_loc):
        state = [p_loc, (p_loc[0] - 1, p_loc[1]), (p_loc[0] + 1, p_loc[1]), (p_loc[0], p_loc[1] - 1),
                   (p_loc[0], p_loc[1] + 1)]
        red_ghost = (self.state.red_ghost.x, self.state.red_ghost.y)
        blue_ghost = (self.state.blue_ghost.x, self.state.blue_ghost.y)
        orange_ghost = (self.state.orange_ghost.x, self.state.orange_ghost.y)
        pink_ghost = (self.state.pink_ghost.x, self.state.pink_ghost.y)
        ghosts_loc = [red_ghost, blue_ghost, orange_ghost, pink_ghost]
        # state gives a list of actions
        # if (game ended):
        # return
        returning = float('-inf')
        best_action = None
        for p_loc in state:
            maxv = self.minVal(p_loc, ghosts_loc, copy.deepcopy(self.grid))
            if maxv > returning:
                best_action = p_loc
                returning = maxv
        return best_action

    def _get_direction(self, p_loc, next_loc):
        if p_loc[0] == next_loc[0]:
            if p_loc[1] < next_loc[1]:
                return PacmanCommand.NORTH
            else:
                return PacmanCommand.SOUTH
        else:
            if p_loc[0] < next_loc[0]:
                return PacmanCommand.EAST
            else:
                return PacmanCommand.WEST

    def _find_paths_to_closest_ghosts(self, pac_loc):
        ghosts = [self.state.red_ghost, self.state.pink_ghost, self.state.orange_ghost, self.state.blue_ghost]
        state_paths = [(ghost.state, bfs(self.grid, pac_loc, (ghost.x, ghost.y), GHOST_CUTOFF)) for ghost in ghosts]
        return [sp for sp in state_paths if sp[1] is not None]

    def _find_distance_of_closest_pellet(self, target_loc):
        return len(bfs(self.grid, target_loc, [o])) - 1

    def _target_is_invalid(self, target_loc):
        return self.grid[target_loc[0]][target_loc[1]] in [I, n]

    def _is_power_pellet_closer(self, path):
        return O in path

    def _get_num_turns(self, p_dir, n_dir):
        lat = [PacmanCommand.WEST, PacmanCommand.EAST]
        lng = [PacmanCommand.SOUTH, PacmanCommand.NORTH]

        if p_dir == n_dir:
            return 0
        elif (p_dir in lat and n_dir in lat) or (p_dir in lng and n_dir in lng):
            return 2
        else:
            return 1

    def _get_target_with_min_turning_direction(self, mins):
        turns = [(self._get_num_turns(self.direction, direct), targ) for direct, targ in mins]
        return min(turns, key=itemgetter(0))[1]

    def _update_game_state(self):
        p_loc = (self.state.pacman.x, self.state.pacman.y)
        if self.grid[p_loc[0]][p_loc[1]] in [o, O]:
            self.grid[p_loc[0]][p_loc[1]] = e

    def _send_command_message_to_target(self, p_loc, target):
        new_msg = PacmanCommand()
        new_msg.dir = self._get_direction(p_loc, target)
        if new_msg.dir == PacmanCommand.NORTH:
            print("N")
        elif new_msg.dir == PacmanCommand.SOUTH:
            print("S")
        elif new_msg.dir == PacmanCommand.EAST:
            print("E")
        else:
            print("W")
        self.write(new_msg.SerializeToString(), MsgType.PACMAN_COMMAND)

    def _send_stop_command(self):
        new_msg = PacmanCommand()
        new_msg.dir = PacmanCommand.STOP
        self.write(new_msg.SerializeToString(), MsgType.PACMAN_COMMAND)

    def msg_received(self, msg, msg_type):
        if msg_type == MsgType.LIGHT_STATE:
            if self.previous_loc != msg.pacman:
                if self.previous_loc is not None:
                    self.direction = self._get_direction((self.previous_loc.x, self.previous_loc.y),
                                                         (msg.pacman.x, msg.pacman.y))
                self.previous_loc = self.state.pacman if self.state else None
            self.state = msg

    def tick(self):
        if self.state and self.state.mode == LightState.RUNNING:
            self._update_game_state()
            p_loc = (self.state.pacman.x, self.state.pacman.y)
            next_loc = self.minMaxAction(p_loc)
            if next_loc != p_loc:
                self._send_command_message_to_target(p_loc, next_loc)
                return
        self._send_stop_command()

# This is awful. Look online to find out blue is supposed to move, and let's just work under the
    # assumption that this function returns that sort of move.
    def _get_next_blue_chase_move(self, p_loc, red_pos, grid):
        pacbot_target = (0, 0)
        if self.game_state.pacbot.direction == up:
            pacbot_target = (
                p_loc[0] - 2, p_loc[1] + 2)
        elif self.game_state.pacbot.direction == down:
            pacbot_target = (
                p_loc[0], p_loc[1] - 2)
        elif self.game_state.pacbot.direction == left:
            pacbot_target = (
                p_loc[0] - 2, p_loc[1])
        elif self.game_state.pacbot.direction == right:
            pacbot_target = (
                p_loc[0] + 2, p_loc[1])
        x = pacbot_target[0] + (pacbot_target[0] -
                                red_pos.x)
        y = pacbot_target[1] + (pacbot_target[1] -
                                red_pos.y)

        return self._get_move_based_on_target((x, y), red_pos, grid)

    # Return the move closest to the space 4 tiles ahead of Pacman in the direction
    # Pacman is currently facing. If Pacman is facing up, then we replicate a bug in
    # the original game and return the move closest to the space 4 tiles above and
    # 4 tiles to the left of Pacman.
    def _get_next_pink_chase_move(self, p_loc, pink_loc, grid):
        (x, y) = (0, 0)

        if self.game_state.pacbot.direction == up:
            x = p_loc[0] - 4
            y = p_loc[1] + 4
        elif self.game_state.pacbot.direction == down:
            x = p_loc[0]
            y = p_loc[1] - 4
        elif self.game_state.pacbot.direction == left:
            x = p_loc[0] - 4
            y = p_loc[1]
        elif self.game_state.pacbot.direction == right:
            x = p_loc[0] + 4
            y = p_loc[1]

        return self._get_move_based_on_target((x, y), pink_loc, grid)

    # Returns the move that will bring the ghost closest to Pacman
    def _get_next_red_chase_move(self, p_loc, red_loc, grid):
        return self._get_move_based_on_target(p_loc, red_loc, grid)

    # If the ghost is close to Pacman, return the move that will bring the ghost closest
    # to its scatter position (bottom left corner). If the ghost is far from Pacman,
    # return the move that will bring the ghost closest to Pacman.
    def _get_next_orange_chase_move(self, p_loc, orange_loc, grid):
        if self._get_euclidian_distance(orange_loc, p_loc) < 8:
            return self._get_move_based_on_target((0, -1), orange_loc, grid)
        return self._get_move_based_on_target(p_loc)

    # Moves to the tile that is the closest to the target by straight-line distance,
    # NOT the tile that is the closest to the target by optimal tile path length.
    def _get_move_based_on_target(self, target, cur_loc, grid):
        possible = self._find_possible_moves(cur_loc, grid)
        distances = []
        for tile in possible:
            distances.append(self._get_euclidian_distance(target, tile))
        (min_distance, index) = min((min_distance, index)
                                    for (index, min_distance) in enumerate(distances))

        return (possible[index], self._get_direction(self.pos["next"], possible[index]))

# Returns the straight-line distance between two points.
    def _get_euclidian_distance(self, pos_a, pos_b):
        return math.hypot(int(pos_a[0])-int(pos_b[0]), int(pos_a[1])-int(pos_b[1]))


# Returns a list of valid tiles for the ghost to move to. If no such tiles exist,
    # return a list containing only the ghost's current position.
    def _find_possible_moves(self, cur_loc, grid):
        (x, y) = self.pos['next']
        possible = []
        if self._is_move_legal((x+1, y), grid, cur_loc):
            possible.append((x+1, y))
        if self._is_move_legal((x, y+1), grid, cur_loc):
            possible.append((x, y+1))
        if self._is_move_legal((x-1, y), grid, cur_loc):
            possible.append((x-1, y))
        if self._is_move_legal((x, y-1), grid, cur_loc):
            possible.append((x, y-1))
        if possible == []:
            possible.append(self.pos["current"])
        return possible

    def _is_move_legal(self, move, grid, cur_loc):
        return (move != (cur_loc.x, cur_loc.y) and
                grid[move[0]][move[1]] != I and
                grid[move[0]][move[1]] != n)




def main():
    module = HeuristicHighLevelModule(ADDRESS, PORT)
    module.run()


if __name__ == "__main__":
    main()
