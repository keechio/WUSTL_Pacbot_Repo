#!/usr/bin/env python3

import os, copy
import robomodules as rm
import math
from operator import itemgetter
from highLevelVariables import *
from grid import grid
from messages import MsgType, message_buffers, LightState, PacmanCommand
from highLevelGhostpaths import *
from highLevelPacbot import *
from highLevelGhostAgent import *
from highLevelGameState import *


ADDRESS = os.environ.get("LOCAL_ADDRESS", "localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 30
GHOST_CUTOFF = 10
MAX_DEPTH = 5


class HeuristicHighLevelModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.LIGHT_STATE]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.state = None
        self.previous_loc = None
        self.direction = PacmanCommand.EAST
        self.pacbot = HighLevelPacBot()
        self.gameState = HighLevelGameState(self.pacbot, copy.deepcopy(grid))

    def _get_direction(self, prev_loc,next_loc):
        if prev_loc[0] == next_loc[0]:
            if prev_loc[1] < next_loc[1]:
                return PacmanCommand.NORTH
            else:
                return PacmanCommand.SOUTH
        else:
            if prev_loc[0] < next_loc[0]:
                return PacmanCommand.EAST
            else:
                return PacmanCommand.WEST
    def pac_get_legal_moves(self, moves, grid):
        return_moves = []
        for move in moves:
            if grid[move[0]][move[1]] != I and grid[move[0]][move[1]] != n:
                return_moves.append(move)
        return return_moves

    """
    max step in minimax
    makes pacman move in each possible direction then calls min step
    returns score if it hits a ghost
    """

    def maxVal(self, gameState, depth):
        if depth > MAX_DEPTH:
            return gameState.score
        v = float('-inf')
        prev_loc = gameState.pacbot.pos
        state = [(prev_loc[0] - 1, prev_loc[1]), (prev_loc[0] + 1, prev_loc[1]),
                 (prev_loc[0], prev_loc[1] - 1),
                 (prev_loc[0], prev_loc[1] + 1)]
        moves = self.pac_get_legal_moves(state, gameState.grid)
        for move in moves:
            new_gameState = copy.deepcopy(gameState)
            new_gameState.pacbot.pos = move
            v = max(v, self.minVal(new_gameState, depth))
        return v

    """
    min step in minimax
    makes each ghost move according to algorithm then calls max step
    returns score if it hits a ghost
    """

    def minVal(self, gameState, depth):
        v = float('inf')
        if gameState.next_step():
            return gameState.score
        if gameState._should_die():
            return gameState.score
        v = min(v, self.maxVal(gameState, depth + 1))
        return v

    """
    function calls min step to start the minmax tree
    """

    def minMaxAction(self):
        # state gives a list of actions
        # if (game ended):
        # return
        returning = float('-inf')
        best_action = None
        prev_loc = self.gameState.pacbot.pos
        state = [(prev_loc[0] - 1, prev_loc[1]), (prev_loc[0] + 1, prev_loc[1]),
                 (prev_loc[0], prev_loc[1] - 1),
                 (prev_loc[0], prev_loc[1] + 1)]
        moves = self.pac_get_legal_moves(state, self.gameState.grid)
        for move in moves:
            new_gamestate = copy.deepcopy(self.gameState)
            new_gamestate.pacbot.pos = move
            maxv = float('-inf')
            if new_gamestate._should_die():
                maxv = new_gamestate.score
            else:
                maxv = self.minVal(new_gamestate, 0)
            if maxv > returning:
                best_action = move
                returning = maxv
        return best_action

    """
    max step in minimax
    makes pacman move in each possible direction then calls min step
    returns score if it hits a ghost
    """

    def maxValAB(self, gameState, depth, alpha, beta):
        if depth > MAX_DEPTH:
            return gameState.score
        v = float('-inf')
        prev_loc = gameState.pacbot.pos
        state = [(prev_loc[0] - 1, prev_loc[1]), (prev_loc[0] + 1, prev_loc[1]),
                 (prev_loc[0], prev_loc[1] - 1),
                 (prev_loc[0], prev_loc[1] + 1)]
        moves = self.pac_get_legal_moves(state, gameState.grid)
        for move in moves:
            new_gameState = copy.deepcopy(gameState)
            new_gameState.pacbot.pos = move
            v = max(v, self.minValAB(new_gameState, depth, alpha, beta))
            if v >= beta:
                return v
            alpha = max(alpha, v)
        return v

    """
    min step in minimax
    makes each ghost move according to algorithm then calls max step
    returns score if it hits a ghost
    """

    def minValAB(self, gameState, depth, alpha, beta):
        v = float('inf')
        if gameState.next_step():
            return gameState.score
        if gameState._should_die():
            return gameState.score
        v = min(v, self.maxValAB(gameState, depth + 1, alpha, beta))
        if v <= alpha:
            return v
        beta = min(beta, v)
        return v

    """
    function calls min step to start the minmax tree
    """

    def minMaxActionAB(self):
        # state gives a list of actions
        # if (game ended):
        # return
        returning = float('-inf')
        best_action = None
        prev_loc = self.gameState.pacbot.pos
        state = [(prev_loc[0] - 1, prev_loc[1]), (prev_loc[0] + 1, prev_loc[1]),
                 (prev_loc[0], prev_loc[1] - 1),
                 (prev_loc[0], prev_loc[1] + 1)]
        moves = self.pac_get_legal_moves(state, self.gameState.grid)
        for move in moves:
            new_gamestate = copy.deepcopy(self.gameState)
            new_gamestate.pacbot.pos = move
            maxv = float('-inf')
            if new_gamestate._should_die():
                maxv = new_gamestate.score
            else:
                maxv = self.minValAB(new_gamestate, 0, float('-inf'), float('inf'))
            if maxv > returning:
                best_action = move
                returning = maxv
        return best_action

    def _target_is_invalid(self, target_loc):
        return self.gameState.grid[target_loc[0]][target_loc[1]] in [I, n]

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
        self.pacbot.pos = (self.state.pacman.x, self.state.pacman.y)
        if self.gameState.grid[self.pacbot.pos[0]][self.pacbot.pos[1]] in [o, O]:
            self.gameState.grid[self.pacbot.pos[0]][self.pacbot.pos[1]] = e

    def _send_command_message_to_target(self, target):
        new_msg = PacmanCommand()
        new_msg.dir = self._get_direction(self.pacbot.pos, target)
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
            self.pacbot.pos = (self.state.pacman.x, self.state.pacman.y)
            next_loc = self.minMaxActionAB()
            if next_loc != self.pacbot.pos:
                self._send_command_message_to_target(next_loc)
                return
        self._send_stop_command()

    # Returns the straight-line distance between two points.
    def _get_euclidian_distance(self, pos_a, pos_b):
        return math.hypot(int(pos_a[0]) - int(pos_b[0]), int(pos_a[1]) - int(pos_b[1]))


def main():
    module = HeuristicHighLevelModule(ADDRESS, PORT)
    module.run()


if __name__ == "__main__":
    main()
