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
from search import bfs


ADDRESS = os.environ.get("LOCAL_ADDRESS", "localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)
PELLET_WEIGHT = 0.65
FREQUENCY = 30
GHOST_CUTOFF = 10
MAX_DEPTH = 1


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

    """
    min step in minimax
    makes each ghost move according to algorithm then calls max step
    returns score if it hits a ghost
    

    def minVal(self, gameState, depth):
        v = float('inf')
        if gameState.next_step():
            return gameState.score
        if gameState._should_die():
            return gameState.score
        v = min(v, self.maxVal(gameState, depth + 1))
        return v
    """

    """
    function calls min step to start the minmax tree
    

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

    """
    max step in minimax
    makes pacman move in each possible direction then calls min step
    returns score if it hits a ghost
    """

    def maxValAB(self, gameState, depth, alpha, beta):
        if depth > MAX_DEPTH:
            return gameState.score + self.get_eval_val(gameState)
        v = float('-inf')
        prev_loc = gameState.pacbot.pos
        state = [(prev_loc[0] - 1, prev_loc[1]), (prev_loc[0] + 1, prev_loc[1]),
                 (prev_loc[0], prev_loc[1] - 1),
                 (prev_loc[0], prev_loc[1] + 1)]
        moves = self.pac_get_legal_moves(state, gameState.grid)
        for move in moves:
            #old_pac_pos = self.pacbot.pos
            new_gameState = copy.deepcopy(gameState)
            new_gameState.pacbot.pos = move
            #prev_pos = [copy.deepcopy(self.gameState.red.pos), copy.deepcopy(self.gameState.pink.pos),
                        #copy.deepcopy(self.gameState.orange.pos), copy.deepcopy(self.gameState.blue.pos)]
            #prev_respawn_counters = [self.gameState.red.respawn_counter, self.gameState.pink.respawn_counter,
                                     #self.gameState.orange.respawn_counter, self.gameState.blue.respawn_counter]
            #prev_frightened_counters = [self.gameState.red.frightened_counter, self.gameState.pink.respawn_counter,
                                        #self.gameState.orange.respawn_counter, self.gameState.blue.respawn_counter]
            v = max(v, self.minValAB(new_gameState, depth, alpha, beta))
            #self.gameState.red.undo_move(prev_pos[0], prev_respawn_counters[0], prev_frightened_counters[0])
            #self.gameState.pink.undo_move(prev_pos[1], prev_respawn_counters[1], prev_frightened_counters[1])
            #self.gameState.orange.undo_move(prev_pos[2], prev_respawn_counters[2], prev_frightened_counters[2])
            #self.gameState.blue.undo_move(prev_pos[3], prev_respawn_counters[3], prev_frightened_counters[3])
            if v >= beta:
                return v
            alpha = max(alpha, v)
            #new_gameState.pacbot.pos = old_pac_pos
        return v


    """
    min step in minimax
    makes each ghost move according to algorithm then calls max step
    returns score if it hits a ghost
    """
    def minValAB(self, gameState, depth, alpha, beta):
        v = float('inf')
        if gameState.next_step():
            return gameState.score + self.get_eval_val(gameState)
        if gameState._should_die():
            return gameState.score + self.get_eval_val(gameState) - 1000
        #prev_game_state_arr = self.gameState.return_instance_variables()
        prev_grid = copy.deepcopy(self.gameState.grid)
        v = min(v, self.maxValAB(gameState, depth + 1, alpha, beta))
        #self.gameState.undo_step(prev_game_state_arr, prev_grid)
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
        #print(prev_loc)
        #print(self.gameState.red.pos['current'])
        #print(self.gameState.blue.pos['current'])
        #print(self.gameState.orange.pos['current'])
        #print(self.gameState.pink.pos['current'])
        for move in moves:
            """
            print("move")
            print(self.pacbot.pos)
            print(move)

            direction = self._get_direction(self.gameState.pacbot.pos, move)
            if (direction == PacmanCommand.NORTH):
                print("N")
            elif (direction == PacmanCommand.SOUTH):
                print("S")
            elif (direction == PacmanCommand.WEST):
                print("W")
            else:
                print("E")
            """

            #old_pac_pos = self.pacbot.pos
            new_gamestate = copy.deepcopy(self.gameState)
            print("curr blue")
            print(self.gameState.blue.pos['current'])
            print("new blue")
            print(new_gamestate.blue.pos['current'])
            new_gamestate.pacbot.pos = move
            maxv = float('-inf')
            if new_gamestate._should_die():
                print("dying")
                maxv = new_gamestate.score + self.get_eval_val(new_gamestate) - 1000
            else:
                #prev_game_state_arr = self.gameState.return_instance_variables()
                #prev_grid = copy.deepcopy(self.gameState.grid)
                maxv = self.minValAB(new_gamestate, 0, float('-inf'), float('inf'))
                #self.gameState.undo_step(prev_game_state_arr, prev_grid)
            if maxv > returning:
                best_action = move
                returning = maxv
            #self.gameState.pacbot.pos = old_pac_pos
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

        self.gameState.red.pos['current'] = (self.state.red_ghost.x, self.state.red_ghost.y)
        self.gameState.pink.pos['current'] = (self.state.pink_ghost.x, self.state.pink_ghost.y)
        self.gameState.orange.pos['current'] = (self.state.orange_ghost.x, self.state.orange_ghost.y)
        self.gameState.blue.pos['current'] = (self.state.blue_ghost.x, self.state.blue_ghost.y)

        #figure out how to update other ghost values and gamestate values

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
            p_loc = (self.state.pacman.x, self.state.pacman.y)
            next_loc = self.minMaxActionAB()
            if next_loc != p_loc:
                self._send_command_message_to_target(next_loc)
                return
        self._send_stop_command()

    # Returns the straight-line distance between two points.
    def _get_euclidian_distance(self, pos_a, pos_b):
        return math.hypot(int(pos_a[0]) - int(pos_b[0]), int(pos_a[1]) - int(pos_b[1]))

    #returns an evaluation of the position
    def get_eval_val(self, gameState):
        eval_val = 0
        min_dist, avg_dist = self.closest_ghost_and_dist(gameState)
        eval_val += min_dist * 1.5
        if min_dist < 5:
            eval_val -= 100
        #if(gameState.state != frightened):
            #eval_val -= avg_dist
        #eval_val += gameState.power_pellets*4
        #eval_val -= gameState.pellets * 2


        return eval_val
    def closest_ghost_and_dist(self, gamestate):
        ghosts = [gamestate.red, gamestate.pink, gamestate.orange, gamestate.blue]
        minDist = float('inf')
        avgDist = 0
        closestGhost = None
        for ghost in ghosts:
            ghostDist = self.manhattan_distance(gamestate.pacbot.pos, [ghost.pos['current'][0], ghost.pos['current'][1]])
            if ghost.color == 4:
                print("pacman pos")
                print(gamestate.pacbot.pos)
                print("closest ghost blue")
                print(self.gameState.blue.pos['current'])
                print("ghost dist")
                print(ghostDist)
            avgDist += ghostDist
            if ghostDist <= minDist:
                closestGhost = ghost
                minDist = ghostDist
        print("mindist")
        print(minDist)
        return minDist, avgDist//4

    def manhattan_distance(self, curr_loc, next_loc):
        #print(curr_loc)
        #print(next_loc)
        distance = abs(next_loc[0] - curr_loc[0]) + abs(next_loc[1] - curr_loc[1])
        #print(distance)
        return distance

    def closest_ghost(self, paths_to_ghosts, target_loc, p_loc):
        closest_ghost = (None, float('inf'))
        ghosts = []
        for state, path in paths_to_ghosts:
            dist = len(path) - 1
            closest_ghost = (state, dist) if dist < closest_ghost[1] else closest_ghost
            ghosts.append((state, dist))
            if self._is_power_pellet_closer(path):
                if target_loc == p_loc:
                    return path[1]
                else:
                    return path[0]

def main():
    module = HeuristicHighLevelModule(ADDRESS, PORT)
    module.run()


if __name__ == "__main__":
    main()
