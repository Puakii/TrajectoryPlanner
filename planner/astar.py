from planner.planner import Planner
import heapq
import copy

class AStar(Planner):

    def plan(self, map, moves, robot, start, goal):
        robot_dimension = min(robot.width, robot.height)

        final_state = None
        opened = []
        heapq.heappush(opened, (0.0, start))
        closed = []

        while opened and final_state is None:
            q = heapq.heappop(opened)[1]
            for move in moves:
                successor = q.try_apply(map, move, robot)
                if successor is not None:
                    if successor.dist_to(goal) < robot_dimension:
                        final_state = successor
                        break
                    successor.g = q.g + successor.dist_to(q)
                    successor.h = successor.dist_to(goal)
                    successor.f = 0.2 * successor.g + successor.h
                    successor.parent = q

                    # TODO ->replace<- with new positions
                    better_in_opened = any(other_successor.is_same_as(successor) and other_f <= successor.f for other_f, other_successor in opened)
                    if not better_in_opened:
                        better_in_closed = any(other_successor.is_same_as(successor) and other_successor.f <= successor.f for other_successor in closed)
                        if not better_in_closed:
                            heapq.heappush(opened, (successor.f, successor))

            closed.append(q)

        if final_state is None:
            return []
        current_state = copy.copy(final_state)
        traj = []
        while True:
            traj.append((current_state.x, current_state.y, current_state.theta))

            current_state = current_state.parent

            if current_state is None:
                break
        traj.reverse()     
        return traj