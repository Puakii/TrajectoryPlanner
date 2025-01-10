from abc import abstractmethod

class Planner:
    @abstractmethod
    def plan(self, map, moves, robot, start, goal):
        pass