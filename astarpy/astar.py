from collections import deque
import numpy as np
import cv2

# pt = [x, y, g, f]
class AStarQueue(object):
    def __init__(self, h):
        """ list of (x,g,f) """
        self._l = []
        self._h = h # heuristic function
    def add(self, x, g):
        lo,hi = 0,len(self._l)
        f = g + self._h(x) # f-value
        while lo < hi:
            mid = (lo+hi)//2
            # sort in reverse order by flipping f
            if -self._l[mid][2] < -f:
                lo = mid + 1
            else:
                hi = mid
        self._l.insert(lo, (x, g, f))
    def pop(self):
        return self._l.pop()
    def n(self):
        return len(self._l)

class EDist(object):
    def __init__(self, x1):
        self._x1 = x1
    def __call__(self, x0):
        res=0
        for e0, e1 in zip(x0,self._x1):
            res += (e0-e1)*(e0-e1)
        return res ** 0.5

class LDist(object):
    def __init__(self, x1):
        self._x1 = x1
    def __call__(self, x0):
        res=0
        for e0,e1 in zip(x0,self._x1):
            res += abs(e0-e1)
        return res

class GDist(EDist):
    def __init__(self, x1, o, scale=1):
        ker = cv2.getStructuringElement(cv2.MORPH_DILATE, (3,3))
        self._o = cv2.dilate(np.float32(o), ker, iterations=2)
        self._s = scale
        super(GDist, self).__init__(x1)
    def __call__(self, x0):
        return super(GDist, self).__call__(x0) + self._s*self._o[x0]
        #return EDist.__call__(x0) + self._o[x0]

class AStar2DGrid(object):
    def __init__(self, grid, init, goal, h=None):
        self._grid = grid
        self._init = init
        self._goal = goal
        if h is None:
            h = EDist(goal)
        self._q = AStarQueue(h)
        self._q.add(init, 0)

        # only valid for 2dgrids ...

        # di,dj
        self._delta = [[-1,0], [0,-1], [1,0], [0,1]]
        self._dviz = ['^', '<', 'v', '>']

        self._grid = grid # occupancy grid
        self._n, self._m = np.shape(grid)
        self._check = np.zeros(np.shape(grid), dtype=np.int32)
        self._viz = np.full(np.shape(grid), ' ', dtype=str)
        self._path = []

    def valid(self, i, j):
        return (i >= 0 and j >= 0 and
                i < self._n and j < self._m and
                self._grid[i][j] == 0 and
                self._check[i][j] == 0)

    def simplify(self, path):
        pass

    def __call__(self):
        while True:
            if self._q.n() == 0:
                return None, None
            (i,j),g,f = self._q.pop() # consider point
            if (i,j) == self._goal:
                break
            for a, (di,dj) in enumerate(self._delta):
                i1,j1 = i+di,j+dj
                if self.valid(i1, j1):
                    self._q.add((i1,j1),g+1)
                    self._check[i1][j1] = a + 1
        # visualize ... 
        self._viz[i][j] = '*'
        self._path = []
        while True:
            self._path.append((i,j))
            if (i,j) == self._init:
                break
            a = self._check[i][j] - 1
            i = i - self._delta[a][0]
            j = j - self._delta[a][1]
            self._viz[i][j] = self._dviz[a]
        self._path.reverse()
        return self._viz, self._path

def main():
    grid = [[0, 1, 0, 0, 0, 0],
            [0, 1, 0, 1, 0, 0],
            [0, 0, 0, 1, 0, 1],
            [1, 0, 1, 1, 0, 0],
            [0, 0, 0, 0, 1, 0]]
    #h = LDist((4,5))
    h = GDist((4,5), o=grid)
    astar = AStar2DGrid(grid, (0,0), (4,5), h)

    print astar()

    #h = EDist((5.0, 5.0))
    #q = AStarQueue(h)
    #q.add((0,0), 0)
    #q.add((0,0), 1)
    #q.add((0,0), 2)
    #q.add((0,0), 3)
    #print q._l

if __name__ == "__main__":
    main()
