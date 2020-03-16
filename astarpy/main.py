#!/usr/bin/python2

from astar import AStar2DGrid as AStar, EDist, GDist
import numpy as np
import cv2

class CVMapper(object):
    def __init__(self, name, img, r):
        self._name = name
        self._img = img
        self._r = r
        self._drawing = False

        # default init-goal
        self._init = (0,0)
        n, m = np.shape(img)[:2]
        self._goal = (n-1, m-1)

        self._redraw=True
        self.redraw()

        print 'L:Init | M:Draw | R:Goal'

    def redraw(self):
        if self._redraw:
            shape = np.shape(self._img)
            if len(shape)<3 or shape[2]<=1:
                self._viz = cv2.cvtColor(self._img, cv2.COLOR_GRAY2BGR)
            else:
                self._viz = np.copy(self._img)
            cv2.circle(self._viz, self._goal[::-1], self._r, 255, 1)
            cv2.circle(self._viz, self._init[::-1], self._r, 255, 1)
            self._redraw = False

    def __call__(self):
        cv2.namedWindow(self._name)
        cv2.setMouseCallback(self._name, self.mouse_cb)
        while(True):
            self.redraw()
            cv2.imshow(self._name, self._viz)
            k = cv2.waitKey(20)
            if k == 27:
                break
        cv2.destroyAllWindows()

    def mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_MBUTTONDOWN:
            self._drawing = True
            self._ix, self._iy = x,y
        elif event == cv2.EVENT_MBUTTONUP:
            self._drawing = False
        elif event == cv2.EVENT_MOUSEMOVE:
            if self._drawing:
                cv2.circle(self._img, (x,y), self._r, 255, -1)
                self._redraw = True
        elif event == cv2.EVENT_LBUTTONDOWN:
            self._redraw = True
            self._init = (y, x)
        elif event == cv2.EVENT_RBUTTONDOWN:
            self._redraw = True
            self._goal = (y, x)

def main():
    # create map
    n,m = 256,256
    grid = np.zeros((n,m), dtype=np.uint8)
    mapper = CVMapper('map', grid, 2)
    mapper()
    grid /= 255 # back to 1
    init, goal = mapper._init, mapper._goal

    print 'Initial Point : {}'.format(init)
    print 'Final Point : {}'.format(goal)

    # h = EDist(goal)
    h = GDist(goal, o=grid, scale=5.)
    astar = AStar(grid, init, goal, h=h)
    _, path = astar()

    if path:
        viz = grid.astype(np.float32)
        viz = cv2.cvtColor(viz, cv2.COLOR_GRAY2BGR)

        for (p0, p1) in zip(path[:-1], path[1:]):
            y0,x0 = p0
            y1,x1 = p1
            #cv2.line( (y0,x0), (y1,x1), (128)
            cv2.line(viz, (x0,y0), (x1,y1), (0,0,1), 1)

        cv2.imshow('viz', viz)

        while True: 
            if cv2.waitKey(0) == 27:
                break
    else:
        print 'Path Not Found'

if __name__ == "__main__":
    main()
