from astar import AStar2DGrid as AStar, EDist
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

        print 'L:Draw | R:Init | M:Goal'

    def __call__(self):
        cv2.namedWindow(self._name)
        cv2.setMouseCallback(self._name, self.mouse_cb)
        while(True):
            cv2.imshow(self._name, self._img)
            k = cv2.waitKey(20)
            if k == 27:
                break
        cv2.destroyAllWindows()

    def mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._drawing = True
            self._ix, self._iy = x,y
        elif event == cv2.EVENT_MOUSEMOVE:
            if self._drawing:
                cv2.circle(self._img, (x,y), self._r, 255, -1)
        elif event == cv2.EVENT_LBUTTONUP:
            self._drawing = False
        elif event == cv2.EVENT_RBUTTONDOWN:
            self._init = (y, x)
        elif event == cv2.EVENT_MBUTTONDOWN:
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
    astar = AStar(grid, init, goal)
    _, path = astar()

    if path:
        viz = grid.astype(np.float32)
        viz = cv2.cvtColor(viz, cv2.COLOR_GRAY2BGR)

        for (p0, p1) in zip(path[:-1], path[1:]):
            y0,x0 = p0
            y1,x1 = p1
            #cv2.line( (y0,x0), (y1,x1), (128)
            cv2.line(viz, (x0,y0), (x1,y1), (0,0,1), 2)

        cv2.imshow('viz', viz)
        cv2.waitKey(0)
    else:
        print 'Path Not Found'



if __name__ == "__main__":
    main()
