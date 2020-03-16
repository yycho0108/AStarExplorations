#!/usr/bin/env python3

import sys
try:
    sys.path.append('build/swig')
    import bglpy as g
except ImportError as e:
    print('bglpy import failed ; build the library first.')
    print('{}'.format(e))
    sys.exit(1)
import numpy as np
import time

from line_profiler import LineProfiler

from phonebot.core.kinematics.workspace import get_workspace
from phonebot.core.kinematics.workspace import rasterize
from matplotlib import pyplot as plt


state = {'mdown' : False }
def main():
    grid = ~rasterize(get_workspace(return_poly=True, lower_half=False), 512)
    v0 = np.int32([39,43]).reshape(2,1)
    v1 = np.int32([147,101]).reshape(2,1)

    start = time.time()
    for _ in range(100):
        path = g.FindPath(grid, v0, v1)
    # print(path)
    end = time.time()
    print( (end - start) / 100 )
    # print(path)


def gui():
    grid = ~rasterize(get_workspace(return_poly=True, lower_half=False), 2048)
    print(grid.dtype)

    fig = plt.gcf()
    h, = plt.plot([],[],'rx-')
    def on_click(event):
        global state
        j, i = int(event.xdata), int(event.ydata)
        if state['mdown']:
            v0 = np.int32([state['i'], state['j']]).reshape(2,1)
            v1 = np.int32([i, j]).reshape(2,1)
            print(v0,v1)
            
            t0=time.time()
            path = g.FindPath(grid, v0, v1)
            t1=time.time()
            print(t1-t0)

            if len(path) > 0:
                print(path)
                path = np.concatenate(path, axis=1)
                h.set_data(path[::-1])
                plt.pause(0.001)
            state['mdown'] = False
        else:
            state['mdown'] = True
            state['i'] = i
            state['j'] = j

    fig.canvas.mpl_connect('button_press_event', on_click)
    plt.imshow(grid)
    plt.show()

if __name__ == '__main__':
    # main()
    gui()

    #prof = LineProfiler()
    #prof.add_function(g.FindPath)
    #prof.enable_by_count()
    # prof.add_function(g.FindPath)
    # prof.print_stats()
