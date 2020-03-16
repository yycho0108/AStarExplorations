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
from matplotlib import pyplot as plt
import time

if False:
    xs, ys = np.mgrid[-1:1:0.01, -1:1:0.01]
    sdf = g.SdfCircle(g.Node2D(0, 0), 0.1)
    print(sdf(g.Node2D(0, 2)))  # 3.9?
    sys.exit(0)
    dmap = np.zeros_like(xs)
    for i, j in np.ndindex(dmap.shape):
        x, y = xs[i, j], ys[i, j]
        d = sdf(g.Node2D(x, y))
        dmap[i, j] = d
        print(x, y)
        print(d, np.sqrt(x*x+y*y) - 0.1)
    print(dmap.max())
    print(dmap.min())
    plt.imshow(dmap)
    plt.show()
    sys.exit(0)


def get_workspace_sdf():
    kHipJointOffset = 0.0110
    kKneeLinkLength = 0.0285
    kHipLinkLength = 0.0175
    kSmallRadius = kKneeLinkLength - kHipLinkLength
    kLargeRadius = kKneeLinkLength + kHipLinkLength

    ca = g.Node2D(kHipJointOffset, 0)  # (0.011, 0.0)
    cb = g.Node2D(-kHipJointOffset, 0)  # (-0.011, 0.0)

    f1 = g.SdfCircle(ca, kSmallRadius)
    f2 = g.SdfCircle(cb, kSmallRadius)
    f3 = g.SdfCircle(ca, kLargeRadius)
    f4 = g.SdfCircle(cb, kLargeRadius)

    # if interior is negative ...
    # return g.SdfIntersect(
    #        g.SdfVector([g.SdfNegate(f1),g.SdfNegate(f2),f3,f4]))

    # if interior is positive ...
    return g.SdfUnion(
        g.SdfVector([f1, f2, g.SdfNegate(f3), g.SdfNegate(f4)]))


def get_distance_map(sdf):
    mnx, mny = -0.035, -0.044
    mxx, mxy = 0.035, 0.044
    xs, ys = np.mgrid[mnx:mxx:0.001, mny:mxy:0.001]
    dmap = np.zeros_like(xs)
    for i, j in np.ndindex(dmap.shape):
        x, y = xs[i, j], ys[i, j]
        d = sdf(g.Node2D(x, y))
        dmap[i, j] = d
    return dmap


sdf = get_workspace_sdf()
s = g.RrtSettings()
s.workspace = g.Workspace(g.Node2D(-0.035, -0.044), g.Node2D(0.035, 0.044))
s.max_iter = 16384

extent = [-0.035, 0.035, -0.044, 0.044]
dmap = get_distance_map(sdf)
rrt = g.Rrt(s, sdf)
rrt.Seed(512)
source = g.Node2D(0.0032096, 0.03201624)
target = g.Node2D(0.01301248, -0.01502493)
start = time.time()
path = rrt.GetTrajectory(source, target)
end = time.time()
print('Took {} sec'.format(end-start))
path = np.asarray([(p.x, p.y) for p in path])

plt.imshow(dmap.T, extent=extent)
plt.imshow(dmap.T > 0, extent=extent, alpha=0.1)
plt.plot(source.x, source.y, 'rx')
plt.plot(target.x, target.y, 'b+')
plt.plot(path[..., 0], path[..., 1])
plt.axis('equal')
plt.gca().set_aspect('equal')
plt.show()
