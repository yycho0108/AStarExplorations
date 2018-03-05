import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial import Voronoi
from utils import *
from shapely.geometry import Polygon

""" Terrain-Based Map Generation """
def centroid(vs, box):
    p0 = Polygon(vs).intersection(box)
    return p0.centroid.xy

class PMap(object):
    def __init__(self, n):
        self._n = n
        self._build(n)
    def _build(self, n):
        pts = np.random.uniform(0, 1, size=(n,2))

        # lloyd ...
        vor = Voronoi(pts, qhull_options='Qbb Qc')
        r, v = voronoi_finite_polygons_2d(vor)
        box = Polygon([[0,0],[0,1],[1,1],[1,0]])
        pts = [centroid(v[e], box) for e in r]
        pts = np.squeeze(pts)

        self._pts = pts
        #plt.scatter(pts[:,0], pts[:,1])
        #plt.show()

if __name__ == "__main__":
    pmap = PMap(2**8)
