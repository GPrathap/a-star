import cv2
import numpy as np

import pyastar

from time import time
from os.path import basename, join, splitext

# input/output files
MAZE_FPATH = join('mazes', 'maze_small.png')
#MAZE_FPATH = join('mazes', 'maze_large.png')
OUTP_FPATH = join('solns', '%s_soln.png' % splitext(basename(MAZE_FPATH))[0])


def main():
    # maze = cv2.imread(MAZE_FPATH)
    # if maze is None:
    #     print('no file found: %s' % (MAZE_FPATH))
    #     return
    # else:
    #     print('loaded maze of shape %r' % (maze.shape[0:2],))

    # grid = cv2.cvtColor(maze, cv2.COLOR_BGR2GRAY).astype(np.float32)
    grid = np.ones((200, 100, 100)).astype(np.float32)
    # grid[grid == 0] = np.inf
    # grid[grid == 255] = 1

    assert grid.min() == 1, 'cost of moving must be at least 1'

    # start is the first white block in the top row
    start = np.array([2,1,3])

    # end is the first white block in the final column
    end = np.array([40, 50, 67])

    t0 = time()

    path = pyastar.astar_path(grid, start, end, allow_diagonal=False)
    dur = time() - t0

    if path.shape[0] > 0:
        print('found path of length %d in %.6fs' % (path.shape[0], dur))
        # grid[path[:, 0], path[:, 1], path[:, 2]] = (0, 0, 255)
        print(path[:, 0])
        print(path[:, 1])
        print(path[:, 2])
        # print('plotting path to %s' % (OUTP_FPATH))
        # cv2.imwrite(OUTP_FPATH, grid)
    else:
        print('no path found')

    print('done')


if __name__ == '__main__':
    main()
