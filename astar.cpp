#include <queue>
#include <limits>
#include <cmath>
#include <array>
#include <algorithm>

// represents a single pixel
class Node {
  public:
    int idx;     // index in the flattened grid
    float cost;  // cost of traversing this pixel

    Node(int i, float c) : idx(i),cost(c) {}
};

// the top of the priority queue is the greatest element by default,
// but we want the smallest, so flip the sign
bool operator<(const Node &n1, const Node &n2) {
  return n1.cost > n2.cost;
}

bool operator==(const Node &n1, const Node &n2) {
  return n1.idx == n2.idx;
}

// See for various grid heuristics:
// http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#S7
// L_\inf norm (diagonal distance)
float linf_norm(int i0, int j0, int i1, int j1) {
  return std::max(std::abs(i0 - i1), std::abs(j0 - j1));
}

// L_1 norm (manhattan distance)
float l1_norm(int i0, int j0, int i1, int j1) {
  return std::abs(i0 - i1) + std::abs(j0 - j1);
}

float linf_norm(int i0, int j0, int i1, int j1, int k0, int k1) {                     
  return std::max({ std::abs(i0 - i1), std::abs(j0 - j1), std::abs(j0 - j1) },
  [](const int s1, int s2) { return s1 < s2; } );
}

// L_1 norm (manhattan distance)
float l1_norm(int i0, int j0, int i1, int j1, int k0, int k1) {
  return std::abs(i0 - i1) + std::abs(j0 - j1) + std::abs(k0 - k1);
}

int to1D( int x, int y, int z, int w, int h ) {
    return (z * w * h) + (y * w) + x;
}

std::array<int, 3> to3D( int idx, int w, int h) {
    std::array<int, 3> converted;
    int z = idx / (w * h);
    idx -= (z * w * h);
    int y = idx / w;
    int x = idx % w;
    converted[0] = x;
    converted[1] = y;
    converted[2] = z;
    return converted;
}

bool isValid(int x, int y, int z, int w, int h, int d) { 
			if (x < 0 || y < 0 || z < 0 || x >= h || y >= w || z >= d) {
				return false;
			}else{
				return true;
			}
	}


std::array<int, 12> getNeighbors(int idx,int w, int h, int d, bool diagonal) {
		
		std::array<int, 12> neighbors;
		auto current = to3D( idx, w, h);
		
    int x = current[0];
    int y = current[1];
    int z = current[2];
		// down
		if(isValid(x-1, y, z, w, h, d)) {
			neighbors[0] = to1D(x-1, y, z, w, h);
		}else{
      neighbors[0] = -1;
    }

		//up
		if(isValid(x+1, y, z, w, h, d)) {
			neighbors[1] = to1D(x+1, y, z, w, h);
		}else{
      neighbors[1] = -1;
    }

		//left
		if(isValid(x, y-1, z, w, h, d)) {
			neighbors[2] = to1D(x, y-1, z, w, h);
		}else{
      neighbors[2] = -1;
    }

		//right
		if(isValid(x, y+1, z, w, h, d)) {
			neighbors[3] = to1D(x, y+1, z, w, h);
		}else{
      neighbors[3] = -1;
    }

		//bottom down
		if(isValid(x-1, y, z-1, w, h, d)) {
			neighbors[4] = to1D(x-1, y, z-1, w, h);
		}else{
      neighbors[4] = -1;
    }

		//bottom up
		if(isValid(x+1, y, z-1, w, h, d)) {
			neighbors[5] = to1D(x+1, y, z-1, w, h);
		}else{
      neighbors[5] = -1;
    }

		//bottom left
		if(isValid(x, y-1, z-1, w, h, d)) {
				neighbors[6] = to1D(x, y-1, z-1, w, h);
		}else{
      neighbors[6] = -1;
    }

		//bottom right
		if(isValid(x, y+1, z-1, w, h, d)) {
			neighbors[7] = to1D(x, y+1, z-1, w, h);
		}else{
      neighbors[7] = -1;
    }

		//top down
		if(isValid(x-1, y, z+1, w, h, d)) {
			neighbors[8] = to1D(x-1, y, z+1, w, h);
		}else{
      neighbors[8] = -1;
    }

		//top up
		if(isValid(x+1, y, z+1, w, h, d)) {
			neighbors[9] = to1D(x+1, y, z+1, w, h);
		}else{
      neighbors[9] = -1;
    }

		//top left
		if(isValid(x, y-1, z+1, w, h, d)) {
			neighbors[10] = to1D(x, y-1, z+1, w, h);
		}else{
      neighbors[10] = -1;
    }

		//top right
		if(isValid(x, y+1, z+1, w, h, d)) {
			neighbors[11] = to1D(x, y+1, z+1, w, h);
		}else{
      neighbors[11] = -1;
    }

		// //8-Way
		// if (diagonal) {
		// 	//left down
		// 	if(isValid(x-1, y-1, z, w, h, d)) {
		// 			neighbors.push_back(allMap[x-1][y-1][z]);
		// 	}
			
		// 	//left up
		// 	if(isValid(x+1, y-1, z, w, h, d)) {
		// 		neighbors.push_back(allMap[x+1][y-1][z]);
		// 	}

		// 	//right down
		// 	if(isValid(x-1, y+1, z, w, h, d)) {
		// 		neighbors.push_back(allMap[x-1][y+1][z]);
		// 	}

		// 	//right up
		// 	if(isValid(x+1, y+1, z, w, h, d)) {
		// 		neighbors.push_back(allMap[x+1][y+1][z]);
		// 	}

		// 	//bottom left down
		// 	if(isValid(x-1, y-1, z-1, w, h, d)) {
		// 			neighbors.push_back(allMap[x-1][y-1][z-1]);
		// 	}
			
		// 	//bottom left up
		// 	if(isValid(x+1, y-1, z-1, w, h, d)) {
		// 		neighbors.push_back(allMap[x+1][y-1][z-1]);
		// 	}

		// 	//bottom right down
		// 	if(isValid(x-1, y+1, z-1, w, h, d)) {
		// 		neighbors.push_back(allMap[x-1][y+1][z-1]);
		// 	}

			//bottom right up
		// 	if(isValid(x+1, y+1, z-1, w, h, d)) {
		// 		neighbors.push_back(allMap[x+1][y+1][z-1]);
		// 	}

		// 	//top left down
		// 	if(isValid(x-1, y-1, z+1, w, h, d)) {
		// 		neighbors.push_back(allMap[x-1][y-1][z+1]);
		// 	}
			
		// 	//top left up
		// 	if(isValid(x+1, y-1, z+1, w, h, d)) {
		// 		neighbors.push_back(allMap[x+1][y-1][z+1]);
		// 	}

		// 	//top right down
		// 	if(isValid(x-1, y+1, z+1, w, h, d)) {
		// 		neighbors.push_back(allMap[x-1][y+1][z+1]);
		// 	}

		// 	//top right up
		// 	if(isValid(x+1, y+1, z+1, w, h, d)) {
		// 		neighbors.push_back(allMap[x+1][y+1][z+1]);
		// 	}
		// }
		return neighbors;
}

// weights:        flattened h x w grid of costs
// h, w:           height and width of grid
// start, goal:    index of start/goal in flattened grid
// diag_ok:        if true, allows diagonal moves (8-conn.)
// paths (output): for each node, stores previous node in path
extern "C" bool astar(
      const float* weights, const int h, const int w, const int d,
      const int start, const int goal, bool diag_ok,
      int* paths) {

  const float INF = std::numeric_limits<float>::infinity();

  Node start_node(start, 0.);
  Node goal_node(goal, 0.);

  float* costs = new float[h * w * d];
  for (int i = 0; i < h * w * d; ++i)
    costs[i] = INF;
  costs[start] = 0.;

  std::priority_queue<Node> nodes_to_visit;
  nodes_to_visit.push(start_node);

  // int* nbrs = new int[12];

  bool solution_found = false;
  while (!nodes_to_visit.empty()) {
    // .top() doesn't actually remove the node
    Node cur = nodes_to_visit.top();

    if (cur == goal_node) {
      solution_found = true;
      break;
    }

    nodes_to_visit.pop();

    // int row = cur.idx / w; // y (width -x, height- y, depth -d)
    // int col = cur.idx % w; // x

    int row = cur.idx % w;
    int col = ( cur.idx / w ) % h;
    int depth = cur.idx / ( w * h );

    // check bounds and find up to eight neighbors: top to bottom, left to right
    // nbrs[0] = (diag_ok && row > 0 && col > 0)          ? cur.idx - w - 1   : -1;
    // nbrs[1] = (row > 0)                                ? cur.idx - w       : -1;
    // nbrs[2] = (diag_ok && row > 0 && col + 1 < w)      ? cur.idx - w + 1   : -1;
    // nbrs[3] = (col > 0)                                ? cur.idx - 1       : -1;
    // nbrs[4] = (col + 1 < w)                            ? cur.idx + 1       : -1;
    // nbrs[5] = (diag_ok && row + 1 < h && col > 0)      ? cur.idx + w - 1   : -1;
    // nbrs[6] = (row + 1 < h)                            ? cur.idx + w       : -1;
    // nbrs[7] = (diag_ok && row + 1 < h && col + 1 < w ) ? cur.idx + w + 1   : -1;

    auto nbrs = getNeighbors(cur.idx, w, h, d, false);

    float heuristic_cost;
    for (int i = 0; i < 12; ++i) {
      if (nbrs[i] >= 0) {
        // the sum of the cost so far and the cost of this move
        float new_cost = costs[cur.idx] + weights[nbrs[i]];
        if (new_cost < costs[nbrs[i]]) {
          // estimate the cost to the goal based on legal moves
          auto current_point = to3D( nbrs[i], w, h);
          auto goal_point = to3D(goal, w, h);

          if (diag_ok) {
            heuristic_cost = linf_norm(current_point[0], current_point[1],
                                       goal_point[0],  goal_point[1],current_point[2],  goal_point[2]);
          }
          else {
            heuristic_cost = l1_norm(current_point[0], current_point[1],
                                       goal_point[0],  goal_point[1],current_point[2],  goal_point[2]);
          }

          // paths with lower expected cost are explored first
          float priority = new_cost + heuristic_cost;
          nodes_to_visit.push(Node(nbrs[i], priority));

          costs[nbrs[i]] = new_cost;
          paths[nbrs[i]] = cur.idx;
        }
      }
    }
  }

  delete[] costs;
  // delete[] nbrs;

  return solution_found;
}

