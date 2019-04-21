## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

For the `Motion_planning.py` script, it provides the following functionality:     

* MotionPlanning, which is the class using ERP to fly the drone, namely, the process of 
* I also implemented the prune_path() function here

For the `planning_util.py` script, it have some supportable fucntion to do the planning, which includes:

* create_grid() . It returns a grid representation of a 2D configuration space  based on given obstacle data, drone altitude and safety distance   arguments.
* valid_actions(). It returns a list of valid actions given a grid and current node.
* a_star(). It returns the path calculated using the A* algorithm, and I make some modification to this function to add the diagonal action available for planning.
* heuristic(). It returns a metric of how close a point is to the goal.

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.
```
f = open('colliders.csv')
first_line = f.readline().rstrip().split(', ') # ['lat0 37.792480', 'lon0 -122.397450']
lat0 = float(first_line[0].split()[1])
lon0 = float(first_line[1].split()[1])
```
Just read the first line and do some manual information extraction.

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

```
self.set_home_position(lon0, lat0, 0)
```

#### 3. Set grid start position from local position
Here I didn't make modification, just left it as default.

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

```
goal_position_global = (-122.397316, 37.793865, 0)
```
Also, the convert from global to grid position is needed.
#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

```
# Define NW, NE, SW, & SE costs
NORTH_WEST = (-1, -1, np.sqrt(2))
NORTH_EAST = (-1,  1, np.sqrt(2))
SOUTH_WEST = ( 1, -1, np.sqrt(2))
SOUTH_EAST = ( 1,  1, np.sqrt(2))


# Remove NW, NE, SW, or SE if the either horizontal or vertical index falls
# out of bounds or if explicity occupied (requiring both indices)
if (north < 0) or (west < 0) or grid[north, west]:
valid_actions.remove(Action.NORTH_WEST)
if (north < 0) or (east > m) or grid[north, east]:
valid_actions.remove(Action.NORTH_EAST)
if (south > n) or (west < 0) or grid[south, west]:
valid_actions.remove(Action.SOUTH_WEST)
if (south > n) or (east > m) or grid[south, east]:
valid_actions.remove(Action.SOUTH_EAST)
```

I modified the `valid_actions` function as well as constants in the `Action` class.


#### 6. Cull waypoints 
If the 3 points are in a line remove the 2nd point. The 3rd point now becomes and 2nd point and the check is redone with a new third point on the next iteration.
```
def collinearity_check(p1, p2, p3, epsilon=1e-6):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    pruned_path = [p for p in path]
    # TODO: prune the path!

    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path
```

### Execute the flight
#### 1. Does it work?
It works!

![Test Case]((./misc/fcnd-project2-motion-planning.gif)


	