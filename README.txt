Using RRT*, plan a path for an Aubo i3 6-axis industrial robot from a starting pose to and ending pose.

Execution:
  From project root:
    build -t rrt . && docker run -it rrt

The planner will run a number of planning sessions, randomly generating a starting and ending pose,
and then print out the entire joint path after each attempt. The output isn't particularly
well-formatted as of this writing, but this project is more of a demo of planning algs and
code quality.

My notes about the implementation and what's left for productionization found in source/inc/rrt_star_planner.h

Suggestions for the prompt itself:
- State explicitly that collision checking intentionally returns false always.
