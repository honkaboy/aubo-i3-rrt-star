Execution:
  From project root:
    build -t rapid . && docker run -it rapid

The planner will run a number of planning sessions, randomly generating a staring and ending pose,
and then print out the entire joint path after each attempt. The output isn't particularly
well-formatted, but I figured the planning algorithm / code quality is what you're mostly looking
at, so for the same of time-boxing development I left it like that.

My notes about the implementation and what's left for productionization found in source/inc/rrt_star_planner.h

Suggestions for the prompt itself:
- State explicitly that collision checking intentionally returns false always.
