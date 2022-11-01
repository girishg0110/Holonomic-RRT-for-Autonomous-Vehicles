# Holonomic RRT for Autonomous Vehicles

The RRT and RRT* algorithms are approaches to efficiently explore a configuration space which can be used to route an autonomous vehicle in an unknown environment (i.e. off-road, areas with low satellite-connectivity or occlusion). 
Both algorithms are implemented as well as tools for visualizing the rapidly-exploring random tree as it evolves over time. 

In addition, the implementation presumes to have direct control over linear and angular velocities instead of positions, approaching real-life use cases where drivers have direct control over acceleration. 
This is the [**kinematic** motion planning]{https://en.wikipedia.org/wiki/Robot_kinematics} setting for a car with [holonomic constraints]{https://en.wikipedia.org/wiki/Holonomic_constraints} on its possible configurations. 

See the report for more details and example visualizations.
