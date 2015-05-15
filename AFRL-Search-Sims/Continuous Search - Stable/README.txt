Author: Drew Ellison
Institute: University of Colorado Boulder
Last Updated: 8/11/14

File to Run: Simulation.m

Description:

The search simulation being performed is that of a single agent, displayed
as a white circle, looking for a single, stationary target, displayed as a pink X. The 
graphic shows the probability distribution of the agents believed pose of the
target as a heat map with blue areas being low probability and red areas being
high. The agent updates this Bayesian belief (currently a discritization of the
map) using its sensor, with sensor footprint shown as a cyan circle surrounding
the agent. The sensor is modelled as a binary sensor, i.e. 1 when the target
is seen and 0 when it is not. The model also includes the possibility of 
false alarms, with probability alpha, and missed detections, with probability
beta. The update phase after each measurement operates on every cell in the 
grid. 

The black areas on the map represent obstacles. The agent successfully performs
obstacle avoidance by creating a grid based on the Delauney triangulation of
1000 randomly sampled points in the free configuration space, i.e. points not associated
with an obstacle. The agent then uses A* to generate an obstacle free path 
to the point on the Delauney triangulation closest to the highest probability 
grid cell. The two grids have been decoupled in order to provide flexibility in
the representation of the probability distribution in future releases. 

The pulsing yellow circle surrounding the agent represents the 3 sigma bound
on its believed position. A Kalman filter is used to keep track of self-pose
using simulated IMU measurements and intermittent GPS signals. The update 
step in the probability grid has been modified to reflect the uncertainty in 
position, which effectively widens the possible placement of the footprint. 