Author: Drew Ellison
Last Updated: 8/5/14

Description: This repository simulated a robotic search agent in an indoor environment searching for a stationary target. A discrete Bayesian probability map is used to store the current belief for a targets position. A prior probability map must be specified by the user for initialization. The algorithm for updating the map uses Bayes’ rule based on modeling a camera as a binary detector of the target with the possibility of false alarms and missed detections. Several search algorithms have been included based on the paper ‘A Decision-Making Framework for Control Strategies in Probabilistic Search’ by Chung and Burdick, including:
1) Drosophila Search (Go to current max belief)
2) Optimal Lookahead (Perform local optimization of expected gain)
3) Sweeping search (ignore distribution and just cover all area uniformly)
4) Random walk