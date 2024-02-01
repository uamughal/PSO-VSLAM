# PSO-VSLAM
- UAVs path planning by particle swarm optimization based on visual-SLAM algorithm

Unmanned Aerial Vehicles (UAVs) have gained tremendous popularity due to its high mobility in various robotics platforms.
We use a state-of-the-art visual simultaneous localization and mapping (VSLAM) method to trace the UAV poses while simultaneously
constructing an incremental and progressive map of the surrounding environment. In this regard, a single UAV is first used to compose
a map of the region of interest by utilizing the monocular vision-based approach. The constructed map is processed as an input mean
for the optimization algorithm to plan multiple UAVs’ optimum paths. We design a path planner based on particle swarm optimization
(PSO) and propose a path updating mechanism based on region sensitivity (RS) to avoid sensitive areas if any hazardous events are
detected while executing the final path. Moreover, we propose a dynamic fitness function (DFF) to evaluate the path planner’s planning
strategy considering various optimization parameters such as flight risk estimation, energy consumption, and operation completion
time. The proposed planner attains the high fitness value and reaches the destination safely following the shortest path while avoiding
all the unexpected hazardous events and restricted areas, which validate the effectiveness of our proposed PSO-VSLAM system as
depicted by the simulation results.




# Citing this work
If you are using our implementation, you are encouraged to cite [our paper](https://www.sciencedirect.com/science/article/pii/S221420962030053X).
``` 

@incollection{mughal2022uavs,
  title={UAVs path planning by particle swarm optimization based on visual-SLAM algorithm},
  author={Mughal, Umair Ahmad and Ahmad, Ishtiaq and Pawase, Chaitali J and Chang, KyungHi},
  booktitle={Intelligent Unmanned Air Vehicles Communications for Public Safety Networks},
  pages={169--197},
  year={2022},
  publisher={Springer}
}

``` 
