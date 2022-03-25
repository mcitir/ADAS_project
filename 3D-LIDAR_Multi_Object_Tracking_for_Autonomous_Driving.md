## Masters Thesis: 3D-LIDAR Multi Object Tracking for Autonomous Driving, A.S. Abdul Rachman, 

### Keywords
- Bayesian Occupancy Filter  [74] (is a Grid-based detection method)
  - Limited to represent the dynamic objects [36]
- 

### Notes


### Contributions



### References
- Segmentation 
  - Ground extraction 
    - Divided into 3 subgroups
      - Grid/cell-based methods
        - Lidar data is divided into polar coordinate cells, and channel
      - Scan-based Methods (Line-based methods + Surface-based methods)
        -Takes lowest z value and apply Random Sample Consensus (RANSAC) fitting to determine possilbe ground [78]
- Clustering
  - 2D clustering [80]
  - 2.5D clustering [79]
  - 3D clustering, e.g. 
    - Radially Bounded Nearest Neigbour (RNN) [63]
    - Density-Based Spatial Clustering with Noise (DBSCAN) [82]
- Pose Estimation
  - Includes information orientation (heading), dimension, velocity, acceleration
  - Model Based P.E.
    - aims to match raw measurement into a known geometry
  - Feature-based 
    - estimation shape of object from a set of feature. 


### Tasks done (Short) 


### Tasks done (Long) 
- 3D lidar sensor has been selected.
- Occlusion-aware detection method is used to utilise the full potential of LIDAR
- For segmentation process:
  - Slope-based channel classification
- For clustering process  
  - 2D Connected Component Clustering 
- Pose estimation process utilizes: [96,15]
  - cluster height information to form a 3D Box. 
  - minimum area rectangle augmented with L-Shape fitting 
- Object Tracking
![](/images/2022-03-25-15-10-50.png)
- Polar to Cartesian Coondinate transformation by Velodyne Lidar Sensor digital processing unit and extension the KITTI dataset
- A point tracking is used instead of extended object tracking [100]
- Tracking target dynamic motion model is described mathematically by a discrete time, stochastic state-space model
![](/images/2022-03-25-15-29-31.png)



### Softwares
