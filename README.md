# Cooperative Driving

# Publications Read
## Demonstrations of Cooperative Perception: Safety and Robustness in Connected and Automated Vehicle Operations, *Mao Shan, 12.2020*
.
### Keywords
- Cooperative Sensing Message (CSM) from AutoNet2030
- Collective Perception
- Cooperative Perception
- Cooperative collision avoidance
- Cooperative advisory warnings
- Vehicle misbehaviour detection
- Integrated Road Safety Units (IRSU)
- Ko-PER CPM
- Data elements (DEs)
- Data Frames (DFs)
- Infrastructure-assisted CP
### Notes
- The ETSI CPMs convey abstract representations of perceived objects instead of raw sensory data
  - facilitating the interoperability between ITS-Ss of different types and from different manufacturers
- sharing raw sensor data is not useful, because of bandwidth
- raw sensor data are often vendor dependent and proprietary causing interoperability issues among communicating ITS-Ss
- The ETSI CPMs convey abstract representations of perceived objects instead of raw sensory data
### Contributions
- to investigate and demonstrate through three representative experiments how a CV and a CAV achieve improved safety and robustness when perceiving and interacting with VRU, using the CP information from an intelligent infrastructure in different traffic environments and with different setups.
- to address the coordinate transformation of perception information considering
the respective uncertainties
- to analyse the influence of ITS-S self-localisation accuracy through numerical simulations.

### References
- Ko-FAS [8]
- high level object fusion framework in CP [10], which combines the local sensor information with the perception data received from other V2X enabled vehicles or roadside units (RSUs)
- Proxy CAM [17-19]
- the concern of redundant data sharing in V2V based CP with the increase
of CAV penetration rate [27]
- To tackle the redundant transmission issue, a probabilistic data selection approach is presented [28]
- Quantitative comparison of V2V and V2I connectivity on improving sensing redundancy and collaborative sensing coverage for CAV applications
-  **Environmental Perception Message (EPM)** for CP, at [13] while based on [9] 
- CPM currently being specified at ETSI, as in [23], is derived from optimising the EPM and combining it with CAM.
- **ETSI CPM generation rules** is investigated in [25]
- An adaptive CPM generation rule considering change in perceived object’s state [29]
- Object filtering scheme in CPM [30]
- a deep reinforcement learning based approach that a vehicle can employ when selecting data to transmit in CP to alleviate communication congestion [31]
- early study of CP illustrating its potential in terms of improved vehicle awareness and extended perception range and field of view [32]
- EPM for obstacle avoidance of two manually driven CAVs, showing that the CP helps gain extra reaction time for the vehicles to avoid obstacles [13]
- the performance gain in extending horizon of CAVs by leveraging V2V based CP [27]
- analytically evaluates the enhancement of environmental perception for CVs at different CP service penetration rates and with different traffic densities [33,34]
- using CP for detecting vehicle misbehaviour due to adversarial attacks in V2X communication [36]
- cooperative driving [5,16]
- cooperative advisory warnings [37,38]
- cooperative collision avoidance [4,13,39]
- intersection assistance [17,40],
- vehicle misbehaviour detection [41]
- quantitative comparison of V2V and V2I connectivity on improving sensing redundancy and collaborative sensing coverage for CAV applications
- Managing Automated Vehicles Enhances Network (MAVEN) [45], TransAID [46] and IMAGinE [47]


### Tasks done
- sensory data fusion of images and lidar point clouds for pedestrian and vehicle detection
- the road users within the images are classified/detected using YOLOv3 that runs on GPU.
- the lidar point clouds are projected to the image coordinate system with proper extrinsic sensor calibration parameters
- The lidar points are then segmented, clustered and labelled by fusing the visual classifier results (in the form of bounding boxes in the images) and the projected lidar points.
- The detection results are then encoded into ETSI CPMs and broadcast by the Cohda Wireless MK5 RSU at 10 Hz
- Gaussian mixture probability hypothesis density (GMPHD) filter [57] is employed to track multiple road users
- the automatic extrinsic calibration toolkit presented in [58], to calibrate camera and lidar to the local coordinate system of the platform
- No perception sensors were used for road user detection
- the multi-beam lidar was enabled only for aiding self-localisation within the map
- Lidar feature maps of the experiment sites were built using a simultaneous localisation and mapping (SLAM) algorithm. 
- The maps are based on pole and building corner features extracted from lidar point clouds [60]
- ETSI CPM through Conda MP5 OBU is decoded from binary ASN.1 and transformed with its uncertainty to local frame of referance of the CAV, which also takes into account the estimated egocentric pose of the CAV in self-localisation
- **Following the coordinate transformation with uncertainty, the perceived objects information from the IRSU are fused into a multiple road user tracking algorithm that is a variant of the GMPHD filter running within the local frame of the receiving CAV.**
- a Lanelet2 map is built for every experiment site, which includes road network,lane layout and traffic rules such as speed limits, traffic lights and right-of-way rules

### Softwares
- [27,28], SUMO 
- [49,50], Veins (Vehicles in Network Simulation) , which integrates SUMO and a discrete-event simulator OMNeT++ for modelling realistic communication patterns
- [21,22,25,30,32,52] Artery framework which wraps SUMO and OMNet++
- [43] Pro-SiVIC for CP related simulations
- [55,56] Carla + SUMO