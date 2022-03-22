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

### Referances
- Ko-FAS [8]
- high level object fusion framework in CP [10], which combines the local sensor information with the perception data received from other V2X enabled vehicles or roadside units (RSUs)
- Proxy CAM [17-19]
- the concern of redundant data sharing in V2V based CP with the increase
of CAV penetration rate [27]
- To tackle the redundant transmission issue, a probabilistic data selection approach is presented [28]
- Quantitative comparison of V2V and V2I connectivity on improving sensing redundancy and collaborative sensing coverage for CAV applications

### Tasks done
- sensory data fusion of images and lidar point clouds for pedestrian and vehicle detection
- the road users within the images are classified/detected using YOLOv3 that runs on GPU.
- the lidar point clouds are projected to the image coordinate system with proper extrinsic sensor calibration parameters
- The lidar points are then segmented, clustered and labelled by fusing the visual classifier results (in the form of bounding boxes in the images) and the projected lidar points.
- The detection results are then encoded into ETSI CPMs and broadcast by the Cohda Wireless MK5 RSU at 10 Hz

### Softwares
- [27,28], SUMO 
- [49,50], Veins (Vehicles in Network Simulation) , which integrates SUMO and a discrete-event simulator OMNeT++ for modelling realistic communication patterns
- [21,22,25,30,32,52] Artery framework which wraps SUMO and OMNet++
- [43] Pro-SiVIC for CP related simulations
- [55,56] Carla + SUMO