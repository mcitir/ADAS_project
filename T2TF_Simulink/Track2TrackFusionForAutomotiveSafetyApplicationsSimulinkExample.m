%% Track-to-Track Fusion for Automotive Safety Applications in Simulink
% This example shows how to perform track-to-track fusion in Simulink(R)
% with Sensor Fusion and Tracking Toolbox(TM). In the context of autonomous
% driving, the example illustrates how to build a decentralized tracking
% architecture using a Track-To-Track Fuser block. In the example, each vehicle
% performs tracking independently as well as fuses tracking information
% received from other vehicles. This example closely follows the
% <docid:fusion_ug#mw_6bc1ae11-9260-43cd-b4d4-27fda4d0f9a6
% Track-to-Track Fusion for Automotive Safety Applications> MATLAB(R)
% example.

% Copyright 2019-2021 The MathWorks, Inc.

%% Introduction
% Automotive safety applications largely rely on the situational awareness
% of the vehicle. A better situational awareness provides the basis to a
% successful decision-making for different situations. To achieve this,
% vehicles can benefit from intervehicle data fusion. This example
% illustrates the workflow in Simulink for fusing data from two vehicles to
% enhance situational awareness of the vehicle.
%
%% Setup and Overview of the Model
load_system('TrackToTrackFusionSimulink');
set_param('TrackToTrackFusionSimulink','SimulationCommand','update');
open_system('TrackToTrackFusionSimulink');
%%
% Prior to running this example, the drivingScenario object was used to
% create the same scenario defined in
% <docid:fusion_ug#mw_6bc1ae11-9260-43cd-b4d4-27fda4d0f9a6 Track-to-Track
% Fusion for Automotive Safety Applications>. The roads and actors from
% this scenario were then saved to the scenario object file TrackToTrackFusionScenario.mat. 
%
%% Tracking and Fusion
%
% In the Tracking and Fusion section of the model there are two subsystems
% that implement the target tracking and fusion capabilities of
% Vehicle 1 and Vehicle 2 in this scenario.
%
% *Vehicle 1 Subsystem*
%%
load_system('TrackToTrackFusionSimulink');
open_system('TrackToTrackFusionSimulink/Vehicle 1');
%%
% This subsystem includes the <docid:driving_ref#mw_0abe0f52-f25a-4829-babb-d9bafe8fdbf3 Scenario
% Reader> block that reads the actor pose data from the saved file. The block
% converts the actor poses from the world coordinates of the scenario into
% ego vehicle coordinates. The actor poses are streamed on a bus generated
% by the block. The actor poses are used by the Sensor Simulation
% subsystem, which generates radar and vision detections. These detections
% are then passed to the JPDA Tracker V1 block, which processes the
% detections to generate a list of tracks. The tracks are then passed into
% a Track Concatenation1 block, which concatenates these input tracks.
% The first input to the Track Concatenation1 block is the local tracks
% from the JPDA tracker and the second input is the tracks received from
% the track fuser of the other vehicle. To transform local tracks to central
% tracks, the track fuser needs the parameter information about the local
% tracks. However, this information is not available from the direct
% outputs of the JPDA tracker. Therefore, a helper Update Pose block is
% used to supply this information by reading the data from the v1Pose.mat
% file. The updated tracks are then broadcasted to T2TF Tracker V1  block
% as an input. Finally, the <docid:fusion_ref#block_track_to_track_fuser track
% fuser> T2TF Tracker V1 block fuse the local vehicle tracks with the
% tracks received from the track fuser of the other vehicle. After each update,
% the track fuser on each vehicle broadcasts its fused tracks to be fed
% into the update of the track fuser of the other vehicle in the next time
% stamp.
%
% *Vehicle 2 Subsystem*
%%
load_system('TrackToTrackFusionSimulink');
open_system('TrackToTrackFusionSimulink/Vehicle 2');
%%
% Vehicle 2 subsystem follows a similar setup as the Vehicle 1 subsystem.
%
% *Visualization*
%
% The Visualization block is implemented using the MATLAB System block and
% is defined using the HelperTrackDisplay block. The block uses
% RunTimeObject parameters Out, Confirmed Tracks, Tracks and Confirmed
% Tracks of Detection Clustering, JPDA Tracker V1, Update Pose V1, T2TF
% Tracker V1 blocks respectively for vehicle 1 and RunTimeObject parameters
% Out, Confirmed Tracks, Tracks and Confirmed Tracks of Detection
% Clustering, JPDA Tracker V2, Update Pose V2, T2TF Tracker V2 blocks
% respectively for vehicle 2 to display their outputs. See
% <docid:simulink_ug#f13-92122 Access Block Data During Simulation> for
% further information on how to access block outputs during simulation.
%
%% Results
% After running the model, you can visualize the results. This
% animation shows the results for this simulation.
%
% The visualization includes two panels. The left panel shows the
% detections, local tracks, and fused tracks that vehicle 1 generated
% during the simulation and represents the situational awareness of 
% vehicle 1. The right panel shows the situational awareness of
% vehicle 2.
%
% The recorded detections are represented by black circles. The local and
% fused tracks from vehicle 1 are represented by a square and a diamond,
% respectively. The local and fused tracks from vehicle 2 represented by a
% solid black square and a diamond. At the start of
% simulation, vehicle 1 detects vehicles parked on the right side of the
% street, and tracks associated with the parked vehicles are confirmed.
% Currently, vehicle 2 only detects vehicle 1 which is immediately in
% front of it. As the simulation continues, the confirmed tracks from
% vehicle 1 are broadcast to the fuser on vehicle 2. After fusing the
% tracks, vehicle 2 becomes aware of the objects prior to detecting these
% objects on its own. Similarly, vehicle 2 tracks are broadcast to
% vehicle 1. Vehicle 1 fuses these tracks and becomes aware of the
% objects prior to detecting them on its own.
%
% In particular, you observe that the pedestrian standing between the blue
% and purple cars on the right side of the street is detected and tracked by
% vehicle 1. Vehicle 2 first becomes aware of the pedestrian by fusing
% the track from Vehicle 1 at around 0.8 seconds. It takes vehicle 2
% roughly 3 seconds before it starts detecting the pedestrian using its own
% sensor. The ability to track a pedestrian based on inputs from vehicle 1
% allows vehicle 2 to extend its situational awareness and to mitigate the
% risk of accident.
%
% <<../Track2TrackFusionResults.gif>>
%% Summary
% This example showed how to perform track-to-track fusion in Simulink. You
% learned how to perform tracking using a decentralized tracking
% architecture, where each vehicle is responsible for maintaining its own
% local tracks, fuse tracks from other vehicles, and communicate the tracks
% to the other vehicle. You also use a JPDA tracker block to generate the
% local tracks.