%Script that takes the internal robot vars as logged by the IHMC logger and
%mocap system and makes them more convenient, also sets up transformation
%matricies and start time for data sync between mocap/internal robot
%logging

% Instructions for use:
% Prereqs: Kabsch.m and absor.m
%
% 1) Import the markers .mat, ihmc .mat files, and start_time.txt from the
% run of interest
% 2) Run preprocess_data.m


%UTC time the sync signal is sent to the mocap system
start_time = int64(importdata('start_time.txt'))*1000;

%Matrices for the Kabsch algorithm describing marker to origin frame
%offsets
Q.pelvis = [-0.1162     0           0           0.057607;
            0           0.15921     -0.15921    -0.11938;
            -0.20149    -0.079571   -0.079571   -0.071697];

%Reduced Q.pelvis for box pickup due to front pelvis marker occlusion
% Q.pelvis = [-0.1162     0           0;
%             0           0.15921     -0.15921;
%             -0.20149    -0.079571   -0.079571];
        
%Reduced Q.pelvis for right_arm_grid trial due to significant occlusion of
%one marker during the trial, 3 is minimum so results are still valid
% Q.pelvis = [-0.1162    0           0.057607;
%             0          -0.15921    -0.11938;
%             -0.20149   -0.079571   -0.071697];
        
Q.r_arm = [0.02608     0.02504      -0.00681;
           -0.01055    -0.01055     -0.01055;
           -0.24875    0.05097      0.06523];
       
Q.l_arm = [-0.02608    -0.02504     0.00681;
           -0.01055    -0.01055     -0.01055;
           -0.24875    0.05097      0.06523];
       
Q.r_foot = [-0.06586    -0.08421  -0.05925;
           0            0.08255    0.07116;
           -0.07872     0.01665   0.17028];
       
Q.l_foot = [-0.06586    -0.08421  -0.05925;
           0            -0.08255  -0.07116;
           -0.07872     0.01665   0.17028];
       
% Q.torso = [-0.30345     -0.30345    -0.000889   -0.000889;
%            -0.07747     0.07747     -0.1487     0.1487;
%            0.1336       0.1336      0.3879      0.3879];

%Reduced Q.torso to 3 markers due to significant occlusion of 1 during
%multiple trials, 3 is minimum so results are OK
Q.torso = [-0.30345     -0.30345  -0.000889;
           -0.07747     0.07747    0.1487;
           0.1336       0.1336      0.3879];

%Robot Timestamp and Robot Time
valkyrie.timestamp = timestamp';
valkyrie.robot_time = robotTime';

%Center of Pressure and Center of Mass
valkyrie.cop = [CenterOfPressureX'; 
                CenterOfPressureY'];
            
valkyrie.com = [estimatedCenterOfMassPositionX'; 
                estimatedCenterOfMassPositionY'; 
                estimatedCenterOfMassPositionZ'];

%Robot State Estimator
valkyrie.state_estimator = [ stateEstimatorInWorldFramePoseX';
                             stateEstimatorInWorldFramePoseY';
                             stateEstimatorInWorldFramePoseZ'];
%Ground Reaction Forces
valkyrie.grf.left_foot_force = [ leftFootStateEstimatorForceX';
                                 leftFootStateEstimatorForceY';
                                 leftFootStateEstimatorForceZ'];
valkyrie.grf.left_foot_torque = [ leftFootStateEstimatorTorqueX';
                                 leftFootStateEstimatorTorqueY';
                                 leftFootStateEstimatorTorqueZ'];
valkyrie.grf.right_foot_force = [ rightFootStateEstimatorForceX';
                                 rightFootStateEstimatorForceY';
                                 rightFootStateEstimatorForceZ'];
valkyrie.grf.right_foot_torque = [ rightFootStateEstimatorTorqueX';
                                 rightFootStateEstimatorTorqueY';
                                 rightFootStateEstimatorTorqueZ'];                             

%Joint Torques
%The 3 neck joints are position controlled so no torque measurements from
%them
valkyrie.tau.left_ankle_pitch = leftAnklePitchTauMeasured';
valkyrie.tau.left_ankle_roll = leftAnkleRollTauMeasured';
valkyrie.tau.left_knee_pitch = leftKneePitchTauMeasured';
valkyrie.tau.left_hip_pitch = leftHipPitchTauMeasured';
valkyrie.tau.left_hip_roll = leftHipRollTauMeasured';
valkyrie.tau.left_hip_yaw = leftHipYawTauMeasured';

valkyrie.tau.right_ankle_pitch = rightAnklePitchTauMeasured';
valkyrie.tau.right_ankle_roll = rightAnkleRollTauMeasured';
valkyrie.tau.right_knee_pitch = rightKneePitchTauMeasured';
valkyrie.tau.right_hip_pitch = rightHipPitchTauMeasured';
valkyrie.tau.right_hip_roll = rightHipRollTauMeasured';
valkyrie.tau.right_hip_yaw = rightHipYawTauMeasured';

valkyrie.tau.left_elbow_pitch = leftElbowPitchTauMeasured';
valkyrie.tau.left_shoulder_pitch = leftShoulderPitchTauMeasured';
valkyrie.tau.left_shoulder_roll = leftShoulderRollTauMeasured';
valkyrie.tau.left_shoulder_yaw = leftShoulderYawTauMeasured';

valkyrie.tau.right_elbow_pitch = rightElbowPitchTauMeasured';
valkyrie.tau.right_shoulder_pitch = rightShoulderPitchTauMeasured';
valkyrie.tau.right_shoulder_roll = rightShoulderRollTauMeasured';
valkyrie.tau.right_shoulder_yaw = rightShoulderYawTauMeasured';

valkyrie.tau.torso_pitch = torsoPitchTauMeasured';
valkyrie.tau.torso_roll = torsoRollTauMeasured';
valkyrie.tau.torso_yaw = torsoYawTauMeasured';

%Joint Displacements
valkyrie.q.left_ankle_pitch = leftAnklePitch_q';
valkyrie.q.left_ankle_roll = leftAnkleRoll_q';
valkyrie.q.left_knee_pitch = leftKneePitch_q';
valkyrie.q.left_hip_pitch = leftHipPitch_q';
valkyrie.q.left_hip_roll = leftHipRoll_q';
valkyrie.q.left_hip_yaw = leftHipYaw_q';

valkyrie.q.right_ankle_pitch = rightAnklePitch_q';
valkyrie.q.right_ankle_roll = rightAnkleRoll_q';
valkyrie.q.right_knee_pitch = rightKneePitch_q';
valkyrie.q.right_hip_pitch = rightHipPitch_q';
valkyrie.q.right_hip_roll = rightHipRoll_q';
valkyrie.q.right_hip_yaw = rightHipYaw_q';

valkyrie.q.left_elbow_pitch = leftElbowPitch_q';
valkyrie.q.left_shoulder_pitch = leftShoulderPitch_q';
valkyrie.q.left_shoulder_roll = leftShoulderRoll_q';
valkyrie.q.left_shoulder_yaw = leftShoulderYaw_q';

valkyrie.q.right_elbow_pitch = rightElbowPitch_q';
valkyrie.q.right_shoulder_pitch = rightShoulderPitch_q';
valkyrie.q.right_shoulder_roll = rightShoulderRoll_q';
valkyrie.q.right_shoulder_yaw = rightShoulderYaw_q';

valkyrie.q.torso_pitch = torsoPitch_q';
valkyrie.q.torso_roll = torsoRoll_q';
valkyrie.q.torso_yaw = torsoYaw_q';

valkyrie.q.neck_upper_pitch = upperNeckPitch_q';
valkyrie.q.neck_lower_pitch = lowerNeckPitch_q';
valkyrie.q.neck_yaw = neckYaw_q';

%Joint Velocities
valkyrie.qd.left_ankle_pitch = leftAnklePitch_qd';
valkyrie.qd.left_ankle_roll = leftAnkleRoll_qd';
valkyrie.qd.left_knee_pitch = leftKneePitch_qd';
valkyrie.qd.left_hip_pitch = leftHipPitch_qd';
valkyrie.qd.left_hip_roll = leftHipRoll_qd';
valkyrie.qd.left_hip_yaw = leftHipYaw_qd';

valkyrie.qd.right_ankle_pitch = rightAnklePitch_qd';
valkyrie.qd.right_ankle_roll = rightAnkleRoll_qd';
valkyrie.qd.right_knee_pitch = rightKneePitch_qd';
valkyrie.qd.right_hip_pitch = rightHipPitch_qd';
valkyrie.qd.right_hip_roll = rightHipRoll_qd';
valkyrie.qd.right_hip_yaw = rightHipYaw_qd';

valkyrie.qd.left_elbow_pitch = leftElbowPitch_qd';
valkyrie.qd.left_shoulder_pitch = leftShoulderPitch_qd';
valkyrie.qd.left_shoulder_roll = leftShoulderRoll_qd';
valkyrie.qd.left_shoulder_yaw = leftShoulderYaw_qd';

valkyrie.qd.right_elbow_pitch = rightElbowPitch_qd';
valkyrie.qd.right_shoulder_pitch = rightShoulderPitch_qd';
valkyrie.qd.right_shoulder_roll = rightShoulderRoll_qd';
valkyrie.qd.right_shoulder_yaw = rightShoulderYaw_qd';

valkyrie.qd.torso_pitch = torsoPitch_qd';
valkyrie.qd.torso_roll = torsoRoll_qd';
valkyrie.qd.torso_yaw = torsoYaw_qd';

valkyrie.qd.neck_upper_pitch = upperNeckPitch_qd';
valkyrie.qd.neck_lower_pitch = lowerNeckPitch_qd';
valkyrie.qd.neck_yaw = neckYaw_qd';

%Pelvis Rear IMU Data
valkyrie.pelvis_imu.q_w = pelvisRearImu_q_w';
valkyrie.pelvis_imu.q_x = pelvisRearImu_q_x';
valkyrie.pelvis_imu.q_y = pelvisRearImu_q_y';
valkyrie.pelvis_imu.q_z = pelvisRearImu_q_z';

valkyrie.pelvis_imu.theta_x = pelvisRearImu_theta_x';
valkyrie.pelvis_imu.theta_y = pelvisRearImu_theta_y';
valkyrie.pelvis_imu.theta_z = pelvisRearImu_theta_z';

valkyrie.pelvis_imu.xdd = pelvisRearImu_xdd';
valkyrie.pelvis_imu.ydd = pelvisRearImu_ydd';
valkyrie.pelvis_imu.zdd = pelvisRearImu_zdd';

%Locate the indecies of each marker in the mocap data
idx.pelvis_back = find(trajectories.Labels==string('pelvis_back'));
idx.pelvis_left = find(trajectories.Labels==string('pelvis_left'));
idx.pelvis_right = find(trajectories.Labels==string('pelvis_right'));
idx.pelvis_front = find(trajectories.Labels==string('pelvis_front'));

idx.r_elbow = find(trajectories.Labels==string('right_elbow'));
idx.r_forearm = find(trajectories.Labels==string('right_forearm'));
idx.r_hand = find(trajectories.Labels==string('right_hand'));

idx.l_elbow = find(trajectories.Labels==string('left_elbow'));
idx.l_forearm = find(trajectories.Labels==string('left_forearm'));
idx.l_hand = find(trajectories.Labels==string('left_hand'));

idx.r_foot_rear = find(trajectories.Labels==string('right_foot_rear'));
idx.r_foot_side = find(trajectories.Labels==string('right_foot_side'));
idx.r_foot_front = find(trajectories.Labels==string('right_foot_front'));

idx.l_foot_rear = find(trajectories.Labels==string('left_foot_rear'));
idx.l_foot_side = find(trajectories.Labels==string('left_foot_side'));
idx.l_foot_front = find(trajectories.Labels==string('left_foot_front'));

idx.back_right = find(trajectories.Labels==string('back_right'));
idx.back_left = find(trajectories.Labels==string('back_left'));
idx.shoulder_right = find(trajectories.Labels==string('right_shoulder'));
idx.shoulder_left = find(trajectories.Labels==string('left_shoulder'));

%Pack the marker data into a structure, change from millimeters to meters,
%and reshape to fit

%Reduced mocap.pelvis for right_arm_grid trial due to significant occlusion of
%one marker during the trial, 3 is minimum so results are still valid
mocap.pelvis = trajectories.Data([idx.pelvis_back, idx.pelvis_left, idx.pelvis_right, idx.pelvis_front],[1:3],:);

%Box Pickup
%mocap.pelvis = trajectories.Data([idx.pelvis_back, idx.pelvis_left, idx.pelvis_right],[1:3],:);
%Right Arm Grid
%mocap.pelvis = trajectories.Data([idx.pelvis_back, idx.pelvis_right, idx.pelvis_front],[1:3],:);
mocap.pelvis = mocap.pelvis/1000;
mocap.pelvis = permute(mocap.pelvis, [2, 1, 3]);

mocap.r_arm = trajectories.Data([idx.r_elbow, idx.r_forearm, idx.r_hand],[1:3],:);
mocap.r_arm = mocap.r_arm/1000;
mocap.r_arm = permute(mocap.r_arm, [2, 1, 3]);

mocap.l_arm = trajectories.Data([idx.l_elbow, idx.l_forearm, idx.l_hand],[1:3],:);
mocap.l_arm = mocap.l_arm/1000;
mocap.l_arm = permute(mocap.l_arm, [2, 1, 3]);

mocap.r_foot = trajectories.Data([idx.r_foot_rear, idx.r_foot_side, idx.r_foot_front],[1:3],:);
mocap.r_foot = mocap.r_foot/1000;
mocap.r_foot = permute(mocap.r_foot, [2, 1, 3]);

mocap.l_foot = trajectories.Data([idx.l_foot_rear, idx.l_foot_side, idx.l_foot_front],[1:3],:);
mocap.l_foot = mocap.l_foot/1000;
mocap.l_foot = permute(mocap.l_foot, [2, 1, 3]);

mocap.torso = trajectories.Data([idx.back_right, idx.back_left, idx.shoulder_left],[1:3],:);
mocap.torso = mocap.torso/1000;
mocap.torso = permute(mocap.torso, [2, 1, 3]);

%Cleanup
clearvars -except valkyrie trajectories mocap start_time Q
