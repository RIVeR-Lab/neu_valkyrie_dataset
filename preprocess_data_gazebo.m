%Script that takes the internal robot vars as logged by the IHMC logger and
%mocap system and makes them more convenient
%USE FOR GAZEBO DATASET ONLY

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

%Downsample the robot logged data (500hz) to match the mocap data (100hz) 
valkyrie_dec.timestamp = downsample(valkyrie.timestamp',5)';
valkyrie_dec.robot_time = downsample(valkyrie.robot_time',5)';
valkyrie_dec.cop = downsample(valkyrie.cop',5)';
valkyrie_dec.com = downsample(valkyrie.com',5)';
valkyrie_dec.state_estimator = downsample(valkyrie.state_estimator',5)';
valkyrie_dec.grf = structfun(@(M) downsample(M',5)',valkyrie.grf,'UniformOutput',false);
valkyrie_dec.tau = structfun(@(M) downsample(M',5)',valkyrie.tau,'UniformOutput',false);
valkyrie_dec.q = structfun(@(M) downsample(M',5)',valkyrie.q,'UniformOutput',false);
valkyrie_dec.qd = structfun(@(M) downsample(M',5)',valkyrie.qd,'UniformOutput',false);
valkyrie_dec.pelvis_imu = structfun(@(M) downsample(M',5)',valkyrie.pelvis_imu,'UniformOutput',false);

%Cleanup
clearvars -except valkyrie valkyrie_dec