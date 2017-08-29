%Script that uses the Kabsch algorithm to match tracked markers to
%appendage frames and absor.m to align robot world frame and motion capture
%world frame. In addition, it temporally aligns the internal robot data and
%motion capture data, and provides a decimated version of the robot data at
%100Hz to match the motion capture data.

% Instructions for use:
% Prereqs: preprocess_data.m has been run
%
% 1) Run process_data.m


%Kabsch Algorithm to calculate the motion of the origin frame for all the
%tracked motion capture markers
%Try/catch blocks take care of times when a marker is occluded resulting in
%the SVD calculation failing
%If statement block off the code from running if those markers are not
%present in a specific trial/run (pelvis markers are always used)

for i = 1:length(mocap.pelvis(1,1,:))
    try
        [gt.U.pelvis(:,:,i), gt.r.pelvis(:,1,i), gt.lrms.pelvis(i)] = Kabsch(Q.pelvis,mocap.pelvis(:,:,i));
    catch
        gt.U.pelvis(:,:,i) = zeros(3); gt.r.pelvis(:,1,i) = zeros(3,1); gt.lrms.pelvis(i) = 0;
    end
end

if ~isempty(mocap.r_arm)
    for i = 1:length(mocap.r_arm(1,1,:))
        try
            [gt.U.r_arm(:,:,i), gt.r.r_arm(:,1,i), gt.lrms.r_arm(i)] = Kabsch(Q.r_arm,mocap.r_arm(:,:,i));
        catch
            gt.U.r_arm(:,:,i) = zeros(3); gt.r.r_arm(:,1,i) = zeros(3,1); gt.lrms.r_arm(i) = 0;
        end
    end
end

if ~isempty(mocap.l_arm)
    for i = 1:length(mocap.l_arm(1,1,:))
        try
            [gt.U.l_arm(:,:,i), gt.r.l_arm(:,1,i), gt.lrms.l_arm(i)] = Kabsch(Q.l_arm,mocap.l_arm(:,:,i));
        catch
            gt.U.l_arm(:,:,i) = zeros(3); gt.r.l_arm(:,1,i) = zeros(3,1); gt.lrms.l_arm(i) = 0;
        end
    end
end

if ~isempty(mocap.r_foot)
    for i = 1:length(mocap.r_foot(1,1,:))
        try
            [gt.U.r_foot(:,:,i), gt.r.r_foot(:,1,i), gt.lrms.r_foot(i)] = Kabsch(Q.r_foot,mocap.r_foot(:,:,i));
        catch
            gt.U.r_foot(:,:,i) = zeros(3); gt.r.r_foot(:,1,i) = zeros(3,1); gt.lrms.r_foot(i) = 0;
        end
    end
end

if ~isempty(mocap.l_foot)
    for i = 1:length(mocap.l_foot(1,1,:))
        try
            [gt.U.l_foot(:,:,i), gt.r.l_foot(:,1,i), gt.lrms.l_foot(i)] = Kabsch(Q.l_foot,mocap.l_foot(:,:,i));
        catch
            gt.U.l_foot(:,:,i) = zeros(3); gt.r.l_foot(:,1,i) = zeros(3,1); gt.lrms.l_foot(i) = 0;
        end
    end
end

if ~isempty(mocap.torso)
    for i = 1:length(mocap.torso(1,1,:))
        try
            [gt.U.torso(:,:,i), gt.r.torso(:,1,i), gt.lrms.torso(i)] = Kabsch(Q.torso,mocap.torso(:,:,i));
        catch
            gt.U.torso(:,:,i) = zeros(3); gt.r.torso(:,1,i) = zeros(3,1); gt.lrms.torso(i) = 0;
        end
    end
end

%Crop the internal robot logged data to the timespan of the motion capture
%data, internal data is at 500Hz and mocap at 100Hz hence the factor of 5
%difference in length
[~,~,bin] = histcounts(valkyrie.timestamp, [0 start_time]);
start_idx = find(bin==0,1);
%-5 to knock off the last entry from the decimate later which is extra
end_idx = start_idx+5*i-5;

valkyrie.timestamp = valkyrie.timestamp(:,[start_idx:end_idx]);
valkyrie.robot_time = valkyrie.robot_time(:,[start_idx:end_idx]);
valkyrie.cop = valkyrie.cop(:,[start_idx:end_idx]);
valkyrie.com = valkyrie.com(:,[start_idx:end_idx]);
valkyrie.state_estimator = valkyrie.state_estimator(:,[start_idx:end_idx]);
valkyrie.grf = structfun(@(M) M(:,[start_idx:end_idx]),valkyrie.grf,'UniformOutput',false);
valkyrie.tau = structfun(@(M) M(:,[start_idx:end_idx]),valkyrie.tau,'UniformOutput',false);
valkyrie.q = structfun(@(M) M(:,[start_idx:end_idx]),valkyrie.q,'UniformOutput',false);
valkyrie.qd = structfun(@(M) M(:,[start_idx:end_idx]),valkyrie.qd,'UniformOutput',false);
valkyrie.pelvis_imu = structfun(@(M) M(:,[start_idx:end_idx]),valkyrie.pelvis_imu,'UniformOutput',false);

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


%Calculate the rotation between the internal robot world frame and mocap
%ground turth world frame using the absor function
[regParams,Bfit,ErrorStats]=absor(valkyrie_dec.state_estimator,squeeze(gt.r.pelvis));

%Check if the first pelvis element is zero and reset it to first valid
%value if the markers were occluded so the global offsets in the next step
%work
if ~any(gt.r.pelvis(:,1,1))
    nonzero_idx = find(gt.r.pelvis(1,1,:),1);
    gt.r.pelvis(:,1,1) = gt.r.pelvis(:,1,nonzero_idx);    
end

%Apply optimal rotation to align robot world frame to mocap frame. Also
%remove translational offset between the two frames at the beginning of the
%trial for all vector quantities
valkyrie.cop = regParams.R([1:2],[1:2])*(valkyrie.cop - valkyrie.cop(:,1)) + gt.r.pelvis([1:2],1,1);
valkyrie_dec.cop = regParams.R([1:2],[1:2])*(valkyrie_dec.cop - valkyrie_dec.cop(:,1)) + gt.r.pelvis([1:2],1,1);

valkyrie.com = regParams.R*(valkyrie.com - valkyrie.com(:,1)) + gt.r.pelvis(:,1,1);
valkyrie_dec.com = regParams.R*(valkyrie_dec.com - valkyrie_dec.com(:,1)) + gt.r.pelvis(:,1,1);

valkyrie.state_estimator = regParams.R*(valkyrie.state_estimator - valkyrie.state_estimator(:,1)) + gt.r.pelvis(:,1,1);
valkyrie_dec.state_estimator = regParams.R*(valkyrie_dec.state_estimator - valkyrie_dec.state_estimator(:,1)) + gt.r.pelvis(:,1,1);

valkyrie.grf.left_foot_force = regParams.R*(valkyrie.grf.left_foot_force - valkyrie.grf.left_foot_force(:,1)) + gt.r.pelvis(:,1,1);
valkyrie_dec.grf.left_foot_force = regParams.R*(valkyrie_dec.grf.left_foot_force - valkyrie_dec.grf.left_foot_force(:,1)) + gt.r.pelvis(:,1,1);
valkyrie.grf.right_foot_force = regParams.R*(valkyrie.grf.right_foot_force - valkyrie.grf.right_foot_force(:,1)) + gt.r.pelvis(:,1,1);
valkyrie_dec.grf.right_foot_force = regParams.R*(valkyrie_dec.grf.right_foot_force - valkyrie_dec.grf.right_foot_force(:,1)) + gt.r.pelvis(:,1,1);

valkyrie.grf.left_foot_torque = regParams.R*(valkyrie.grf.left_foot_torque - valkyrie.grf.left_foot_torque(:,1)) + gt.r.pelvis(:,1,1);
valkyrie_dec.grf.left_foot_torque = regParams.R*(valkyrie_dec.grf.left_foot_torque - valkyrie_dec.grf.left_foot_torque(:,1)) + gt.r.pelvis(:,1,1);
valkyrie.grf.right_foot_torque = regParams.R*(valkyrie.grf.right_foot_torque - valkyrie.grf.right_foot_torque(:,1)) + gt.r.pelvis(:,1,1);
valkyrie_dec.grf.right_foot_torque = regParams.R*(valkyrie_dec.grf.right_foot_torque - valkyrie_dec.grf.right_foot_torque(:,1)) + gt.r.pelvis(:,1,1);

%Cleanup
clear bin idx trajectories i Bfit ErrorStats start_idx end_idx

%Plot for sanity check
plot3(squeeze(gt.r.pelvis(1,1,:)), squeeze(gt.r.pelvis(2,1,:)), squeeze(gt.r.pelvis(3,1,:)));
hold on
plot3(valkyrie_dec.state_estimator(1,:), valkyrie_dec.state_estimator(2,:), valkyrie_dec.state_estimator(3,:));

daspect([1 1 1])