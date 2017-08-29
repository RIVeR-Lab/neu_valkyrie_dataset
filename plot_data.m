% plot_data is a script that visualizes the ground truth data and state
% estimator data in a 3D plot from the Northeastern's NASA Valkyrie
% Humanoid Robot Dataset

% Instructions for use:
% 1) Download the dataset
% 2) Import one of the Matlab data files into your workspace corresponding
% to the trial you wish to view
% 3) Run plot_data from the command line or the "Run" button

%Clear any existing figures and query the colors Matlab uses for graphs
clf
colors = get(gca, 'colororder');

%Plot the pelvis ground truth and state estimator
plot3(squeeze(gt.r.pelvis(1,1,:)), squeeze(gt.r.pelvis(2,1,:)), squeeze(gt.r.pelvis(3,1,:)), 'Color',colors(1,:));
hold on
plot3(valkyrie_dec.state_estimator(1,:), valkyrie_dec.state_estimator(2,:), valkyrie_dec.state_estimator(3,:), 'Color',colors(2,:));

%Plot the other tracked appendages
%Use try since markers don't exist in every set for every joint
%Squeeze removes the singlar columns to fit plot3's requirements
try plot3(squeeze(gt.r.r_arm(1,1,:)), squeeze(gt.r.r_arm(2,1,:)), squeeze(gt.r.r_arm(3,1,:)),  'Color',colors(3,:)); end;
try plot3(squeeze(gt.r.l_arm(1,1,:)), squeeze(gt.r.l_arm(2,1,:)), squeeze(gt.r.l_arm(3,1,:)),       'Color',colors(4,:)); end;
try plot3(squeeze(gt.r.r_foot(1,1,:)), squeeze(gt.r.r_foot(2,1,:)), squeeze(gt.r.r_foot(3,1,:)),    'Color',colors(5,:)); end;
try plot3(squeeze(gt.r.l_foot(1,1,:)), squeeze(gt.r.l_foot(2,1,:)), squeeze(gt.r.l_foot(3,1,:)),    'Color',colors(6,:)); end;
try plot3(squeeze(gt.r.torso(1,1,:)), squeeze(gt.r.torso(2,1,:)), squeeze(gt.r.torso(3,1,:)),      'Color',colors(7,:)); end;

%Insert legend
legend('State Estimator','Pelvis', 'Right Arm', 'Left Arm', 'Right Foot', 'Left Foot', 'Torso');

%Make the title and ylabel
title('Forward Walking Trial')
ylabel('Meters')

%Set background to white and make the aspect ratio square
set(gcf,'color','white')
daspect([1 1 1])

