% plot_data_animate is a script that visualizes the ground truth data and
% state estimator data in a 3D plot. In addition, it plots the raw marker
% positions that tracked the appendages. The plot is animated and rotated
% give a better sense of the motion of the robot.

% Instructions for use:
% 1) Download the dataset
% 2) Import one of the Matlab data files into your workspace corresponding
% to the trial you wish to view
% 3) Run plot_data_animate from the command line or the "Run" button

% The rotating view is modified from CaptureFigVid.m created by Alan
% Jennings under the following license:
%
% Copyright (c) 2013, Alan Jennings 
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without 
% modification, are permitted provided that the following conditions are 
% met:
% 
% * Redistributions of source code must retain the above copyright 
% notice, this list of conditions and the following disclaimer. 
% * Redistributions in binary form must reproduce the above copyright 
% notice, this list of conditions and the following disclaimer in 
% the documentation and/or other materials provided with the distribution
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
% POSSIBILITY OF SUCH DAMAGE.
%

% NOTE: This requires a relatively powerful computer to visualize at a
% reasonable speed, play with speed parameter which skips over data to play
% faster
speed = 5;

%Clear any existing figures and query the colors Matlab uses for graphs
colors = get(gca, 'colororder');
alpha = 0.5;

%Setup the animated lines for the ground truth
a = animatedline('Color',[colors(1,:), alpha],'LineWidth',2);
b = animatedline('Color',[colors(2,:), alpha],'LineWidth',2);
c = animatedline('Color',[colors(3,:), alpha],'LineWidth',2);
d = animatedline('Color',[colors(4,:), alpha],'LineWidth',2);
e = animatedline('Color',[colors(5,:), alpha],'LineWidth',2);
f = animatedline('Color',[colors(6,:), alpha],'LineWidth',2);
g = animatedline('Color',[colors(7,:), alpha],'LineWidth',2);

%Setup the animated lines for the markers
%Use the same colors as above to match ground truth data
%Set the number of maximum points to plot so only the marker is shown and
%not a "trail" of the marker history
bb = animatedline('Color',colors(2,:),'LineStyle','none','Marker','.', 'MarkerSize', 16,'MaximumNumPoints',4);
cc = animatedline('Color',colors(3,:),'LineStyle','none','Marker','.', 'MarkerSize', 16,'MaximumNumPoints',3);
dd = animatedline('Color',colors(4,:),'LineStyle','none','Marker','.', 'MarkerSize', 16,'MaximumNumPoints',3);
ee = animatedline('Color',colors(5,:),'LineStyle','none','Marker','.', 'MarkerSize', 16,'MaximumNumPoints',3);
ff = animatedline('Color',colors(6,:),'LineStyle','none','Marker','.', 'MarkerSize', 16,'MaximumNumPoints',3);
gg = animatedline('Color',colors(7,:),'LineStyle','none','Marker','.', 'MarkerSize', 16,'MaximumNumPoints',3);

%Manually set the axes and square aspect ratio
axis([-1 1 -1.5 1.5 0 1.25]);
daspect([1 1 1]);

%White background, text bigger, normalize units
set(gcf,'color','white')
set(gca,'FontSize',16)
set(gcf,'units','normalized','outerposition',[0 0 1 1])

%Legend and Label
legend('State Estimator','Pelvis', 'Right Arm', 'Left Arm', 'Right Foot', 'Left Foot', 'Torso')
ylabel('Meters')

%Setup to support the rotation view
%Copied from CaptureFigVid.m, see copyright notice above
ViewZ = [-20,10;-110,10;-190,80;-290,10;-380,10];
temp_n=round(length(valkyrie_dec.state_estimator(1,:)/speed)); % number frames
temp_p=(temp_n-1)/(size(ViewZ,1)-1); % length of each interval
ViewZ_new=zeros(temp_n,2);
% space view angles, if needed
for inis=1:(size(ViewZ,1)-1)
    ViewZ_new(round(temp_p*(inis-1)+1):round(temp_p*inis+1),:)=...
        [linspace(ViewZ(inis,1),ViewZ(inis+1,1),...
         round(temp_p*inis)-round(temp_p*(inis-1))+1).',...
         linspace(ViewZ(inis,2),ViewZ(inis+1,2),...
         round(temp_p*inis)-round(temp_p*(inis-1))+1).'];
end
ViewZ=ViewZ_new;
ViewZ=ViewZ(1:(end-1),:);

%Loop that cycles through the views as controlled by speed param
for k = 1:speed:length(valkyrie_dec.state_estimator(1,:))  
    
    %Change the view
    view(ViewZ(k,:));
    
    %Plot the ground truth
    addpoints(a,valkyrie_dec.state_estimator(1,k),valkyrie_dec.state_estimator(2,k),valkyrie_dec.state_estimator(3,k));
    addpoints(b,gt.r.pelvis(1,k),gt.r.pelvis(2,k),gt.r.pelvis(3,k));
    
    addpoints(c,gt.r.r_arm(1,k), gt.r.r_arm(2,k), gt.r.r_arm(3,k));
    addpoints(d,gt.r.l_arm(1,k), gt.r.l_arm(2,k), gt.r.l_arm(3,k));
    addpoints(e,gt.r.r_foot(1,k), gt.r.r_foot(2,k), gt.r.r_foot(3,k));
    addpoints(f,gt.r.l_foot(1,k), gt.r.l_foot(2,k), gt.r.l_foot(3,k));
    addpoints(g,gt.r.torso(1,k), gt.r.torso(2,k), gt.r.torso(3,k));
    
    %Plot the markers
    addpoints(bb,mocap.pelvis(1,:,k),mocap.pelvis(2,:,k),mocap.pelvis(3,:,k));
    addpoints(cc,mocap.r_arm(1,:,k),mocap.r_arm(2,:,k),mocap.r_arm(3,:,k));
    addpoints(dd,mocap.l_arm(1,:,k),mocap.l_arm(2,:,k),mocap.l_arm(3,:,k));
    addpoints(ee,mocap.r_foot(1,:,k),mocap.r_foot(2,:,k),mocap.r_foot(3,:,k));
    addpoints(ff,mocap.l_foot(1,:,k),mocap.l_foot(2,:,k),mocap.l_foot(3,:,k));
    addpoints(gg,mocap.torso(1,:,k),mocap.torso(2,:,k),mocap.torso(3,:,k));
    
    %Draw the animation
    drawnow
end
