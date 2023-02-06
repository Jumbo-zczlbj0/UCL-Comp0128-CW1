%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;

%% figure properties
%Define total width, length and height of flight arena (metres)
spaceDim = 20;
spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];

%do you want to draw a ground image on the figure?
draw_ground = false;
if(draw_ground)
    ground_img = imread('ground.png');

end


%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
axis equal;
axis(spaceLimits)
grid ON
grid MINOR
caxis(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d


%% initiate a collection of drones
num_drones = 1;
timelimit = 20.0;

%instantiate a drone object, input the axis and arena limits
drones = [];
for i = 1:num_drones
    drones = [drones Drone(ax1, spaceDim, num_drones)];
end

% Q1
% q_input.drone_q1(drones,num_drones,spaceDim,draw_ground,ax1)

% Q2
% q_input.drone_q2(drones,num_drones,spaceDim,draw_ground,ax1,timelimit)               

% Q3_a
% q_input.drone_q3(drones,num_drones,spaceDim,draw_ground,ax1,false)

% Q3_b
% q_input.drone_q3(drones,num_drones,spaceDim,draw_ground,ax1,true)

% Q3_c (Run wind modul when noise_en == 3)
q_input.drone_q3(drones,num_drones,spaceDim,draw_ground,ax1,3)

