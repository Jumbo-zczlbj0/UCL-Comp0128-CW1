%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Drone class, feel free to add functionality as you see fit
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone < handle
    %% constant properties
    properties (Constant)
        
        % constants for simulation
        m = 0.2; % mass of the drone
        g = 9.2; % gravitational constant
        k = 1; % dimensioned constant i.e.K_v*K_tau/K_t for hovering state(pg.3)
        kd = [0.1;0.1;0.1];% three separate friction constants
        I = [1, 0, 0;
            0, 1, 0;
            0, 0, 0.5];% diagonal inertia matrix
        L = 0.2; % distance from the center of the quadcopter to any of the propellers
        b = 0.1; % drag coeff

        % width, length, height offset between centre and rotors
        body = [0.6 0.6 0.0];
        
        % time interval for simulation (seconds)
        time_interval = 0.02;% 0.02
        
        % size of floating window that follows drone
        axis_size = 2.;
        
        % colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];
        
        % Follows the drone within the figure
        % Don't use if you are simulating more than one drone!
        % Switch to false to see the overall world view
        drone_follow = false;
    end
    
    %% interchanging properties
    properties
        % axis to draw on
        axis
        
        % length of one side of the flight arena
        spaceDim
        
        % limits of flight arena
        spaceLimits
        
        % drone position
        pos
        
        % drone rotation matrix
        R
        
        % Simulation time
        time
        
        % input(4)=0
        v;
        
        % parameter to start drone in random position
        pos_offset
        
        % number of drones
        num_drones
        
        % xdot
        posdot
        
        % theta is the eular angles of the drone
        theta% rollx pitchy yawz

        % thetadot
        thetadot
        
        % input
        gamma
        
        % for disturbance generation
        deviation

        % continuous system
        cs

        % discrete system
        ds

        % summary of states
        X

        % Log output
        LogOut

        % LTI system Log
        LogSys

        % for PID
        pos_3a %destination
        
        % Sum U and torques 
        U_out

        % desired reference phi theta psi
        theta_d

        % destination
        pos_d

        % state_machine
        state_machine

        % Timing (Stays at (5,5,5) for 10 seconds)
        time_change

        % wind speed
        windspeed

        % for PID position
        pid1 = struct( ...
            'kd', 0, ...
            'kp', 0, ...
            'ki', 0, ...
            'ed', 0, ...
            'ei', 0, ...
            'e', 0, ...
            'eprev', 0, ...
            'out', 0)

        % for PID angular controller
        pid2 = struct( ...
            'kd', 0, ...
            'kp', 0, ...
            'ki', 0, ...
            'ed', 0, ...
            'ei', 0, ...
            'e', 0, ...
            'eprev', 0, ...
            'out', 0, ...
            'log', struct([]));

        sysRan = false;
    end
      
    %% methods
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone(axis, spaceDim, num_drones)
            if nargin > 1
                obj.axis = axis;
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                
                obj.pos = [0;0;5];
                
                obj.pos_offset = [5.*(rand - 0.5),5.*(rand - 0.5),2.5.*(rand)];
                
                obj.R = eul2rotm(deg2rad([200, 200, 300]));%[1,0,0;0,1,0;0,0,1];
                
                obj.time = 0;
                
                obj.num_drones = num_drones;
                
                obj.posdot = [0;0;0];
                
                % obj.gamma = [0.460; 0.3; 0.460; 0.320]; 
                obj.gamma = obj.m*obj.g/4/obj.k*ones(4,1);%0.460;
                % obj.gamma = zeros(4,1);

                obj.deviation = 100;
                
                %obj.thetadot = deg2rad(2 * obj.deviation * rand(3, 1) - obj.deviation); 
                obj.theta = zeros(3,1);

                % thetadot
                obj.thetadot = zeros(3,1);

                % obj.U_out: Sum U + torques
                obj.U_out = zeros(4,1);

                % theta is the eular angles of the drone
                obj.theta_d = [0 ; 0 ; 1.2246e-16];

                % destination
                obj.pos_d = [0;0;0];

                % state_machine
                obj.state_machine = 0;

                % Timing (Stays at (5,5,5) for 10 seconds)
                obj.time_change = 0;

                % wind speed
                obj.windspeed = [0;0;0];
                
                obj.LogOut = struct( ...
                    'pos',      obj.pos', ...
                    'posdot',   obj.posdot', ...
                    'R',        obj.R,...
                    'theta',    obj.theta', ...
                    'thetadot', obj.thetadot', ...
                    'gamma',    obj.gamma', ...
                    'time',     obj.time');

                %%%%%%%%%%%%%%%%%%%%%%%% PID1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                obj.pid1.kd =  0.08;

                obj.pid1.kp = 0.11;

                obj.pid1.ki = 0.0004;

                obj.pid1.ed = [0;0;0];

                obj.pid1.ei = [0;0;0];

                % previous error 
                obj.pid1.eprev = [0;0;0];  

                obj.pid1.e = [0;0;0];
                
                % PID1 output
                obj.pid1.out = [0;0;0];

                % 3_a destination
                obj.pos_3a = [5 5 -5 -5 0 0;
                              5 -5 -5 5 0 0;
                              5 10  6 2 2 0];

                %%%%%%%%%%%%%%%%%%%%%%%%% PID2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                obj.pid2.kd = 1.7;

                obj.pid2.kp = 0.38;

                obj.pid2.ki = 0.001;

                obj.pid2.ei = [0;0;0];

                obj.pid2.ed = [0;0;0];
                
                % PID2 output
                obj.pid2.out = [0;0;0];

                obj.pid2.e = [0;0;0];

                obj.pid2.eprev = [0;0;0];

                obj.pid2.log = struct(...
                    'e', [],...
                    'ei', [],...
                    'ed', [],...
                    'out', []);
                
            else
                error('Drone not initialised correctly')
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SIMULATION FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function update_q1(obj)
            %update simulation time
            obj.time = obj.time + obj.time_interval;
            obj = q_input.q1_input_controll(obj);
            
            obj = q1_simulation.simulation(obj, true);

            %draw drone on figure
            draw_func.draw(obj);
            obj = draw_func.logOut(obj);
            plot3( obj.LogOut.pos(:,1),obj.LogOut.pos(:,2),obj.LogOut.pos(:,3),...
                "Color",'m')%obj.axis,
            
%             draw_func.plot_dir(obj)

            %LTI simulator
%            obj.LogSys = sys(obj);
%             n = obj.time/obj.time_interval;
%             plot3(obj.LogSys(1:n,1),obj.LogSys(1:n,2),obj.LogSys(1:n,3));
        end

        function update_q3(obj,noise_en)
            % update simulation time
            obj.time = obj.time + obj.time_interval; 
            % set (false) to true to enable function
            obj = q_input.q3_input_controll(obj);

            %[obj.posdot, obj.windspeed] = wind_model(obj.windspeed, obj.pos, obj.posdot, obj.time_interval);
            % change position and orientation of drone
            % obj = change_pos_and_orientation(obj);
            obj = q1_simulation.simulation(obj, true);

            if noise_en == 3
              % add wind to the model
              [obj.posdot,obj.windspeed] = wind_model(obj.windspeed,obj.pos, obj.posdot, obj.time_interval);
            end
            
			%  PID controller (include: pos_controller & angular_controller)
            % q3_b if pid_controller == true,, add gaussian noise to the sensor
            obj = Pid_controller.pid_controller(obj,noise_en);
            % obj = Pid_controller.pid_controller(obj,false);

            % draw drone on figure
            scatter3(obj.pos_d(1), obj.pos_d(2), obj.pos_d(3),'Marker','o','MarkerEdgeColor','r')

            plot(obj.time, obj.theta);

                             
            draw_func.draw(obj);
            obj = draw_func.logOut(obj);
            plot3( obj.LogOut.pos(:,1),obj.LogOut.pos(:,2),obj.LogOut.pos(:,3),...
                         "Color",'m')% obj.axis,
            
        end
    end
end
