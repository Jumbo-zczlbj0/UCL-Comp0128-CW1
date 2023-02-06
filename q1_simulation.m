classdef q1_simulation
    methods (Static) 
    function obj = simulation(obj, enable)
            % exit if function is disabled
            if enable == false
                return;
            end

            % temporary curernt input: u is the user_defined current input
            i = obj.gamma;
            
            omega = q1_simulation.thetadot2omega(obj.thetadot, obj.theta);
            
            % Compute linear and angular accelerations.
            a = q1_simulation.acceleration(i, obj.theta, obj.posdot, ...
                obj.m, obj.g, obj.k, obj.kd);
            omegadot = q1_simulation.angular_acceleration(i, omega, ...
                obj.I, obj.L, obj.b, obj.k);

            % Update states theta, xdot, x
            omega = omega + obj.time_interval * omegadot;

            obj.thetadot = q1_simulation.omega2thetadot(omega, obj.theta);
            
            % Log theta
            obj.theta = obj.theta + obj.time_interval * obj.thetadot;
            
            % Log pos
            obj.posdot = obj.posdot + obj.time_interval * a;
            obj.pos = obj.pos + obj.time_interval * obj.posdot;
            
            % Log theta to R for plotting
            obj.R = eul2rotm([obj.theta(1), obj.theta(3), obj.theta(2)]);
            % z coordinate cannot be smaller than 0, the drone stops
            % rotating at ground
            if obj.pos(3)<=0
                obj.posdot = zeros(3,1);
                obj.pos(3) = 0;
                obj.thetadot = zeros(3,1);
            end
       end

        %% Linear acceleration
        function T = thrust(gamma, k)
            % Inputs are values for wi^2
            T = [0; 0; k * sum(gamma)];
        end

        function a = acceleration(gamma, theta, xdot, m, g, k, kd)
            % Args:
            %   gamma: 4by1 individual square of motor angular
            %       acceleration. i.e. gammai = wi^2
            %       (1,3 clockwise)
            %   theta: 3by1 theta, angle in inertial frame [yaw pitch roll]
            %   xdot: 3by1 differentiated state
            %   m: single mass of the drone
            %   k: single dimensioned constant
            %   kd: 3by1 friction constant
            % Return: 
            %   a: 3by1 the linear acceleration in inertial frame
            gravity = [0; 0; -g];
            %R = eul2rotm([theta(3),theta(2),theta(1)],"XYZ");%bc x is in [x,y,z]
            %R = eul2rotm(theta',"XYZ");
            R = q1_simulation.rotation(theta);
            T = R * q1_simulation.thrust(gamma, k);
            Fd = -kd .* xdot;
            a = gravity + 1/m*T + 1/m*Fd;
        end
        %% Angular acceleration
        function tau = torques(input, L, b, k)
            % Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
            % Inputs are values for wi^2
            % Outputs are [x y z]
            tau = [
                L * k * (input(1) - input(3))%roll
                L * k * (input(2) - input(4))%pitch
                b * (input(1) - input(2) + input(3) - input(4))%yaw
                ];
        end        
        
        function omegadot = angular_acceleration(gamma, omega, I, L, b, k)
            % args:
            %   gamma: 4by1 individual square of motor angular
            %       acceleration. i.e. gammai = wi^2
            %       (1,3 clockwise)
            %   omega: 3by1 angular velocity in body frame
            %   I: 3by3 diagonal inertia matrix
            %   L: length of the arm of the drone (from centre to the motor)
            %   b: drag coeff
            %   k: dimensioned constant
            % Return:
            %   omegadot: time derivative of omega[yaw; pitch; roll] 
            tau = q1_simulation.torques(gamma, L, b, k);% compute the torque
            omegadot = inv(I) * (tau - cross(omega, I * omega));
        end
        
        %% angle conversion
        function thetadot = omega2thetadot(omega, theta)
            % Args: 
            %     omega: 3-by-1 angular velocity omega
            %     theta: 3-by-1 angular velocity in inertial frame
            % Return: 
            %     thetadot: 3-by-1 angular acceleration in inertial frame
            assert(isequal(size(theta),[3,1]),"dimension of theta is not 3 by 1")
            assert(isequal(size(omega),[3,1]),"dimension of omega is not 3 by 1")
            
            conv = [1,	0,              -sin(theta(2));
                0,	cos(theta(1)),	cos(theta(2))*sin(theta(1));
                0,	-sin(theta(1)), cos(theta(2))*cos(theta(1))];
            thetadot = inv(conv)*omega;
        end


        function omega = thetadot2omega(thetadot, theta)
            % Args:
            %     thetadot: time derivatives of theta[yaw, pitch, roll]
            %     theta: [yaw, pitch, roll] in inertial frame
            % Return:
            %     omega: the angular velocities in body frame
            assert(isequal(size(theta),[3,1]),"dimension of theta is not 3 by 1")
            assert(isequal(size(thetadot),[3,1]),"dimension of thetadot is not 3 by 1")
            
            conv = [1,	0,              -sin(theta(2));
                    0,	cos(theta(1)),	cos(theta(2))*sin(theta(1));
                    0,	-sin(theta(1)), cos(theta(2))*cos(theta(1))];
            omega = conv*thetadot;
        end
        
        %% rotation
        function angle_inertial = rotation(angle_body)
            % Input 
            %   angle_body： 3-by-1 angle_body for angles in body frame
            %       [yaw, pitch, roll]
            % Output
            %   angle_inertial： 3-by-3 vtr_inertial angles in inertial frame
            roll = angle_body(1);% X axis
            pitch = angle_body(2);% Y axis
            yaw = angle_body(3);% Z axis
            % rotation matrix
            Rx = [1     0           0;
                0       cos(roll)    -sin(roll);
                0       sin(roll)    cos(roll)];

            Ry = [cos(pitch) 0          sin(pitch);
                    0        1          0;
                -sin(pitch)  0          cos(pitch)];

            Rz = [cos(yaw) -sin(yaw)    0;
                sin(yaw)    cos(yaw)    0;
                0           0           1];

            angle_inertial = Rz*Ry*Rx;
        end
        
    end
end    