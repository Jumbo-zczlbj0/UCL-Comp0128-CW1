classdef Pid_controller
    methods (Static) 
       function obj = angular_controller(obj)
            % angular PID controller
            obj.pid2.e = obj.theta_d - obj.theta;
            obj.pid2.ed = (obj.pid2.e - obj.pid2.eprev)/obj.time_interval;
            obj.pid2.ei = obj.pid2.ei + obj.pid2.e*obj.time_interval;
            obj.pid2.out = (obj.pid2.kd * obj.pid2.ed) + (obj.pid2.ki * obj.pid2.ei) + (obj.pid2.kp * obj.pid2.e) ;
            obj.pid2.eprev = obj.pid2.e;
            
            % torques
            obj.U_out(2) = obj.pid2.out(1);
            obj.U_out(3) = obj.pid2.out(2);
            obj.U_out(4) = obj.pid2.out(3);
          
            % U_matrix: Coefficient of gamma
            U_matrix = [obj.k obj.k obj.k obj.k;
                       obj.L*obj.k 0 -obj.L*obj.k 0
                       0 obj.L*obj.k 0 -obj.L*obj.k;
                       obj.b -obj.b obj.b -obj.b ];

            % obj.U_out: Sum U + torques
            % Solve linear system of equations, 
            obj.gamma = linsolve(U_matrix,obj.U_out);
        end

        function obj = pid_controller(obj, noise_en) 
            
            if noise_en == true
                 % add gaussian noise to the sensor
                 obj.pos = awgn(obj.pos, 100);
                 obj.theta = awgn(obj.theta, 100);
            end

            % position PID controller 
            obj.pid1.e = obj.pos_d - obj.pos;
            obj.pid1.ed = (obj.pid1.e - obj.pid1.eprev)/obj.time_interval;
            obj.pid1.ei = obj.pid1.ei + obj.pid1.e*obj.time_interval;
            obj.pid1.out = (obj.pid1.kd * obj.pid1.ed) + (obj.pid1.ki * obj.pid1.ei) + (obj.pid1.kp * obj.pid1.e) ;
            obj.pid1.eprev = obj.pid1.e;

            % Inverse solution
            obj.U_out(1) = obj.m*(sqrt(obj.pid1.out(1)^2+obj.pid1.out(2)^2+(obj.pid1.out(3)+obj.g)^2));
            obj.theta_d(1) = asin((obj.m/obj.U_out(1))*(obj.pid1.out(1)*sin(obj.theta_d(3)) - obj.pid1.out(2)*cos(obj.theta_d(3)))); 
            obj.theta_d(2) = asin((obj.m*obj.pid1.out(1)-obj.U_out(1)*sin(obj.theta_d(1))*sin(obj.theta_d(3)))/(obj.U_out(1)*cos(obj.theta_d(1))*cos(obj.theta_d(3)))); 
            % angular PID controller
            obj = Pid_controller.angular_controller(obj);
        end
    end
end