classdef q_input
    methods (Static)         
       function obj = q1_input_controll(obj)
            if obj.time<2
                obj.gamma = obj.m*obj.g/4/obj.k*ones(4,1);
            elseif 2<=obj.time && obj.time<=4
                obj.gamma = obj.m*obj.g/4/obj.k*ones(4,1)*1.2;
            elseif 4<=obj.time && obj.time<=8
                obj.gamma(1) = obj.m*obj.g/4/obj.k*1*1.2;
                obj.gamma(2) = obj.m*obj.g/4/obj.k*1*1.2;
                obj.gamma(3) = obj.m*obj.g/4/obj.k*1*1.2;
                obj.gamma(4) = 0;
            end
       end

       function obj = q3_input_controll(obj)

            if obj.state_machine == 0
                % The drone will go to the next target location if its three-axis position changes by less than 0.1.
                if (abs(obj.pos - 5*ones(3,1)) <= 0.1*ones(3,1))
                    %Status machine update
                    obj.state_machine = obj.state_machine + 1;
                else
                    %Proceed to the first target location.
                    obj.pos_d = obj.pos_3a(:,1);
                end
            elseif   obj.state_machine == 1
                    % If greater than ten seconds then go to next point
                if obj.time_change >= 10
                    %Status machine update
                    obj.state_machine = obj.state_machine + 1;
                else
                    obj.pos_d = obj.pos_3a(:,1);
                    % Timing (Stays at (5,5,5) for 10 seconds)
                    obj.time_change = obj.time_change + obj.time_interval;
                end
            elseif   obj.state_machine == 2
                if (abs(obj.pos(1) - 5) <= 0.1) && (abs(obj.pos(2) + 5) <= 0.1) && (abs(obj.pos(3) - 10) <= 0.1)
                    %Status machine update
                    obj.state_machine = obj.state_machine + 1;
                else
                    obj.pos_d = obj.pos_3a(:,2);
                end
            elseif   obj.state_machine == 3
                if (abs(obj.pos(1) + 5) <= 0.1) && (abs(obj.pos(2) + 5) <= 0.1) && (abs(obj.pos(3) - 6) <= 0.1)
                    %Status machine update
                    obj.state_machine = obj.state_machine + 1;
                else
                    obj.pos_d = obj.pos_3a(:,3);
                end
            elseif   obj.state_machine == 4
                if (abs(obj.pos(1) + 5) <= 0.1) && (abs(obj.pos(2) - 5) <= 0.1) && (abs(obj.pos(3) - 2) <= 0.1)
                    %Status machine update
                    obj.state_machine = obj.state_machine + 1;
                else
                    obj.pos_d = obj.pos_3a(:,4);
                end
            elseif   obj.state_machine == 5
                if (abs(obj.pos(1)) <= 0.1) && (abs(obj.pos(2)) <= 0.1) && (abs(obj.pos(3) - 2) <= 0.1)
                    %Status machine update
                    obj.state_machine = obj.state_machine + 1;
                else
                    obj.pos_d = obj.pos_3a(:,5);
                end
            elseif   obj.state_machine == 6
                if (abs(obj.pos(1)) <= 0.1) && (abs(obj.pos(2)) <= 0.1) && (abs(obj.pos(3)) <= 0.1)
                    %Status machine update
                    obj.state_machine = obj.state_machine + 1;
                else 
                    %When the UAV's speed is more than 0.05m/s, it hovers.
                    if obj.posdot(3) > 0.05
                        %The drone's location is kept the same as it was before.
                        obj.pos_d = obj.pos;
                    else
                        obj.pos_d = obj.pos_3a(:,6);
                    end
                end
            end
       end

       function drone_q1(drones,num_drones,spaceDim,draw_ground,ax1)
            %% draw drone
            while(drones(1).time < 9.0)
                %clear axis
                cla(ax1);
    
                %update and draw drones
                for i = 1:num_drones
                    update_q1(drones(i))
                end
    
                %optionally draw the ground image
                if(draw_ground)
                    imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
                end
    
                %apply fancy lighting (optional)
                camlight
    
                %update figure
                drawnow
                pause(0.01)
            end
            draw_func.q1_draw_func(drones(1).LogOut.pos)
        end

        function drone_q2(drones,num_drones,spaceDim,draw_ground,ax1,timelimit)
                [y,t] = drone_sys_U(drones(1), timelimit);
                q_input.drone_q1(drones,num_drones,spaceDim,draw_ground,ax1)
                draw_func.q2_draw_func(drones,y,t)
        end

        function drone_q3(drones,num_drones,spaceDim,draw_ground,ax1,noise_en)
            
            %% draw drone
             drones(1).pos = [ 0 ; 0 ; 0 ];
             drones(1).LogOut.pos = [ 0 ; 0 ; 0 ]';

            while(drones(1).time < 90)
                %clear axis
                cla(ax1);
    
                %update and draw drones
                for i = 1:num_drones
                    update_q3(drones(i),noise_en)
                end
    
                %optionally draw the ground image
                if(draw_ground)
                    imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
                end
    
                %apply fancy lighting (optional)
                camlight
    
                %update figure
                drawnow
                pause(0.01)
            end
        %%% draw_q3
        draw_func.q3_draw_func(drones(1).LogOut.time, drones(1).LogOut.pos, drones(1).LogOut.theta)
        end

    end
end