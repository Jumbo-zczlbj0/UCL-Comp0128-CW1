classdef draw_func
    methods (Static) 
        function q1_draw_func(pos)
            %%%% draw trajectories
            figure
            plot3(pos(:,1),pos(:,2),pos(:,3),'LineWidth',2)
            grid on
            hold on
            xlabel('X/m')
            ylabel('Y/m')
            zlabel('Z/m')
        end

        function q2_draw_func(drones,y,t)
            close all;
            for i = 2:height(drones(1).LogOut.pos)
                if drones(1).LogOut.pos(i,3)==0 && drones(1).LogOut.pos(i-1,3)~=0
                    break;
                end
            end
            
            t1 = t(1:i);
            l = 1:length(t1);
            figure('Name','position')
            subplot(321),plot(t1,y(l,1)),title('LTI simylation: X direction of the drone against time')
            xlabel('X/m'),ylabel('time/s')
            subplot(323),plot(t1,y(l,2)),title('LTI simylation: Y direction of the drone against time')
            xlabel('Y/m'),ylabel('time/s')
            subplot(325),plot(t1,y(l,3)),title('LTI simylation: Z direction of the drone against time')
            xlabel('Z/m'),ylabel('time/s')
            subplot(322),plot(t1, drones(1).LogOut.pos(l,1)),title('Dynamic simulation: X direction of the drone against time')
            xlabel('X/m'),ylabel('time/s')
            subplot(324),plot(t1, drones(1).LogOut.pos(l,2)),title('Dynamic simulation: Y direction of the drone against time')
            xlabel('Y/m'),ylabel('time/s')
            subplot(326),plot(t1, drones(1).LogOut.pos(l,3)),title('Dynamic simulation: Z direction of the drone against time')
            xlabel('Z/m'),ylabel('time/s')
            
            figure('Name','velocity')
            subplot(321),plot(t1,y(l,4)),title('LTI simylation: X velocity direction of the drone against time')
            xlabel('velocity X(m/s)'),ylabel('time/s')
            subplot(323),plot(t1,y(l,5)),title('LTI simylation: Y velocity direction of the drone against time')
            xlabel('velocity Y(m/s)'),ylabel('time/s')
            subplot(325),plot(t1,y(l,6)),title('LTI simylation: Z velocity direction of the drone against time')
            xlabel('velocity Z(m/s)'),ylabel('time/s')
            subplot(322),plot(t1, drones(1).LogOut.posdot(l,1)),title('Dynamic simulation: X velocity direction of the drone against time')
            xlabel('velocity X(m/s)'),ylabel('time/s')
            subplot(324),plot(t1, drones(1).LogOut.posdot(l,2)),title('Dynamic simulation: Y velocity direction of the drone against time')
            xlabel('velocity Y(m/s)'),ylabel('time/s')
            subplot(326),plot(t1, drones(1).LogOut.posdot(l,3)),title('Dynamic simulation: Z velocity direction of the drone against time')
            xlabel('velocity Z(m/s)'),ylabel('time/s')
            
            figure("Name",'angles')
            subplot(321),plot(t1,y(l,7)),title('LTI simylation: roll of the drone against time')
            xlabel('roll/rad'),ylabel('time/s')
            subplot(323),plot(t1,y(l,8)),title('LTI simylation: pitch of the drone against time')
            xlabel('pitch/rad'),ylabel('time/s')
            subplot(325),plot(t1,y(l,9)),title('LTI simylation: yaw of the drone against time')
            xlabel('yaw/rad'),ylabel('time/s')
            subplot(322),plot(t1, drones(1).LogOut.theta(l,1)),title('Dynamic simulation: roll of the drone against time')
            xlabel('roll/rad'),ylabel('time/s')
            subplot(324),plot(t1, drones(1).LogOut.theta(l,2)),title('Dynamic simulation: pitch of the drone against time')
            xlabel('pitch/rad'),ylabel('time/s')
            subplot(326),plot(t1, drones(1).LogOut.theta(l,3)),title('Dynamic simulation: yaw of the drone against time')
            xlabel('yaw/rad'),ylabel('time/s')
            
            
            figure("Name",'\omega')
            subplot(321),plot(t1,y(l,10)),title('LTI simylation: \omega_{roll} of the drone against time')
            xlabel('\omega_{roll}(rad/s)'),ylabel('time/s')
            subplot(323),plot(t1,y(l,11)),title('LTI simylation: \omega_{pitch} of the drone against time')
            xlabel('\omega_{pitch}(rad/s)'),ylabel('time/s')
            subplot(325),plot(t1,y(l,12)),title('LTI simylation: \omega_{yaw} of the drone against time')
            xlabel('\omega_{yaw}(rad/s)'),ylabel('time/s')
            subplot(322),plot(t1, drones(1).LogOut.thetadot(l,1)),title('Dynamic simulation: \omega_{roll} of the drone against time')
            xlabel('\omega_{roll}(rad/s)'),ylabel('time/s')
            subplot(324),plot(t1, drones(1).LogOut.thetadot(l,2)),title('Dynamic simulation: \omega_{pitch} of the drone against time')
            xlabel('\omega_{pitch}(rad/s)'),ylabel('time/s')
            subplot(326),plot(t1, drones(1).LogOut.thetadot(l,3)),title('Dynamic simulation: \omega_{yaw} of the drone against time')
            xlabel('\omega_{yaw}(rad/s)'),ylabel('time/s')
        end

        function q3_draw_func(time,pos,theta)
            %%%%% draw x y z phi theta psi
            subplot(321),plot(time,pos(:,1));
            xlabel('Time/s')
            ylabel('X/m')
            title('X direction of the UAV against time ')

            subplot(322),plot(time,pos(:,2));
            xlabel('Time/s')
            ylabel('Y/m')
            title('Y direction of the UAV against time ')

            subplot(323),plot(time,pos(:,3))
            xlabel('Time/s')
            ylabel('Z/m')
            title('Z direction of the UAV against time ')

            subplot(324),plot(time,theta(:,1));
            grid on
            xlabel('Time/s')
            ylabel('phi')
            title('phi angle of the UAV against time ')

            subplot(325),plot(time,theta(:,2));
            grid on
            xlabel('Time/s')
            ylabel('theta')
            title('theta angle of the UAV against time')
            
            subplot(326),plot(time,theta(:,3))
            grid on
            xlabel('Time/s')
            ylabel('psi')
            title('psi angle of the UAV against time')

            %%%% draw trajectories
            figure
            plot3(pos(:,1),pos(:,2),pos(:,3),'LineWidth',2)
            grid on
            hold on
            plot3( [0 5 5 -5 -5 0 0],[0 5 -5 -5 5 0 0],[0 5 10 6 2 2 0],'o')
            xlabel('X/m')
            ylabel('Y/m')
            zlabel('Z/m')
       end
   
        function obj = logOut(obj)
                obj.LogOut.pos = [obj.LogOut.pos; obj.pos.'];
                obj.LogOut.R = [obj.LogOut.R; obj.R.'];
                obj.LogOut.time = [obj.LogOut.time; obj.time.'];
                obj.LogOut.gamma = [obj.LogOut.gamma; obj.gamma.'];
                obj.LogOut.posdot = [obj.LogOut.posdot; obj.posdot.'];
                obj.LogOut.theta = [obj.LogOut.theta; obj.theta.'];
                obj.LogOut.thetadot = [obj.LogOut.thetadot; obj.thetadot.'];
                %obj.LogOut. = [obj.LogOut.; obj.];
        end

        function plot_dir(obj)
            o = obj.pos;
            r = obj.R;

            phi = obj.theta(3);
            theta = obj.theta(2);
            psi1 = obj.theta(1);
            Rx = [1     0           0;
                0       cos(phi)    -sin(phi);
                0       sin(phi)    cos(phi)];
            Ry = [cos(theta) 0          sin(theta);
                    0        1          0;
                -sin(theta)  0          cos(theta)];
            
            Rz = [cos(psi1) -sin(psi1)    0;
                sin(psi1)    cos(psi1)    0;
                0           0           1];
            px = Rx*(o+[1;0;0]);
            py = Ry*(o+[0;1;0]);
            pz = Rz*(o+[0;0;1]);
           
%             quiver3(o(1),o(2),o(3),px(1),px(2),px(3),0)
%             quiver3(o(1),o(2),o(3),py(1),py(2),py(3),0)
%             quiver3(o(1),o(2),o(3),pz(1),pz(2),pz(3),0)
%             plot3([o(1),px(1)], [o(2),px(2)],[o(3),px(3)],'Color','green')
%             plot3([o(1),py(1)], [o(2),py(2)],[o(3),py(3)],'Color','blue')
%             plot3([o(1),pz(1)], [o(2),pz(2)],[o(3),pz(3)],'Color','red')
        end

        function draw(obj)
            % how big should the moving window be
            cL = obj.axis_size;
            
            % set to false if you want to see world view
            if(obj.drone_follow)
               axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            end
            
            % create middle sphere
            [X Y Z] = sphere(8);
            % [X Y Z] = (obj.body(1)/5.).*[X Y Z];
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.axis,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            % create side spheres
            % front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = draw_func.bodyToInertial(obj,rotorPosBody);
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.axis,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
            obj.axis.Title.String = ['Sim Time = ',num2str(obj.time,'%f'),' seconds'];
        end
        function vectorInertial = bodyToInertial(obj, vectorBody)
            vectorInertial = obj.R*vectorBody;
        end
        
    end
end