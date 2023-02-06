function [posdot, windspeed] = wind_model(windspeed, pos, posdot, dt)
    % meter to feet
    pwr = -100; % dwB power of the white noise
    pz=0.00000000001;% altitude of the drone, in this section, we use the average altitude of London
    u_20 = 0.000015*1.68780986; %15 knots for 20 feet height
    posdot = posdot*3.2808399;% change the unit to feet
    pos(3) = pos(3)*3.2808399;% change the unit to feet
    V = sum(posdot.^2)^0.5; %airspeed of the platform
    rand_c = rand;

    %-pz is an altitude,u_20 is 20 feet to the ground  
    L_w = (pz+pos(3));%turbulance scale length 1
    L_v = (pz+pos(3))/((0.177+0.000823*pos(3))^1.2);%turbulance scale length 2 & 3
    L_u = L_v;
    
    sigma_w = 0.1*u_20; %intensity of 1 of turbulence
    sigma_v = sigma_w/((0.177+0.000823*pos(3))^0.4);%intensity of 2 & 3  of turbulence
    sigma_u = sigma_v;
    %total turbulence,linear velocity vector,V is the airspeed (scalar) of platform, x，y，z and T(dt) is discretization
    %period, white noise perturbations with variances 
    windspeed = [((1-V*dt/L_u)*windspeed(1))+((2*V*dt/L_u).^0.5)*wgn(1,1,pwr)*sigma_u;
                 ((1-V*dt/L_v)*windspeed(2))+((2*V*dt/L_v).^0.5)*wgn(1,1,pwr)*sigma_v;
                 ((1-V*dt/L_w)*windspeed(3))+((2*V*dt/L_w).^0.5)*wgn(1,1,pwr)*sigma_w];
        if pos(3)>20
            % Add turbulence to the x/y axis at random
            if  rand_c > 0.5
                 mean_wind = [u_20*(log((pz+pos(3))/0.15))/(log(20/0.15));0;0];%mean wind field vector
                windspeed = windspeed + mean_wind;
            else
                  mean_wind = [0;u_20*(log((pz+pos(3))/0.15))/(log(20/0.15));0];%mean wind field vector
                  windspeed = windspeed + mean_wind; 
            end
        end     

    % feet to meter
    posdot = (windspeed + posdot)/3.2808399;
end
