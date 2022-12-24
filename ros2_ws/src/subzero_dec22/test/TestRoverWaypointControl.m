%% UNI. OF TUEBINGEN ROVER CONTROL MODEL
%   Name: TestRoverWaypointControl.m
%
%   Auth: J.D. Hawkins
%   Date: 2021-12-13
%
%   Designed based on code from Polar Research Equipment and M. R. Ershadi.
%
%   Contributions to code and algorithm R. Drews and I. Koch
%

%% ROVER PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r_pos = [0;0];      % rover starting position
r_hdg = [1;0];      % pover heading
r_vel = 1;          % rover velocity
r_ang = 0;          % rover angular velocity
vel_max = 2;        % rover max velocity
vel_min = 0.1;      % rover min velocity (if moving)
ang_max = 1;        % rover max angular velocity

nav_radius = 30;    % navigation radius parameter

acceptable_target_proximity = 0.5; % acceptable target proximity at which
                                   % waypoint is considered visited.

% Rover waypoint
wpt = [20 40 41 42 43 44 45 46 47 48 49 50; 5 5 5 5 5 5 5 5 5 5 5 5];
% Insert starting position as first waypoint
wpt = [r_pos wpt];

% PID coefficients
ANG_I_COEFF = 0.5;
ANG_P_COEFF = 0.5;
VEL_P_COEFF = 0.5;

%% GPS PARAMETER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Assumes GPS error is normally distributed with:
%   sigma = 0.025 m for RTK (i.e. 95% in 5cm)
%   sigma = 2.500 m for C/A (i.e. 95% in 5m) 
gps_error = 0.05/2; %5/2; 

% Number of GPS positions to average over when calculating current and
% heading positions
n_gps_pos = 2; %should be minimum of 2 to calculate heading

% GPS update interval at which a new GPS position is received by the rover
gps_update = 1;

% INTERNAL PARAMETERS FOR GPS
% Internal parameter to track last GPS update time
last_gps_update = 0;
% Store GPS positions
gps_pos = repmat(r_pos + gps_error * randn(2,1), 1, n_gps_pos);
gps_hdg = 0;
% Store current waypoint (starts from 2 as first waypoint is origin)
wpt_idx = 2; % Assume that wpt_idx=1 is start pos
% Integral variable for heading error
hdg_error_int = 0;
% Initial heading error
hdg_error = 0;

%% MODEL PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Time step interval
dt = 0.1;
% Current time 
t = 0;
% Max time to run model until
Tmax = 100;

%% RESET GRAPHIC STATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,2,1)
hold off
plot(wpt(1,:), wpt(2,:), 'bx')
axis equal
hold on
subplot(2,2,2)
hold off
plot(wpt(1,:), wpt(2,:), 'bx')
axis equal
hold on
subplot(2,1,2)
hold off

%% RUN UPDATE LOOP FOR ROVER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Will keep going until t = Tmax
while t < Tmax
    
    %% UPDATE ROVER POSITION
    % Rotate rover heading by angular velocity * timestep
    r_hdg = [cos(r_ang*dt) -sin(r_ang*dt); sin(r_ang*dt) cos(r_ang*dt)] * r_hdg;
    % Update rover position from velocity
    r_pos = r_pos + dt * r_vel * r_hdg;
    
    %% FIND ERROR IN NEXT TARGET
    % Vector to target
    delta_target = wpt(:,wpt_idx) - mean(gps_pos, 2);
    % Target distance
    dist_target = norm(delta_target);
    
    % This is implementing the same as line 34 in original function
    if dist_target > nav_radius
        % Get goal distance
        goal_dist = dist_target - nav_radius;
        % Find path from previous waypoint to next
        wpt_path = wpt(:,wpt_idx-1) - wpt(:,wpt_idx);
        % Normalise
        wpt_norm = wpt_path / norm(wpt_path);
        % Move goal_dist back along the path
        target_point = wpt(:,wpt_idx) + goal_dist * wpt_norm;
        % Update delta_target and dist_target
        delta_target = target_point - mean(gps_pos, 2);
        dist_target = norm(delta_target);
    end
    
    %% CALCULATE HEADING ERROR
    hdg_target = atan2(delta_target(2), delta_target(1));
    hdg_error = hdg_target - gps_hdg;
    
    % Resolve heading error
    if abs(hdg_error) <= pi
        if abs(hdg_error) == pi
            hdg_error = abs(hdg_error);
        end
    elseif hdg_target > gps_hdg
        hdg_error = abs(hdg_error) - 2*pi;
    else
        hdg_error = pi - hdg_error;
    end
    
    % Add to PID integral
    hdg_error_int = hdg_error_int + dt * hdg_error;
    
    %% CHECK WAYPOINT VISITED AND IF SO, STOP OR MOVE ON
    if dist_target < acceptable_target_proximity
        % Update graph to show waypoint visited
        subplot(2,2,2)
        plot(wpt(1,wpt_idx), wpt(2,wpt_idx), 'ro')
        % Increment waypoint index
        wpt_idx = wpt_idx + 1;
        % Are we at the end
        if wpt_idx > size(wpt,2)
            t = T;
        end
        t = t + dt;
        continue
    end
    
    %% UPDATE GPS POSITION AND DISPLAY
    if t - last_gps_update >= gps_update
        % Reset last_gps_time
        last_gps_udpate = t;
        % Shift GPS positions
        gps_pos(:,1:end-1) = gps_pos(:,2:end);
        % Add a new GPS position 
        gps_pos(:,end) = r_pos + randn(2,1) * gps_error;
        % Calculate heading error
        delta_gps = mean(gps_pos(:,2:end) - gps_pos(:,1:end-1),2);
        gps_hdg = atan2(delta_gps(2), delta_gps(1));
        %% ATTENTION: 
        %   This resets the hdg_error integral every time a new GPS
        %   position is acquired.  Comment out to see effect of continuous
        %   heading error integral as is currently implmeneted.
        hdg_error_int = 0;
        
        %% DRAW
        subplot(2,2,1)
        plot(r_pos(1), r_pos(2), 'ro')
        title(num2str(t))
        subplot(2,2,2)
        plot(gps_pos(1), gps_pos(2), 'mx')
        subplot(2,1,2)
        plot(t, gps_hdg, 'rx')
        plot(t, hdg_error, 'go')
        plot(t, hdg_target, 'b*')
        hold on
        drawnow
        
    end
    
    %% CONSTRAIN HEADING ERROR TO MIN/MAX
    if hdg_error_int > ang_max
        hdg_error_int = ang_max;
    elseif hdg_error_int < -ang_max
        hdg_error_int = -ang_max;
    end
    
    %% ATTENTION:
    %   This is also not implemented in the code but adjusts the rover
    %   velocity so it is proprotional to the sqrt of distance to the
    %   target within a MIN/MAX bound.
    r_vel = VEL_P_COEFF * sqrt(dist_target);
    if r_vel > vel_max
        r_vel = vel_max;
    elseif r_vel < vel_min
        r_vel = vel_min;
    end
    
    %% UPDATE ANGULAR VELOCITY AND CONSTRAIN
    r_ang = ANG_P_COEFF * hdg_error + ANG_I_COEFF * hdg_error_int;
    if r_ang > ang_max
        r_ang = ang_max;
    elseif r_ang < -ang_max
        r_ang = -ang_max;
    end
    
    % Increment time step
    t = t + dt;
    
end

% Make plots nice at the end with labels and legend
subplot(2,2,1)
xlabel('Rover x (m)')
ylabel('Rover y (m)')
subplot(2,2,2)
title('Measured GPS Position')
xlabel('Rover x (m)')
ylabel('Rover y (m)')
subplot(2,1,2)
xlabel('Time (s)')
ylabel('Angle (rad)')
legend('gps heading', 'heading error', 'target heading')