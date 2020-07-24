%% SetUp
% Connect to ROS master
clear
close all

setenv('ROS_MASTER_URI','http://192.168.1.3:11311')
setenv('ROS_IP','192.168.1.2')

rosshutdown;
IP = '192.168.1.3';
rosinit(IP,11311);

%Creat ROS subscribers and Publishers
posSub = rossubscriber('/cur_pos');
ftSub = rossubscriber('/ft_data');
trackSub = rossubscriber('/track_flag');
initSub = rossubscriber('/init_flg');
dockedSub = rossubscriber('/docked_flg');
[goalPub, goalMsg] = rospublisher('/goal_pos','geometry_msgs/Twist');

%% initialize variables
X_pos = [];
Y_pos = [];
Z_pos = [];
ft_vec = [0 0 1];
v_tool = [0 0 1]; % desired tool orientation
f_vec = [0 0 1]; % desired force vector
vel_last = [0 0 0 0 0 0];
t = 1;

start_track = 0; % start tracking flag
docked_flag = 0;
update_p_flag = 1;
msg_track_hist = ones(1,5)*2; % trakcing flag history data

error_linear = [0,0,0]; % positioning integral error for PID controller
error_force = [0,0,0]; % force integral error for PID controller
error_angular = [0,0,0]; % angular integral error for PID controller
linear_last_out = 0;
angular_last_out = 0;
force_last_out = 0;
dt = 0.1;

last_ft_msg = 0;
last_pos_msg=0;
%% initiliaze trajectory and plot

figure(1)
axis([-150 150 -150 150 -150 150])

r = 30; %[mm]
theta = 0:pi/60:2*pi;
X = zeros(121,1);
Y = zeros(121,1);
Z = zeros(121,1);
pitch = 0;
R_y = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];

v_tool = R_y * v_tool'; % perpendicular to the surface

Traj = zeros(121,3);

for i=1:121
    Y(i) = r*cos(theta(i));
    Z(i) = r*sin(theta(i));
end

Traj(:,1) = X(:);
Traj(:,2) = Y(:);
Traj(:,3) = Z(:);

Traj = (R_y*Traj')';
P_last = [0, 0, 0];
center = [0, 0, 0];
figure(1)
h0=plot3(Traj(:,1),Traj(:,2),Traj(:,3));
xlabel('x')
ylabel('y')
zlabel('z')
grid on
axis([-100 100 -100 100 -100 100])
hold on 
h1 = scatter3(X_pos,Y_pos,Z_pos);
h2 = scatter3(NaN,NaN,NaN,'d');


%% Mian Loop
while(1)
    % sub tracking start flag msg
    msg_track = trackSub.LatestMessage;
    if ~isempty(msg_track)
        msg_track_hist(1,1:4) = msg_track_hist(1,2:5);
        msg_track_hist(1,5) = msg_track.Data;
    else
        msg_track_hist(1,1:4) = msg_track_hist(1,2:5);
        msg_track_hist(1,5) = 0; 
    end
    start_flag = check_start(msg_track_hist);

    if start_flag ==1
        start_track = 1;
    elseif start_flag ==0
        start_track = 0;
    end
    
    %sub docked flag
    msg_docked = dockedSub.LatestMessage;
    if ~isempty(msg_docked)
        docked_flag = msg_docked.Data;
    end
    % sub initialization flag msg
    msg_init = initSub.LatestMessage;
    if ~isempty(msg_init) && msg_init.Data==1
       fprintf ('initializing....\n')
       X_pos = [];
       Y_pos = [];
       Z_pos = [];
       t=1;
       start_track=0;
       update_p_flag=1;
       continue
    end
    
    % sub ft sensor data msg
    msg_ft = ftSub.LatestMessage;
    ft_vec(1) = round(msg_ft.Linear.X, 0);
    ft_vec(2) = round(msg_ft.Linear.Y, 0);
    ft_vec(3) = round(msg_ft.Linear.Z, 0);
    

    % sub current position of end effect msg (it's the flange of the ur robot, not the TCP)
    msg = posSub.LatestMessage;
    
    if msg == last_pos_msg
        continue
    end
    last_pos_msg = msg;
    cur_X = round(msg.X,4);
    cur_Y = round(msg.Y,4);
    cur_Z = round(msg.Z,4);
    cur_linear = [cur_X,cur_Y,cur_Z];
    
    X_pos = [X_pos, cur_X];
    Y_pos = [Y_pos, cur_Y];
    Z_pos = [Z_pos, cur_Z];
    
    % remove repeated position point
    if length(X_pos)>2 && X_pos(end) == X_pos(end-1) && Y_pos(end-1) == Y_pos(end) && Z_pos(end-1) ==Z_pos(end)
        X_pos = X_pos(1:end-1);
        Y_pos = Y_pos(1:end-1);
        Z_pos = Z_pos(1:end-1);
    end
    
    %update start position
    if start_track==0 && ~isempty(X_pos) && ~isequal(P_last, [X_pos(end) Y_pos(end) Z_pos(end)]) && update_p_flag==1
        center_offset = [X_pos(end)-P_last(1), Y_pos(end)-P_last(2), Z_pos(end)-P_last(3)];
        center = center + center_offset;
        Traj(:,1) = (Traj(:,1)/1000+center_offset(1))*1000;
        Traj(:,2) = (Traj(:,2)/1000+center_offset(2))*1000;
        Traj(:,3) = (Traj(:,3)/1000+center_offset(3))*1000;
        P_last = [X_pos(end) Y_pos(end) Z_pos(end)];
    end

    % main control section
    if start_track==1 && ~isempty(X_pos)
        update_p_flag = 0;
        % impedance control
        [disp_X, disp_Y, disp_Z, error_force,force_last_out] = Force_PID(f_vec, ft_vec, 1, 0, 0, error_force, force_last_out, dt);
%         disp([disp_X, disp_Y, disp_Z])
        % positiongin control
        goal_linear_X = Traj(t,1)/1000;%+disp_X; % match frame offset
        goal_linear_Y = Traj(t,2)/1000;%+disp_Y; % match frame offset
        goal_linear_Z = Traj(t,3)/1000;%+disp_Z; % match frame offset
        goal_linear = [goal_linear_X, goal_linear_Y, goal_linear_Z];
        
        [goalMsg.Linear.X, goalMsg.Linear.Y, goalMsg.Linear.Z,error_linear,linear_last_out] = Linear_PID(goal_linear, cur_linear, 2, 0, 0.01, error_linear, linear_last_out, dt);
        
        % orientation alignment control
        if docked_flag ==1
            [roll, pitch, yaw] = orien_align(ft_vec, v_tool');
            dif_angular_X = 0;
            dif_angular_Y = pitch;
            dif_angular_Z = roll;
            dif_RPY = [dif_angular_X, dif_angular_Y, dif_angular_Z];
            [goalMsg.Angular.X, goalMsg.Angular.Y, goalMsg.Angular.Z, error_angular,angular_last_out] = Angular_PID(dif_RPY, 0,0,0, error_angular, angular_last_out, dt);
            
            goalMsg.Angular.X=0;
            goalMsg.Angular.Y=0;
            goalMsg.Angular.Z=0;
            
%             fprintf("angular control on, %b", msg_docked.Data)
        else
            goalMsg.Angular.X=0;
            goalMsg.Angular.Y=0;
            goalMsg.Angular.Z=0;
%             fprintf("angular control off, %b", msg_docked.Data)

        end
        
        goalMsg.Linear.Y = scaler(goalMsg.Linear.Y + goalMsg.Angular.Z,1);
        goalMsg.Linear.Z = scaler(goalMsg.Linear.Z - goalMsg.Angular.Y,1);
        
        vel_in = [goalMsg.Linear.X, goalMsg.Linear.Y, goalMsg.Linear.Z,goalMsg.Angular.X, goalMsg.Angular.Y, goalMsg.Angular.Z];
        [goalMsg.Linear.X, goalMsg.Linear.Y, goalMsg.Linear.Z,goalMsg.Angular.X, goalMsg.Angular.Y, goalMsg.Angular.Z]=vel_smooth(vel_last, vel_in, 0.2);
        vel_last = [goalMsg.Linear.X, goalMsg.Linear.Y, goalMsg.Linear.Z,goalMsg.Angular.X, goalMsg.Angular.Y, goalMsg.Angular.Z];
        
%         fprintf('Angular_X: %f Angular_Y: %f, Angular_Z: %f \n',goalMsg.Angular.X, goalMsg.Angular.Y, goalMsg.Angular.Z);
%         fprintf('linear_X: %f linear_Y: %f, linear_Z: %f \n',goalMsg.Linear.X, goalMsg.Linear.Y, goalMsg.Linear.Z);
%         disp(norm(goal_linear - cur_linear)*1000)

        send(goalPub,goalMsg);
    end
    % update robot end-effect trajectory plot
    if ~isempty(X_pos)
    % set(h1, 'XData', (X_pos(end)-X_pos(1))*1000, 'YData', (Y_pos(end)-Y_pos(1))*1000, 'ZData', (Z_pos(end)-Z_pos(1))*1000)
    set(h1, 'XData', (X_pos-center(1))*1000, 'YData', (Y_pos-center(2))*1000, 'ZData', (Z_pos-center(3))*1000, 'cData', jet(length(X_pos)));
    set(h2, 'XData', (Traj(t,1)/1000-center(1))*1000, 'YData', (Traj(t,2)/1000-center(2))*1000, 'ZData', (Traj(t,3)/1000-center(3))*1000);
    set(h0, 'XData',(Traj(:,1)/1000-center(1))*1000,'YData', (Traj(:,2)/1000-center(2))*1000,'ZData',(Traj(:,3)/1000-center(3))*1000);
    drawnow
    end
    
    
    % update goal watpoint
    if start_track == 1 && norm(goal_linear - cur_linear)*1000 < 1.5
        t = max(1, mod(t+1,122));
%         t=1;
    end    
end

function [flag] = check_start(hist_matrix)
% add a fliter to smooth the check on tracking start flag
    n = length(hist_matrix);
    count_start = 0;
    count_halt = 0;
    flag = -1;
    for i = 1:n
        if hist_matrix(i)==-2
            count_start = count_start +1;
        elseif hist_matrix(i) == 2
            count_halt = count_halt+1;
        end
    end    
    
    if count_start == n
        flag = 1;
    elseif count_halt == n
        flag = 0;
    end
end

function [output] = scaler(input, weight)
% scale the output in range of [-1, 1]
    input = input * weight;
    if input >= 1
        output =1;
    elseif 0<= input && input <=0.05
        output = 0.05;
    elseif input < 0 && input >= -0.05
        output = -0.05;
    elseif input <=-1
        output = -1;
    else
        output = input;
    end
end

function [rotation_matrix] = rotation_matrix_from_vectors(vec1, vec2)
    % Find the rotation matrix that aligns vec1 to vec2
    %param vec1: A 3d "source" vector
    %param vec2: A 3d "destination" vector
    %return rot: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    if isequal(vec1, [0 0 0]) || isequal(vec1, [0 0 -1])
        vec1=vec2;
    end
    
    a = reshape(vec1 / norm(vec1),[1,3]);
    b = reshape(vec2 / norm(vec2), [1,3]);
    v = cross(a, b);
    c = dot(a, b);
%     s = norm(v);
    kmat = [[0, -v(3), v(2)]; [v(3), 0, -v(1)]; [-v(2), v(1), 0]];
    rotation_matrix = eye(3) + kmat + kmat*kmat  / (1+c);
end

function [r, p, y] = cal_RPY(rot)
    r21 = rot(2,1);
    r11 = rot(1,1);
    r31 = rot(3,1);
    r32 = rot(3,2);
    r33 = rot(3,3);
    
    r = atan2(r32, r33);
    p = atan2(-r31, sqrt(r32^2+r33^2));
    y = atan2(r21, r11);

end

function [roll, pitch, yaw] = orien_align(vec1, vec2)
    rot = rotation_matrix_from_vectors(vec1, vec2);
    [roll, pitch, yaw] = cal_RPY(rot);
end

function [linear_X, linear_Y, linear_Z, error_out,last_out] = Linear_PID(goal, cur, Kp, Ki, Kd, error_in, last_in, dt)
    % error_* is the integral historical error
    e = goal - cur; % difference between goal and current value
    error_out = error_in + e .*1000 .* dt; % update integral historical error
    last_out = e;
    output = Kp .* e .*1000 + Ki.* error_out - Kd/dt .* (e - last_in).*1000; %PID controller
    
    linear_X = scaler(output(1), 0.1);
    linear_Y = scaler(output(2), 0.1);
    linear_Z = scaler(output(3), 0.1);
    
end

function [angular_X, angular_Y, angular_Z, error_out,last_out] = Angular_PID(dif_RPY, Kp, Ki, Kd, error_in, last_in, dt)
    % error_* is the integral historical error
    
    error_out = error_in + dif_RPY .* dt; % update integral historical error
    last_out = dif_RPY;
    output = Kp .* dif_RPY + Ki.* error_out - Kd/dt .* (dif_RPY - last_in); %PID controller
    
    angular_X = 0;
    angular_Y = scaler(output(2), 0.1);
    angular_Z = scaler(output(3), 0.1);   
end

function [disp_X, disp_Y, disp_Z, error_out,last_out] = Force_PID(goal_force, cur_force, Kp, Ki, Kd, error_in, last_in, dt)
    stf_factor = [35e6, 35e6, 30e6]; %N/m
    e = goal_force - floor(cur_force);
    error_out = error_in + e;
    last_out = e;
    output = Kp .* e ./ stf_factor + Ki.* error_out./stf_factor - Kd/dt .* (e - last_in)./stf_factor; %PID controller
    disp_X = output(1) * 1000;
    disp_Y = output(2) * 1000;
    disp_Z = output(3) * 1000;
end

function [out] = compare_vel(diff, step)
    if abs(diff) > step
        out = step * sign(diff);
    else
        out = 0;
    end
end

function [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z] = vel_smooth(vel_last, vel_in ,step)
    diff = vel_in-vel_last;
    linear_x = vel_last(1) + compare_vel(diff(1), step);
    linear_y = vel_last(2) + compare_vel(diff(2), step);
    linear_z = vel_last(3) + compare_vel(diff(3), step);
    angular_x = vel_last(4) + compare_vel(diff(4), step);
    angular_y = vel_last(5) + compare_vel(diff(5), step);
    angular_z = vel_last(6) + compare_vel(diff(6), step);
end

% function rot = rotation_mtrx(roll, pitch, yaw)
%     Rx =  [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
%     Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
%     Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
%     
%     rot = Rx * Ry * Rz;
% end