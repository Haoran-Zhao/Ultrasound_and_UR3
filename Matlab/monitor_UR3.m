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

%% initialize variables
Y_pos = [];
Z_pos = [];

Y_goal = [];
Z_goal = [];

start_track = 0; % start tracking flag
docked_flag = 0;
msg_track_hist = ones(1,5)*2; % trakcing flag history data
ft_hist = [];
ft_hist = [ft_hist; ft_vec];

last_ft_msg = 0;
last_pos_msg=0;
%% initiliaze trajectory and plot

figure(1)
set(gcf, 'Position', [100,60, 500, 950])
subplot(2,1,1)
axis([-150 150 -150 150 -150 150])

figure(1)
h0=plot(Y_goal,Z_goal);
xlabel('Y')
ylabel('Z')
grid on
axis([-100 100 -100 100])
hold on 
h1 = scatter(Y_pos,Z_pos);
h2 = scatter(Y_goal,Z_goal,'d');

subplot(2,1,2)
h3 = plot(ft_hist(:,3));
hold on 
h4 = plot(smoothdata(ft_hist(:,3),'movmedian',5),'r');
h5 = plot(1:length(ft_hist), f_vec(3)* ones(length(ft_hist),1), 'g');

%% Mian Loop
while(1)
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
       ft_hist = [];
       t=1;
       continue
    end
    
    % sub ft sensor data msg
    msg_ft = ftSub.LatestMessage;
    ft_vec(1) = round(msg_ft.Linear.X, 1);
    ft_vec(2) = round(msg_ft.Linear.Y, 1);
    ft_vec(3) = round(msg_ft.Linear.Z, 1);
    
    ft_hist = [ft_hist; ft_vec];
    
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
    
    % update robot end-effect trajectory plot
    if ~isempty(X_pos)
    % set(h1, 'XData', (X_pos(end)-X_pos(1))*1000, 'YData', (Y_pos(end)-Y_pos(1))*1000, 'ZData', (Z_pos(end)-Z_pos(1))*1000)
    set(h0, 'XData', Y_goal, 'YData', Z_goal);
    set(h1, 'XData', Y_pos, 'YData', Z_pos, 'cData', jet(length(X_pos)));
    set(h2, 'XData', Y_goal, 'YData', Z_goal);
    set(h3, 'XData', 1:length(ft_hist) ,'YData', ft_hist(:,3));
    set(h4, 'XData', 1:length(ft_hist) ,'YData', smoothdata(ft_hist(:,3),'movmedian',20));
    set(h5, 'XData', 1:length(ft_hist) ,'YData',  f_vec(3)* ones(length(ft_hist),1));

    drawnow
    end     
end