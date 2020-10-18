clear
close all

setenv('ROS_MASTER_URI','http://192.168.1.3:11311')
setenv('ROS_IP','192.168.1.2')

rosshutdown;
IP = '192.168.1.3';
rosinit(IP,11311);

[framePub, frameMsg] = rospublisher('/FrameRaw','sensor_msgs/Image');
ftSub = rossubscriber('/ft_data');
f_vec = [0 0 -4]; % desired force vector
ft_vec = [0 0 1];
ft_hist = [];
ft_hist = [ft_hist; ft_vec];
last_ft_msg = 0;
hist_t = [0.00000001];
iter=1;

figure(1)
set(gcf, 'Position', [100,60, 500, 400])
h3 = plot(ft_hist(:,3));
hold on 
h4 = plot(smoothdata(ft_hist(:,3),'movmedian',5),'r');
h5 = plot(1:length(ft_hist), f_vec(3)* ones(length(ft_hist),1), 'g');

while(1)
%     filename = [pwd '/Verasonics_Haoran/Images/scene',num2str(iter),'.png'];
    tic
    directory = [pwd '/Verasonics_Haoran/Images'];
    imagename = 'scene1.png';
    filename = [directory,'/',imagename];
    if isfile(filename)
        try
            image = imread(filename);
            %             imshow(image);
            frameMsg.Encoding = 'mono8';
            writeImage(frameMsg,image);
            send(framePub, frameMsg);
%             iter = iter + 1;
        catch ME
            fprintf('There was a problem reading file %s\n', imagename);
        end
    end
    
    % sub ft sensor data msg
    msg_ft = ftSub.LatestMessage;
    ft_vec(1) = round(msg_ft.Linear.X, 1);
    ft_vec(2) = round(msg_ft.Linear.Y, 1);
    ft_vec(3) = round(msg_ft.Linear.Z, 1);
    
    ft_hist = [ft_hist; ft_vec];
    set(h3, 'XData', 1:length(ft_hist) ,'YData', ft_hist(:,3));
    set(h4, 'XData', 1:length(ft_hist) ,'YData', smoothdata(ft_hist(:,3),'movmedian',20));
    set(h5, 'XData', 1:length(ft_hist) ,'YData',  f_vec(3)* ones(length(ft_hist),1));
    drawnow
    t = toc; 
    hist_t = [hist_t, hist_t(end)+t];
end