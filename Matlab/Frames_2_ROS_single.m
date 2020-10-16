clear
close all

setenv('ROS_MASTER_URI','http://192.168.1.3:11311')
setenv('ROS_IP','192.168.1.2')

rosshutdown;
IP = '192.168.1.3';
rosinit(IP,11311);

[framePub, frameMsg] = rospublisher('/FrameRaw','sensor_msgs/Image');
iter=1;

while(1)
%     filename = [pwd '/Verasonics_Haoran/Images/scene',num2str(iter),'.png'];
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
end