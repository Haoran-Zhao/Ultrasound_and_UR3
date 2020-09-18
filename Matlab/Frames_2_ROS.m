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
    
    dirc = dir(directory);
    %Filter out all the folders.
    dirc = dirc(~cellfun(@isfolder,{dirc(:).name}));
    if ~isempty(dirc)
        imagename = getlatestfile(dirc);
        filename = [directory,'/',imagename];
%         filename = ['C:\Users\Julien Leclerc\Documents\Vantage-4.1.1-1910101200/Verasonics_Haoran/Images/scene2913.png'];
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
end


function latestfile = getlatestfile(directory)
%This function returns the latest file from the directory passsed as input
%argument

% %Get the directory contents
% dirc = dir(directory);
% 
% %Filter out all the folders.
% dirc = dirc(~cellfun(@isfolder,{dirc(:).name}));

%I contains the index to the biggest number which is the latest file
% [~,I] = max([directory(:).datenum]);

    names = regexp([directory(:).name], '(?<=scene)\d+', 'match');
    namesnum = cellfun(@str2num, names);
    [~,I] = max(namesnum);

    if ~isempty(I)
        latestfile = directory(I).name;
    end

end