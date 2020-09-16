clear
close all

setenv('ROS_MASTER_URI','http://192.168.1.3:11311')
setenv('ROS_IP','192.168.1.2')

rosshutdown;
IP = '192.168.1.3';
rosinit(IP,11311);

ftSub = rossubscriber('/ft_data');

while(1)
    msg_ft = ftSub.LatestMessage;
    if ~isempty(msg_ft)
        ft_vec(1) = round(msg_ft.Linear.X, 1);
        ft_vec(2) = round(msg_ft.Linear.Y, 1);
        ft_vec(3) = round(msg_ft.Linear.Z, 1);
        disp(ft_vec(3));
    end
    filename = ['C:/Users/Julien Leclerc/Documents/Vantage-4.1.1-1910101200/Verasonics_Haoran/Images/scene',num2str(iter),'.png'];
    if isfile(filename)
        image = imread(filename);
        imshow(image);
        iter = iter + 1;
    end
end