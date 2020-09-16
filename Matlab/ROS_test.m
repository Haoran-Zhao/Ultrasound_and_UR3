setenv('ROS_MASTER_URI','http://192.168.1.2:11311')
setenv('ROS_IP','192.168.1.3')

rosshutdown;
IP = '192.168.1.2';
rosinit(IP,11311);

ftSub = rossubscriber('/ft_data');

while(1)
    msg_ft = ftSub.LatestMessage;
    ft_vec(1) = round(msg_ft.Linear.X, 1);
    ft_vec(2) = round(msg_ft.Linear.Y, 1);
    ft_vec(3) = round(msg_ft.Linear.Z, 1);
    disp(ft_vec(3));
    
    filename = ['C:/Users/zhaoh/OneDrive/Desktop/Vantage-4.2.0-2001220500/Verasonics_training/Images/scene',num2str(iter),'.png'];
    if isfile(filename)
        image = imread(filename);
        imshow(image);
        iter = iter + 1;
    end
end