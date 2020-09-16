clear 
close all

iter = 1;
while 1
    filename = ['C:/Users/zhaoh/OneDrive/Desktop/Vantage-4.2.0-2001220500/Verasonics_training/Images/scene',num2str(iter),'.png'];
    if isfile(filename)
        image = imread(filename);
        imshow(image);
        iter = iter + 1;
    end
end