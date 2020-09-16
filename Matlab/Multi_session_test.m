clear 
close all

iter = 1;
while 1
    filename = ['C:/Users/Julien Leclerc/Documents/Vantage-4.1.1-1910101200/Verasonics_Haoran/Images/scene',num2str(iter),'.png'];    
    if isfile(filename)
        image = imread(filename);
        imshow(image);
        iter = iter + 1;
    end
end