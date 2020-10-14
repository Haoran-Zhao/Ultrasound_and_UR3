clc
clear all close all

%create racetrack trajectory

Length=30; %mm
Width=10; %mm
%turns radius = width/2
Angle=[]
for i=0:100
    Angle=[Angle;i*pi/100];
end

X=Width/2*sin(Angle)+Length/2
Y=-Width/2*cos(Angle)

X2=-Width/2*sin(Angle)-Length/2
Y2=Width/2*cos(Angle)

Xtraj=[X;X2;X(1)]
Ytraj=[Y;Y2;Y(1)]
figure(1)
plot(Xtraj,Ytraj)
axis equal