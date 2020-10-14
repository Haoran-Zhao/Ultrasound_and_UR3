1/0.003
function [I] = InverseMagnetics(Field,P,coil_radius,T)
%INVERSEMAGNETICS Summary of this function goes here

%This function takes as input the magnetic flux density and force on d axis
%and returen the current to apply

%INUTS: 
%Field: 1x6 vector containg the 3 components of the flux density to apply and
%the 3 componenets of the force to apply [Bx;By;Bz;Fx;Fy;Fz]

%coil_radius: average radius of the coil [m]

%P: [m] Position of the robot [x,y,z]
%m: magnetic moment vector [mx;my;mz]
%T: [m] distance between oposit coils

%Outputs: 

%I: [A] curremt vector


%Build the actuation matrix 

G1=GFunction(1,P,coil_radius,T);
G2=GFunction(2,P,coil_radius,T);
G3=GFunction(3,P,coil_radius,T);
G4=GFunction(4,P,coil_radius,T);
G5=GFunction(5,P,coil_radius,T);
G6=GFunction(6,P,coil_radius,T);

Gx=[G1(1) G2(1) G3(1) G4(1) G5(1) G6(1)];
Gy=[G1(2) G2(2) G3(2) G4(2) G5(2) G6(2)];
Gz=[G1(3) G2(3) G3(3) G4(3) G5(3) G6(3)];

%Actuation Matrix
A=[Gx;Gy;Gz];

%Inverse the matrix and return the current
I=A'/(A*A')*Field;
end

