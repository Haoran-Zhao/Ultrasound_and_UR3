function [ G ] = dGFunction( coil_number,P,a,AxisDerivative,T )

%/////////////////////////////////////////////////////////////////////////
%This program was made by Julien Leclerc Ph.D, Robotic Swarm Control
%Laboratory, University of Houston, Department of Electrical and Computer
%Engineering
%/////////////////////////////////////////////////////////////////////
%Last modification : 10/10/2018 by Haoran Zhao

%Axis==1 => x
%Axis==2 => y
%Axis==3 => z

dl=0.0001; %delta for differenciation [m]

if AxisDerivative==1
    Gplus=GFunction( coil_number,[P(1)+dl;P(2);P(3)],a, T);
    Gminus=GFunction( coil_number,[P(1)-dl;P(2);P(3)],a, T );
end

if AxisDerivative==2
    Gplus=GFunction( coil_number,[P(1);P(2)+dl;P(3)],a,T );
    Gminus=GFunction( coil_number,[P(1);P(2)-dl;P(3)],a ,T);
end

if AxisDerivative==3
    Gplus=GFunction( coil_number,[P(1);P(2);P(3)+dl],a,T );
    Gminus=GFunction( coil_number,[P(1);P(2);P(3)-dl],a ,T);
end



G=(Gplus-Gminus)./(2*dl);

end

