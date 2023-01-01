close all;
clc;
disp('Start');

%TIME SPAN
dt=1e-2;
q=quat;
tspan=linspace(0,simtime,length(q));
w0=[w0x,w0y,w0z];

state0=[q(1,:),w0];

vertices=[ 226.3/2 -226.3/2 -366/2;
           226.3/2 226.3/2 -366/2;  
          -226.3/2 226.3/2 -366/2; 
          -226.3/2 -226.3/2 -366/2;
           226.3/2 -226.3/2 366/2; 
           226.3/2 226.3/2 366/2; 
          -226.3/2 226.3/2 366/2 ;
          -226.3/2 -226.3/2 366/2; 
          226.3/2  -226.3/2 366/2-10;
          226.3/2  226.3/2  366/2-10;
          366+226.3/2 226.3/2 366/2-10;   %11
          366+226.3/2 -226.3/2 366/2-10;
          366+226.3/2 -226.3/2 366/2;
          366+226.3/2 226.3/2  366/2;
          -366-226.3/2 226.3/2 366/2-10;
          -366-226.3/2 -226.3/2 366/2-10;  %16
          -366-226.3/2 226.3/2 366/2;
          -366-226.3/2 -226.3/2 366/2;
          -226.3/2  226.3/2  366/2-10;
          -226.3/2  -226.3/2 366/2-10; 
          ] ;
% vertices(:,1) = vertices(:,1) - R_nx(1);
% vertices(:,2) = vertices(:,2) - R_ny(1);
% vertices(:,3) = vertices(:,3) - R_nz(1);

faces=[1 2 6 5;
       2 3 7 6;
       3 4 8 7;
       4 1 5 8;
       1 2 3 4;
       5 6 7 8;
       9 10 11 12;
       5 6 10 9;
       5 6 14 13;
       5 9 12 13;
       11 12 13 14;
       6 10 11 14;
       7 19 15 17;
       8 20 16 18;
       7 8 20 19;
       15 16 18 17;
       20 19 15 16;
       8 7 17 18;];

color='b';

%INERTIA
 
inertia=diag([Ix Iy Iz]);

BLOCK = block(state0,q,inertia,tspan,dt,vertices,faces,color);
 BLOCK.simulate()
 BLOCK.animate()
 


disp('Done');