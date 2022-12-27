
%% INITIAL DATAS

 clearvars; 
 clc;
 close all;
 
 %Geometry
%  Ix=100.9*1e-2; %[Kg/m^2] 
%  Iy=25.1*1e-2;
%  Iz=91.6*1e-2;
%  
 %Orientation 
 %Main body
 body.n1=[1;0;0];
 body.n2=[0;1;0];
 body.n3=[0;0;1];
 body.n4=-body.n1;
 body.n5=-body.n2;
 body.n6=-body.n3;

 %Solar panels
 body.n7=[0;0;1];
 body.n8=-body.n7;
 body.n9=[0;0;1];
 body.n10=-body.n9;
 
 body.r_g=[0.027;0.012;0.04];  %Centre of mass position
 
 %Areas
 body.A1=0.2263*0.366;
 body.A2=body.A1;
 body.A3=0.2263^2;
 body.A4=body.A1;
 body.A5=body.A2;
 body.A6=body.A3;
 body.A7=0.2263*0.366;
 body.A8=body.A7;
 body.A9=body.A8;
 body.A10=body.A9;
 
 %Center of pressure for all faces
 body.r_cp_1 = [(0.2263/2);0;0];
 body.r_cp_2 = [0;0.2263/2;0];
 body.r_cp_3 = [0;0;0.366/2];
 body.r_cp_4 = -body.r_cp_1;
 body.r_cp_5 = -body.r_cp_2;
 body.r_cp_6 = -body.r_cp_3;
 body.r_cp_7 = [0.366+0.2263/2;0;0.366/2];
 body.r_cp_8 = body.r_cp_7;
 body.r_cp_9 = [-(0.366+0.2263/2);0;0.366/2];
 body.r_cp_10 = body.r_cp_9;
 
 %Distance between cp_i and Center of gravity
 body.r1 = body.r_cp_1 - body.r_g;
 body.r2 = body.r_cp_2 - body.r_g;
 body.r3 = body.r_cp_3 - body.r_g;
 body.r4 = body.r_cp_4 - body.r_g;
 body.r5 = body.r_cp_5 - body.r_g;
 body.r6 = body.r_cp_6 - body.r_g;
 body.r7 = body.r_cp_7 - body.r_g;
 body.r8 = body.r_cp_8 - body.r_g;
 body.r9 = body.r_cp_9 - body.r_g;
 body.r10 = body.r_cp_10 - body.r_g;
 




%Orbit model
orbit.R_E = astroConstants(23);
orbit.mu = astroConstants(13);
orbit.w_E = 15.04*(pi/180)*(1/3600);
orbit.G = 6.67408e-20;
orbit.Mt = 5.97219e24;
orbit.rp = orbit.R_E+400;
orbit.ra = orbit.R_E+500;
orbit.e = (orbit.ra-orbit.rp)/(orbit.ra+orbit.rp);
orbit.a = (orbit.ra + orbit.rp)/2;
orbit.p = orbit.a * (1 - orbit.e^2);
orbit.incl=deg2rad(0);  % Minimum error near equator,more near poles
orbit.omega=0;
orbit.w_p=0;
orbit.n=sqrt(orbit.G*orbit.Mt/orbit.a^3);


%SRP
Rs=147098074;
Fe=1358;
c=299792458;
P=Fe/c;
rho_s_main=0.5;
rho_d_main=0.1;
rho_s_panel=0.1;
rho_d_panel=0.1;

%Magnetic field
g1_0=-29615*1e-9;
g1_1=-1728*1e-9;
h1_1=5186*1e-9;
H0=sqrt(g1_0^2+g1_1^2+h1_1^2);
axis_i=deg2rad(11.5);          %Inclination of the magnetic axis;
m_para=[0.03;0.04;0.05]; 


%Air Drag
Cd=2.3;


%Initial condition

%Use this for De-Tumbling
w0x= 0.044; w0y = 0.025 ;w0z = 0.05; w0r=0; 

%Use this for Tracking
w0x= 0.01; w0y = orbit.n + 0.05 ;w0z = 0.02; w0r=0; 

Ix = 0.07; Iy = 0.0504; Iz = 0.0109;

I = [Ix 0 0; 0 Iy 0 ; 0 0 Iz];


Kx = (Iy-Iz)/Ix; Krx=0;
Ky = (Iz-Ix)/Iy; Kry=0;
Kz = (Ix-Iy)/Iz;
Mr=0;

%Sensors
accuracy_mag = deg2rad(300/60);
accuracy_earth = deg2rad(10/60);
%ortog = (1/3 * deg2rad(1)^2);
ortog = 0;

%Q-METHOD
alpha_m = 0.4;
alpha_s = 1 - alpha_m;

%Release attitude for slew motion
A_R = [0 0 1;
       1 0 0;
       0 1 0]';


%Quaternion at the release
q4r = 0.5 * ( 1 + A_R(1,1) + A_R(2,2) + A_R(3,3))^(1/2);
q3r = 1/(4*q4r) * (A_R(1,2) - A_R(2,1));
q2r = 1/(4*q4r) * (A_R(3,1) - A_R(1,3));
q1r = 1/(4*q4r) * (A_R(2,3) - A_R(3,2));
q_r = [q1r q2r q3r q4r];

%Target Attitude : Pointing the Earth so the target is given by the matrix
%A_B/L
A_T =[1 0 0;
      0  0 1;
      0 -1 0];

%Control
kp_detum = [0.0000008 0.000005 0.000005];
kd_detum = 2 .* kp_detum;
kp_track = kp_detum;
kd_track = kd_detum;
kp_slew = [0.0005 0.0004 0.0004];
kd_slew = 2 * kp_slew;
k1 = 0.002;
k2 = 0.0025;

%Thrusters
%Position of Actuators
thrus.ACT_x1 = [body.r_g(1) ; -0.2263/2 ; -0.366/2] - body.r_g;
thrus.ACT_x2 = [body.r_g(1) ; 0.2263/2; -0.366/2] - body.r_g;
thrus.ACT_y1 = [0.2263/2 ; body.r_g(2) ; -0.366/2] - body.r_g;
thrus.ACT_y2 = [-0.2263/2; body.r_g(2) ; -0.366/2] - body.r_g;
thrus.ACT_z1 = [body.r_g(1) ; 0.2263/2 ; body.r_g(3)] - body.r_g;
thrus.ACT_z2 = [body.r_g(1) ; -0.2263/2; body.r_g(3)] - body.r_g;

thrus.up_sat = 0.1; %N
thrus.low_sat = 0.003; %N

l = 0.183;
X1 = 0.07;
X2 = 0.08;
alpha = deg2rad(40);

Thrus = [l * sin(alpha) ,      0 ,          0
         0                          , l * cos(alpha) , 0
         0      ,       0     ,      X1 * cos(alpha) - X2 * sin(alpha)] ;

Thr_pos = [1 , -1 , -1 , 1
           1 , 1 , -1 , 1 
           1 , -1 , 1 , -1] ;

R_thr = Thrus * Thr_pos;

R = pinv(R_thr);

%Orbit
orbit.year=60*60*24*365;
orbit.eclip=deg2rad(23.45);
orbit.n_sun=2*pi/orbit.year;
orbit.T=2*pi/orbit.n;
simtime=2*orbit.T;

% Target after De-tumbling
w_track = [0; orbit.n ; 0];
zero_w = [ 0; 0; 0];

out=sim('Att_project_simulink',simtime);
%%
%quaternion found directly
 quat=out.quaternions.Data;
%quat=[quat(:,4), quat(:,1),quat(:,2),quat(:,3)];
 R_nx = out.R_n.Data(:,1);
 R_ny = out.R_n.Data(:,2);
 R_nz = out.R_n.Data(:,3);


 R_bx = out.R_b.Data(:,1);
 R_by = out.R_b.Data(:,2);
 R_bz = out.R_b.Data(:,3);

%dcmt = out.dcm_t.Data;
% 
% for i = 1:length(R_bz)
%  k(:,:,i) = out.k.signals.values(:,:,i);
% 
%  [evec,eval] = eig(k(:,:,i));
% d = diag(eval);
% [max_eval(i), index] = max(real(d));
% max_evec(:,i) = real(evec(:,index));
% %  [max_eval(i), index] = max(max(eval));
% % max_evec(:,i) = evec(:,index);
%     qu(i,1) = max_evec(2,i);
%     qu(i,2) = max_evec(3,i);
%     qu(i,3) = max_evec(4,i);
%     qu(i,4) = max_evec(1,i);
% Abar(:,:,i) = quat2dcm(qu(i,:));
% id(:,:,i) = Abar(:,:,i) * dcmt(:,:,i);
% 
% end



% lam_vec = out.lam_vec.Data(:,:);
% lambda = out.lambda.Data(:);
% 
% for i = 1:length(lam_vec)
%     qu(i,1) = lam_vec(i,2);
%     qu(i,2) = lam_vec(i,3);
%     qu(i,3) = lam_vec(i,4);
%     qu(i,4) = lam_vec(i,1);
% end


% for j = 1:length(lam_vec) - 1
%     for i = 1:4
%         diff(j,i) = lam_vec(j+1,i) - lam_vec(j,i);
%         if (diff(j,i) > 1.99 * abs(lam_vec(j,i)) ||diff(j,i) < -1.99 * abs(lam_vec(j,i)) )
%             lam_vec(j+1,i) =   lam_vec(j,i);      %Change sign
%         end
%     end
% end

 for i=1:length(quat)
     N(i,1)=quat(i,1)^2+quat(i,2)^2+quat(i,3)^2+quat(i,4)^2; 
     RT(i,1) = sqrt(R_nx(i)^2 + R_ny(i)^2 + R_nz(i)^2);
     posiz(i,:) = [R_nx(i), R_ny(i), R_nz(i)];
     
 end

 %Average error attitude determination
 med_err_triad = mean(out.err_triad.signals.values);

 w = out.W_vec.Data;
 wx = w(:,1);
 wy = w(:,2);
 wz = w(:,3);
 
 
 A=quat2dcm(quat);
 Hb_vec=zeros(length(A),3);
 Hn=zeros(length(A),3);
 wx=w(:,1);
 wy=w(:,2);
 wz=w(:,3);
 hb0=sqrt(Ix^2*w(1,1).^2 + Iy^2*w(1,2).^2 + Iz^2*w(1,3).^2); 
 hn0_vec=inv(A(:,:,1))*hb0';
 hn0=norm(hn0_vec);
 K0 = 0.5*(Ix*w(i,1).^2 + Iy*w(i,2).^2 +Iz*w(i,3).^2);

 for i=1:length(A)
     deter(i,1)=det(A(:,:,i));
     hb(i,1)=sqrt(Ix^2*w(i,1).^2 + Iy^2*w(i,2).^2 + Iz^2*w(i,3).^2);  %Angular momentum body
     Hb_vec(i,:)=[Ix*w(i,1) Iy*w(i,2) Iz*w(i,3)];
     Hn(i,:)=inv(A(:,:,i))*Hb_vec(i,:)';             
     Hn_mod(i,1)=norm(Hn(i,:));
     Hb_mod(i,1)=norm(Hb_vec(i,:));        
     K(i,1)=0.5*(Ix*w(i,1).^2 + Iy*w(i,2).^2 + Iz*w(i,3).^2);
     T_mod(i,1)=norm(K(i,:));
     Hn_var(i,:) = Hn_mod(i,:) - hn0;
     K_var(i)=K(i,1)-K0;
 end
 
t=linspace(0,simtime,length(A));

% figure(88)
% plot(t,qu(:,1),t,qu(:,2),t, qu(:,3),t,qu(:,4))


% figure(11)
% plot(t',movmean(lam_vec(:,1),4))
% hold on;
%     plot(t',movmean(lam_vec(:,2),4))
%     plot(t',movmean(lam_vec(:,3),4))
%     plot(t',movmean(lam_vec(:,4),4))

figure(1)
 plot(t,deter)
 grid on;
 legend('DetA from quaternions'); 
 
 %check for the conservation of angular momentum
 figure(2)
 plot(t,Hb_mod,t,Hn_mod,t,K);
 grid on;
 legend('Angular momentum body','Ang mom inertial frame','Kinetic energy');
 
 figure(9)
 plot(t,Hn_var);
 title('Variation angular mom inertial from first time step');

 figure(10)
 plot(t,K_var);
 title('Variation Kinetic energy from 1 time step');

 %figure(11)
 %plot(t,A_var(1,1),t,A_var(2,1),t,A_var(3,1),t,A_var(1,2),t,A_var(2,2),t,A_var(3,2),t,A_var(1,3),t,A_var(2,3),t,A_var(3,3));
 
 figure(3)
 plot(t,quat(:,1),t,quat(:,2),t, quat(:,3),t,quat(:,4))
 title('Quaternions');
 legend('q1','q2','q3','q4');

 figure(4)
 plot3(wx,wy,wz)
 title('Angular velocities');
 grid on;

 figure(5)
 plot(t,wx,t,wy,t,wz);
 legend('Wx','Wy','Wz');
 title('Ang. velocities');

 
 
 figure(6)
 plot(t,Hb_vec);
 title('Angular mom body components');
 legend('Hx_{b}','Hy_{b}','Hz_{b}');
 grid on;

 figure(7)
 plot(t,Hn);
 grid on;
 title('Ang mom inertial components');
 legend('Hx_{in}','Hy_{in}','Hz_{in}');


 
 
 %Orbita plot
 figure(8)
 DisegnaOrbita(orbit.a,orbit.e,orbit.w_p,orbit.incl,orbit.omega,orbit.mu);
 hold on;
 DisegnaPunto(orbit.p,orbit.e,orbit.w_p,orbit.incl,orbit.omega,0);
 hold on;
 earth_sphere;
 hold off;
 grid on;
 legend('','Release','');
 title('Orbit');

 
 