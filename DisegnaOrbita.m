function DisegnaOrbita(a,e,w,i,omega,mu)
p=a*(1-e^2);
theta_dis=linspace(0,2*pi,1000);
R= p./(1+e.*cos(theta_dis));
x=R.*cos(theta_dis);
y=R.*sin(theta_dis);
z=0.*theta_dis;
Rw=[cos(w) sin(w) 0;-sin(w) cos(w) 0; 0 0 1];
Ri=[1 0 0;0 cos(i) sin(i); 0 -sin(i) cos(i)];
Romega=[cos(omega) sin(omega) 0; -sin(omega) cos(omega) 0;0 0 1];
Rge2pf=Rw*Ri*Romega;
Rpf2ge=inv(Rge2pf);
for j=1:size(theta_dis,2)
    rpf=[x(j);y(j);z(j)];
    r_dis=Rpf2ge*rpf;
    x_dis(j)=r_dis(1);
    y_dis(j)=r_dis(2);
    z_dis(j)=r_dis(3);
end
plot3(x_dis,y_dis,z_dis,'linewidth',2);