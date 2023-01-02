function DisegnaPunto(p,e,w,i,omega,theta)
dist=p/(1+e*cos(theta));
    p_x=dist*cos(theta);
    p_y=dist*sin(theta);     
    p_z=0;
    posizione=[p_x;p_y;p_z];
    Rw=[cos(w) sin(w) 0;-sin(w) cos(w) 0; 0 0 1];
    Ri=[1 0 0;0 cos(i) sin(i); 0 -sin(i) cos(i)];
    Romega=[cos(omega) sin(omega) 0; -sin(omega) cos(omega) 0;0 0 1];
    Rge2pf=Rw*Ri*Romega;
    Rpf2ge=inv(Rge2pf);
    p_IJK=Rpf2ge*posizione; 
    plot3(p_IJK(1),p_IJK(2),p_IJK(3),'.','Markersize',25);
    
   