classdef block < handle
    %BLOCK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
      state0
      stateS
      quat
      %Rb
      inertia
      tspan
      dt
      
      vertices
      faces 
      color
      X
      Y
      Z
      
      h
      
    end
    
    methods
        function obj = block(state0,quat,inertia,tspan,dt,vertices,faces,color)
            %BLOCK Construct an instance of this class
            %   Detailed explanation goes here
            obj.state0 = state0;
            obj.inertia=inertia;
            %obj.Rb = Rb;
            obj.quat=quat;
            obj.tspan=tspan;
            obj.dt=dt;
            obj.vertices=vertices;
            obj.faces=faces;
            obj.color=color;
            obj.draw();
        end
        
            
        function [] = simulate(obj)
            obj.stateS = zeros(length(obj.tspan), length(obj.state0));
            obj.stateS(1,:) = obj.state0;
            for a = 1:length(obj.tspan)-1
                obj.stateS(a+1,1:4) = obj.quat(a+1,:);
               
            end
        end

        function  [] = animate(obj)
                   xlim([-500 500])
                   ylim([-500 500])
                   zlim([-500 500])
                view(45,20)
            

                for i= 1:length(obj.tspan)
                    rotmat = quat2dcm(obj.stateS(i, 1:4));
% %                     obj.vertices(:,1) = obj.vertices(:,1) + obj.Rn(i,1);
% %                     obj.vertices(:,2) = obj.vertices(:,2) + obj.Rn(i,2);
% %                     obj.vertices(:,3) = obj.vertices(:,3) + obj.Rn(i,3);
                    obj.updateAttitude(rotmat)
                    title(num2str(obj.tspan(i) ,'%.1f'))
                    xNew = rotmat * [300 0 0]';
                    yNew = rotmat * [0 300 0]';
                    zNew = rotmat * [ 0 0 300]';
%                     obj.X = quiver3(0,0,0,xNew(1), xNew(2), xNew(3),'r','linewidth',2);
%                     obj.Y = quiver3(0,0,0,yNew(1), yNew(2),yNew(3),'g','LineWidth',2);
%                     obj.Z = quiver3(0,0,0,zNew(1),zNew(2),zNew(3),'k','LineWidth',2);
                    drawnow;
                    if length(obj.tspan)<10000
                    pause(1/100)
                    else
                        pause(1/1000)
                    end
                end
        end

                function [] = updateAttitude(obj, rotmat )
                    vNew = (rotmat * (obj.vertices)')';
                    set(obj.h, 'vertices',vNew);
                    set(obj.X,'Visible','off')
                    set(obj.Y,'Visible','off')
                    set(obj.Z,'Visible','off')
                end

        function [] = draw(obj)
           
           
            obj.h=patch('faces',obj.faces,'vertices',obj.vertices,'facecolor',obj.color);
            view(45,20);
            axis equal;
            grid on;
            rotate3d on;
            hold on;

            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            
            
            
        end
    end
end


