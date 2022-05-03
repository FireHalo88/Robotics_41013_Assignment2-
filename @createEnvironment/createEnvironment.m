classdef createEnvironment < handle
    properties

        %DH parameters of each object created (ignoring robots)
        workspace = [];
        x = [];

    end

    methods%%   
function self = createEnvironment(workspace)
            %%x = 1;          
        % Save Workspace
        self.workspace = workspace;
        
        hold on;
        
            %Makes the floor 
        surf([self.workspace(1,1), self.workspace(1,1); self.workspace(1,2), self.workspace(1,2)], ...
            [self.workspace(1,3), self.workspace(1,4); self.workspace(1,3), self.workspace(1,4)], ...
            [0.01, 0.01; 0.01, 0.01], ...
            'CData',imread('woodenFloor.jpg'), 'FaceColor','texturemap');
        
        %Makes the brick wall along the x axis
        surf([self.workspace(1,1), self.workspace(1,2); self.workspace(1,1), self.workspace(1,2)], ...
            [self.workspace(1,4), self.workspace(1,4); self.workspace(1,4), self.workspace(1,4)], ...
            [self.workspace(1,6), self.workspace(1,6); 0.0, 0.0], ...
            'CData',imread('galleryWall.jpg'), 'FaceColor','texturemap');
        
        %Makes the brick wall along the y axis
        surf([workspace(1,1), workspace(1,1); workspace(1,1), workspace(1,1)], ...
            [workspace(1,4), workspace(1,3); workspace(1,4), workspace(1,3)], ...
            [workspace(1,6), workspace(1,6); 0, 0], ...
            'CData',imread('galleryWall.jpg'), 'FaceColor','texturemap');
        
        % Emergency Exit Sign
        % Stretch Pattern ([X Coords], [Y Coords], [Z Coords])
        surf([self.workspace(1,1)+0.01, self.workspace(1,1)+0.01;
              self.workspace(1,1)+0.01, self.workspace(1,1)+0.01], ...
             [self.workspace(1,3)+0.1, self.workspace(1,3)+0.3;
              self.workspace(1,3)+0.1, self.workspace(1,3)+0.3], ...
             [self.workspace(1,6)-0.05, self.workspace(1,6)-0.05; ...
             self.workspace(1,6)-0.2, self.workspace(1,6)-0.2], ...
             'CData',imread('e_exitsign.jpg'),'FaceColor','texturemap'); 
        
        %self.placeObjects();
end
function placeObjects(~, paint_pot_1, paint_pot_2, paint_pot_3, canvas) 
    L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360,360]), 'offset', 0);     
    qa = zeros(1,1);  
    %For cups
    for i = 1:3
        %Placing a cup in the environment as '1-link robot'                                                                                                       
        [faceData,vertexData] = plyread('cup.ply','tri');  
        %Creating Bricks as '1-Link' objects and placing them in the
        %environment based on P transforms
        translation = [0 0 0];
        switch i
            case 1
                display("Pot 1");
                Cup = SerialLink([L1],'name','cup','base',transl(paint_pot_1));          
                Cup.faces = {faceData,[]};                           % Referring to robotcows.m file
                Cup.points = {vertexData,[]};                        % Inputting brick faces and vertices                                        % Creating initial joint angles as zero for all bricks
                Cup.plot3d(qa,'scale',1);                  
            case 2
                display("Pot 2");
                Cup2 = SerialLink([L1],'name','cup2','base',transl(paint_pot_2));          
                Cup2.faces = {faceData,[]};                           % Referring to robotcows.m file
                Cup2.points = {vertexData,[]};                        % Inputting brick faces and vertices                                        % Creating initial joint angles as zero for all bricks
                Cup2.plot3d(qa,'scale',1);                 
            case 3
                
                display("Pot 3"); 
                Cup3 = SerialLink([L1],'name','cup3','base',transl(paint_pot_3));          
                Cup3.faces = {faceData,[]};                           % Referring to robotcows.m file
                Cup3.points = {vertexData,[]};                        % Inputting brick faces and vertices                                        % Creating initial joint angles as zero for all bricks
                Cup3.plot3d(qa,'scale',1); 
        end

        if isempty(findobj(get(gca,'Children'),'Type','Light'))
            camlight
        end  
     end

%     %For Canvas
        %Placing a cup in the environment as '1-link robot'                                                                                                       
        [faceData,vertexData] = plyread('canvas3.ply','tri');  
        %Creating Bricks as '1-Link' objects and placing them in the
        %environment based on P transforms
        Canvas = SerialLink([L1],'name','canvas','base',transl(canvas));     % Inputting 9 bricks     
        Canvas.faces = {faceData,[]};                           % Referring to robotcows.m file
        Canvas.points = {vertexData,[]};                        % Inputting brick faces and vertices                                        % Creating initial joint angles as zero for all bricks
        Canvas.plot3d(qa,'scale',0.0001); 
end
function[object_h, objectVertices] = placeObjectsBetter(~, canvas,table, ...
        pen1, pen2, pen3, pen4, safetyBarrier1, safetyBarrier2, ...
        safetyBarrier3, safetyBarrier4, guard, fireExtinguisher)
    
        % Plotting Objects
        object_h = {0 0 0 0 0 0 0 0 0 0 0 0 0 0}; % Cell array for storing token handles
        objectVertices = {0 0 0 0 0 0 0 0 0 0 0 0 0 0}; % Cell array for storing token vertices
        for i = 1:1:12 % Plotting all tokens
            switch i
                %Canvas
                case 1 
                    object_h{i} = PlaceObject('canvas2.ply'); % Importing Canvas
                    objectVertices{i} = get(object_h{i},'Vertices'); % Extracting vertices data
                    transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * canvas'; % Transforming vertices
                    set(object_h{i},'Vertices',transformedVertices(:,1:3)); % Updating token location
                    drawnow; % Update simulation
                    pause(0.001); % Wait before execution                    
                case 2
                    object_h{i} = PlaceObject('brown_table.ply');
                    objectVertices{i} = get(object_h{i},'Vertices'); % Extracting vertices data
                    transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * table'; % Transforming vertices    
                    set(object_h{i},'Vertices',transformedVertices(:,1:3)); % Updating token location
                    drawnow; % Update simulation
                    pause(0.001); % Wait before execution                    
                case 3
                    object_h{i} = PlaceObject('black_pen.ply');
                    objectVertices{i} = get(object_h{i},'Vertices'); % Extracting vertices data
                    transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * pen1'; % Transforming vertices
                    set(object_h{i},'Vertices',transformedVertices(:,1:3)); % Updating token location
                    drawnow; % Update simulation
                    pause(0.001); % Wait before execution                    
                case 4
                    object_h{i} = PlaceObject('red_pen.ply');
                    objectVertices{i} = get(object_h{i},'Vertices'); % Extracting vertices data
                    transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * pen2'; % Transforming vertices
                    set(object_h{i},'Vertices',transformedVertices(:,1:3)); % Updating token location
                    drawnow; % Update simulation
                    pause(0.001); % Wait before execution                    
                case 5
                    object_h{i} = PlaceObject('green_pen.ply'); % Importing Pen
                    objectVertices{i} = get(object_h{i},'Vertices'); % Extracting vertices data
                    transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * pen3'; % Transforming vertices
                    set(object_h{i},'Vertices',transformedVertices(:,1:3)); % Updating token location
                    drawnow; % Update simulation
                    pause(0.001); % Wait before execution  
                case 6
                    object_h{i} = PlaceObject('blue_pen.ply'); % Importing Pen
                    objectVertices{i} = get(object_h{i},'Vertices'); % Extracting vertices data
                    transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * pen4'; % Transforming vertices
                    set(object_h{i},'Vertices',transformedVertices(:,1:3)); % Updating token location
                    drawnow; % Update simulation
                    pause(0.001); % Wait before execution                     
                case 7
                    object_h{i} = PlaceObject('safetyBarrier2.ply');
                    objectVertices{i} = get(object_h{i},'Vertices'); % Extracting vertices data
                    transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * safetyBarrier1'; % Transforming vertices
                    set(object_h{i},'Vertices',transformedVertices(:,1:3)); % Updating token location
                    drawnow; % Update simulation
                    pause(0.001); % Wait before execution                    
                case 8
                    object_h{i} = PlaceObject('safetyBarrier2.ply');
                    objectVertices{i} = get(object_h{i},'Vertices'); % Extracting vertices data
                    transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * safetyBarrier2'; % Transforming vertices
                    set(object_h{i},'Vertices',transformedVertices(:,1:3)); % Updating token location
                    drawnow; % Update simulation
                    pause(0.001); % Wait before execution                    
                case 9
                    object_h{i} = PlaceObject('safetyBarrier2.ply');
                    objectVertices{i} = get(object_h{i},'Vertices'); % Extracting vertices data
                    transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * safetyBarrier3'; % Transforming vertices
                    set(object_h{i},'Vertices',transformedVertices(:,1:3)); % Updating token location
                    drawnow; % Update simulation
                    pause(0.001); % Wait before execution                    
                case 10
                    object_h{i} = PlaceObject('safetyBarrier2.ply'); 
                    objectVertices{i} = get(object_h{i},'Vertices'); % Extracting vertices data
                    transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * safetyBarrier4'; % Transforming vertices
                    set(object_h{i},'Vertices',transformedVertices(:,1:3)); % Updating token location
                    drawnow; % Update simulation
                    pause(0.001); % Wait before execution                    
                case 11
                    object_h{i} = PlaceObject('guard.ply');
                    objectVertices{i} = get(object_h{i},'Vertices'); % Extracting vertices data
                    transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * guard'; % Transforming vertices
                    set(object_h{i},'Vertices',transformedVertices(:,1:3)); % Updating token location
                    drawnow; % Update simulation
                    pause(0.001); % Wait before execution                    
                case 12
                    object_h{i} = PlaceObject('fireExtinguisher.ply');
                    objectVertices{i} = get(object_h{i},'Vertices'); % Extracting vertices data
                    transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * fireExtinguisher'; % Transforming vertices
                    set(object_h{i},'Vertices',transformedVertices(:,1:3)); % Updating token location
                    drawnow; % Update simulation
                    pause(0.001); % Wait before execution  
            end
%             %objectVertices{i} = get(object_h{i},'Vertices'); % Extracting vertices data
%             switch i
%                 case 1
%                     transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * canvas'; % Transforming vertices
%                 case 2
%                     transformedVertices = [objectVertices{i},ones(size(objectVertices{i},1),1)] * pen'; % Transforming vertices
%             end
% %             set(object_h{i},'Vertices',transformedVertices(:,1:3)); % Updating token location
% %             drawnow; % Update simulation
% %             pause(0.001); % Wait before execution            
        end
        camlight
end
    end
end