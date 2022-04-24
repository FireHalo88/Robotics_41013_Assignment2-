classdef createEnvironment < handle
    properties

        %DH parameters of each object created (ignoring robots)
        workspace = [];
        x = [];

    end

    methods%%   
function self = createEnvironment(workspace)
            %%x = 1;
            %Makes the floor 
        surf([workspace(1,1), workspace(1,1); workspace(1,2), workspace(1,2)], ...
            [workspace(1,3), workspace(1,4); workspace(1,3), workspace(1,4)], ...
            [0.01, 0.01; 0.01, 0.01], ...
            'CData',imread('woodenFloor.jpg'), 'FaceColor','texturemap');
        hold on;
        %Makes the brick wall along the x axis
        surf([workspace(1,1), workspace(1,2); workspace(1,1), workspace(1,2)], ...
            [workspace(1,3), workspace(1,3); workspace(1,3), workspace(1,3)], ...
            [workspace(1,6), workspace(1,6); 0.0, 0.0], ...
            'CData',imread('galleryWall.jpg'), 'FaceColor','texturemap');
        hold on;
        %Makes the brick wall along the y axis
        surf([workspace(1,1), workspace(1,1); workspace(1,1), workspace(1,1)], ...
            [workspace(1,4), workspace(1,3); workspace(1,4), workspace(1,3)], ...
            [workspace(1,6), workspace(1,6); 0, 0], ...
            'CData',imread('galleryWall.jpg'), 'FaceColor','texturemap');  
        hold on;
        %self.placeObjects();
end
function placeObjects(class,paint_pot_1, paint_pot_2, paint_pot_3, canvas) 
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
end
end