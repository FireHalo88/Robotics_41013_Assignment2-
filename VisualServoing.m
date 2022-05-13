classdef VisualServoing < handle
    % This class will handle the movement of SerialLink Robots and Mesh
    % Objects in a defined workspace environment. It will log details to a
    % log file defined as a property of the class.
    
    properties
        L;      % LogFile
        stopSign_h; % Stop Sign Surface Plot
        ssMesh_h;   % Stop Sign PLY Mesh
    end
    
    methods
        function self = VisualServoing()
           self.L = log4matlab('Visual_Servoing_Log.log');
        end
        
        function[stopSign_h] = plotSign(self, robot, depth)
            try delete(self.stopSign_h); end
            
            % Get Robot EE 4x4 Transform and XYZ Position
            q = robot.getpos();
            EE_TR = robot.fkine(q);
            ee_XYZ = EE_TR(1:3, 4);
            
            % Plot Stop Sign
            startX = -0.3;
            startY = 0;
            startZ = 0.5;
            size = 0.2;
            % Stretch Pattern ([X Coords], [Y Coords], [Z Coords])
            stopSign_h = surf([startX, startX; startX, startX], ...
                              [startY+size/2, startY-size/2; startY+size/2, startY-size/2], ...
                              [startZ+size/2, startZ+size/2; startZ-size/2, startZ-size/2], ...
                              'CData',imread('Stop_sign.png'),'FaceColor','texturemap');
            self.stopSign_h = stopSign_h;
        end
        
        function [ssMesh_h, ssVertices] = CreateSign(self, sign_T)
            % This function will use the 'Brick.ply' file to create an instance ...
            % of a brick mesh, and then get its vertices and place it in the 
            % environment at a specified location.

            % Add a Brick with the 'PlaceObject' function and using ...
            % 'Brick.ply' as the input argument. Then get the vertices ...
            % out of the mesh handle to use later on.
            ssMesh_h = PlaceObject('StopSign_Resized2_PLY.ply');
            %axis equal
            ssVertices = get(ssMesh_h,'Vertices');

            % Translate the vertices that make up the Brick Mesh by ...
            % a 4x4 Homogenous Matrix to place the Brick in a specific ...
            % location. Note that as the vertices are an Nx3 Matrix, a ...
            % column vector of ones needed to be placed onto the matrix ...
            % to allow for multiplication of a 4x4 Matrix.
            ssTransformVertices = [ssVertices,ones(size(ssVertices,1),1)] * sign_T';

            % Set the Vertices property in the Brick Mesh using the ...
            % first 3 columns (x,y,z values) of the transformed vertices.
            % (:, 1:3) = All rows in columns 1-3.
            set(ssMesh_h, 'Vertices', ssTransformVertices(:, 1:3));
            
            self.ssMesh_h = ssMesh_h;
        end
        
        function MoveSign(~, objMesh_h, objVertices, sign_T)
            % Transform Object to this Pose
            objTransformVertices = [objVertices,ones(size(objVertices,1),1)] ...
                * sign_T';
            set(objMesh_h, 'Vertices', objTransformVertices(:,1:3));
               
            drawnow();
        end
        
        function deleteSign(self)
            disp('Hello?');
            try delete(self.ssMesh_h); end
            try delete(self.stopSign_h); end
        end
        
    end
end