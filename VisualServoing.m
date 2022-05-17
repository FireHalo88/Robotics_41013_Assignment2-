classdef VisualServoing < handle
    % This class will handle the movement of SerialLink Robots and Mesh
    % Objects in a defined workspace environment. It will log details to a
    % log file defined as a property of the class.
    
    properties
        L;      % LogFile
        stopSign_h; % Stop Sign Surface Plot
        ssMesh_h;   % Stop Sign PLY Mesh
        sign_T;     % Global Transform of the Sign
        signLength = 0.15;   % Length of the sign in m.
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
            %ssMesh_h = PlaceObject('StopSign_Resized2_PLY.ply');
            ssMesh_h = PlaceObject('StopSign_15x15_PLY.ply');
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
            self.sign_T = sign_T;
        end
        
        function MoveSign(self, objMesh_h, objVertices, sign_T)
            % Transform Object to this Pose
            objTransformVertices = [objVertices,ones(size(objVertices,1),1)] ...
                * sign_T';
            set(objMesh_h, 'Vertices', objTransformVertices(:,1:3));
            
            self.sign_T = sign_T;
               
            drawnow();
        end
        
        function [signFromEE_T] = signFromEE(self, robot)
            % This function takes a Robot Manipulator and determines the
            % transform from the End Effector to the current pose of the
            % Stop Sign
            currQ = robot.getpos();
            EE_TR = robot.fkine(currQ);
            self.sign_T
            signFromEE_T = inv(EE_TR)*self.sign_T;
        end
        
        function [P] = determineCartesianCornerPoints(self, sign_T)
            % This function will return a matrix of the XYZ points defining
            % the eight corners of the Stop Sign Octagon, given its 4x4
            % transform (which is the centre of the sign) - Note that the
            % thickness of the sign is negligable.
            
            % Pre-allocate size for the P array
            P = zeros(3, 8);    % 3 Rows for XYZ, 8 Columns for 8 Corner Coordinates
            % Get XYZ of Sign Centre
            centreSign = sign_T(1:3, 4);
            
            % Defining Corners (TOP RIGHT -> CCW)
            % Top Right
            topRight = [centreSign(1); centreSign(2)+self.signLength/4; ...
                centreSign(3)+self.signLength/2];
            P(:, 1) = topRight;
            
            % Top Left
            topLeft = [centreSign(1); centreSign(2)-self.signLength/4; ...
                centreSign(3)+self.signLength/2];
            P(:, 2) = topLeft;
            
            % Top Middle Left
            topMiddleLeft = [centreSign(1); centreSign(2)-self.signLength/2; ...
                centreSign(3)+self.signLength/4];
            P(:, 3) = topMiddleLeft;
            
            % Bottom Middle Left
            bottomMiddleLeft = [centreSign(1); centreSign(2)-self.signLength/2; ...
                centreSign(3)-self.signLength/4];
            P(:, 4) = bottomMiddleLeft;
            
            % Bottom Left
            bottomLeft = [centreSign(1); centreSign(2)-self.signLength/4; ...
                centreSign(3)-self.signLength/2];
            P(:, 5) = bottomLeft;
            
            % Bottom Right
            bottomRight = [centreSign(1); centreSign(2)+self.signLength/4; ...
                centreSign(3)-self.signLength/2];
            P(:, 6) = bottomRight;
            
            % Bottom Middle Right
            bottomMiddleRight = [centreSign(1); centreSign(2)+self.signLength/2; ...
                centreSign(3)-self.signLength/4];
            P(:, 7) = bottomMiddleRight;
            
            % Top Middle Right
            topMiddleRight = [centreSign(1); centreSign(2)+self.signLength/2; ...
                centreSign(3)+self.signLength/4];
            P(:, 8) = topMiddleRight;
            
        end
        
        function deleteSign(self)
            disp('Hello?');
            try delete(self.ssMesh_h); end
            try delete(self.stopSign_h); end
        end
        
    end
end