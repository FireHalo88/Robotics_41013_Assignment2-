classdef EnvironmentCreation < handle
    % This class will handle the creation of object and object meshes in
    % the workspace.
    
    properties
        L;      % LogFile for debugging purposes
        
    end
    
    methods        
        function [mesh_h, vertices] = CreateObject(self, name, T)
            % This function will use the '<object>.ply' file to create an ... 
            % instance of an object mesh, and then get its vertices and ...
            % place it in the environment at a specified pose (4x4 Matrix).
            funcName = 'CreateObject';

            % Add a object with the 'PlaceObject' function and using ...
            % '<object>.ply' as the input argument. Then get the vertices ...
            % out of the mesh handle to use later on.
            mesh_h = PlaceObject(name);
            vertices = get(mesh_h,'Vertices');

            % Translate the vertices that make up the Brick Mesh by ...
            % a 4x4 Homogenous Matrix to place the Brick in a specific ...
            % location. Note that as the vertices are an Nx3 Matrix, a ...
            % column vector of ones needed to be placed onto the matrix ...
            % to allow for multiplication of a 4x4 Matrix.
            transformVertices = [vertices,ones(size(vertices,1),1)] * T';

            % Set the Vertices property in the Brick Mesh using the ...
            % first 3 columns (x,y,z values) of the transformed vertices.
            % (:, 1:3) = All rows in columns 1-3.
            set(mesh_h, 'Vertices', transformVertices(:, 1:3));
            
            % Log a message to the log file to confirm that this object has
            % been successfully loaded/placed in the environment.
            self.L.mlog = {self.L.DEBUG,funcName,['New object added to ', ...
                'environment: ', name, char(13)]};
            
        end       
    end
            
end