%% PLY Collision Detection
function [collision] = PLY_Collision_Detection(state, LocationX,LocationY, objectLength, objectWidth,workspaceHeight, robot, q1,c1,c2,c3,side_length)
    switch state
        case 1 %Tests whether a PLY File shaped as a rectangle will interact with the area covered by the safety barriers
            % Create the workspace boundary
            Corner1 = [-0.47, -0.45, 0.0];
            Corner2 = [-0.47, 0.5, 0.0];
            Corner3 = [0.35, -0.45, 0.0];
            Corner4 = [0.35, 0.5, 0.0];
            IntersectX = false;
            IntersectY = false;
            %display(Corner1(1,2));
            % Check for Object Intersecting the X of the workspace boundary
            if ((LocationX+objectLength/2) <= Corner3(1,1)) && ((LocationX-objectLength/2) >= Corner1(1,1))
                IntersectX = true;
            end
            if ((LocationY+objectWidth/2) <= Corner2(1,2)) && ((LocationY-objectWidth/2) >= Corner3(1,2))
                IntersectY = true;
            end
            if IntersectX == true && IntersectY == true
                collision = true;
            else
                collision = false;
            end
        case 2 %Test whether the hans cute robot will collide with a ply file shaped as a rectangle            
            %plottingCollisionDetection(robot,q1,q1,c1,c2,c3,side_length);
            collision = plottingCollisionDetection(robot,q1,c1,c2,c3,side_length);
            %display(collision);
    end

end
%% Check for collisions between Hans Cute and 
function [output] = plottingCollisionDetection(robot,q1,c1,c2,c3,side_length)
    output = false;
    q = zeros(1,3); 
    centerpnt = [c1,c2,c3];
    plotOptions.plotFaces = true;
    [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side_length/2, centerpnt+side_length/2,plotOptions);
    axis equal

    % Get the transform of every joint (i.e. start and end of every link)
    tr = zeros(4,4,robot.n+1);
    tr(:,:,1) = robot.base;
    L = robot.links;
    for i = 1 : robot.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end

    qMatrix = jtraj(q1,q1,1);
    result = true(1,1);
    result(1) = IsCollision(robot,qMatrix(1,:),faces,vertex,faceNormals,false);
    if result(1) == true
        output = true;
    end          
end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
    if nargin < 6
        returnOnceFound = true;
    end
    result = false;
    
    for qIndex = 1:size(qMatrix,1)
        % Get the transform of every joint (i.e. start and end of every link)
        tr = GetLinkPoses(qMatrix(qIndex,:), robot);
    
        % Go through each link and also each triangle face
        for i = 1 : size(tr,3)-1    
            for faceIndex = 1:size(faces,1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                    plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                    display('Intersection');
                    result = true;
                    if returnOnceFound
                        return
                    end
                end
            end    
        end
    end
end
%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)
    links = robot.links;
    transforms = zeros(4, 4, length(links) + 1);
    transforms(:,:,1) = robot.base;
    
    for i = 1:length(links)
        L = links(1,i);
        
        current_transform = transforms(:,:, i);
        
        current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
        transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
        transforms(:,:,i + 1) = current_transform;
    end
end