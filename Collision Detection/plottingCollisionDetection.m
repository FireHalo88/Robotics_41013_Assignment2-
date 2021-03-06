%% Collision Detection Code
%   Create the workspace area
function [collision] = plottingCollisionDetection(robot,q1,q2,centerpnt, translation, width, depth, height)
    collision = false;
%     q = zeros(1,3); 
    plotOptions.plotFaces = true;
    [vertex,faces,faceNormals] = ActualRectangularPrism(centerpnt, translation, width, depth, height ,plotOptions);
    axis equal
    
    % Get the transform of every joint (i.e. start and end of every link)
    tr = zeros(4,4,robot.n+1);
    tr(:,:,1) = robot.base;
    L = robot.links;
    for i = 1 : robot.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end
    
    %Go through until there are no step sizes larger than 1 degree
    steps = 2;
    while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
        steps = steps + 1;
    end
    qMatrix = jtraj(q1,q2,steps);
    
    % 2.7
    result = true(steps,1);
    for i = 1: steps
        result(i) = IsCollision(robot,qMatrix(i,:),faces,vertex,faceNormals,true);
        if result(i) == true
            collision = true;
            %robot.animate(qMatrix(i,:));
            break
        end
        %robot.animate(qMatrix(i,:));
        %pause(0.02);
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
                    %plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                    %display('Intersection');
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
    %display(links);
    transforms = zeros(4, 4, length(links) + 1);
    transforms(:,:,1) = robot.base;
    
    for i = 1:length(links)
        L = links(1,i);
        
        current_transform = transforms(:,:, i);
%         display("First Current Transform: ")
%         display(current_transform);
        current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
        transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
%         display("Second Current Transform: ");
%         display(current_transform);
        transforms(:,:,i + 1) = current_transform;
    end
end