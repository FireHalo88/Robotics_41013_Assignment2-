function [vertex,face,faceNormals] = ActualRectangularPrism(centerpnt, translation, width, depth, height ,plotOptions,axis_h)
if nargin<4
        axis_h=gca;
    if nargin<3
        plotOptions.plotVerts=false;
        plotOptions.plotEdges=true;
        plotOptions.plotFaces=true;
    end
end

vertex(1,:) = [centerpnt(1)-width+translation(1), centerpnt(2)-depth+translation(2), translation(3)];
vertex(2,:) = [centerpnt(1)+width+translation(1), centerpnt(2)-depth+translation(2), translation(3)]; %?
vertex(3,:) = [centerpnt(1)+width+translation(1), centerpnt(2)+depth+translation(2), translation(3)];
vertex(4,:) = [centerpnt(1)+width+translation(1), centerpnt(2)-depth+translation(2), height+translation(3)];
vertex(5,:) = [centerpnt(1)-width+translation(1), centerpnt(2)+depth+translation(2), height+translation(3)];
vertex(6,:) = [centerpnt(1)-width+translation(1), centerpnt(2)-depth+translation(2), height+translation(3)];
vertex(7,:) = [centerpnt(1)-width+translation(1), centerpnt(2)+depth+translation(2), translation(3)];
vertex(8,:) = [centerpnt(1)+width+translation(1), centerpnt(2)+depth+translation(2), height+translation(3)];

% vertex(1,:)=[-0.2 0 0];
% vertex(2,:)=[0.14 0 0];
% vertex(3,:)=[0.14 0.2 0];
% vertex(4,:)=[0.14 0 0.48];
% vertex(5,:)=[-0.2 0.2 0.48];
% vertex(6,:)=[-0.2 0 0.48];
% vertex(7,:)=[-0.2 0.2 0];
% vertex(8,:)=[0.14 0.2 0.48];

face=[1,2,3;1,3,7;
     1,6,5;1,7,5;
     1,6,4;1,4,2;
     6,4,8;6,5,8;
     2,4,8;2,3,8;
     3,7,5;3,8,5;
     6,5,8;6,4,8];

if 2 < nargout    
    faceNormals = zeros(size(face,1),3);
    for faceIndex = 1:size(face,1)
        v1 = vertex(face(faceIndex,1)',:);
        v2 = vertex(face(faceIndex,2)',:);
        v3 = vertex(face(faceIndex,3)',:);
        faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end
end
%% If plot verticies
hold on
if isfield(plotOptions,'plotVerts') && plotOptions.plotVerts
    for i=1:size(vertex,1);
        plot3(vertex(i,1),vertex(i,2),vertex(i,3),'r*');
        text(vertex(i,1),vertex(i,2),vertex(i,3),num2str(i));
    end
end

%% If you want to plot the edges
if isfield(plotOptions,'plotEdges') && plotOptions.plotEdges
    links=[1,2;
        2,3;
        3,7;
        7,1;
        1,6;
        5,6;
        5,7;
        4,8;
        5,8;
        6,4;
        4,2;
        8,3];

    for i=1:size(links,1)
        hold on
        plot3(axis_h,[vertex(links(i,1),1),vertex(links(i,2),1)],...
            [vertex(links(i,1),2),vertex(links(i,2),2)],...
            [vertex(links(i,1),3),vertex(links(i,2),3)],'k')
    end
end

%% If you want to plot the edges
if isfield(plotOptions,'plotFaces') && plotOptions.plotFaces
    hold on
    tcolor = [ 0.3 1.44 2.55];
    plot = patch('Faces',face,'Vertices',vertex,'FaceVertexCData',tcolor,'FaceColor','flat','lineStyle','none');
    plot.FaceVertexAlphaData = 0.00002;
    plot.FaceAlpha = 0.05;
    delete(plot);
end

end