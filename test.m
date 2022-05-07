    collision = false;
    q = zeros(1,3); 
    centerpnt = [-0.035,0.1]; %[-0.03,0.1,0.2275];
    translation = [2 2 2];  
    length = 2;
%     lower = centerpnt-length/2;
%     upper = centerpnt+length/2;
    plotOptions.plotFaces = true;

    width = 0.17;

    depth = 0.1;

    height = 0.48;    
    
%     corner1 = [centerpnt(1)-width+translation(1), centerpnt(2)-depth+translation(2), translation(3)];
%     corner2 = [centerpnt(1)+width+translation(1), translation(2), translation(3)];
% 
%     corner3 = [centerpnt(1)+width+translation(1), centerpnt(2)+depth+translation(2), translation(3)];
% 
%     corner4 = [centerpnt(1)+width+translation(1), centerpnt(2)-depth+translation(2), height+translation(3)];
% 
%     corner5 = [centerpnt(1)-width+translation(1), centerpnt(2)+depth+translation(2), height+translation(3)];
% 
%     corner6 = [centerpnt(1)-width+translation(1), centerpnt(2)-depth+translation(2), height+translation(3)];
% 
%     corner7 = [centerpnt(1)-width+translation(1), centerpnt(2)+depth+translation(2), translation(3)];
%     corner8 = [centerpnt(1)+width+translation(1), centerpnt(2)+depth+translation(2), height+translation(3)];

%     vertex(1,:)=[-0.2 0 0];
% vertex(2,:)=[0.14 0 0];
% vertex(3,:)=[0.14 0.2 0];
% vertex(4,:)=[0.14 0 0.48];
% vertex(5,:)=[-0.2 0.2 0.48];
% vertex(6,:)=[-0.2 0 0.48];
% vertex(7,:)=[-0.2 0.2 0];
% vertex(8,:)=[0.14 0.2 0.48];
%     vertex(1,:)=lower
%     %vertex(2,:)=[upper(1),lower(2:3)]
%     vertex(2,:)=[1 -1 -1];
%     vertex(3,:)=[upper(1:2),lower(3)]
%     vertex(4,:)=[upper(1),lower(2),upper(3)]
%     vertex(5,:)=[lower(1),upper(2:3)]
%     vertex(6,:)=[lower(1:2),upper(3)]
%     vertex(7,:)=[lower(1),upper(2),lower(3)]
%     vertex(8,:)=upper

    [vertex,faces,faceNormals] = ActualRectangularPrism(centerpnt, translation, width, depth, height ,plotOptions);
    axis equal