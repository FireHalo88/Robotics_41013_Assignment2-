   
% translation = [0 0 0];
% 
% % guardDimensions = [0.12 0.1 0.7];
% centerpnt = [0.19 0.2];
% width = 0.19;
% depth = 0.2;
% height = 0.032;
% 
% plotOptions.plotFaces = true;
% [vertex,faces,faceNormals] = ActualRectangularPrism(centerpnt, translation, width, depth, height ,plotOptions);
% axis equal
plotOptions.plotFaces = true;
            %Sets up collision rectangles for the canvas and table
            table_translation = [0.0, 0.0, -0.005];    
            canvas_translation = [-0.3, -0.2, 0.22];                        
        
            %Places Collision Detection for Table            
            [table_centerpnt, table_width, table_depth, table_height] = PLY_Obstacle_Dimensions(3,0);        
            [table_vertex,table_faces,table_faceNormals] = ActualRectangularPrism(table_centerpnt, table_translation, table_width, table_depth, table_height ,plotOptions);
            axis equal
        
            %Places Collision Detection for Canvas
            [canvas_centerpnt, canvas_width, canvas_depth, canvas_height] = PLY_Obstacle_Dimensions(4,0);
            [canvas_vertex,canvas_faces,canvas_faceNormals] = ActualRectangularPrism(canvas_centerpnt, canvas_translation, canvas_width, canvas_depth, canvas_height ,plotOptions);
            axis equal

% translation = transl(0.0, 0.0, 0.0);    
% canvas_translation = transl(-1.0, 1.0, 10);   
% %Places Collision Detection for Table
% [table_centerpnt, table_width, table_depth, table_height] = PLY_Obstacle_Dimensions(3,1);
% plotOptions.plotFaces = true;
% [table_vertex,table_faces,table_faceNormals] = ActualRectangularPrism(table_centerpnt, translation, table_width, table_depth, table_height ,plotOptions);
% axis equal
% 
% %Places Collision Detection for Canvas
% canvas_translation = [-0.3, -0.2, 0.22];
% canvas_translation = [0, 0, 0.22];
% [canvas_centerpnt, canvas_width, canvas_depth, canvas_height] = PLY_Obstacle_Dimensions(4,0);
% plotOptions.plotFaces = true;
% [canvas_vertex,canvas_faces,canvas_faceNormals] = ActualRectangularPrism(canvas_centerpnt, canvas_translation, canvas_width, canvas_depth, canvas_height ,plotOptions);
% axis equal

% translation = [0, -0.19, 0.145];
% [centerpnt, width, depth, height] = PLY_Obstacle_Dimensions(1,1);
% 
%     plotOptions.plotFaces = true;
%     [vertex,faces,faceNormals] = ActualRectangularPrism(centerpnt, translation, width, depth, height ,plotOptions);
%     axis equal

%         line([4 4],[0 10],'color','b') % Blue line from (4,0) to (4,10)
        %line([0 10],[9 9],'color','r') % Red line from (0,9) to (10,9)
        %axis([0 10 0 10]) % Set the axis limits
%         x = 23750;
%         y = mod(x,1)