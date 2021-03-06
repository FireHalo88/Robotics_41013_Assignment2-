   
% translation = [0 0 0];
% 
% % guardDimensions = [0.12 0.1 0.7];
    safetyBarrierPoint1 = [-0.43, -0.52, 0.0];
    safetyBarrierPoint2 = [-0.43, 0.46, 0.0];    
    safetyBarrierPoint3 = [0.42, -0.52, 0.0];
    safetyBarrierPoint4 = [0.42, 0.46, 0.0];
    object = 2;
    translation = [-0.45, 0, 0];
    switch object
        case 1 %For Boy entering workspace
            collision = false;
            centerpnt = [0.055,-0.35];
            translation = [translation(1) + centerpnt(1), translation(2) + centerpnt(2), translation(3)];
            %check if the translation enters the workspace
            if(translation (1,1) > safetyBarrierPoint1(1,1) && translation (1,1) < safetyBarrierPoint4(1,1))
                if(translation (1,2) > safetyBarrierPoint1(1,2) && translation (1,2) < safetyBarrierPoint4(1,2))
                    collision = true;
                end
            end
        case 2 %Check if joint are outside the light curtain area
            collision = false;
            if(translation (1,1) < safetyBarrierPoint1(1,1) || translation (1,1) > safetyBarrierPoint4(1,1))
                if(translation (1,2) < safetyBarrierPoint1(1,2) || translation (1,2) > safetyBarrierPoint4(1,2))
                    collision = true; %Out Of Bounds
                end
            end
        end

% centerpnt = [0.19 0.2];
% width = 0.19;
% depth = 0.2;
% height = 0.032;
% 
% plotOptions.plotFaces = true;
% [vertex,faces,faceNormals] = ActualRectangularPrism(centerpnt, translation, width, depth, height ,plotOptions);
% axis equal
% plotOptions.plotFaces = true;
%             %Sets up collision rectangles for the canvas and table
%             table_translation = [0.0, 0.0, -0.005];    
%             canvas_translation = [-0.3, -0.2, 0.22];                        
%         
%             %Places Collision Detection for Table            
%             [table_centerpnt, table_width, table_depth, table_height] = PLY_Obstacle_Dimensions(3,0);        
%             [table_vertex,table_faces,table_faceNormals] = ActualRectangularPrism(table_centerpnt, table_translation, table_width, table_depth, table_height ,plotOptions);
%             axis equal
%         
%             %Places Collision Detection for Canvas
%             [canvas_centerpnt, canvas_width, canvas_depth, canvas_height] = PLY_Obstacle_Dimensions(4,0);
%             [canvas_vertex,canvas_faces,canvas_faceNormals] = ActualRectangularPrism(canvas_centerpnt, canvas_translation, canvas_width, canvas_depth, canvas_height ,plotOptions);
%             axis equal

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