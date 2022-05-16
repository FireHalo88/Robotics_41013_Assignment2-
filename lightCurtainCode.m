function [collision] = lightCurtainCode(robot, translation,object)
    safetyBarrierPoint1 = [-0.43, -0.52, 0.0];
    safetyBarrierPoint2 = [-0.43, 0.46, 0.0];    
    safetyBarrierPoint3 = [0.42, -0.52, 0.0];
    safetyBarrierPoint4 = [0.42, 0.46, 0.0];

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
%             display(translation(1,4)+ " "+ translation(2,4));
            if(translation (1,4) < safetyBarrierPoint1(1,1) || translation (1,4) > safetyBarrierPoint4(1,1))
                collision = true; %Out Of Bounds   
            end                
            if(translation (2,4) < safetyBarrierPoint1(1,2) || translation (2,4) > safetyBarrierPoint4(1,2))                    
                collision = true; %Out Of Bounds                
            end

end