function [collision] = lightCurtainCode(translation,object)
    safetyBarrierPoint1 = [-0.43, -0.52, 0.0];
    safetyBarrierPoint2 = [-0.43, 0.46, 0.0];    
    safetyBarrierPoint3 = [0.42, -0.52, 0.0];
    safetyBarrierPoint4 = [0.42, 0.46, 0.0];

    curtain1 = [-0.425, -0.515, 0.0];
    curtain2 = [-0.425, 0.455, 0.0];    
    curtain3 = [0.415, -0.515, 0.0];
    curtain4 = [0.415, 0.455, 0.0];

    switch object
        case 1 %For Boy entering workspace
            collision = false;
            centerpnt = [0.12,-0.35];
            boyTranslation = [translation(1) - centerpnt(1), translation(2), translation(3)];
            %check the whole width of the boy
            for i = 0.0275:-0.0025:-0.0275
                %Check the whole length of the boy
                for j = 0.03:-0.0025:-0.03
                    %check if the translation enters the workspace
                    if((boyTranslation (1,1)+i) > safetyBarrierPoint1(1,1) && (boyTranslation (1,1)+i) < safetyBarrierPoint4(1,1))
                        %display("1st");
                        if((boyTranslation (1,2)+j) > safetyBarrierPoint1(1,2) && (boyTranslation (1,2)+j) < safetyBarrierPoint4(1,2))
                            %display("2nd");
                            %Checks if object is past the light curtain
                            if((boyTranslation(1,1)+i) < curtain1(1,1) || (boyTranslation(1,1)+i) > curtain4(1,1))
                                %display("3rd");
                                if((boyTranslation(1,2)+j) > curtain1(1,2) || (boyTranslation(1,2)+j) > curtain2(1,2))
                                    %display("4th");
                                    collision = true;
                                end
                            end
                        end
                    end
                end
            end
            %check if the translation enters the workspace
%             if(translation (1,1) > safetyBarrierPoint1(1,1) && translation (1,1) < safetyBarrierPoint4(1,1))
%                 if(translation (1,2) > safetyBarrierPoint1(1,2) && translation (1,2) < safetyBarrierPoint4(1,2))
%                     collision = true;
%                 end
%             end


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