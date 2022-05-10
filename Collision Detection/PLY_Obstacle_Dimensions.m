%% Dimensions for Testable PLY Files
function [centerpnt, width, depth, height] = PLY_Obstacle_Dimensions(type, facing)
    %
    switch type
        case 1 %For boy8 file dimensions
            %boyDimensions = [0.07 0.125 0.48]
            %centerpnt = [-0.03,0.1];
            centerpnt = [0.03,-0.1];
            width = 0.055;
            depth = 0.06;
            height = 0.48;
        case 2 %For guard file dimensions
            %guardDimensions = [0.091 0.18 0.48];
            centerpnt = [0, 0];
            width = 0.08;
            depth = 0.0455;
            height = 0.7;
        otherwise
            Display("Dimension Type Not Found")
            centerpnt = [0, 0];
            width = 0.0;
            depth = 0.0;
            height = 0.0;
    end 
    switch facing
        case 0
            %Nothing Happens here as it is assumed that this part is
            %irrelevant
        case 1 %Assuming the file is facing towards negative Y
            centerpnt = -centerpnt;
    end   
end