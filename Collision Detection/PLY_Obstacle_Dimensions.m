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
        case 3 %For table file dimensions
            centerpnt = [0.0, 0];
            width = 0.41;
            depth = 0.365;
            height = 0.22;
        case 4 %For canvas file dimensions
            centerpnt = [0.2 0.2];
            width = 0.2;
            depth = 0.2;
            height = 0.032;
        case 5 %For safety barrier area dimensions
            centerpnt = [0.2 0.2];
            width = 0.2;
            depth = 0.2;
            height = 0.032;
        otherwise
            display("Dimension Type Not Found")
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
