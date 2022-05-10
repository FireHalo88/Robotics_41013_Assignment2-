    collision = false;
    q = zeros(1,3); 
    plotOptions.plotFaces = true;

    centerpnt = [-0.035,0.1];
    translation = [2 2 2];      
    width = 0.17;
    depth = 0.1;
    height = 0.48;    
    
    [vertex,faces,faceNormals] = ActualRectangularPrism(centerpnt, translation, width, depth, height ,plotOptions);
    axis equal