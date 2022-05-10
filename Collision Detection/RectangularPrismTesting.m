
centerpnt = [0 0];
   
translation = [0 0 0];

guardDimensions = [0.12 0.1 0.7];
width = 0.06;
depth = 0.03;
height = 0.7;

plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = ActualRectangularPrism(centerpnt, translation, width, depth, height ,plotOptions);
axis equal