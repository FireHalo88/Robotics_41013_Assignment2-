 PlaceObject('boy9.ply')
 axis equal
 camlight

 translation = [0 0 0];
centerpnt = [0,0.0];
width = 0.055;
depth = 0.06;
height = 0.48;

plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = ActualRectangularPrism(centerpnt, translation, width, depth, height ,plotOptions);
axis equal


%guardDimensions = [0.12 0.1 0.7];