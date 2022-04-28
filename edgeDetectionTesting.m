clc
clear all
im = imread('test_bw_image.png');
im_edge = edge(im);
[x, y] = find(im_edge==1);
coordinates = [x y];

%Scaling the image so that the dobot can get to each position
for i = 1:1:size(coordinates,1)
    coordinates(i,1) = (coordinates(i,1)/128)+5;
    coordinates(i,2) = (coordinates(i,2)/128)+5;
end

for i = 1:1:size(coordinates,1)
    grid on
    h = plot(coordinates(i,1), coordinates(i,2),'.--');
    hold on
end

%coordinates(1,2)
% size(coordinates,1)

% BW = imread('test_bw_image.png');
% s = regionprops(BW,'centroid');
% centroids = cat(1,s.Centroid);
% 
% stats = regionprops('table',BW,'Centroid',...
%     'MajorAxisLength','MinorAxisLength');
% centers = stats.Centroid;
% diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
% radii = diameters/2;
% for i = 1:1:size(centroids,1)
%     h = plot(centers(i,1), centers(i,2),'.--');
%     hold on
% end