clc
clear all
im = imread('test_bw_image.png');
im_edge = edge(im);
[x, y] = find(im_edge==1);
coordinates = [x y];
%coordinates(1,2)
for i = 1:1:3851
    h = plot(coordinates(i,1), coordinates(i,2),'.--');
    hold on
end