clear all;close all;clc;

im = rgb2gray(imread('dot.png'));
%Find edges of image
im_edge = edge(im,'canny');

%range of radius
radius = 50;



[row, col] = size(im);

H = zeros(row,col); %Accumulation matrix
res = [];

for rad_ind = 1:length(radius) 
    r = radius(rad_ind);
    for x = 1:row           %For each row
        for y = 1:col           %For each column
            if im_edge(x,y)         %If an edge
                [rr, cc]=meshgrid(1:row,1:col);
                C = round(sqrt((cc-x).^2+(rr-y).^2))==r;
                H(C) = H(C)+1; 
            end
        end
    end
    res = [res H/max(max(H))];
end
imshow(res)


