% Test segmentation de couleur
close all;clear all;clc;

%-------------------------------------------------------------------------%
%Lecture image
%-------------------------------------------------------------------------%
Im = imread('hestain.png');

%-------------------------------------------------------------------------%
%Convert Image from RGB Color Space to L*a*b* Color Space
%-------------------------------------------------------------------------%
cform = makecform('srgb2lab');
lab_Im = applycform(Im,cform);
%-------------------------------------------------------------------------%
%Classify the Colors in 'a*b*' Space Using K-Means Clustering
%-------------------------------------------------------------------------%
ab = double(lab_Im(:,:,2:3));
nrows = size(ab,1);
ncols = size(ab,2);
ab = reshape(ab,nrows*ncols,2);

nColors = 10;
%repeat the clustering 3 times to avoid local minima
[cluster_idx, cluster_center] = kmeans(ab,nColors,'distance','sqEuclidean', ...
                                      'Replicates',3);
                                  
%-------------------------------------------------------------------------%
%Label Every Pixel in the Image Using the Results from KMEANS
%-------------------------------------------------------------------------%
pixel_labels = reshape(cluster_idx,nrows,ncols);
figure
imshow(pixel_labels,[]), title('image labeled by cluster index');

%-------------------------------------------------------------------------%
%Create Images that Segment the H&E Image by Color.
%-------------------------------------------------------------------------%
segmented_images = cell(1,3);
rgb_label = repmat(pixel_labels,[1 1 3]);

for k = 1:nColors
    color = Im;
    color(rgb_label ~= k) = 0;
    segmented_images{k} = color;
    figure
    imshow(segmented_images{k}), title(['objects in cluster : ' num2str(k)]);
end
