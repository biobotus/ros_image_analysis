function [N, Centroid,Eccentricity,Area,Perimeter,BoundingBox,BW_ridge ] = Run_watershed(BW, min_pix)

%-------------------------------------------------------------------------%
%--------------------------------Watershed--------------------------------%
%-------------------------------------------------------------------------%
% Distance transform
Dist_transf         = -bwdist(~BW);
% Watershed
Dist_transf(~BW)    = Inf;
L                   = watershed(Dist_transf,8);
BW_ridge            = BW;
BW_ridge(L == 0)    = 0;

%-------------------------------------------------------------------------%
%-------------------------Characteristic of regions-----------------------%
%-------------------------------------------------------------------------%

% Scrap small regions
BW_ridge = bwareaopen(BW_ridge, min_pix);


% Extract info   
caract = regionprops(BW_ridge,'Centroid','Eccentricity','Area','Perimeter','BoundingBox');

Centroid            = cat(1,caract.Centroid);
Eccentricity        = cat(1,caract.Eccentricity);
Area                = cat(1,caract.Area);
Perimeter           = cat(1,caract.Perimeter);
BoundingBox         = cat(1,caract.BoundingBox);
N                   = length(Area);







