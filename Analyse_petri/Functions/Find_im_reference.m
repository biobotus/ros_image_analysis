function [ref_coord,ref_scale] = Find_im_reference(im_gray,polarity, radius_range, scan_sens, scale, mask, known_dist, disp_fig)
%-------------------------------------------------------------------------%
%---------------------------Input parameters------------------------------%
%-------------------------------------------------------------------------%
% polarity            Object color 'bright' or 'dark'
% radius_range        Range of radius of petri dish (in pixel)
% scan_sens           Sensibility of scan        
% scale               Scaling factor on image 
% known_dist          Distance in mm between ref points
% disp_fig            Bool to display images

%-------------------------------------------------------------------------%
%---------------------------Output parameters-----------------------------%
%-------------------------------------------------------------------------%
% ref_coord           Coordonates of the reference point in corner of image
% ref_scale           Coordonates of the reference point in corner of image


%Find radius range with scaling factor
radius_range = ceil(radius_range*scale);

% Scale image
im_scale = imresize(im_gray,scale);

% Gray scale
im_gray_scale = im_scale;

%-------------------------------------------------------------------------%
%-----------------------Find reference circles----------------------------%
%-------------------------------------------------------------------------%
[ref_centers, radii] = imfindcircles(im_gray_scale,radius_range,'sensitivity',...
    scan_sens,'ObjectPolarity',polarity);


%-------------------------------------------------------------------------%
%---------------------Calculate references coordinates--------------------%
%-------------------------------------------------------------------------%

%Adjust to real size
ref_centers = ref_centers./scale;

% Validate the centers of references are outside the mask
buf_centers = [];
buf_radii   = [];
for i=1:size(ref_centers,1)
    if ~(mask(round(ref_centers(i,2)),round(ref_centers(i,1))))
        buf_centers = [buf_centers; ref_centers(i,:)];
        buf_radii   = [buf_radii radii(i,:)];
    end
end
ref_centers = buf_centers;
radii       = buf_radii;


% Number of reference points found
N_ref = size(ref_centers,1);

% Validate the size of reference points are similare
radii_mean = mean(radii);
max_error = max(abs(radii_mean - radii(:))/radii_mean);


% Return nothing if N_ref is not as expected or reference points size
% differ too much
if N_ref ~= 3 || max_error > .05
    disp('Error finding reference points')
    ref_coord = [];
    ref_scale = [];
    return;
end



% Locate the reference point in corner of image 
dist_centers = sqrt(ref_centers(:,1).^2+ref_centers(:,2).^2);
[~,I] = min(dist_centers);
ref_coord = ref_centers(I,:);

% Calculate the mean distence between ref points
dist = [];
for i=1:N_ref
    if i ~= I
        dist = [dist pdist2(ref_centers(I,:),ref_centers(i,:))];
    end
end

% Calculate the scale of image in mm/pixel
ref_scale = known_dist/mean(dist);


%-------------------------------------------------------------------------%
%----------------------------Display results------------------------------%
%-------------------------------------------------------------------------%

if disp_fig
    figure
    imshow(im_gray)
    hold on
    plot(ref_centers(:,1),ref_centers(:,2), 'rx')
    hold off
    
end




