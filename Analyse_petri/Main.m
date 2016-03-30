clear all;close all;clc
addpath('Functions')

%load image
Im_name = 'DSC_0328_ref.JPG';
Im_folder = [pwd '\' 'Images\'];
im_o = imread([Im_folder Im_name]);

figure
imshow(im_o)
%-------------------------------------------------------------------------%
%---------------------------Global parameters-----------------------------%
%-------------------------------------------------------------------------%
Save_output = 0;




%-------------------------------------------------------------------------%
%---------------------------Petri detection-------------------------------%
%-------------------------------------------------------------------------%
polarity        = 'dark';       % Object color ('bright' or 'dark')
radius_range    = [1500 1600];  % Range of radius of petri dish (in pixel)
scan_sens       = .99;          % Sensibility of scan        
scale           = .1;           % Scaling factor on image
r_scale         = .90;         % Scaling factor on radius
disp_fig        = 0;            % Bool to display images

[mask, r, center] =  Find_dish(im_o, polarity,...
                     radius_range,scan_sens, ...
                     scale, r_scale, disp_fig);
                 
im_gray = rgb2gray(im_o);
im_gray_mask = im_gray.*uint8(mask);
%% -----------------------------------------------------------------------%
%------------------------Find image reference scale-----------------------%
%-------------------------------------------------------------------------%                 
polarity        = 'bright';     % Object color ('bright' or 'dark')
radius_range    = [75 125];     % Range of radius of petri dish (in pixel)
scan_sens       = .85;          % Sensibility of scan        
scale           = .25;          % Scaling factor on image
known_dist      = 100;          % Distance in mm between ref points
disp_fig        = 0;            % Bool to display images

[ref_coord, ref_scale] =    Find_im_reference(im_o(:,:,1),polarity,...
                            radius_range, scan_sens, scale,...
                            mask, known_dist,disp_fig);

%% -----------------------------------------------------------------------%
%------------------------------Image anaysis------------------------------%
%-------------------------------------------------------------------------%
close all;


% for i = 1:3 % For R, G, and B
%     im = im_crop(:,:,i);
%     % Mask image
%     im(~mask) = 0;
%     % Find back ground tone
%     BG_tone = Bkgrnd_tone(im);
%     % Apply background tone to masked area
%     im(~mask) = BG_tone;
%     B(:,:,i) = im;
%     
%     B1(:,:,i) = im2bw(im, BG_tone*(1+.2)/255);
%     B2(:,:,i) = im2bw(imcomplement(im), BG_tone*(1-.2)/255);
% end
% level = multithresh(im_gray_crop(mask),2);
% seg_I = imquantize(im_gray_crop.*uint8(mask),level);
% imagesc(seg_I)


level = Bkgrnd_tone(im_gray_mask)*(1+.2)/255;
I =im_gray.*uint8(mask);
B1 = im2bw(I, level);


% Erode image and fill holes
se = strel('disk',3);        
B1 = imerode(B1,se);
B1 = imfill(B1,'holes');

imshow(B1)


% Run watershed
[N_colonies, Centroid, Eccentricity, Area, Perimeter, BoundingBox, BW_ridge] = Run_watershed(B1, 50);

% Fin distances in mm from the reference point
Centroid_ref(:,1)   = Centroid(:,1)  - ref_coord(1);
Centroid_ref(:,2)   = Centroid(:,2)  - ref_coord(2);
Centroid_ref        = Centroid_ref * ref_scale;

% Calculate the area in (mm)^2
Area                = Area * ref_scale^2;
% Calculate the perimeter in mm
Perimeter           = Perimeter .*ref_scale;



%% -----------------------------------------------------------------------%
%------------------------------Display results----------------------------%
%-------------------------------------------------------------------------%

f=figure;
mask_rgb = repmat(uint8(mask),[1 1 3]);
imshow(im_o)
title('Original image')
set(gca,'DataAspectRatio',[1 1 1]);
hold on
% for i=1:N_colonies
%     %text(centroids(i,1),centroids(i,2),num2str(s(i).Perimeter))
%     rectangle('Position',BoundingBox(i,:),'EdgeColor','r','LineWidth',3)
% end
plot(Centroid(:,1),Centroid(:,2), 'rx')
plot(ref_coord(1),ref_coord(2), 'bx','LineWidth',3)
hold off
xlabel(['Nb colonies: ' num2str(N_colonies)])


%% -----------------------------------------------------------------------%
%------------------------------Save results-------------------------------%
%-------------------------------------------------------------------------%

if Save_output == 1
    folder      = 'Output\';
    Date = char(datetime('now','Format','yyyy-MM-dd_''T_''HH_mm_ss'));
    filename    = [folder matlab.lang.makeValidName(Date)]; 
    
    % Save image
    saveas(f,filename,'png')


    % Save characteristics in Excel
    header ={'Center_X', 'Center_Y', 'Eccentricity', 'Area', 'Perimeter'};
    Mat = [Centroid Eccentricity Area Perimeter];

    Sheetname   = 'Experience 1';
    xlswrite(filename, header, Sheetname,'A1') 
    xlswrite(filename,Mat,Sheetname,'A2')
end

