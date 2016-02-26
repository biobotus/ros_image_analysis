clear all;close all;clc
addpath('Functions')



%load image
Im_name = 'DSC_0330.JPG';
Im_folder = [pwd '\' 'Images\'];
im_o = imread([Im_folder Im_name]);

figure
imshow(im_o)
%imdistline(gca);

%-------------------------------------------------------------------------%
%---------------------------Petri detection-------------------------------%
%-------------------------------------------------------------------------%
polarity        = 'dark';       % Object color ('bright' or 'dark')
radius_range    = [1500 1600];  % Range of radius of petri dish (in pixel)
scan_sens       = .99;          % Sensibility of scan        
scale           = .1;           % Scaling factor on image
r_scale          = .90;         % Scaling factor on radius
disp_fig        = 0;            % Bool to display images

[im_crop, im_gray_crop,mask_org, mask_crop, r, center] =  Find_dish(im_o, polarity,...
                                            radius_range,scan_sens, ...
                                            scale, r_scale, disp_fig);

                                
  
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

level = Bkgrnd_tone(im_gray_crop)*(1+.2)/255;
I =im_gray_crop.*uint8(mask_crop);

B1 = im2bw(I, level);
figure
imshow(B1)



% Erode image
se = strel('disk',3);        
B1 = imerode(B1,se);
B1 = imfill(B1,'holes');

imshow(B1)


% Run watershed
[N_colonies, Centroid, Eccentricity, Area, Perimeter, BoundingBox, BW_ridge ] = Run_watershed(B1, 50);


%-------------------------------------------------------------------------%
%------------------------------Display results----------------------------%
%-------------------------------------------------------------------------%

f=figure;
imshow(im_crop.*repmat(uint8(mask_crop),[1 1 3]))
title('Original image')
set(gca,'DataAspectRatio',[1 1 1]);
hold on
% for i=1:N_colonies
%     %text(centroids(i,1),centroids(i,2),num2str(s(i).Perimeter))
%     rectangle('Position',BoundingBox(i,:),'EdgeColor','r','LineWidth',3)
% end
plot(Centroid(:,1),Centroid(:,2), 'rx')
hold off
xlabel(['Nb colonies: ' num2str(N_colonies)])


%% -----------------------------------------------------------------------%
%------------------------------Save results-------------------------------%
%-------------------------------------------------------------------------%
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


