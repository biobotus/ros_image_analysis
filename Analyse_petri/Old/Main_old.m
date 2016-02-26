clear all;close all;clc

%load image
Im_folder = [pwd '\' 'Images\'];
Im_name = 'DSC_0328.JPG';
im_o = imread([Im_folder Im_name]);

figure
imshow(im_o)
imdistline(gca);

%-------------------------------------------------------------------------%
%---------------------------Petri detection-------------------------------%
%-------------------------------------------------------------------------%
polarity        = 'dark';     % Object color ('bright' or 'dark')
radius_range    = [1500 1600];  % Range of radius of petri dish (in pixel)
scan_sens       = .99;          % Sensibility of scan        
med_order       = 20;           % Order for the median filter
scale           = .1;           % Scaling factor 
disp_fig        = 1;            % Bool to display images

[im_crop, im_gray_crop, mask, r] = Find_dish(im_o, polarity,...
                                radius_range,scan_sens, ...
                                med_order, scale, disp_fig);

                       
                   
% %% -----------------------------------------------------------------------%
% %------------------------------Image anaysis------------------------------%
% %-------------------------------------------------------------------------%
% erod_factor = 5 ;%
% 
% % Adjust mask to background colour
% for chan = 1:3 % For each channel
%     im_chan = im_crop(:,:,chan);
%         
%     % Median filter
%     im_chan = medfilt2(im_chan,[med_order med_order]);
%     
%     % Masking channel
%     im_chan(~mask)=0;
%     
%     %find background colour
%     [counts,x] = imhist(im_chan,100);
%     counts(1) = 0; %discard the mask color
%     counts = smooth(counts);
%     
%     [~,indx]= max(counts);
%     
%     % Apply background colour to masked region
%     im_chan(~mask) = 256*indx/100;
%     im_chan = imsharpen(im_chan);
%     % Adjust contrast
%     %im_chan = imadjust(im_chan);
% 
%     % Binarize image
%     level = graythresh(im_chan);
%     BW(:,:,chan) = im2bw(im_chan, level);
%     
%    
%     %Change polarity if necessery
%     if BW(1,1,chan) ~= 0
%        BW(:,:,chan) = ~BW(:,:,chan);
%     end
%     
%     %Erode to help find colony centers
%     se = strel('disk',erod_factor);        
%    % BW = imerode(BW,se);
%     
%     % Reform image
%     im2(:,:,chan) = im_chan;
% end
% 
% figure
% imshow(im2)
% 
% figure('Name','Binarized image')
% for i = 1:3
%     subplot(1,3,i)
%     imshow(BW(:,:,i))
%     title(['Channel: ' num2str(i)])
% end
% 
% 
% %imwrite(im2,'colony.jpg')











