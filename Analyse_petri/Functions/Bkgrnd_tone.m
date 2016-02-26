function [tone] = Bkgrnd_tone(Im)
%Find background color
n = 100;
counts = imhist(Im,n);
counts(1) = 0;
[~,Indx] = max(counts);
tone = Indx*255/n;










