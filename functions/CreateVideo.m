function CreateVideo(pathname)
%Write results to video from folder containing JPG images
imagefiles = dir(append(pathname,'\','*.jpg'));
nfiles = length(imagefiles);

video = VideoWriter(append(pathname,'/','ImpactVideo.avi'),'Motion JPEG AVI'); %create the video object
video.FrameRate = 10;
open(video); %open the file for writing
for ii = 1:nfiles
    img = imread(append(imagefiles(ii).folder,'/',imagefiles(ii).name));
    writeVideo(video,img); %write the image to file
end
close(video); %close the file
end
