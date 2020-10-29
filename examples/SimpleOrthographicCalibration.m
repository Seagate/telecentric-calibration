%% Add path if needed
% addpath('../src/');

%% Read files
% Much better result if no degeneracy in calibration images
folder = "../2020-08-19/";
images = read_images(folder,'.tif');

%% Run calibration
[calib_struct, target_struct]= calibrate(images, 2.2);

%% Visualize extrinsics
visualize_extrinsics(calib_struct.extrinsics, target_struct, 'z_offset', 100,...
    'origin', 'camera');

%% Correct for distortion and visualize
imgs_correct = correctDistortion(images, calib_struct.intrinsics);
imshow(imgs_correct{1});
title("Corrected image 1");