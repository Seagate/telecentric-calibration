% Filename: batch_calibration_v1.m
% Description: orthographic calibration script using a number of checkerboard images

%% Read files
% Much better result if no degeneracy in calibration images
folder = "../2020-08-19/";
images = read_images(folder,'.tif');

%% Estimate parameteres with linear least squares
[v_param0, target_struct, batch_img_pts, homographies] = estimate_params(images, 2.2);

%% Optimization on all images (image coordinates distortion)
% 'rad_tan' and 'full' distortion model show similar results, with the
% latter converges faster.
fit_struct_img_plane = refine_params(v_param0, target_struct, batch_img_pts, ...
    'distort_plane', 'pixel', 'distort_model', 'full');

%% Optimization on all images (normalized plane distortion)
% No difference in residual
fit_struct_normal_plane = refine_params(v_param0, target_struct, batch_img_pts, ...
    'distort_plane', 'normal', 'distort_model', 'full');    

%% Visualize target orientations
visualize_extrinsics(fit_struct_normal_plane.extrinsics, target_struct, 'z_offset', 100,...
    'origin', 'camera');

%% Compare residuals
% Homographic projection
residual_homo = get_residual(target_struct.w_coord, batch_img_pts, 'homographic', homographies);
% Orthographic projection with no distortion
residual_pre = get_residual(target_struct.w_coord, batch_img_pts, 'parametric', v_param0,...
    'distort_model', 'none');
% Orthographic projection with normal plane distortion
residual_nl_norm = get_residual(target_struct.w_coord, batch_img_pts, 'parametric',...
    fit_struct_normal_plane.vector, 'distort_model', 'full', 'distort_plane', 'normal');
% Orthographic projection with pixel plane distortion
residual_nl_pixel = get_residual(target_struct.w_coord, batch_img_pts, 'parametric',...
    fit_struct_img_plane.vector, 'distort_model', 'full', 'distort_plane', 'pixel');

%% Apply correction 
% pixel plane
imgs_correct_pixel = correctDistortion(images, fit_struct_img_plane.intrinsics,...
    'distort_model','full','distort_plane','pixel');
% normal plane
imgs_correct_normal = correctDistortion(images, fit_struct_normal_plane.intrinsics,...
    'distort_model', 'full', 'distort_plane', 'normal');

%% Compare differences between top and bottom sides of target images
% Original
diff_orig = top_v_bottom(target_struct, batch_img_pts);
% Corrected (normal plane)
diff_correct_normal = top_v_bottom(target_struct, imgs_correct_normal);
% Corrected (pixel plane)
diff_correct_pixel = top_v_bottom(target_struct, imgs_correct_pixel);
