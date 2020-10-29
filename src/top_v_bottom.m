function difference = top_v_bottom(target_struct, arg1)
% ** Inputs **
%   @taget_struct -- target structure
%   @arg1 -- image cell array or calibration points coordinates
% ** Output **
%   @difference -- difference between top and bottom side of the target
%   image.

    assert(isstruct(target_struct));
    rows = target_struct.rows;
    cols = target_struct.cols;
    % Parse arg1, populate batch_img_pts
    if iscell(arg1)
        images = arg1;
        assert(isvector(images));
        num_imgs = length(images);
        assert(ismatrix(images{1}));
        [imagePts,~] = detectCheckerboardPoints(images{1});
        batch_img_pts = zeros([size(imagePts),num_imgs]);
        batch_img_pts(:,:,1) = imagePts;
        for k = 2:num_imgs
            image = images{k};
            assert(ismatrix(image));
            assert(isnumeric(image));
            [batch_img_pts(:,:,k),~] = detectCheckerboardPoints(image); 
        end        
    elseif isnumeric(arg1)      
        batch_img_pts = arg1;
        assert(length(size(batch_img_pts))==3);
        assert(size(batch_img_pts,2)==2);        
    else
        error("Unknown argument type")
    end   
    % Find difference
    difference = zeros(size(batch_img_pts,3),1);
    for k = 1:size(batch_img_pts,3)
        ul = batch_img_pts(1,:,k);
        bl = batch_img_pts(rows,:,k);
        ur = batch_img_pts(rows*cols-rows+1,:,k);
        br = batch_img_pts(rows*cols,:,k);
        % Extract sides
        top_side = norm(ul-ur);
        bottom_side = norm(bl-br);
        difference(k) = top_side - bottom_side;
    end     
end