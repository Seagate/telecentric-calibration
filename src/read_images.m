function images = read_images(folder,extension)
    assert(startsWith(extension,'.'), "Must start with .");
    assert(ischar(extension), "Must be char array");
    img_files = dir(fullfile(folder,['*' extension]));
    img_num = length(img_files);
    images = cell(img_num,1);
    for k = 1:img_num
        images{k} = rescale(imread(strcat(folder,img_files(k).name)));
    end
end