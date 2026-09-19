% 读取图片
img = imread('2.jpg');

% 获取原始图片的尺寸
[original_height, original_width, ~] = size(img);

% 计算缩放后的尺寸
new_width = floor(original_width / 7);
new_height = floor(original_height / 7);

% 缩放图片
resized_img = imresize(img, [new_height, new_width]);

% 保存缩放后的图片
imwrite(resized_img, '2_resized.jpg');

% 显示处理结果
disp(['图片已成功缩放并保存为 ''Mona_Lisa_resized.jpg''，新尺寸为 ' num2str(new_width) 'x' num2str(new_height) ' 像素。']);
