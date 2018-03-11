function [rgb,points3d]=read_depth_cloud(depth_map,rgb_img,K,transToWolrd)
%     K is [fx 0 cx; 0 fy cy; 0 0 1];  
%     for uncrop image crop =[1,1];
%  depth_map  is absolute depth image, double in meters.
%  rgb_img could be [].  transToWolrd can be provided if want to transfrom.

    depth_size = size(depth_map);

    if (size(rgb_img,1)>0)
        rgb = im2double(rgb_img);
    else
        rgb = double(cat(3,zeros(depth_map(1),depth_map(2)),...
                    ones(depth_map(1),depth_map(2)),...
                    zeros(depth_map(1),depth_map(2))));
    end
    rgb = reshape(rgb, [], 3);
    
    cx = K(1,3); cy = K(2,3);  
    fx = K(1,1); fy = K(2,2); 
    invalid = depth_map==0;   % depth zero means invalid pixels
    [x,y] = meshgrid(1:depth_size(2), 1:depth_size(1));   
    x3 = (x-cx).*depth_map*1/fx;  
    y3 = (y-cy).*depth_map*1/fy;  
    z3 = depth_map;
    points3d = [x3(:) y3(:) z3(:)];
    points3d(invalid(:),:) =NaN;  % points are n*3
%     points3d=(Rot*points3d')';
        
    points3d = points3d';
%     size(points3d)
%     size([points3d; ones(1,size(points3d,2))])
%     size(transToWolrd)
%     transToWolrd
    if (size(transToWolrd,1)==4) % if full 4*4 matrix
        points3d = transToWolrd * [points3d; ones(1,size(points3d,2))];    
        points3d = (homo_to_real_coord(points3d))';   % n*3
    else  % if only 3*3 matrix
        points3d = transToWolrd * points3d;
        points3d = points3d';
    end
    
    
end