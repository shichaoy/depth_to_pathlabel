close all;
clear all;

addpath(genpath(strcat(pwd,'/utils')));

%% read offline trajectory set, each path has 28 points
load('path_library/gk_path_set_5_proc.mat');  % load offline path library
figure(1); hold on;
path_num=size(all_paths,2);
for i=1:path_num
    plot(all_paths(i).y,all_paths(i).x,'b');
end
axis equal;grid on;title('path library');xlabel('x');ylabel('y');


%% for each image create 3D world using camera calibration matrix
depth_map = double(imread('sample_img/0000_depth.png'))/1000; % depth in meter
rgb_img = imread('sample_img/0000_rgb.jpg');

T = [1,0,0, 5;   % camera pose to world. not important
     0,0,1, 0;
     0,-1,0, 5;
     0 0 0 1];
K = [582.6245  0      313.0448;  % kalibration matrix
        0   582.6910  238.4439;
        0      0         1 ];
    
[rgb, world_cloud] = read_depth_cloud(depth_map,rgb_img,K,T);
figure(3); clf; title('3D cuboids, Red truth, Blue pred');
vis_point_cloud(world_cloud,rgb);
xlabel('x');ylabel('y');zlabel('z');hold on;view([-0.5,-1,1]);


num_of_img = 1;
obst_dists = zeros(path_num,size(all_paths(1).x,2),num_of_img);  % for each image, store each path, each point's obstacle distance, used for latter cost.
grid_map = zeros(500,500,500); % camera x(left),y (down) z(forward) frame each grid is 0.02m

world_grid = int32(world_cloud/0.02);
for pt_id=1:size(world_grid,1)
    if (sum(world_grid(pt_id,:)<=500)==3) && (sum(world_grid(pt_id,:)>=1)==3)
        grid_map(world_grid(pt_id,2),world_grid(pt_id,1),world_grid(pt_id,3))=1; % swap axis just for visualization
    end
end
distance_map = bwdist(grid_map);
figure(3);hold on;
for i=1:path_num
    sumdist=0;
    traj=[all_paths(i).x' all_paths(i).y']; 
    world_traj=[traj(:,2)+5,traj(:,1)+0.1,ones(size(traj(:,1)))*5]; % move forward 0.1m
    grid_traj=int32(world_traj/0.02);
    for j=1:1:size(grid_traj,1)
        obst_dists(i,j,1)=distance_map(grid_traj(j,2),grid_traj(j,1),grid_traj(j,3));
    end
    scatter3(world_traj(:,1),world_traj(:,2),world_traj(:,3),'o')
end
obst_dists=single(obst_dists);

% figure(4);
% isosurface(distance_map,1), axis equal, view(3);
% xlabel('x');ylabel('y');zlabel('z');


%% compute fused cost of smooth and obstacle cost and output trajectory.

% define smoothness cost as squared acceleration
smooth_costs=zeros(path_num,1);
for i=1:path_num
    traj=[all_paths(i).x' all_paths(i).y']; 
    acce=traj(1:1:end-2,:)+traj(3:1:end,:)-2*traj(2:1:end-1,:);
    smooth_costs(i)=sum(sum(acce.^2))*1000;    
end
smooth_costs=(smooth_costs-2)*1000; % will be scaled and weighted later



% obst_dists=obst_dists(:,1:20,:);  % for evalute whether trajectory hits obstacles only use nearby points!!!
obst_cost=(180-obst_dists).^2;
obst_cost(obst_cost>(180-5)^2)=1e8;  % penalize points being very close to obstalces (5 grid=0.1m))

obst_cost=sum(obst_cost,2);obst_cost=reshape(obst_cost,size(obst_dists,1),size(obst_dists,3));
obst_cost=obst_cost'/size(obst_dists,2);

scale_obst_cost=(obst_cost-repmat(min(obst_cost,[],2),1,size(obst_dists,1)))./(repmat(30000-min(obst_cost,[],2),1,size(obst_dists,1)));  % rescale each row to [0,1]
scale_obst_cost(scale_obst_cost>1)=10;
scale_obst_cost=scale_obst_cost(:,1:path_num);

% must needs smooth cost, otherwise, it tends to select the leftmost and rightmost paths
scale_smooth_costs=(smooth_costs-3500)./(5000-3500);  % rescale each row to [0,1]

smooth_weight=0.6;
fused_cost=scale_obst_cost(:,1:path_num)+smooth_weight*repmat(scale_smooth_costs',size(obst_cost,1),1);

[~, path_label]=min(fused_cost,[], 2 ); % path label

[~, path_sort_labels]=sort(fused_cost,2);
path_label_top2=path_sort_labels(:,1:2); % top 2 path label

disp(['Select the best path ID:  ' num2str(path_label)]);

% plot the best trajectory
figure(3);hold on;
traj=[all_paths(path_label).x' all_paths(path_label).y']; 
world_traj=[traj(:,2)+5,traj(:,1)+0.1,ones(size(traj(:,1)))*5]; % move forward 0.1m
scatter3(world_traj(:,1),world_traj(:,2),world_traj(:,3),'x')
plot3(world_traj(:,1),world_traj(:,2),world_traj(:,3),'r','LineWidth',2);
