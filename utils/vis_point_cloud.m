% Visualizes a 3D point cloud.
%
% Args:
%   points3d - Nx3 or Nx2 point cloud where N is the number of points.
%   colors - (optional) Nx3 vector of colors or Nx1 vector of values which which 
%            be scaled for visualization.
%   marker_sizes - (optional) Nx1 vector of point sizes or a scalar value which is applied
%           to every point in the cloud.
%   visualize_samples - (optional) the maximum number of points to show. Note that since matlab
%                is slow, around 5000 points is a good sampleSize is practice.
%
function vis_point_cloud(points, colors, marker_sizes, visualize_samples)  
removeNaN = sum(isnan(points),2)>0;
points(removeNaN,:)=[];
  [N, D] = size(points);
  assert(D == 2 || D == 3, 'points must be Nx2 or Nx3');
  
  if ~exist('colors', 'var') || isempty(colors)
    norms = sqrt(sum(points.^2, 2));
    colors = values2colors(norms);
  else
    colors(removeNaN,:)=[];
    
  end
  
  if ~exist('sizes', 'var') || isempty(marker_sizes)
    marker_sizes = ones(N, 1) * 10;
  elseif numel(marker_sizes) == 1  
    marker_sizes = marker_sizes * ones(N, 1);
  elseif numel(marker_sizes) ~= N
    error('sizes:size', 'sizes must be Nx1');
  end
  
  if ~exist('sampleSize', 'var') || isempty(visualize_samples)
    visualize_samples = 5000; %5000   100000  sample some points
  end

  % Sample the points, colors and sizes.
  N = size(points, 1);
  if N > visualize_samples
    seq = randperm(size(points, 1));
    seq = seq(1:visualize_samples);
    
    points = points(seq, :);
    colors = colors(seq, :);
    marker_sizes = marker_sizes(seq);
  end
  
  switch size(points, 2)
    case 2        
      vis_2d(points, colors, marker_sizes);
    case 3
      vis_3d(points, colors, marker_sizes);
    otherwise 
      error('Points must be either 2 or 3d');
  end
  
  axis equal;
  %view(0,90);
%s  view(0,8);
end

function colors = values2colors(values)
  values = scale_values(values);
  inds = ceil(values * 255) + 1;
  h = colormap(jet(256));
  colors = h(inds, :);
end

function values = scale_values(values)
if length(values(:))>1,
  values = values - min(values(:));
  values = values ./ max(values(:));
end
end

function vis_3d(points, colors, sizes)
  X = points(:,1);
  Y = points(:,2);
  Z = points(:,3);
  scatter3(X, Y, Z, sizes, colors, 'filled');
end

function vis_2d(points, colors, sizes)
  X = points(:,1);
  Y = points(:,2);
  scatter(X, Y, sizes, colors, 'filled');
  
  xlabel('x');
  ylabel('y');
end
