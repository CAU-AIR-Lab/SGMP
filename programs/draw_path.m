function draw_path( map, path )

%DRAW_PATH    Draw obtained path on given map.
%   DRAW_PATH draws the obtained path specified in "path".
%   from the map speicified in the variable "map".
%
%   Usage: DRAW_PATH( map, path )
%
%   The "map" is a boolean square matrix variable specifying given map.
%      "TRUE" means obstacle node, "FALSE" means free-space node.
%
%   Example:
%        0 1 0 0 0
%        0 1 0 0 0
%        0 1 1 1 0
%        0 0 0 1 0
%        0 0 0 0 0
%
%   The "path" is a array variable contains obtained paths.
%
%   Example:
%        [24,45,93,36]
%
%      where the values of "sp" and "dp" are 24 and 36, respectively.
%      In this example, the path traverses 24 -> 45 -> 93 -> 36.      
%      For more detailed information, please visit our homepage:
%      http://ai.cau.ac.kr/?f=softwares&m=cave

colSize = size( map, 2 );
coords = zeros( length(path), 2 );
for k=1:length(path)
    [coords(k,1),coords(k,2)] = indexToRowCol( path(k), colSize );
end

imshow(map);
colormap( 1-gray );
hold on
plot( coords(:,2), coords(:,1), 'r-', 'LineWidth', 3 );
hold off
end % End of "draw_path" function


function [row,col] = indexToRowCol(idx, colSize)

% A function for changing the index of node "idx"
% to vertical and horizontal coordinate.

row = floor((idx - 1) / colSize) + 1;
col = mod(idx - 1, colSize) + 1;
end % End of "indexToRowCol" function.
