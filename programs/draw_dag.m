function draw_dag(map, dag)

%DRAW_DAG    Draw obtained Directed Acyclic Graph by SMGP method.
%   DRAW_DAG draws the obtained directed acyclic graph specified in "dag".
%
%   Usage: draw_dag( map, dag )
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
%   The "dag" is a variable specifying the obtained DAG by SGMP.
%
%   Example:
%                       Node          Parents
%      1st row:    115    1   115  |  0     0 : starting Node
%      2nd row:   4120    9   120  |  1   NaN : one parent (1st row)
%      3rd row:   4624   10   124  |  2     1 : two parents (2nd, 1st)
%      4th row:   5625   12   125  |  3     2 : two parents (3rd, 2nd)
%      5th row:   4113    9   113  |  2     1 : two parents (2nd, 1st)
%
%   The structure of "dag" is
%      1st column: the index of node
%      2nd column: vertical coordinate of node
%      3rd column: horizontal coordinate of node
%      4th column: The first parent of node
%      5th to end column: Remaining parents of node

% For simplicity, we assumed the given map is a square.
% If the map is a rectangle, then it can be easily transformed to
% a square by filling obstacle nodes.
% The variable "mapRowSize" is not required for further procedure.
% However, for compatibility issue of MATLAB, we specified
% "mapRowSize" to the output variables in this implementation.
[mapRowSize,mapColSize] = size(map);

figure
for k = 2 : size(dag,1)
    % k = 1: starting node. Thus, there is no parent node 
    %        and the algorithm does not need to draw the edge.
    
    % The current node is identified.
    % For clarity, the algorithm save the node to "node".
    node = dag(k, 1);
    for eachParent = 4 : size(dag,2)
        % Identifiy the parent node's index in the dag structure
        parent = dag(k, eachParent);
        if isnan(parent) || parent == 0
            % If there is no parent, then the algorithm 
            % does not need to draw the edge.
            continue;
        end
        
        % The parent node of "k"-th node is identified.
        % For clarity, save the index of parent node to "parentNode".
        parentNode = dag(parent, 1);
        
        % Obtain (vertical,horizontal)-coordinate of current node, "node".
        [spRow,spCol] = indexToRowCol(node, mapColSize);
        
        % Obtain (vertical,horizontal)-coordinate of parent node, "parentNode".
        [dpRow,dpCol] = indexToRowCol(parentNode, mapColSize);
        
        % Draw the intermediate nodes between "node" and "parentNode".
        % "bresenham" algorithm returns (x,y)-coordinate pairs
        % between "node" and "parentNode".
        [x,y] = bresenham(spRow, spCol, dpRow, dpCol);
        for m = 1 : size(x, 1)
            map(x(m), y(m)) = 1;
        end
    end
end

imshow(map);
colormap(1-gray);
end

function [row,col] = indexToRowCol(idx, colSize)
    row = floor((idx - 1) / colSize) + 1;
    col = mod(idx - 1, colSize) + 1;
end

function [x,y] = bresenham(x1,y1,x2,y2)

% Modified Bresenham Algorithm.
% Returns (x,y)-coordinates between <(x1,y1),(x2,y2)>.
% If it stuck by obstacle nodes then terminate immediately,
% and returns coordinates identified so far.

global map;
x1=round(x1); x2=round(x2);
y1=round(y1); y2=round(y2);
dx=abs(x2-x1);
dy=abs(y2-y1);
steep=abs(dy)>abs(dx);
if steep
    t=dx;dx=dy;dy=t;
end

if dy==0 
    q=zeros(dx+1,1);
else
    q=[0;diff(mod([floor(dx/2):-dy:-dy*dx+floor(dx/2)]',dx))>=0];
end

if steep
    if y1<=y2 y=[y1:y2]'; else y=[y1:-1:y2]'; end
    if x1<=x2 x=x1+cumsum(q);else x=x1-cumsum(q); end
else
    if x1<=x2 x=[x1:x2]'; else x=[x1:-1:x2]'; end
    if y1<=y2 y=y1+cumsum(q);else y=y1-cumsum(q); end
end

otherQ = zeros(size(q, 1) + sum(q == 1), 1);
cum = 0;
newX = zeros(size(otherQ, 1), 1);
newY = zeros(size(otherQ, 1), 1);
if steep
    for i = 1 : size(q, 1)
        if q(i) == 1
            newX(i+cum) = x(i-1);
            newY(i+cum) = y(i);
            if map(newX(i+cum), newY(i+cum)) == 1
                newX = newX(1:i-1, 1);
                newY = newY(1:i-1, 1);
                break;
            end
            cum = cum + 1;
            otherQ(i+cum) = 1;
            newX(i+cum) = x(i);
            newY(i+cum) = y(i);
            if map(newX(i+cum), newY(i+cum)) == 1
                newX = newX(1:i-1, 1);
                newY = newY(1:i-1, 1);
                break;
            end
        else
            newX(i+cum) = x(i);
            newY(i+cum) = y(i);
            if map(newX(i+cum), newY(i+cum)) == 1
                newX = newX(1:i-1, 1);
                newY = newY(1:i-1, 1);
                break;
            end
        end
    end
else
    for i = 1 : size(q, 1)
        if q(i) == 1
            newX(i+cum) = x(i);
            newY(i+cum) = y(i-1);
            if map(newX(i+cum), newY(i+cum)) == 1
                newX = newX(1:i-1, 1);
                newY = newY(1:i-1, 1);
                break;
            end
            cum = cum + 1;
            otherQ(i+cum) = 1;
            newX(i+cum) = x(i);
            newY(i+cum) = y(i);
            if map(newX(i+cum), newY(i+cum)) == 1
                newX = newX(1:i-1, 1);
                newY = newY(1:i-1, 1);
                break;
            end
        else
            newX(i+cum) = x(i);
            newY(i+cum) = y(i);
            if map(newX(i+cum), newY(i+cum)) == 1
                newX = newX(1:i-1, 1);
                newY = newY(1:i-1, 1);
                break;
            end
        end
    end 
end

x = newX;
y = newY;
end % End of "bresenham" function.