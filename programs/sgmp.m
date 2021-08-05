function [ pathSet, execTime, dag ] = sgmp( IMAP, sp, dp, setSize, numPt )

%SGMP    Create a Directed Acyclic Graph (DAG) by exploring given map.
%   SGMP creates a DAG that starts from "sp" and ends from "dp"
%   from the map speicified in the variable "IMAP".
%
%   Usage: [pathSet, execTime, dag] = SGMP( IMAP, sp, dp, setSize, numPt )
%
%   INPUT PARAMETERS
%   
%   The "IMAP" is a boolean square matrix variable specifying given map.
%      "TRUE" means obstacle node, "FALSE" means free-space node.
%
%   Example:
%        0 1 0 0 0
%        0 1 0 0 0
%        0 1 1 1 0
%        0 0 0 1 0
%        0 0 0 0 0
%
%   The "sp" is a variable specifying the index of starting node.
%      It should be an integer value. If you download the maps from
%      our official site, you can find the "sp" variable from the
%      workspace when you load .mat file for each map.
%      Please visit our homepage for download the maps:
%      http://ai.cau.ac.kr/?f=softwares&m=cave
%
%   The "dp" is a variable specifying the index of destination node.
%      It should be an integer value.
%
%   The "setSize" is a variable specifying the size of path set, or
%      the number of paths to be created. It should be an integer value.
%
%   The "numPt" is a variable specifiying the maximum allowed number of
%      parents for each node. Based on this value, SGMP assigns
%      maximum "numPt" parents to each node.
%
%   OUTPUT PARAMETERS
%
%   The "pathSet" is a cell-array variable contains obtained paths.
%      Each cell contains the trajectory of path where starting from "sp"
%      and ends from "dp".
%
%   Example:
%        1st cell: [24,45,93,36]
%        2nd cell: [24,85,57,79,19,48,20,21,46,36]
%        3rd cell: [24,11,39,31,52,8,36]
%        4th cell: [24,9,34,71,4,93,24,5,36]
%        :
%
%      where the values of "sp" and "dp" are 24 and 36, respectively.
%      For example, the path specified by 1st cell is
%      24 (Starting node) -> 45 -> 93 -> 36 (Destination node)
%      For more detailed information, please visit our homepage:
%      http://ai.cau.ac.kr/?f=softwares&m=cave
%
%   The "execTime" is a variable specifying the execution time for
%      creating each path.
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

% Below variables are used in the most context of SGMP function.
% Thus, we initialize these variables as global variables.
global OBSTACLE_PENALTY;
global mapColSize;
global map;
map = IMAP;

% Initialize "pathSet" according to the number of paths to be created.
pathSet = cell(setSize, 1);

% Initialize "execTime" according to the number of paths to be created.
execTime = zeros(setSize, 1);

% Identify the size of given map specified in "map".
[mapRowSize, mapColSize] = size(map);

% To identify the infeasible edge, we initialize "OBSTACLE_PENALTY"
% by using a value that cannot be a value of distance from given map.
% OBSTACLE_PENALTY = mapColSize * 2;
OBSTACLE_PENALTY = inf;

% Identify the index of obtacle nodes.
% It is useful to accelerate the process of creating DAG.
[obstacleRow,obstacleCol] = find(map == 1);
obstacle = (obstacleRow - 1) * mapRowSize + obstacleCol;

% Identify the index of free-space nodes.
% It is useful to accelerate the process of creating DAG.
feasibleCell = 1:mapRowSize*mapColSize;

% Because starting node and destination node cannot be included to
% the path as the intermediate nodes, we treat them as obstacle.
feasibleCell([obstacle; sp; dp]) = [];

% Because the variable "dag" grows according to the number of included
% node, the size of dag will be changed dynamically.
% However, in MATLAB implementation, it is not preferable that employing
% dynamic variables since it significantly increases execution time.
% Thus, we initialize the size of "dag" as 2000, and increase
% the size of "dag" when it is necessary.
dag = zeros(2000, numPt + 3);

% Initialize the size of "dag". 
% "dagSize" can be used as the index of to be included node on "dag".
dagSize = 0;

% To find the detailed information about "tNode", go to Line 151.
tNode = zeros(1, 3);

% To find the detailed information about "knnNodes", go to Line 172.
knnNodes = NaN(numPt, 1);

% Because DAG always starts from the starting node,
% it should be included to the first row of "dag".
dag(1,1) = sp;

% The second and third column of "dag" specifies the vertical axis
% and horizontal axis of the node given in the first column.
%
% Notice: The function "indexToRowCol" returns the vertical and horizontal
%         axis of given index of node.
[dag(1,2), dag(1, 3)] = indexToRowCol(sp, mapColSize);

% Increase the size of "dag" because "sp" was included in the "dag".
dagSize = dagSize + 1;

% Start TIC for measuring the execution time.
tic;

% SGMP runs until "dp" is included to the end of DAG.
% Thus, it will fall into infinite loop if there is no feasible path 
% connecting starting and destination node. So, be careful.
while dag(1, end) ~= dp
    
    % Identify a direction node "tNode" to branch the "dag"
    % by choosing a free-space node from "feasibleCell".
    % The "tNode" variable is 1 by 3 array that contains
    % [Index of node,Vertical Axis,Horizontal Axis].
    % Note that "tNode" may not be included to the "dag" 
    % because there can be obstacle nodes between DAG and "tNode".
    tNode(1) = feasibleCell(randi(size(feasibleCell, 2)));
    [tNode(2), tNode(3)] = indexToRowCol(tNode(1), mapColSize);
    
    % For identifying the nearest nodes of "tNode" that are already
    % included in the "dag", the values of euclidean distance
    % between "tNode" and all nodes in "dag" are calculated.
    euDist = sum((dag(1:dagSize, 2:3) - repmat( tNode(2:3), dagSize, 1 )).^2,2).^0.5;
    
    % The variable "idx" contains the position (row) in "dag" of 
    % nearest nodes from "tNode".
    % The variable "tmp" is useless and may be specified as "~", 
    % but we included it because the compatibility issue of MATLAB.
    [tmp,idx] = sort(euDist);

    % Identify nearest nodes that can be used as candidate parent nodes.
    knnNodes(1:min(numPt, dagSize)) = idx(1:min(numPt, dagSize));
    
    % Identify a node to be actually included to "dag".
    % The function "C = nextState(A,B)" returns the furthest node "C"
    % from "A" that is a node between the edge "<A,B>".
    actualNode = nextState(dag(knnNodes(1), 2:3), tNode(1, 2:3));
    
    % According to the situation of DAG, there may not exist a node
    % to be included to DAG as follows:
    % 1) When the nearest node adjoins the obstacle nodes, so that DAG
    %    cannot be branched any further to "tNode"
    % 2) When DAG already contains the "actualNode"
    % 3) If there are obstacle nodes between nearest node and actual node.
    % In those cases, it is failed to find a new node for branching DAG.
    % 
    % Notice: The function "map_dist_realtime(A,B)" returns the length of
    %         <A,B>. If it intersects any obstacle node, then it returns
    %         a value specified in "OBTACLE_PENALTY".
    if isempty(actualNode) || ...
       actualNode == dag(dagSize, 1) || ...
       isinf(map_dist_realtime(dag(knnNodes(1), 1), actualNode))
        continue;
    end

    % Include a new node "actualNode" to "dag".
    dag(dagSize+1, 1) = actualNode;
    [dag(dagSize+1, 2), dag(dagSize+1, 3)] = indexToRowCol(actualNode, mapColSize);

    % Connect additional parent nodes specified in "numPt" if possible.
    % If it is unable to include additional parent nodes, mark it as "NaN" value.
    for each = 2 : numPt
        if isnan(knnNodes(each)) || ...
           isinf(map_dist_realtime(dag(knnNodes(each), 1), actualNode))
            knnNodes(each) = NaN;
        end
    end
    
    % Assign the position of parent nodes in "dag".
    dag(dagSize+1, 4:end) = knnNodes;
    
    % Increase the size of "dag" because "actualNode" was included in the "dag".
    dagSize = dagSize + 1;
    
    % Test the last-added node "actualNode" can be connected.
    % If it is possible, terminate while-loop (Line 149).    
    if ~isinf(map_dist_realtime(dag(dagSize, 1), dp))
        dag(dagSize+1, 1) = dp;
        [dag(dagSize+1, 2), dag(dagSize+1, 3)] = indexToRowCol(dp, mapColSize);
        dag(dagSize+1, 4) = dagSize;
        dagSize = dagSize + 1;
        break;
    end
end

% Because "dag" was initialized with a fixed length (2000),
% it may contain useless rows that should be eliminated for clarity.
dag = dag(1:dagSize, 1:end);

% Generate the path based on obtained "dag".
for pop = 1 : setSize
    pathSet{pop} = createPathFrom(dag);
    execTime(pop) = toc;
    tic;
end
end % End of SGMP function


function [path] = createPathFrom(dag)

% A function for creating multiple paths from "dag".

% Because "dp" always has only one parent and the path is created by
% back-tracking from "dp" to "sp", the variables "node" and "parent"
% are initialized as follows.
node = dag(end, 1);
parent = dag(end, 4);
path = [];

% In the variable "dag", the parent of "sp" is initialized as the value 0.
% Thus, if the parent is set to 0 then it means a path is created.
while parent ~= 0
    path = [node, path];
    node = dag(parent, 1);
    
    % Identify parents of "node".
    parents = dag(parent, 4:end);
    
    % If the parents are specified as "NaN" then eliminate them.
    parents(isnan(parents)) = [];
    
    % Select a parent randomly from specified parents.
    parent = parents(randi(size(parents, 2)));
end

% Include "sp" to the head of "path".
path = [node, path];
end % End of "createPathFrom" function.


% ------------------------------------------------------------------
% Belows are implementation of MISC functions for execution of SGMP.
% ------------------------------------------------------------------
function [actualNode] = nextState(nearestCell, tNode)

% A function for identifying "actualNode" that will be included to "dag".

global mapColSize;

nRow = nearestCell(1);
nCol = nearestCell(2);
rRow = tNode(1);
rCol = tNode(2);

[x,y] = bresenham(nRow, nCol, rRow, rCol, true );
if isempty(x) || isempty(y)
    actualNode = [];
else
    actualNode = rowColToIndex(x(end), y(end), mapColSize);
end
end % End of "nextState" function.


function [row,col] = indexToRowCol(idx, colSize)

% A function for changing the index of node "idx"
% to vertical and horizontal coordinate.

row = floor((idx - 1) / colSize) + 1;
col = mod(idx - 1, colSize) + 1;
end % End of "indexToRowCol" function.


function [idx] = rowColToIndex(row, col, colSize)

% A function for changing the vertical and horizontal coordinate
% to the index of node "idx".

idx = (row - 1) * colSize + col;
end % End of "rowColToIndex" function.


function [x,y] = bresenham(x1,y1,x2,y2,flag)

% Create Feasible Edge Function (or Modified Bresenham Algorithm)
% The function returns (x,y)-coordinates between <(x1,y1),(x2,y2)>.
% If "flag" is set to "true", the function will terminate immediately 
% when it stuck by obstacle nodes, and returns coordinates identified so far.
% In other case, the function will perform modified Bresenham Algorithm.

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

% If "flag" is set to true, then it finds furtheset node.
% In other case, the function performs modified Bresenham algorithm.
if ~flag, return; end

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


function res = map_dist_realtime( sp, ep )

% A function for calculating the Euclidean distance of <sp, ep>.
% Please do not care about the "realtime".

global map
global OBSTACLE_PENALTY

% The map should be n by n matrix
n = size( map, 1 );
penalty = OBSTACLE_PENALTY;

sp = [ceil(sp/n) mod(sp,n)];
if sp(2) == 0
    sp(2) = sp(2) + n;
end
ep = [ceil(ep/n) mod(ep,n)];
if ep(2) == 0
    ep(2) = ep(2) + n;
end

% Distance between two nodes
sdist = sum((sp-ep).^2)^0.5;

% Check if there is any obstacle node intersecting the edge
if abs((ep(2) - sp(2)) / (ep(1) - sp(1))) == 1
    startR = min(sp(1), ep(1));
    startC = min(sp(2), ep(2));
    endR = max(sp(1), ep(1));
    endC = max(sp(2), ep(2));
    sdist = max( sdist, sum(diag(map( startR:endR, startC:endC  ))) * penalty);
elseif sp(1) == ep(1)
    sdist = max( sdist, sum( map( sp(1), min(sp(2),ep(2)):max(sp(2),ep(2)) ) ) * penalty );
elseif sp(2) == ep(2)
    sdist = max( sdist, sum( map( min(sp(1),ep(1)):max(sp(1),ep(1)), sp(2) ) ) * penalty );
else
    obsCount = 0;
    [x, y] = bresenham(sp(1), sp(2), ep(1), ep(2), false);
    for i = 1 : size(x, 1)
        if map(x(i), y(i)) == 1
            obsCount = obsCount + 1;
        end
    end
    sdist = max(sdist, obsCount * penalty);
end

res = sdist;
end % End of "map_dist_realtime" function.