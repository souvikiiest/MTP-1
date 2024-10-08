close all;
% clear all;
clearvars -except phiii fi_values isTunnelChanging_dash d_by_B d_B_values b_idx row_i row_j b_by_B b_by_B_values results;

plot_enabled = true;
draw_tunnel = true;
isTunnelChanging = isTunnelChanging_dash;

%% Footing Inputs
B=1;                                             % width of footing
Df=1;                                           % Depth of footing
Df_fan_mesh = B/2;                     % height of fan mesh
L_bound = 10;                              % Length of boundary 16
D_bound = 5;                               % Depth of boundary  8
L=L_bound/2 - B/2;                     % left edge distance from origin
L_fan_mesh_each_side = B/2;
N_foot_soil_interf = 10;                % for half footing width

%% Control Parameters
% b_by_B = 0;
% d_by_B = 7;
gamma = 0;
p = 12;
fi =phiii;
c = 20;

%% Tunnel Inputs
radius = 0.5;
no_of_interface = 24;
distance_of_first_square = radius*1.5;
x_coor_circle_center = (L+B/2 + b_by_B * B);
if (d_by_B==0 || Df == 0)
    y_coor_circle_center = -(distance_of_first_square+15);
else
    y_coor_circle_center = -(Df + d_by_B * B);
end
no_of_circles = 2;

%% creating the folder and fileName

folderName = ['Df_B_' num2str(Df/B)];
if ~exist(folderName, 'dir')
    mkdir(folderName);  
end
subFolderName = [folderName '/b_B_' num2str(b_by_B)];
if ~exist(subFolderName, 'dir')
    mkdir(subFolderName);  
end
fileName = [subFolderName '/b_B_' num2str(b_by_B) '_d_Df_' num2str(d_by_B) '.mat'];


spacing = (8*distance_of_first_square)/(no_of_interface); % spacing of the interfaces

%% Bunch of function calls

[midpointX,midpointY,EmbeddedCoordX,EmbeddedCoordY] = drawRectangularMesh(N_foot_soil_interf,Df_fan_mesh...
    ,B,Df,L_fan_mesh_each_side,L,spacing,plot_enabled);


%% spacing and division of net mesh
vertical_division_right = 0:spacing:L_bound;  % actually total vertical spacing
horizontal_division = -(0:spacing:(D_bound));

%% functions to draw the net mesh without tunnel region
footing_bound_coord = [L-L_fan_mesh_each_side,-(Df+Df_fan_mesh);(L+B+L_fan_mesh_each_side),-(Df+Df_fan_mesh);(L+B+L_fan_mesh_each_side),0; (L-L_fan_mesh_each_side),0];
center_coords=[x_coor_circle_center,y_coor_circle_center];
[adjusted_square_coords,intersections] = drawEmbeddedMeshWithoutTunnel(vertical_division_right, horizontal_division, D_bound, distance_of_first_square, center_coords,footing_bound_coord,plot_enabled);
new_x_coord = adjusted_square_coords(1,1)+distance_of_first_square; % got the new_x_coord after swapping to nearest x-values
new_y_coord = adjusted_square_coords(1,2)+distance_of_first_square;

%% this is for tunnel, but here because new x and  y coord are given as input
if draw_tunnel
    [radial_increment,All_radial_points,square_points] = DrawInterface(radius,no_of_interface,new_x_coord,new_y_coord,no_of_circles,distance_of_first_square,plot_enabled);
    [All_radial_and_sqaure_points,Midpoints_radial_square_x,Midpoints_radial_square_y] = ArrangeAllRadial_SquarePoints(All_radial_points,square_points,no_of_interface,no_of_circles,plot_enabled);
end
[result,midPoints] = processMesh(intersections,plot_enabled);

% footing_x=[L,L,L+B,L+B];
% footing_y = [-Df+0.2,-Df,-Df,-Df+0.2];
% fill(footing_x,footing_y,'cyan');

%% Functions to store nodes anticlockwise
[final_nodes_array_main_mesh] = storeNodesACWMainMesh(result,midPoints);

if draw_tunnel
    [final_nodes_array_tunnel] = storeNodeACWTunnel(All_radial_and_sqaure_points,Midpoints_radial_square_x,Midpoints_radial_square_y);
end
embedded_ACW_nodes = storeNodesACWembedment(midpointX,midpointY,EmbeddedCoordX,EmbeddedCoordY,L,B);


%%%%%
if draw_tunnel
    final_nodes_array = [final_nodes_array_main_mesh;final_nodes_array_tunnel; embedded_ACW_nodes]; % concatenating all,,,, final_nodes_array_tunnel;
else
     final_nodes_array = [final_nodes_array_main_mesh; embedded_ACW_nodes];
end
no_of_element = (size(final_nodes_array,1)/3);
disp("Number of element: "+ no_of_element);

%% storing nodes for different conditions
if isTunnelChanging
    [result_edges,new_node] = findMatchingEdges(final_nodes_array);
else
     if exist(fileName, 'file')
        load(fileName, 'result_edges', 'new_node');
    else
        error('Saved file does not exist: %s', fileName);
    end
end
X=[L,L+B]; Y=[-Df,-Df];
top_boundary_array = find_top_boundary(new_node, X, Y); % finding GL boudnary
side_boundary_array = find_embeddment_side_boundary(final_nodes_array,L, B, Df); % finding embedment side boudnary
tunnel_interface_array=[];
if draw_tunnel
    tunnel_interface = All_radial_points(1:no_of_interface,:);
    tunnel_interface_array = find_tunnel_interface_array(tunnel_interface, final_nodes_array);
end
footing_interface_array = find_footing_interface_array(EmbeddedCoordX,EmbeddedCoordY,B,final_nodes_array);



%% Function call for different conditions
if isTunnelChanging
    [A_element, B_element] = ElementEquilibrium(final_nodes_array, no_of_element, gamma);
    [A_bound, B_Boundary] = BoundaryCondition(Df, top_boundary_array, side_boundary_array, tunnel_interface_array, footing_interface_array, no_of_element, draw_tunnel);
    [A_discontinuity, B_discontinuity] = DiscontinuityEquilibrium(result_edges, no_of_element);

    save(fileName, 'A_element', 'B_element', 'A_bound', 'B_Boundary', 'A_discontinuity', 'B_discontinuity','new_node','result_edges');
else
    % Load the saved data if no need to recalculate
    if exist(fileName, 'file')
        load(fileName, 'A_element', 'B_element', 'A_bound', 'B_Boundary', 'A_discontinuity', 'B_discontinuity');
    else
        error('Saved file does not exist: %s', fileName);
    end
end
% A_yield and B_yield are always recalculated
[A_Yield, B_yield] = YieldEquilibrium(no_of_element, p, fi, c);
C_matrix = ObjectiveFunction(footing_interface_array,no_of_element, side_boundary_array,tunnel_interface_array);



%% checking functions
% checkNodesStored(final_nodes_array);
% checkNodesStored(final_nodes_array_tunnel);
% checkNodesStored(embedded_ACW_nodes);
% plotNodesWithNumbersInside(final_nodes_array);
% CheckBoudnaryNodes(top_boundary_array,'c-o');
% CheckBoudnaryNodes(side_boundary_array,'b-o');
% CheckBoudnaryNodes(tunnel_interface_array,'g-o');
% CheckBoudnaryNodes(footing_interface_array,'y-o');
% DrawDiscontinuity(final_nodes_array,result_edges);

%% SOCP



%% mosek application
ElemX = final_nodes_array(:,1);
ElemY = final_nodes_array(:,2);
A1 = [A_element;A_discontinuity;A_bound];
zerosMatrix = sparse(height(A1),width(A1));
A1 = [A1,zerosMatrix];
B1 = [B_element;B_discontinuity;B_Boundary];
A2 = A_Yield;
B2 = B_yield;


%% this draws both rectangular as well as the radial mesh for the footing
function [midpointX,midpointY,EmbeddedCoordX,EmbeddedCoordY] = drawRectangularMesh(N, Df_fan_mesh, B, Df, L_fan_mesh_each_side, L, spacing,plot_enabled)
    footing_base_left = linspace(L, L + B / 2, N + 1);
    footing_middle_right = -(Df:spacing:(Df + Df_fan_mesh));
    y_coords_active = zeros(size(footing_middle_right, 2), length(footing_base_left));

    % this is the loop for active zone 
    for i = 1:size(footing_middle_right, 2)
        x1 = L; y1 = -Df;
        x2 = L + B / 2; y2 = footing_middle_right(i);
        if(x2-x1)==0
            m=0;
        else
            m = (y2 - y1) / (x2 - x1);
        end
       
        c = y1 - m * x1;
        y_coords_active(i, :) = m * footing_base_left + c;
    end
    x_coords_active = repmat(footing_base_left, [size(y_coords_active, 1), 1]);
    % Calculate the coordinates for the fan mesh base
    fan_mesh_base = L - L_fan_mesh_each_side:spacing: L + B / 2;
    const_y_coords = y_coords_active(end, :);
    x_coords_radial = zeros(length(fan_mesh_base), length(const_y_coords));
    
    % this is the loop for radial zone 
    for i = 1:length(fan_mesh_base)
        x1 = L; y1 = -Df;
        x2 = fan_mesh_base(i); y2 = -Df - Df_fan_mesh;
        m = (y2 - y1) / (x2 - x1);
        c = y1 - m * x1;
        if(isinf(m))
            x_coords_radial(i, :) = L;
        else
            x_coords_radial(i, :) = (const_y_coords - c) / m;
        end
        
    end
    y_coords_radial = repmat(const_y_coords, [size(x_coords_radial, 1), 1]);

    footing_base_left_left = linspace(L - L_fan_mesh_each_side, L, N + 1);
    left_vert_bound_y_coord = -(Df:spacing:(Df + Df_fan_mesh));
    y_coords_passive = zeros(size(left_vert_bound_y_coord, 2), length(footing_base_left_left));
    hold on;

    % this is the loop for passive zone 
    for i = 1:size(left_vert_bound_y_coord, 2)
        x1 = L; y1 = -Df;
        x2 = L - B / 2; y2 = left_vert_bound_y_coord(i);
        m =  (y2 - y1) / (x2 - x1);
        c = y1 - m * x1;
        y_coords_passive(i, :) = m * footing_base_left_left + c;
    end
    x_coords_passive = repmat(footing_base_left_left, [size(y_coords_passive, 1), 1]);
 if plot_enabled
    % Plot for the rectangular lines
    for i = 1:size(footing_base_left, 2)
        plot(x_coords_active(:, i), y_coords_active(:, i), 'k-');
    end
    
    for i = 1:size(footing_base_left, 2)
        plot(x_coords_radial(:, i), y_coords_radial(:, i), 'k-');
    end
    
    for i = 1:size(footing_base_left, 2)
        plot(x_coords_passive(:, i), y_coords_passive(:, i), 'k-');
    end

    % for radial lines.
    for i = 1:size(y_coords_active, 1)
        plot(x_coords_active(i, :), y_coords_active(i, :), 'k-');
    end
    
    for i = 1:size(y_coords_radial, 1)
        plot(x_coords_radial(i, :), y_coords_radial(i, :), 'k-');
    end
    
    for i = 1:size(y_coords_passive, 1)
        plot(x_coords_passive(i, :), y_coords_passive(i, :), 'k-');
    end

    % plot the straight up lines
    for i=1:size(footing_base_left_left,2)
        plot([footing_base_left_left(i),footing_base_left_left(i)],[-Df,0],'k-');
    end
end
    [x_intersections_vertical, y_intersections_vertical] = plotAndStoreIntersectionsInVerticalEmbedment(Df,spacing,footing_base_left_left,plot_enabled);
    
    footing_base_right_right = footing_base_left_left + B + L_fan_mesh_each_side;
    hold on;
    %For right half of footing base
    vertical_line_x = L + B/2; %about this line mirroring will happen
   mirrorCoordinates(x_coords_active, y_coords_active, x_coords_radial, y_coords_radial, x_coords_passive,...
        y_coords_passive, vertical_line_x,footing_base_left,footing_base_right_right,Df,spacing, plot_enabled);

    % Rotating some coords so as to maintain unformity
    x_coords_radial = flipud(x_coords_radial); x_coords_radial(1,:)=[]; x_coords_radial(end,:)=[];
    y_coords_radial = flipud(y_coords_radial); y_coords_radial(1,:)=[]; y_coords_radial(end,:)=[];
    x_coords_passive = rot90(x_coords_passive,2); x_coords_passive(end,:)=[];
    y_coords_passive = rot90(y_coords_passive,2); y_coords_passive(end,:)=[];
    x_intersections_vertical = fliplr(x_intersections_vertical);
    y_intersections_vertical = fliplr(y_intersections_vertical);
    
    all_x_coord_footing_left = [x_coords_active',x_coords_radial',x_coords_passive',x_intersections_vertical'];
    all_y_coord_footing_left = [y_coords_active',y_coords_radial',y_coords_passive',y_intersections_vertical'];
    
    

    mid_points_footing_left_x=[];
    mid_points_footing_left_y=[];
    for j=1:size(all_x_coord_footing_left,2)-1
        for i=1:size(all_x_coord_footing_left,1)-1
            x1 = all_x_coord_footing_left(i,j); y1 = all_y_coord_footing_left(i,j);
            x2 = all_x_coord_footing_left(i+1,j); y2 = all_y_coord_footing_left(i+1,j);
            x3 = all_x_coord_footing_left(i,j+1); y3 = all_y_coord_footing_left(i,j+1);
            x4 = all_x_coord_footing_left(i+1,j+1); y4 = all_y_coord_footing_left(i+1,j+1);
            tol = 1e-6; % Tolerance value
            if (abs(x1 - x4) > tol || abs(y1 - y4) > tol) && (abs(x2 - x3) > tol || abs(y2 - y3) > tol)
                [x_coord_midpoint,y_coord_midpoint] = Divide_Quadri (x1,x2,x3,x4,y1,y2,y3,y4,'k-',plot_enabled);
                mid_points_footing_left_x(i,j) = x_coord_midpoint;
                mid_points_footing_left_y(i,j) = y_coord_midpoint;
            end
             
        end
    end
    % mid_points_footing_left_x = round(mid_points_footing_left_x,3);
    % mid_points_footing_left_y = round(mid_points_footing_left_y,3);
    % all_x_coord_footing_left = round(all_x_coord_footing_left,3);
    % all_y_coord_footing_left = round(all_y_coord_footing_left,3);
   midpointX.footing_left = mid_points_footing_left_x;
   midpointY.footing_left = mid_points_footing_left_y;
   EmbeddedCoordX.footing_left = all_x_coord_footing_left;
   EmbeddedCoordY.footing_left = all_y_coord_footing_left;

   %% Mirroring them
    x_reflection_line = L + B / 2;
    
    all_x_coord_footing_right = 2 * x_reflection_line - all_x_coord_footing_left;
    all_y_coord_footing_right = all_y_coord_footing_left;  % y-coordinates remain the same
    
    mid_points_footing_right_x = [];
    mid_points_footing_right_y = [];

    for j = 1:size(all_x_coord_footing_right, 2) - 1
        for i = 1:size(all_x_coord_footing_right, 1) - 1
            x1 = all_x_coord_footing_right(i, j);  y1 = all_y_coord_footing_right(i, j);    
            x2 = all_x_coord_footing_right(i + 1, j); y2 = all_y_coord_footing_right(i + 1, j);    
            x3 = all_x_coord_footing_right(i, j + 1); y3 = all_y_coord_footing_right(i, j + 1);   
            x4 = all_x_coord_footing_right(i + 1, j + 1); y4 = all_y_coord_footing_right(i + 1, j + 1);
            
            if ~( (x1 == x2 && y1 == y2) || (x1 == x3 && y1 == y3) || (x1 == x4 && y1 == y4) || ...
      (x2 == x3 && y2 == y3) || (x2 == x4 && y2 == y4) || (x3 == x4 && y3 == y4) )
                [x_coord_midpoint, y_coord_midpoint] = Divide_Quadri(x1, x2, x3, x4, y1, y2, y3, y4,'k-',plot_enabled);
                
                mid_points_footing_right_x(i, j) = x_coord_midpoint;
                mid_points_footing_right_y(i, j) = y_coord_midpoint;
            end
        end
    end
    % all_x_coord_footing_right = round(all_x_coord_footing_right,3);
    % all_y_coord_footing_right = round(all_y_coord_footing_right,3);
    % mid_points_footing_right_x = round(mid_points_footing_right_x,3);
    % mid_points_footing_right_y = round(mid_points_footing_right_y,3);

    EmbeddedCoordX.footing_right = all_x_coord_footing_right;
    EmbeddedCoordY.footing_right = all_y_coord_footing_right;
    midpointX.footing_right = mid_points_footing_right_x;
    midpointY.footing_right = mid_points_footing_right_y;
    
end

%% Tunnel drawing
function  [radial_increment,All_radial_points,square_points] = DrawInterface(radius,divisions,x_coor_circle_center,...
    y_coor_circle_center,no_of_circles,distance_of_first_square,plot_enabled)
    hold on;
    %drawing the first square
    DrawSquare(distance_of_first_square,x_coor_circle_center,y_coor_circle_center);
    
    % Now i need to divide the (distance_of_first_square - radius) in no_of_circles
    radial_increment = (distance_of_first_square - radius) / (no_of_circles+1);
    old_radius = radius;
    theta_increment = 2*pi/divisions;
    All_radial_points =zeros(divisions*no_of_circles,2);
    
    for j = 1:no_of_circles+1
        new_radius = old_radius + radial_increment*(j-1);
        temp_radial_points = zeros(divisions,2);
    
        for i = 1:divisions
            theta = (i - 1) * theta_increment;
            x = new_radius * cos(theta) + x_coor_circle_center;
            y = new_radius * sin(theta) + y_coor_circle_center;
            temp_radial_points(i, :) = [x, y];
        end
    
        row_start = (j - 1) * (divisions) + 1;
        row_end = j * (divisions);
        All_radial_points(row_start : row_end , :) = temp_radial_points;
        temp_radial_points(divisions + 1, :) = temp_radial_points(1, :);
        if plot_enabled
            plot(temp_radial_points(:, 1), temp_radial_points(:, 2), 'b-');
            
        end
        hold on;
    end
    % to join the first and last circle by straight line
    first_division_points = All_radial_points(1:divisions, :);
    last_division_start = no_of_circles * divisions + 1;
    last_division_points = All_radial_points(last_division_start:last_division_start + divisions - 1, :);
    for i = 1:divisions
        if plot_enabled
            plot([first_division_points(i, 1), last_division_points(i, 1)], ...
            [first_division_points(i, 2), last_division_points(i, 2)], 'k-');
        end
    end
    hold on;
    square_points = DivideSquarePerimeter(x_coor_circle_center, y_coor_circle_center, distance_of_first_square, divisions);
    % Now to join the last divisions coordiantes with the square divisions.
    Last_circle_coords = All_radial_points(end-divisions+1:end,:);
    JoinSqaureAndLastCircle(Last_circle_coords,square_points,divisions,plot_enabled);

end

function DrawSquare(distance_of_square,x_coor_circle_center,y_coor_circle_center)
    %Draw the square outside
    
    half_side_length = distance_of_square;
    
    square_x = [x_coor_circle_center - half_side_length, x_coor_circle_center + half_side_length, ...
        x_coor_circle_center + half_side_length, x_coor_circle_center - half_side_length, ...
        x_coor_circle_center - half_side_length];
    square_y = [y_coor_circle_center - half_side_length, y_coor_circle_center - half_side_length, ...
        y_coor_circle_center + half_side_length, y_coor_circle_center + half_side_length, ...
        y_coor_circle_center - half_side_length];
    
    plot(square_x, square_y, 'k-');
    hold on;
    %square draw ends
end

function square_division_points = DivideSquarePerimeter(cx, cy, half_side_length, divisions)

side_length = 2 * half_side_length;
perimeter = 4 * side_length;
division_length = perimeter / divisions;
square_division_points = zeros(divisions, 2);

for i = 1:divisions
    current_perimeter = division_length * (i - 1);
    if current_perimeter < side_length
        % Top edge (moving left)
        x = cx + half_side_length - current_perimeter;
        y = cy + half_side_length;
    elseif current_perimeter < 2 * side_length
        % Left edge (moving down)
        x = cx - half_side_length;
        y = cy + half_side_length - (current_perimeter - side_length);
    elseif current_perimeter < 3 * side_length
        % Bottom edge (moving right)
        x = cx - half_side_length + (current_perimeter - 2 * side_length);
        y = cy - half_side_length;
    else
        % Right edge (moving up)
        x = cx + half_side_length;
        y = cy - half_side_length + (current_perimeter - 3 * side_length);
    end
    square_division_points(i, :) = [x, y];
end
n=divisions/8;
new_array = [square_division_points(end-n+1:end, :); square_division_points(1:end-n, :)]; %To place the coordinates
%such that it starts from the right middle(0 degree). previously it was
%storing from 45 degree.
square_division_points = new_array;
end

function JoinSqaureAndLastCircle(Last_circle_coords,square_points,divisions,plot_enabled)
for i=1:divisions
    square_point = square_points(i,:);
    circle_point = Last_circle_coords(i,:);
    if plot_enabled
        plot([square_point(1),circle_point(1)],[square_point(2),circle_point(2)],'k-');
    end
end

end

function [All_radial_and_sqaure_points,Midpoints_radial_square_x,Midpoints_radial_square_y] = ...
    ArrangeAllRadial_SquarePoints(All_radial_points,square_points,divisions,circles,plot_enabled)


All_radial_and_sqaure_points=[];
for i=1:circles+1
    temp_division = [];
    row_start = divisions*(i-1)+1;
    row_end = divisions*(i);
    temp_division = All_radial_points(row_start:row_end,:);
    All_radial_and_sqaure_points=[All_radial_and_sqaure_points,temp_division];
end
All_radial_and_sqaure_points = [All_radial_and_sqaure_points,square_points];
All_radial_and_sqaure_points(end+1,:) = All_radial_and_sqaure_points(1,:);

Midpoints_radial_square_x = [];
Midpoints_radial_square_y = [];
for i=1:circles+1
    for j=1:divisions
        x1=All_radial_and_sqaure_points(j,2*(i-1)+1); y1=All_radial_and_sqaure_points(j,2*(i-1)+2);
        x2=All_radial_and_sqaure_points(j+1,2*(i-1)+1); y2=All_radial_and_sqaure_points(j+1,2*(i-1)+2);
        x3=All_radial_and_sqaure_points(j,2*i+1); y3=All_radial_and_sqaure_points(j,2*i+2);
        x4=All_radial_and_sqaure_points(j+1,2*i+1); y4=All_radial_and_sqaure_points(j+1,2*i+2);
        [x_coord_midpoint,y_coord_midpoint] = Divide_Quadri(x1,x2,x3,x4,y1,y2,y3,y4,'r-',plot_enabled);
        Midpoints_radial_square_x(i,j) = x_coord_midpoint;
        Midpoints_radial_square_y(i,j) = y_coord_midpoint;
    end
end

end

%% Embedded Mesh drawing without tunnel and footing

function [adjusted_square_coords, intersections] = drawEmbeddedMeshWithoutTunnel(vertical_division_right, horizontal_division, D_bound, half_square_side_length, center_coords, footing_bound_coord,plot_enabled)
    hold on;
    
    % Define the square coordinates
    half_length = half_square_side_length;
    square_coords = [
        center_coords(1) - half_length, center_coords(2) - half_length;
        center_coords(1) + half_length, center_coords(2) - half_length;
        center_coords(1) + half_length, center_coords(2) + half_length;
        center_coords(1) - half_length, center_coords(2) + half_length
    ];
    
    % Find nearest points in the mesh for each corner of the square
    for i = 1:size(square_coords, 1)
        [~, nearest_x_index] = min(abs(vertical_division_right - square_coords(i, 1)));
        [~, nearest_y_index] = min(abs(horizontal_division - square_coords(i, 2)));
        
        square_coords(i, 1) = vertical_division_right(nearest_x_index);
        square_coords(i, 2) = horizontal_division(nearest_y_index);
    end
    
    adjusted_square_coords = square_coords;
    
    % Calculate the boundaries of the tunnel
x_min = min(square_coords(:, 1));
x_max = max(square_coords(:, 1));
y_min = min(square_coords(:, 2));
y_max = max(square_coords(:, 2));

% Boundaries of the footing region
x_min_f = min(footing_bound_coord(:, 1));
x_max_f = max(footing_bound_coord(:, 1));
y_min_f = min(footing_bound_coord(:, 2));
y_max_f = max(footing_bound_coord(:, 2));

intersections = [];

% Plotting vertical lines, omitting tunnel and footing regions
    for i = 1:length(vertical_division_right)
        x = vertical_division_right(i);
        
        % Check if the line is outside both the tunnel and footing regions
        if (x <= x_min || x >= x_max) && (x <= x_min_f || x >= x_max_f)
            if plot_enabled
                plot([x; x], [0; -D_bound], 'k-');
            end
            intersections = [intersections; repmat(x, length(horizontal_division), 1), horizontal_division'];
        else
            % Vertical lines within the tunnel region but outside the footing region
            if x >= x_min && x <= x_max && (x <= x_min_f || x >=x_max_f)
                if plot_enabled
                    plot([x; x], [0; y_max], 'k-');
                    plot([x; x], [y_min; -D_bound], 'k-');
                end
                intersections = [intersections; repmat(x, length(horizontal_division(horizontal_division <= y_min | horizontal_division >= y_max)), 1), horizontal_division(horizontal_division <= y_min | horizontal_division >= y_max)'];
            end
            
            % Vertical lines within the footing region but outside the tunnel region
            if x >= x_min_f && x <= x_max_f && (x <= x_min || x >= x_max)
                if plot_enabled
                    plot([x; x], [y_min_f; -D_bound], 'k-');
                end
                intersections = [intersections; repmat(x, length(horizontal_division(horizontal_division <= y_min_f | horizontal_division >= y_max_f)), 1), horizontal_division(horizontal_division <= y_min_f | horizontal_division >= y_max_f)'];
            end
            
            % Vertical lines within both the tunnel and footing regions
            if x >= x_min && x <= x_max && x >= x_min_f && x <= x_max_f
                if plot_enabled
                    plot([x; x], [y_min_f; y_max], 'k-');
                    plot([x; x], [y_min; -D_bound], 'k-');
                end
                 valid_y = horizontal_division(horizontal_division <= y_min | ...
                                          (horizontal_division >= y_max_f & horizontal_division <= y_max)|...
                                          (horizontal_division >= y_max & horizontal_division <= y_min_f));
                intersections = [intersections; repmat(x, length(valid_y), 1), valid_y'];            
            end
        end
    end
    
    % Plotting horizontal lines, omitting tunnel and footing regions
    for i = 1:length(horizontal_division)
        y = horizontal_division(i);
        if (y <= y_min || y >= y_max) && (y <= y_min_f || y >= y_max_f)
            if plot_enabled
                plot([min(vertical_division_right), max(vertical_division_right)], [y, y], 'k-');
            end
        else
            if y >= y_min && y <= y_max % checks if y is within tunnel area
                if y >= y_min_f && y <= y_max_f  % checks if y is within footing area
                    if plot_enabled
                        plot([x_max_f,x_min],[y,y],'k-');
                        plot([x_max,max(vertical_division_right)],[y,y],'k-');
                    end
                else
                    if plot_enabled
                        plot([min(vertical_division_right),x_min],[y,y],'k-');
                        plot([x_max,max(vertical_division_right)],[y,y],'k-');
                    end
                end
            end
            if y >= y_min_f && y <= y_max_f %checks if y is within footing area
                % to plot the horizontal lines on the left side of footing
                if plot_enabled
                    plot([min(vertical_division_right),x_min_f],[y,y],'k-');
                end
                if  y >= y_max || y <= y_min % checks if the y is above tunnel but within footing area
                    if plot_enabled
                        plot([x_max_f,max(vertical_division_right)],[y,y],'k-');
                    end
                end
                
            end
            
        end
    end
    
    hold off;
end

function [x_intersections, y_intersections,x_coords_active_mirrored,y_coords_active_mirrored,...
    x_coords_radial_mirrored,y_coords_radial_mirrored, x_coords_passive_mirrored, ...
    y_coords_passive_mirrored ]= mirrorCoordinates(x_coords_active, y_coords_active,...
    x_coords_radial, y_coords_radial, x_coords_passive,y_coords_passive, vertical_line_x,...
    footing_base_left,footing_base_right_right,Df,spacing,plot_enabled)
 if plot_enabled
        % Mirror x_coords_active about the vertical line
        x_coords_active_mirrored = 2 * vertical_line_x - x_coords_active;
        y_coords_active_mirrored = y_coords_active;
    
        % Mirror x_coords_radial about the vertical line
        x_coords_radial_mirrored = 2 * vertical_line_x - x_coords_radial;
        y_coords_radial_mirrored = y_coords_radial;
    
        % Mirror x_coords_passive about the vertical line
        x_coords_passive_mirrored = 2 * vertical_line_x - x_coords_passive;
        y_coords_passive_mirrored = y_coords_passive;
    
        % plot the radial lines
        for i = 1:size(y_coords_active_mirrored, 1)
            plot(x_coords_active_mirrored(i, :), y_coords_active_mirrored(i, :), 'k-');
        end
        for i = 1:size(y_coords_radial_mirrored, 1)
            plot(x_coords_radial_mirrored(i, :), y_coords_radial_mirrored(i, :), 'k-');
        end
        for i = 1:size(y_coords_passive_mirrored, 1)
            plot(x_coords_passive_mirrored(i, :), y_coords_passive_mirrored(i, :), 'k-');
        end
    
        % plot the rectangular lines
        for i = 1:size(footing_base_left, 2)
            plot(x_coords_active_mirrored(:, i), y_coords_active_mirrored(:, i), 'k-');
        end
        
        for i = 1:size(footing_base_left, 2)
            plot(x_coords_radial_mirrored(:, i), y_coords_radial_mirrored(:, i), 'k-');
        end
        
        for i = 1:size(footing_base_left, 2)
            plot(x_coords_passive_mirrored(:, i), y_coords_passive_mirrored(:, i), 'k-');
        end
    
        %for plotting the vertical lines in embedded depth
        for i=1:size(footing_base_right_right,2)
            plot([footing_base_right_right(i),footing_base_right_right(i)],[-Df,0],'k-');
        end
end
   [x_intersections, y_intersections] = plotAndStoreIntersectionsInVerticalEmbedment( Df, spacing,footing_base_right_right,plot_enabled);
end

function [x_intersections, y_intersections] = plotAndStoreIntersectionsInVerticalEmbedment( Df, spacing,footing_base_left_left,plot_enabled)
    % Define the x-coordinates for the vertical lines
    %footing_base_left_left = linspace(L - L_fan_mesh_each_side, L, N + 1);
    
    % Define the y-coordinates for the horizontal lines
    horizontal_lines_y = -(Df:-spacing:0);

    % Initialize arrays to store the intersection coordinates
    x_intersections = zeros(length(horizontal_lines_y), length(footing_base_left_left));
    y_intersections = zeros(length(horizontal_lines_y), length(footing_base_left_left));
    
    
    % Plot the horizontal lines and store intersections
    for j = 1:length(horizontal_lines_y)
        if plot_enabled
            plot([footing_base_left_left(1), footing_base_left_left(end)], [horizontal_lines_y(j), horizontal_lines_y(j)], 'k-');
        end
        % Store intersection coordinates
        x_intersections(j, :) = footing_base_left_left;
        y_intersections(j, :) = horizontal_lines_y(j);
    end
    
     hold on;
end

%% store coordinates in arranged way
function [result,midPoints] = processMesh(intersection,plot_enabled)
    % Find unique x-coordinates
    uniqueX = unique(intersection(:, 1)); 

    % Find unique y-coordinates
    uniqueY = unique(intersection(:, 2));
    
    outputArray = NaN(length(uniqueY), 2*length(uniqueX));
    outputArray(:, 1) = uniqueX(1);
    outputArray(:, 2) = uniqueY;
    
    for i = 2:length(uniqueX)
        x = uniqueX(i);
        outputArray(:, 2*i-1) = x; % filling x-coordinates
        
        for j = 1:length(uniqueY)
            y = uniqueY(j);
            
            % Check if this (x, y) pair exists in the intersection
            index = find(intersection(:, 1) == x & intersection(:, 2) == y, 1);
            
            if ~isempty(index)
                % If the pair exists, fill the corresponding value
                outputArray(j, 2*i) = intersection(index, 2);
            else
                % If the pair does not exist, put NaN
                outputArray(j, 2*i) = NaN;
            end
        end
    end
    
  result = outputArray;
  midPoints = NaN(size(outputArray, 1) - 1, (size(outputArray, 2) - 3) / 2 * 2);
  for j = 1:2:size(outputArray, 2) - 3
      columnIndex = (j + 1)/2;
        for i = 1:size(outputArray, 1) - 1
            x1 = outputArray(i, j);
            y1 = outputArray(i, j + 1);
            x3 = outputArray(i, j + 2);
            y3 = outputArray(i, j + 3); 
            
            x2 = outputArray(i + 1, j);
            y2 = outputArray(i + 1, j + 1);
            x4 = outputArray(i + 1, j + 2);
            y4 = outputArray(i + 1, j + 3);
    
            % Check for NaN values
            if ~any(isnan([x1, x2, x3, x4, y1, y2, y3, y4]))
               [x_coord_midpoint,y_coord_midpoint] = Divide_Quadri(x1, x2, x3, x4, y1, y2, y3, y4,'k-',plot_enabled);
                midPoints(i, 2 * columnIndex - 1) = x_coord_midpoint;
                midPoints(i, 2 * columnIndex) = y_coord_midpoint;
            end
        end
   end



end

%% divide quadrilateral into triangles
function [x_coord_midpoint, y_coord_midpoint] = Divide_Quadri(x1, x2, x3, x4, y1, y2, y3, y4, color,plot_enabled)
    % Plot the lines
    % plot([x1; x4], [y1; y4], color);
    % plot([x2; x3], [y2; y3], color);
    % hold on;
    
    % Coordinates for the two lines
    % X1 = [x1, x4];
    % X2 = [x2, x3];
    % Y1 = [y1, y4];
    % Y2 = [y2, y3];
    % 
    % % Check if either line is vertical (m = inf)
    % if X1(2) - X1(1) == 0
    %     m1 = inf;
    %     c1 = X1(1); % Vertical line, c1 is the x-coordinate
    % else
    %     m1 = (Y1(2) - Y1(1)) / (X1(2) - X1(1));
    %     c1 = Y1(1) - m1 * X1(1);
    % end
    % 
    % if X2(2) - X2(1) == 0
    %     m2 = inf;
    %     c2 = X2(1); % Vertical line, c2 is the x-coordinate
    % else
    %     m2 = (Y2(2) - Y2(1)) / (X2(2) - X2(1));
    %     c2 = Y2(1) - m2 * X2(1);
    % end
    % 
    % % Calculate the intersection point
    % if isinf(m1) % If first line is vertical
    %     x_coord_midpoint = c1;
    %     y_coord_midpoint = m2 * x_coord_midpoint + c2;
    % 
    % elseif isinf(m2) % If second line is vertical
    %     x_coord_midpoint = c2;
    %     y_coord_midpoint = m1 * x_coord_midpoint + c1;
    % 
    % else
    %     x_coord_midpoint = (c1 - c2) / (m2 - m1);
    %     y_coord_midpoint = m2 * x_coord_midpoint + c2;
    % end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
if (x1 == x2 && y1 == y2) || (x1 == x3 && y1 == y3) || (x1 == x4 && y1 == y4)
        x_coord_midpoint = x1;
        y_coord_midpoint = y1;
    elseif (x2 == x3 && y2 == y3) || (x2 == x4 && y2 == y4)
        x_coord_midpoint = x2;
        y_coord_midpoint = y2;
    elseif (x3 == x4 && y3 == y4)
        x_coord_midpoint = x3;
        y_coord_midpoint = y3;
    else
        % Calculate the midpoints for a quadrilateral
        x_coord_midpoint = (x1 + x2 + x3 + x4) / 4;
        y_coord_midpoint = (y1 + y2 + y3 + y4) / 4;
end

    % Plot x1, y1, midpoint, and x4, y4
    if plot_enabled
        plot([x1 x_coord_midpoint x4], [y1 y_coord_midpoint y4], color);
        % Plot x2, y2, midpoint, and x3, y3
        plot([x2 x_coord_midpoint x3], [y2 y_coord_midpoint y3],color);
    end

end


%% Store the coordinates in acti-clockwise direction
function [final_nodes_array] = storeNodesACWMainMesh(result, midPoints)
    result = flipud(result);
    midPoints = flipud(midPoints);
    
    final_nodes_array = [];
    rescol=1; resrow=1;
    for midcol = 1:2:size(midPoints, 2)
        resrow =1;
        for midrow = 1:size(midPoints, 1)
            % Get the midpoint (node 5)
            x5 = midPoints(midrow, midcol);
            y5 = midPoints(midrow, midcol+1);
            
            % Get the four corner nodes (node 1, 2, 3, 4)
            x1 = result(resrow, rescol); y1 = result(resrow, rescol+1);            
            x2 = result(resrow+1 , rescol); y2 = result(resrow + 1, rescol+1);    
            x3 = result(resrow + 1, rescol+2);  y3 = result(resrow+1, rescol+3);
            x4 = result(resrow , rescol+2);  y4 = result(resrow, rescol+3);
           
            if ~any(isnan([x1, x2, x3, x4, x5, y1, y2, y3, y4, y5]))
                final_nodes_array = [final_nodes_array; 
                x5, y5; x1, y1; x2, y2;   % Triangle 1
                x5, y5; x2, y2; x3, y3;   % Triangle 2
                x5, y5; x3, y3; x4, y4;   % Triangle 3
                x5, y5; x4, y4; x1, y1;   % Triangle 4
                ];
            end
            resrow = resrow + 1;

        end
        
        rescol=rescol+2;
    end
end

function [final_nodes_array] = storeNodeACWTunnel(All_radial_and_sqaure_points,Midpoints_radial_square_x,Midpoints_radial_square_y)
    Midpoints_radial_square_x = Midpoints_radial_square_x';
    Midpoints_radial_square_y = Midpoints_radial_square_y';
    final_nodes_array_tunnel = 1;
    for i=1:size(Midpoints_radial_square_y,2)
        Midpoint_all_x_y(:,2*i-1) = Midpoints_radial_square_x(:,i);
        Midpoint_all_x_y(:,2*i) = Midpoints_radial_square_y(:,i);
    end
    midPoints = Midpoint_all_x_y;
    result = All_radial_and_sqaure_points;
    final_nodes_array = [];
    rescol=1; 
    for midcol = 1:2:size(midPoints, 2)
        resrow =1;
        for midrow = 1:size(midPoints, 1)
            % Get the midpoint (node 5)
            x5 = midPoints(midrow, midcol);
            y5 = midPoints(midrow, midcol+1);
            
            % Get the four corner nodes (node 1, 2, 3, 4)
            x2 = result(resrow, rescol); y2 = result(resrow, rescol+1);            
            x1 = result(resrow+1 , rescol); y1 = result(resrow + 1, rescol+1);    
            x4 = result(resrow + 1, rescol+2);  y4 = result(resrow+1, rescol+3);
            x3 = result(resrow , rescol+2);  y3 = result(resrow, rescol+3);
           
            if ~any(isnan([x1, x2, x3, x4, x5, y1, y2, y3, y4, y5]))
                final_nodes_array = [final_nodes_array; 
                x5, y5; x1, y1; x2, y2; % Triangle 1
                x5, y5; x2, y2; x3, y3; % Triangle 2
                x5, y5; x3, y3; x4, y4; % Triangle 3
                x5, y5; x4, y4; x1, y1;   % Triangle 4
                ];
            end
            resrow = resrow + 1;

        end
        
        rescol=rescol+2;
    end

end

function embedded_ACW_nodes = storeNodesACWembedment(midpointX,midpointY,EmbeddedCoordX,EmbeddedCoordY,L,B)
    % Initialize final_nodes_array
    final_nodes_array = [];
    
    % Extract relevant coordinates for left and right sides
    all_x_left = EmbeddedCoordX.footing_left;
    all_y_left = EmbeddedCoordY.footing_left;
    mid_x_left = midpointX.footing_left;
    mid_y_left = midpointY.footing_left;

    all_x_right = EmbeddedCoordX.footing_right;
    all_y_right = EmbeddedCoordY.footing_right;
    mid_x_right = midpointX.footing_right;
    mid_y_right = midpointY.footing_right;

    % Left side footing coordinates
    for midcol = 1:size(mid_x_left, 2)
        for midrow = 1:size(mid_x_left, 1)
            % Get the midpoint (node 5)
            x5 = mid_x_left(midrow, midcol);
            y5 = mid_y_left(midrow, midcol);

            % Get the four corner nodes (node 1, 2, 3, 4)
            x1 = all_x_left(midrow, midcol); y1 = all_y_left(midrow, midcol);
            x2 = all_x_left(midrow + 1, midcol); y2 = all_y_left(midrow + 1, midcol);
            x4 = all_x_left(midrow + 1, midcol + 1); y4 = all_y_left(midrow + 1, midcol + 1);
            x3 = all_x_left(midrow, midcol + 1); y3 = all_y_left(midrow, midcol + 1);

            % Handle midpoint case
            if x1==x3 && y1 == y3
                final_nodes_array = [final_nodes_array; x1, y1; x4, y4; x2, y2];
            else
                final_nodes_array = [final_nodes_array;
                    x5, y5;  x1, y1;  x3, y3;   % Triangle 1
                    x5, y5;  x3, y3;  x4, y4;   % Triangle 2
                    x5, y5;  x4, y4;  x2, y2;   % Triangle 3
                    x5, y5;  x2, y2;  x1, y1;   % Triangle 4
                ];
            end
        end
    end
    
    % Right side footing coordinates
    final_nodes_array_m = [];  % Initialize final_nodes_array_m
    for midcol = 1:size(mid_x_right, 2)
        for midrow = 1:size(mid_x_right, 1)
            % Get the midpoint (node 5)
            x5_m = mid_x_right(midrow, midcol);
            y5_m = mid_y_right(midrow, midcol);

            % Get the four corner nodes (node 1, 2, 3, 4)
            x1_m = all_x_right(midrow, midcol); y1_m = all_y_right(midrow, midcol);
            x2_m = all_x_right(midrow + 1, midcol); y2_m = all_y_right(midrow + 1, midcol);
            x3_m = all_x_right(midrow + 1, midcol + 1); y3_m = all_y_right(midrow + 1, midcol + 1);
            x4_m = all_x_right(midrow, midcol + 1); y4_m = all_y_right(midrow, midcol + 1);

            % Handle midpoint case
            if x1_m==x4_m && y1_m == y4_m
                final_nodes_array_m = [final_nodes_array_m; x1_m, y1_m; x2_m, y2_m; x3_m, y3_m;];
            else
                final_nodes_array_m = [final_nodes_array_m;
                    x5_m, y5_m;  x4_m, y4_m;  x1_m, y1_m;   % Triangle 1
                    x5_m, y5_m;  x1_m, y1_m;  x2_m, y2_m;   % Triangle 2
                    x5_m, y5_m;  x2_m, y2_m;  x3_m, y3_m;   % Triangle 3
                    x5_m, y5_m;  x3_m, y3_m;  x4_m, y4_m;   % Triangle 4
                ];
            end
        end
    end
    
    % Combine left and right side nodes
    embedded_ACW_nodes = [final_nodes_array; final_nodes_array_m];
end


%% checking the nodes stored are correct or not

function checkNodesStored(final_nodes_array)
    for i = 1:4:size(final_nodes_array, 1)  % Iterate through in steps of 4
        x1 = final_nodes_array(i, 1);     y1 = final_nodes_array(i, 2);
        x2 = final_nodes_array(i+1, 1);   y2 = final_nodes_array(i+1, 2);
        x3 = final_nodes_array(i+2, 1);   y3 = final_nodes_array(i+2, 2);
        x4 = final_nodes_array(i+3, 1);   y4 = final_nodes_array(i+3, 2);

        plot([x1; x2; x3; x4], [y1; y2; y3; y4], 'g-');
        hold on; 
    end
     % Release hold after plotting all triangles
end

function CheckBoudnaryNodes(top_boundary_array,color)
    for i = 1:2:size(top_boundary_array, 1) - 1
        x1 = top_boundary_array(i, 2);
        y1 = top_boundary_array(i, 3);
        x2 = top_boundary_array(i + 1, 2);
        y2 = top_boundary_array(i + 1, 3);
        
        plot([x1, x2], [y1, y2], color, 'LineWidth', 2, 'MarkerSize', 6);
        hold on;
    end
end

function plotNodesWithNumbersInside(total_node_table)
    % Get the number of nodes
    numNodes = size(total_node_table, 1);
 
    for i = 1:3:numNodes
        % Extract the coordinates of the three nodes forming the triangle
        x_coords = total_node_table(i:i+2, 1);
        y_coords = total_node_table(i:i+2, 2);
        

        area = abs(x_coords(1)*(y_coords(2)-y_coords(3)) + x_coords(2)*(y_coords(3)-y_coords(1)) + x_coords(3)*(y_coords(1)-y_coords(2))) / 2;
        
        % Adjust the offset based on the area
        offset = 0.1 * sqrt(area);  

        % Calculate the centroid of the triangle
        centroid_x = mean(x_coords);
        centroid_y = mean(y_coords);
      
        
        % Annotate the nodes inside the triangle
       text(centroid_x - offset, centroid_y, num2str(i), 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'right');
        text(centroid_x + offset, centroid_y, num2str(i+1), 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'left');
        text(centroid_x, centroid_y + offset, num2str(i+2), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'center');
    end
    
    
end

function DrawDiscontinuity(total_node_table,result)
    hold on;
    for i=1:size(result,1)
        X=[total_node_table(result(i,2),1),total_node_table(result(i,3),1)];
        Y=[total_node_table(result(i,2),2),total_node_table(result(i,3),2)];
        plot(X,Y,'r-','LineWidth',1);
    end
   
end


%% storig nodes for different condition calls
function top_boundary_array = find_top_boundary(final_nodes_array, X, Y)
    result = [];

    for i = 1:(size(final_nodes_array, 1) - 1)
        if final_nodes_array(i, 3) == 0 && final_nodes_array(i + 1, 3) == 0  
            if ~(final_nodes_array(i, 2) > X(1) && final_nodes_array(i, 2) <= X(2) && ...
                 final_nodes_array(i, 3) >= Y(1) && final_nodes_array(i, 3) <= Y(2))
                node_number1 = final_nodes_array(i,1); node_number2 = final_nodes_array(i+1,1);

                result = [result; node_number1, final_nodes_array(i, 2:3);node_number2,final_nodes_array(i+1,2:3)];
                
            end
        end
        
    end
    % result = [result;result(end,1)-20,8,0;result(end,1)-18,8.167,0]; % comment when depth changes
    top_boundary_array = result;
end

function side_boundary_array = find_embeddment_side_boundary(final_nodes_array, L, B, Df)
    side_boundary_array = [];

    for i = 1:size(final_nodes_array, 1) - 1
        x1 = final_nodes_array(i, 1);
        y1 = final_nodes_array(i, 2);
        x2 = final_nodes_array(i + 1, 1);
        y2 = final_nodes_array(i + 1, 2);

       
        if y1 <= 0 && y1 >= -Df && y2 <= 0 && y2 >= -Df
           
            if (x1 == L && x2 == L ) || (x1 == L + B && x2 == L + B)
                
                side_boundary_array = [side_boundary_array;i, x1, y1;i+1, x2, y2];
            end
        end
    end
end

function tunnel_interface_array = find_tunnel_interface_array(tunnel_interface, final_nodes_array)
    tunnel_interface_array = [];
    tunnel_interface = [tunnel_interface;tunnel_interface(1,:)];
    num_tunnel_coords = size(tunnel_interface, 1);

    for i = 1:size(final_nodes_array, 1)-1
        x1 = final_nodes_array(i, 1);
        y1 = final_nodes_array(i, 2);
        x2 = final_nodes_array(i + 1, 1);
        y2 = final_nodes_array(i + 1, 2);

        for j = 1:num_tunnel_coords - 1
            x_t1 = tunnel_interface(j, 1);
            y_t1 = tunnel_interface(j, 2);
            x_t2 = tunnel_interface(j + 1, 1);
            y_t2 = tunnel_interface(j + 1, 2);

            if (x1 == x_t1 && y1 == y_t1 && x2 == x_t2 && y2 == y_t2) || ...
               (x1 == x_t2 && y1 == y_t2 && x2 == x_t1 && y2 == y_t1)
                tunnel_interface_array = [tunnel_interface_array; i, x1, y1; i + 1, x2, y2];
                break; 
            end
        end
    end
    for i=1:2:size(tunnel_interface_array,1)
        x1 = tunnel_interface_array(i,2); y1 = tunnel_interface_array(i,3);
        x2 = tunnel_interface_array(i+1,2); y2 = tunnel_interface_array(i+1,3);
        m = (y2-y1)/(x2-x1);
        if isinf(m)
            m=deg2rad(90);
            tunnel_interface_array(i:i+1,4) = atan(m);
        else
            tunnel_interface_array(i:i+1,4) = atan(m);
        end
    end
end

function footing_interface_array = find_footing_interface_array(EmbeddedCoordX,EmbeddedCoordY,B,final_nodes_array)
    footing_interface_array = [];
    x_coord = EmbeddedCoordX.footing_left(:,1);
    y_coord = EmbeddedCoordY.footing_left(:,1);
    for i=1:size(x_coord,1)-1
        footing_interface_array = [footing_interface_array;x_coord(i,1),y_coord(i,1);x_coord(i+1,1),y_coord(i+1,1)];
    end
    
    mirrored_coord = footing_interface_array;
    mirrored_coord(:,1) = footing_interface_array(:,1) + B/2;
    footing_interface_array = [footing_interface_array;mirrored_coord];
    % finding footing_interface in final_nodes_array
    footing_interface = footing_interface_array ;
    footing_interface_array = [];
    

    for j = 1:size(footing_interface, 1) - 1
        x_t1 = footing_interface(j, 1);
        y_t1 = footing_interface(j, 2);
        x_t2 = footing_interface(j + 1, 1);
        y_t2 = footing_interface(j + 1, 2);

        for i = 1:size(final_nodes_array, 1) - 2
            x1 = final_nodes_array(i, 1);
            y1 = final_nodes_array(i, 2);
            x2 = final_nodes_array(i + 1, 1);
            y2 = final_nodes_array(i + 1, 2);
            x3 = final_nodes_array(i + 2, 1);
            y3 = final_nodes_array(i + 2, 2);

            % If footing_interface's first point matches with final_nodes_array's first point,
            % and footing_interface's second point matches with final_nodes_array's third point
            if j == 1
                if (x1 == x_t1 && y1 == y_t1 && x3 == x_t2 && y3 == y_t2) || ...
                   (x1 == x_t2 && y1 == y_t2 && x3 == x_t1 && y3 == y_t1)
                    footing_interface_array = [footing_interface_array; i, x_t1, y_t1; i + 1, x_t2, y_t2];
                    break;
                end
            end

            % Regular two-point matching
            if (x1 == x_t1 && y1 == y_t1 && x2 == x_t2 && y2 == y_t2) || ...
               (x1 == x_t2 && y1 == y_t2 && x2 == x_t1 && y2 == y_t1)
                footing_interface_array = [footing_interface_array; i, x_t1, y_t1; i + 1, x_t2, y_t2];
                break;
            end
        end
    end
end

function [result,new_node] = findMatchingEdges(nodes_array)
    result = [];
    n = size(nodes_array, 1);
    new_node=[];
    index_array = (1:n)';
    new_nodes_array = [index_array,nodes_array];

    i=1;
    tol = 1e-4; % Define a small tolerance value
    
    while i<n-1
        triangle = new_nodes_array(i:i+2,:);
        new_node = [new_node;triangle;triangle(1,:)]; % repeats first triangle node at the end
        i=i+3;
    end

    n = size(new_node, 1);
    for i = 1:n
        if mod(i,4)==0  % skips first node repetition (of first element) with next element
            continue;
        end
        x1 = new_node(i, 2);
        y1 = new_node(i, 3);
        x2 = new_node(i+1, 2);
        y2 = new_node(i+1, 3);
        
        % Skips boundary edges
        if ~(abs(x1) < tol && abs(x2) < tol) || ~(abs(y1) < tol && abs(y2) < tol)
            
            for j = i+2:n-1
                x3 = new_node(j, 2);
                y3 = new_node(j, 3);
                x4 = new_node(j+1, 2);
                y4 = new_node(j+1, 3);
                
                % Prevent same node number for comparison
                if new_node(j,1) ~= new_node(i,1) && new_node(j,1) ~= new_node(i+1,1) && ...
                        new_node(j+1,1) ~= new_node(i,1) && new_node(j+1,1) ~= new_node(i+1,1) 
                    
                    % Check with tolerance for equality of coordinates
                    if (abs(x1 - x3) < tol && abs(y1 - y3) < tol && abs(x2 - x4) < tol && abs(y2 - y4) < tol) || ...
                        (abs(x1 - x4) < tol && abs(y1 - y4) < tol && abs(x2 - x3) < tol && abs(y2 - y3) < tol)
                        
                        dy = (y2 - y1);
                        dx = (x2 - x1);
                        
                        angle_radian = atan2(dy,dx);
                        angle_degree = rad2deg(angle_radian);
                        if(abs(angle_degree) < 10^-4)
                            angle_degree = 0;
                        end
                        result = [result; new_node(i,1),new_node(j+1,1), new_node(i+1,1), new_node(j,1),angle_radian, angle_degree];
                    end
                end
            end
        end
    end
end

%% All equilibrium funcitons

function [A_element,B_element]= ElementEquilibrium(total_node_table, no_of_element,gamma)
    row = 2*no_of_element;
    col =  9 * no_of_element;  %9 * no_of_element for linprog 18 for SOCP.
    A_element = sparse(row,col);
    B_element = repmat([0;gamma],no_of_element,1);
    for i = 1:no_of_element
       
        start_row = (i-1) * 2 + 1;
        start_col = (i-1) * 9 + 1;
        
        x1 = total_node_table((i-1)*3 + 1, 1); y1 = total_node_table((i-1)*3 + 1, 2);
        x2 = total_node_table((i-1)*3 + 2, 1); y2 = total_node_table((i-1)*3 + 2, 2);
        x3 = total_node_table((i-1)*3 + 3, 1); y3 = total_node_table((i-1)*3 + 3, 2);
        
        e1 = y2 - y3; e2 = y3 - y1; e3 = y1 - y2;
        z1 = x3 - x2; z2 = x1 - x3; z3 = x2 - x1;
        
        twice_area = abs(e1 * z2 - e2 * z1);
    
        A_individual = (1 / twice_area) * [
            e1, 0, z1, e2, 0, z2, e3, 0, z3;
            0, z1, e1, 0, z2, e2, 0, z3, e3
        ];
        
        A_element(start_row:start_row+1, start_col:start_col+8) = A_individual;
    end
   
end

function [A_bound,B_Boundary] = BoundaryCondition(Df,top_boundary_array,side_boundary_array,tunnel_interface_array,footing_interface_array,no_of_element,draw_tunnel)
    if draw_tunnel
        row = 2*(size(top_boundary_array,1) + height(tunnel_interface_array) ) ;  % + height(footing_interface_array); %+ size(side_boundary_array,1)
    else
        row = 2*(size(top_boundary_array,1)) ;
    end
    col = 9 * no_of_element;  
    A_bound =sparse(row,col);
    

    T_top = [0, 1, 0; 0, 0, 1];   % for top boundary theta=0
    T_emb_side = [1,0,0; 0,0,-1]; % for right boundary theta=90
    T_footing_base = [0,0,1];     % for right boundary theta=0 q1=0
    
    % Footing embedment surface
    left_embedded_array = side_boundary_array(1:height(side_boundary_array)/2,:);
    right_embedded_array = side_boundary_array(height(side_boundary_array)/2 +1: end,:);
    
    row_start = 1;
        
    % b_counter=1;
    % 
    %     %  for left side
    %         for i = 1:size(left_embedded_array, 1)
    %             index = left_embedded_array(i, 1);
    % 
    %             col_start = 3 * (index - 1) + 1;
    %             A_bound(row_start, col_start:col_start+2) = T_emb_side(1,:);
    %             % A_bound(row_start+1, col_start:col_start+2) = T_emb_side(2,:);
    %             B_Boundary_left(b_counter,1)=0; % q1
    %             % B_Boundary_left(b_counter+1,1)=-1; % t1
    %             row_start = row_start + 1;
    %             b_counter=b_counter+1;
    %         end
    % 
    % 
    %      b_counter=1;
    %     % for right side
    %     for i = 1:size(right_embedded_array, 1)
    %         index = right_embedded_array(i, 1);
    % 
    %         col_start = 3 * (index - 1) + 1;
    %         A_bound(row_start, col_start:col_start+2) = T_emb_side(1,:);
    %         % A_bound(row_start+1, col_start:col_start+2) = T_emb_side(2,:);
    %         B_Boundary_right(b_counter,1)=0; % q1
    %         % B_Boundary_right(b_counter+1,1)=1; % t1
    %         row_start = row_start + 1;
    %         b_counter=b_counter + 1;
    %     end
    %     B_Boundary_side = [B_Boundary_left;B_Boundary_right];
    
    % Top surface i.e GL 
    b_counter=1;
    for i = 1:size(top_boundary_array, 1)
        index = top_boundary_array(i, 1);
        col_start = 3 * (index - 1) + 1;
        A_bound(row_start, col_start:col_start+2) = T_top(1,:);
        A_bound(row_start+1, col_start:col_start+2) = T_top(2,:);
        B_Boundary_top(b_counter,1) = 0; % q1
        B_Boundary_top(b_counter+1,1) = 0; % t1
        row_start = row_start + 2;
        b_counter = b_counter+2;
    end 
    

    % Tunnel inteface for bondary
    if draw_tunnel
         b_counter=1;
        for i=1:size(tunnel_interface_array,1)
            index = tunnel_interface_array(i,1);
            col_start = 3*(index-1)+1;
            theta = tunnel_interface_array(i,4);
            T_tunnel = [(sin(theta))^2, (cos(theta))^2, -sin(2*theta);
                -0.5*(sin(2*theta)), 0.5*(sin(2*theta)), cos(2*theta)];
            A_bound(row_start,col_start:col_start+2)=T_tunnel(1,:);
            A_bound(row_start+1, col_start:col_start+2) = T_tunnel(2,:);
            B_Boundary_tunnel(b_counter,1) = 0; %q1
            B_Boundary_tunnel(b_counter+1,1) = 0; %t1
            row_start = row_start + 2;
            b_counter=b_counter+2;
        end
    end
    % footing inteface for objective function
    % b_counter =1;
    % for i = 1:size(footing_interface_array, 1)
    %     index = footing_interface_array(i, 1);
    %     col_start = 3 * (index - 1) + 1;
    %     A_bound(row_start, col_start:col_start+2) = T_footing_base;
    %     B_Boundary_footing_interface(b_counter,1) = 0; % t1
    %     row_start = row_start + 1;
    %     b_counter=b_counter+1;
    % end 

    if draw_tunnel
        B_Boundary=[B_Boundary_top;B_Boundary_tunnel] ;
    else
        B_Boundary=B_Boundary_top ; % B_Boundary_side; ;B_Boundary_footing_interface
    end
    % side; top; footing; tunnel
    
end

function [A_discontinuity,B_discontinuity] = DiscontinuityEquilibrium(result_edges,no_of_element)
    no_of_discont_plane = size(result_edges,1);
    disp("Discontinuity Plane: "+no_of_discont_plane);
    A_discontinuity = sparse(4*no_of_discont_plane, 9 * no_of_element); %4*no_of_discont_plane,3*no_of_element 
    B_discontinuity = sparse(4*no_of_discont_plane,1); %9 * no_of_element for linprog 18 for SOCP.
    for i=1:no_of_discont_plane
        theta_radian = result_edges(i,5);
        T=[sin(theta_radian).^2, cos(theta_radian).^2, -sin(2*theta_radian); -0.5*sin(2*theta_radian), 0.5*sin(2*theta_radian), cos(2*theta_radian)];
        col1_start = (result_edges(i,1)-1)*3+1;
        col2_start = (result_edges(i,2)-1)*3+1;
        col3_start = (result_edges(i,3)-1)*3+1;
        col4_start = (result_edges(i,4)-1)*3+1;

        row = (i-1)*4+1;
        A_discontinuity(row,col1_start:col1_start+2) = T(1,:);
        A_discontinuity(row,col2_start:col2_start+2) = -T(1,:);

        A_discontinuity(row+1,col1_start:col1_start+2) = T(2,:);
        A_discontinuity(row+1,col2_start:col2_start+2) = -T(2,:);

        A_discontinuity(row+2,col3_start:col3_start+2) = T(1,:);
        A_discontinuity(row+2,col4_start:col4_start+2) = -T(1,:);

        A_discontinuity(row+3,col3_start:col3_start+2) = T(2,:);
        A_discontinuity(row+3,col4_start:col4_start+2) = -T(2,:);
    end
end

function [A_Yield, B_yield] = YieldEquilibrium(no_of_element, p, fi,c)
    % Total number of nodes
    fi=(pi/180)*fi;
    total_nodes = no_of_element * 3;
    num_decimal_places = 3;

    
    
   %%%%% SOCP Yield
    A_yield = [sin(fi), sin(fi), 0 ;
       -1, 1, 0;
       0, 0, -2];
    D = [2 *c * cos(fi); 0;0];

    A_Yield = sparse(total_nodes * 3, total_nodes * 3);

    for i = 1:total_nodes
        row_start = (i-1) * 3 + 1;
        row_end = i * 3;
        col_start = (i-1) * 3 + 1;
        col_end = i * 3;
        A_Yield(row_start:row_end, col_start:col_end) = A_yield;
    end
    I_matrix = speye(height(A_Yield));

    A_Yield = [A_Yield,I_matrix];
    B_yield = repmat(D, height(A_Yield)/3 ,1);
    
    
    
end

function C_matrix = ObjectiveFunction(middle_bound_result,no_of_element,side_boundary_array,tunnel_interface_array)

    size_middle = size(middle_bound_result,1);
    C_matrix_footing = sparse(1,9*no_of_element);
    for i=1:2:size_middle
        x1 = middle_bound_result(i,2);
        y1 = middle_bound_result(i,3);
        x2 = middle_bound_result(i+1,2);
        y2 = middle_bound_result(i+1,3);

        Length = sqrt((x1-x2)^2+(y1-y2)^2);
        T = (Length/2)*[0,1,0];
        index_1 = middle_bound_result(i,1);
        index_2 = middle_bound_result(i+1,1);
        col_start_1 = (index_1-1)*3+1;
        col_start_2 = (index_2-1)*3+1;
        C_matrix_footing(1,col_start_1:col_start_1+2)=T;
        C_matrix_footing(1,col_start_2:col_start_2+2)=T;
    end
    % C_matrix = C_matrix_footing;
    % zeros_matrix = sparse(height(C_matrix),width(C_matrix));
    % C_matrix = [C_matrix,zeros_matrix];

    %%%% Tunnel becomes objective function
    % size_tunnel = size(tunnel_interface_array,1);
    % C_matrix_tunnel = sparse(1,9*no_of_element);
    % for i=1:2:size_tunnel
    %     x1 = tunnel_interface_array(i,2);
    %     y1 = tunnel_interface_array(i,3);
    %     x2 = tunnel_interface_array(i+1,2);
    %     y2 = tunnel_interface_array(i+1,3);
    %     theta = tunnel_interface_array(i,4);
    %     Length = sqrt((x1-x2)^2+(y1-y2)^2);
    %     T = (Length/2)*[(sin(theta))^2, (cos(theta))^2, -sin(2*theta)];
    %     index_1 = tunnel_interface_array(i,1);
    %     index_2 = tunnel_interface_array(i+1,1);
    %     col_start_1 = (index_1-1)*3+1;
    %     col_start_2 = (index_2-1)*3+1;
    %     C_matrix_tunnel(1,col_start_1:col_start_1+2)=T;
    %     C_matrix_tunnel(1,col_start_2:col_start_2+2)=T;
    % end
    % C_matrix = C_matrix_tunnel;
    % zeros_matrix = sparse(height(C_matrix),width(C_matrix));
    % C_matrix = [C_matrix,zeros_matrix];
    
    %%%% for embedded footing
    left_embedded_array = side_boundary_array(1:height(side_boundary_array)/2,:);
    right_embedded_array = side_boundary_array(height(side_boundary_array)/2 +1: end,:);

    C_matrix_left_side = sparse(1,9*no_of_element);
    size_left = size(left_embedded_array,1);
    for i=1:2:size_left
        x1 = left_embedded_array(i,2);
        y1 = left_embedded_array(i,3);
        x2 = left_embedded_array(i+1,2);
        y2 = left_embedded_array(i+1,3);

        Length = sqrt((x1-x2)^2+(y1-y2)^2);
        T = (Length)*[0, 0, -1];
        index_1 = left_embedded_array(i,1);
        index_2 = left_embedded_array(i+1,1);
        col_start_1 = (index_1-1)*3+1;
        col_start_2 = (index_2-1)*3+1;
        C_matrix_left_side(1,col_start_1:col_start_1+2)= T;
        C_matrix_left_side(1,col_start_2:col_start_2+2)= T;
    end

    C_matrix_right_side = sparse(1,9*no_of_element);
    size_right = size(right_embedded_array,1);
    theta = 1.5708; %90 degree

    for i=1:2:size_right
        x1 = right_embedded_array(i,2);
        y1 = right_embedded_array(i,3);
        x2 = right_embedded_array(i+1,2);
        y2 = right_embedded_array(i+1,3);

        Length = sqrt((x1-x2)^2+(y1-y2)^2);
        T = (Length)*[0, 0, -1];
        index_1 = right_embedded_array(i,1);
        index_2 = right_embedded_array(i+1,1);
        col_start_1 = (index_1-1)*3+1;
        col_start_2 = (index_2-1)*3+1;
        C_matrix_right_side(1,col_start_1:col_start_1+2)= T;
        C_matrix_right_side(1,col_start_2:col_start_2+2)= T;
    end
    C_matrix = -C_matrix_footing - C_matrix_left_side + C_matrix_right_side;
    zeros_matrix = sparse(height(C_matrix),width(C_matrix));
    C_matrix = [C_matrix,zeros_matrix];
end



