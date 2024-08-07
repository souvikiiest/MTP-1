clc;
clear all;
close all;

%% Footing Inputs
B=1;    % width of footing

Df=2;   % Depth of footing
Df_fan_mesh = B/2;  % height of fan mesh
L_bound = 20;   %Length of boundary
D_bound = 10;   % Depth of boundary
L=L_bound/2 - B/2;    % left edge distance from origin
L_fan_mesh_each_side = B/2;
N_foot_soil_interf = 6; % for half footing width
%% Control Parameters
b_by_B = 4;
d_by_Df = 2;



%% Tunnel Inputs
radius = 0.5;
no_of_interface = 24;
x_coor_circle_center = (L+B/2 + b_by_B * B);
y_coor_circle_center = -(Df + d_by_Df * Df);
no_of_circles = 2;
distance_of_first_square = radius*1.5;




spacing = (8*distance_of_first_square)/(no_of_interface); % spacing of the interfaces

%% Bunch of function calls
 
drawRectangularMesh(N_foot_soil_interf,Df_fan_mesh,B,Df,L_fan_mesh_each_side,L,spacing);

%% spacing and division of net mesh
vertical_division_right = 0:spacing:L_bound;  % actually total vertical spacing
horizontal_division = -(0:spacing:(D_bound));

%% functions to draw the net mesh without tunnel region
footing_bound_coord = [L-L_fan_mesh_each_side,-(Df+Df_fan_mesh);(L+B+L_fan_mesh_each_side),-(Df+Df_fan_mesh);(L+B+L_fan_mesh_each_side),0; (L-L_fan_mesh_each_side),0];

center_coords=[x_coor_circle_center,y_coor_circle_center];
[adjusted_square_coords,intersections] = drawEmbeddedMeshWithoutTunnel(vertical_division_right, horizontal_division, D_bound, distance_of_first_square, center_coords,footing_bound_coord);
new_x_coord = adjusted_square_coords(1,1)+distance_of_first_square; % got the new_x_coord after swapping to nearest x-values
new_y_coord = adjusted_square_coords(1,2)+distance_of_first_square;

%% this is for tunnel, but here because new x and  y coord are given as input
[radial_increment,All_radial_points,square_points] = DrawInterface(radius,no_of_interface,new_x_coord,new_y_coord,no_of_circles,distance_of_first_square);
[All_radial_and_sqaure_points] = ArrangeAllRadial_SquarePoints(All_radial_points,square_points,no_of_interface,no_of_circles);

result = processMesh(intersections);

footing_x=[L,L,L+B,L+B];
footing_y = [-Df+0.2,-Df,-Df,-Df+0.2];
fill(footing_x,footing_y,'cyan');


% this draws both rectangular as well as the radial mesh
function drawRectangularMesh(N, Df_fan_mesh, B, Df, L_fan_mesh_each_side, L, spacing)
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
    plotAndStoreIntersectionsInverticalEmbedment(Df,spacing,footing_base_left_left);

    footing_base_right_right = footing_base_left_left + B + L_fan_mesh_each_side;
    hold on;
    %For right half of footing base
    vertical_line_x = L + B/2; %about this line mirroring will happen
    mirrorCoordinates(x_coords_active, y_coords_active, x_coords_radial, y_coords_radial, x_coords_passive,...
        y_coords_passive, vertical_line_x,footing_base_left,footing_base_right_right,Df,spacing);
end

%% Tunnel drawing
function  [radial_increment,All_radial_points,square_points] = DrawInterface(radius,divisions,x_coor_circle_center,...
    y_coor_circle_center,no_of_circles,distance_of_first_square)
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
        plot(temp_radial_points(:, 1), temp_radial_points(:, 2), 'b-');
        hold on;
    end
    % to join the first and last circle by straight line
    first_division_points = All_radial_points(1:divisions, :);
    last_division_start = no_of_circles * divisions + 1;
    last_division_points = All_radial_points(last_division_start:last_division_start + divisions - 1, :);
    for i = 1:divisions
        plot([first_division_points(i, 1), last_division_points(i, 1)], ...
            [first_division_points(i, 2), last_division_points(i, 2)], 'k-');
    end
    hold on;
    square_points = DivideSquarePerimeter(x_coor_circle_center, y_coor_circle_center, distance_of_first_square, divisions);
    % Now to join the last divisions coordiantes with the square divisions.
    Last_circle_coords = All_radial_points(end-divisions+1:end,:);
    JoinSqaureAndLastCircle(Last_circle_coords,square_points,divisions);

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

function JoinSqaureAndLastCircle(Last_circle_coords,square_points,divisions)
for i=1:divisions
    square_point = square_points(i,:);
    circle_point = Last_circle_coords(i,:);
    plot([square_point(1),circle_point(1)],[square_point(2),circle_point(2)],'k-');
end

end

function [All_radial_and_sqaure_points] = ArrangeAllRadial_SquarePoints(All_radial_points,square_points,divisions,circles)


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
        [x_coord_midpoint,y_coord_midpoint] = Divide_Quadri(x1,x2,x3,x4,y1,y2,y3,y4);
        Midpoints_radial_square_x(i,j) = x_coord_midpoint;
        Midpoints_radial_square_y(i,j) = y_coord_midpoint;
    end
end

end

%% Embedded Mesh drawing without tunnel and footing

function [adjusted_square_coords, intersections] = drawEmbeddedMeshWithoutTunnel(vertical_division_right, horizontal_division, D_bound, half_square_side_length, center_coords, footing_bound_coord)
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
        if (x <= x_min || x >= x_max) && (x <= x_min_f || x >= x_max_f)
            
            plot([x; x], [0; -D_bound], 'k-');
            intersections = [intersections; repmat(x, length(horizontal_division), 1), horizontal_division'];
        else
            % Vertical lines up to the tunnel and after the tunnel
            if x >= x_min && x <= x_max 
                if x >= x_min_f && x <= x_max_f
                    plot([x;x],[y_min_f,y_max],'k-');
                    
                else
                    plot([x; x], [0; y_max], 'k-');
                    plot([x; x], [y_min; -D_bound], 'k-');
                end
                
            end
            % Vertical lines up to the footing and after the footing
            if x >= x_min_f && x <= x_max_f
                if x >= x_min && x <= x_max 
                    plot([x;x],[y_min_f,y_max],'k-');
                    plot([x;x],[y_min,-D_bound],'k-');
                else
                    plot([x; x], [y_min_f; -D_bound], 'k-');
                end
                
            end
            % Store intersection points for the regions
            if x >= x_min && x <= x_max
                intersections = [intersections; repmat(x, length(horizontal_division(horizontal_division <= y_min | horizontal_division >= y_max)), 1), horizontal_division(horizontal_division <= y_min | horizontal_division >= y_max)'];
            elseif x >= x_min_f && x <= x_max_f
                intersections = [intersections; repmat(x, length(horizontal_division(horizontal_division <= y_min_f | horizontal_division >= y_max_f)), 1), horizontal_division(horizontal_division <= y_min_f | horizontal_division >= y_max_f)'];
            end
        end
    end
    
    % Plotting horizontal lines, omitting tunnel and footing regions
    for i = 1:length(horizontal_division)
        y = horizontal_division(i);
        if (y < y_min || y > y_max) && (y < y_min_f || y > y_max_f)
            plot([min(vertical_division_right), max(vertical_division_right)], [y, y], 'k-');
        else
            if y >= y_min && y <= y_max % checks if y is within tunnel area
                if y >= y_min_f && y <= y_max_f  % checks if y is within footing area
                    
                    plot([x_max_f,x_min],[y,y],'k-');
                    plot([x_max,max(vertical_division_right)],[y,y],'k-');
                else
                    plot([min(vertical_division_right),x_min],[y,y],'k-');
                    plot([x_max,max(vertical_division_right)],[y,y],'k-');
                end
            end
            if y >= y_min_f && y <= y_max_f %checks if y is within footing area
                % to plot the horizontal lines on the left side of footing
                plot([min(vertical_division_right),x_min_f],[y,y],'k-');
                if  y >= y_max || y <= y_min % checks if the y is above tunnel but within footing area
                    plot([x_max_f,max(vertical_division_right)],[y,y],'k-');
                end
                
            end
            
        end
    end
    
    hold off;
end

function mirrorCoordinates(x_coords_active, y_coords_active, x_coords_radial, y_coords_radial, x_coords_passive,y_coords_passive, vertical_line_x,footing_base_left,footing_base_right_right,Df,spacing)
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
    plotAndStoreIntersectionsInverticalEmbedment( Df, spacing,footing_base_right_right);
end

function [x_intersections, y_intersections] = plotAndStoreIntersectionsInverticalEmbedment( Df, spacing,footing_base_left_left)
    % Define the x-coordinates for the vertical lines
    %footing_base_left_left = linspace(L - L_fan_mesh_each_side, L, N + 1);
    
    % Define the y-coordinates for the horizontal lines
    horizontal_lines_y = -(Df:-spacing:0);

    % Initialize arrays to store the intersection coordinates
    x_intersections = zeros(length(horizontal_lines_y), length(footing_base_left_left));
    y_intersections = zeros(length(horizontal_lines_y), length(footing_base_left_left));
    
    
    % Plot the horizontal lines and store intersections
    for j = 1:length(horizontal_lines_y)
        plot([footing_base_left_left(1), footing_base_left_left(end)], [horizontal_lines_y(j), horizontal_lines_y(j)], 'k-');
        % Store intersection coordinates
        x_intersections(j, :) = footing_base_left_left;
        y_intersections(j, :) = horizontal_lines_y(j);
    end
    
     hold on;
end


%% store coordinated in arranged way
function result = processMesh(intersection)
    % Find unique x-coordinates
    uniqueX = unique(intersection(:, 1)); 

    % Find unique y-coordinates
    uniqueY = unique(intersection(:, 2));
    
    % Initialize output array
    outputArray = NaN(length(uniqueY), 2*length(uniqueX));
    outputArray(:, 1) = uniqueX(1);
    outputArray(:, 2) = uniqueY;
    
    % Populate the array
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
  for j = 1:2:size(outputArray, 2) - 3
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
                Divide_Quadri(x1, x2, x3, x4, y1, y2, y3, y4);
            end
        end
   end



end


%% divide quadrilateral into triangles
function [x_coord_midpoint,y_coord_midpoint] = Divide_Quadri (x1,x2,x3,x4,y1,y2,y3,y4)
    plot([x1;x4], [y1;y4], 'k-');
    plot([x2;x3], [y2;y3], 'k-');
    hold on;
    %To find intersecting point coordinates
    X1=[x1,x4];
    X2 = [x2,x3];
    Y1=[y1,y4];
    Y2=[y2,y3];
    
    eqn1 = polyfit(X1,Y1,1);
    eqn2 = polyfit(X2,Y2,1);
    
    m1=eqn1(1);m2=eqn2(1);
    c1=eqn1(2);c2=eqn2(2);
    
    x_coord_midpoint = (c1-c2)/(m2-m1);
    y_coord_midpoint = m2*x_coord_midpoint + c2;

end

