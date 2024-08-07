clear all;
clc;
close all;

no_of_interface = 32; %Always be an multiple of 8.
radius = 0.5;

no_of_circles = 2; % No of circle inside the square
distance_of_first_square = 0.7; % This is the distance of first square from centre, will control the size of square 
total_depth = 6;
total_length = 30;
H_D_ratio = 1;
H = H_D_ratio * radius*2;
x_coor_circle_center = 5;  %should be >= total_depth/2
y_coor_circle_center = -(H+radius);

%Input for GP mesh
r=1.1;
r_main_gp = 1.1;
N_of_gp_mesh = 10;


[radial_increment,All_radial_points,square_points] = DrawInterface...
    (radius,no_of_interface,x_coor_circle_center,y_coor_circle_center,no_of_circles,distance_of_first_square,total_depth);

%[All_radial_and_sqaure_points] = ArrangeAllRadial_SquarePoints(All_radial_points,square_points,no_of_interface,no_of_circles);

%[Left_line2_coords,Right_line2_coords,initial_distance_for_gp] = ParentFunctionToMakeMainSquareMesh(r,N_of_gp_mesh,distance_of_first_square,total_depth,H,radius,x_coor_circle_center,...
  %  y_coor_circle_center,no_of_interface,radial_increment);



%% Left_line2_coord is the left square side cooord and right square edge. (MAIN MESH)
% DrawOuterMesh(Left_line2_coords,Right_line2_coords,total_length,total_depth,...
%     initial_distance_for_gp,no_of_interface,r_main_gp);



function [Left_line2_coords,Right_line2_coords,initial_distance_for_gp] = ParentFunctionToMakeMainSquareMesh...
    (r,N_of_gp_mesh,distance_of_first_square,total_depth,H,radius,x_coor_circle_center,y_coor_circle_center,no_of_interface,radial_increment)
    
    
    left_right_distance_of_last_square = total_depth/2;
    %defining the coordinates, Please see the image for reference
    x1 = x_coor_circle_center - distance_of_first_square;  y1 = y_coor_circle_center + distance_of_first_square;
    x2 = x_coor_circle_center + distance_of_first_square;  y2 = y1;
    x3 = x2;                                                         y3 = y_coor_circle_center - distance_of_first_square;
    x4 = x1;                                                         y4 = y3;
    x5 = x_coor_circle_center - left_right_distance_of_last_square;   y5 = 0;
    x6 = x_coor_circle_center + left_right_distance_of_last_square;   y6 = y5;
    x7 = x6;                                                          y7 = -total_depth;
    x8 = x5;                                                          y8 = y7;
    
    %% This are the coordinates of the last square and the first square
    num_divisions = no_of_interface/4;
   [Top_line1_coords, Top_line2_coords] = DivideLinesAndJoinThem(x1, y1, x2, y2, x5, y5, x6, y6, num_divisions); %For top part
   [Left_line1_coords, Left_line2_coords] = DivideLinesAndJoinThem(x1, y1, x4, y4, x5, y5, x8, y8, num_divisions); %For left part
   [Bottom_line1_coords, Bottom_line2_coords] = DivideLinesAndJoinThem(x4, y4, x3, y3, x8, y8, x7, y7, num_divisions) ;%For bottom part
   [Right_line1_coords, Right_line2_coords] = DivideLinesAndJoinThem(x3, y3, x2, y2, x7, y7, x6, y6, num_divisions); %For right part
 

   a=radial_increment*1.5 ;
   n=N_of_gp_mesh; L=-1*y_coor_circle_center - distance_of_first_square;
   initial_value = -L;
   cumulative_distances = GpSeries(a,r,n,L,initial_value); % dividing top portion only, 
   % cumulative_dist will have y-coordinate of the top portion
 
   %% loop will run for no of lines i.e interface/4
  [all_x_coord_top_lines,all_y_coord_top_lines] = PlotfanMeshGpLinesTop(Top_line1_coords,Top_line2_coords,cumulative_distances,no_of_interface);
  cum_distance_for_left = all_x_coord_top_lines(1,:);

  [all_x_coord_left_lines,all_y_coord_left_lines] = PlotfanMeshGpLinesleft(Left_line1_coords,Left_line2_coords,cum_distance_for_left,no_of_interface);
  cum_distance_for_bottom = all_y_coord_left_lines(end,:); 
  initial_distance_for_gp = abs(all_x_coord_left_lines(1,end)-all_x_coord_left_lines(1,end-1)); %This is not needed here, but in another function

  [all_x_coord_bottom_lines,all_y_coord_bottom_lines] = PlotfanMeshGpLinesBottom(Bottom_line1_coords, Bottom_line2_coords,cum_distance_for_bottom,no_of_interface);
  cum_distance_for_right = all_x_coord_bottom_lines(end,:);

  [all_x_coord_right_lines,all_y_coord_right_lines] = PlotfanMeshGpLinesRight(Right_line1_coords, Right_line2_coords,cum_distance_for_right,no_of_interface);
  
  hold on;
  %% So this all_x_coord_lines doesn't have the first series of coordinates i.e the first square coordinates
   all_x_coord_top_lines = round([Top_line1_coords(:,1),all_x_coord_top_lines],3);
   all_y_coord_top_lines = round([Top_line1_coords(:,2),all_y_coord_top_lines],3);
  
   all_x_coord_left_lines = round([Left_line1_coords(:,1),all_x_coord_left_lines],3);
   all_y_coord_left_lines = round([Left_line1_coords(:,2),all_y_coord_left_lines],3);

   all_x_coord_bottom_lines = round([Bottom_line1_coords(:,1),all_x_coord_bottom_lines],3);
   all_y_coord_bottom_lines = round([Bottom_line1_coords(:,2),all_y_coord_bottom_lines],3);

   all_x_coord_right_lines = round([Right_line1_coords(:,1),all_x_coord_right_lines],3);
   all_y_coord_right_lines = round([Right_line1_coords(:,2),all_y_coord_right_lines],3);
    
  %% Here i  called the quadri function to divide them and send the midpoint
  DivideGPinTriangleStoreQuads(all_x_coord_top_lines,all_y_coord_top_lines,all_x_coord_left_lines,all_y_coord_left_lines,...
      all_x_coord_bottom_lines,all_y_coord_bottom_lines,all_x_coord_right_lines,all_y_coord_right_lines);
  
end

function  [radial_increment,All_radial_points,square_points] = DrawInterface(radius,divisions,x_coor_circle_center,...
    y_coor_circle_center,no_of_circles,distance_of_first_square,total_depth)
    
    %drawing the first square
    DrawSquare(distance_of_first_square,x_coor_circle_center,y_coor_circle_center);
    distance_of_last_square = total_depth/2;
    % To draw the outer square but now with reference to the ground level
    DrawSquare(distance_of_last_square,x_coor_circle_center,-distance_of_last_square);
    
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

function DrawOuterMesh(Left_line2_coords,Right_line2_coords,total_length,total_depth,...
    initial_distance_for_gp,no_of_interface,r_main_gp)

    length_each_side  = (total_length - total_depth)/2;
 
    New_Left_Line_Coords = Left_line2_coords;
    New_Left_Line_Coords(:, 1) = New_Left_Line_Coords(:, 1) - length_each_side;
    
    New_Right_Line_Coords = Right_line2_coords;
    New_Right_Line_Coords(:, 1) = New_Right_Line_Coords(:, 1) + length_each_side;
    
    plot([Left_line2_coords(:, 1)'; New_Left_Line_Coords(:, 1)'],[Left_line2_coords(:, 2)';New_Left_Line_Coords(:, 2)'], 'k-');
    plot([Right_line2_coords(:,1)'; New_Right_Line_Coords(:, 1)'], [Right_line2_coords(:,2)';New_Right_Line_Coords(:, 2)'], 'k-');
    hold on;
    
    % Here to plot the vertical GP mesh
    
    a=initial_distance_for_gp * 1.05; r=r_main_gp; L=length_each_side; 
    
    initial_value_right = Right_line2_coords(1,1);
    initial_value_left = Left_line2_coords(1,1);
  
    cumulative_distances_right = GpSeriesModified(a,r,L,initial_value_right);
    cumulative_distances_left = GpSeriesModified(a,r,L,initial_value_left);
    cumulative_distances_left = 2*initial_value_left - cumulative_distances_left;
    [all_x_coord_right_main_lines, all_y_coord_right_main_lines] = PlotfanMeshGpLinesRight(Right_line2_coords, New_Right_Line_Coords, cumulative_distances_right, no_of_interface);
    [all_x_coord_left_main_lines, all_y_coord_left_main_lines] = PlotfanMeshGpLinesleft(Left_line2_coords, New_Left_Line_Coords, cumulative_distances_left, no_of_interface);
    %% again they dont have first set of x and y coordinates, so need to append them
    
    all_x_coord_left_main_lines = round([Left_line2_coords(:,1),all_x_coord_left_main_lines],3);
    all_y_coord_left_main_lines = round([Left_line2_coords(:,2),all_y_coord_left_main_lines],3);
    all_x_coord_right_main_lines = round([Right_line2_coords(:,1),all_x_coord_right_main_lines],3);
    all_y_coord_right_main_lines = round([Right_line2_coords(:,2), all_y_coord_right_main_lines],3);
    
    [x_midpoints_left_main_lines,y_midpoints_left_main_lines] = DivideMainMeshGPinTRiangStoreCoord(all_x_coord_left_main_lines,all_y_coord_left_main_lines);
    [x_midpoints_right_main_lines,y_midpoints_right_main_lines] = DivideMainMeshGPinTRiangStoreCoord(all_x_coord_right_main_lines,all_y_coord_right_main_lines);
end

%% Storing all radial and sqaure points so that i join the cross lines and also returns the midpoint coordinates.
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

%% function to calculate the division on the square.
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


%% function where i will give four coord and no of divisions, it will divide the two lines and join them

function [line1_coords, line2_coords] = DivideLinesAndJoinThem(x1, y1, x2, y2, x3, y3, x4, y4, num_divisions)

    line1_x = linspace(x1, x2, num_divisions + 1);
    line1_y = linspace(y1, y2, num_divisions + 1);
    
    line2_x = linspace(x3, x4, num_divisions + 1);
    line2_y = linspace(y3, y4, num_divisions + 1);
    
    line1_coords = round([line1_x', line1_y'],3);
    line2_coords = round([line2_x', line2_y'],3);
    
   
    
    for i = 1:(num_divisions + 1)
        plot([line1_x(i), line2_x(i)], [line1_y(i), line2_y(i)], 'k-');
    end
    hold on;
end


function cumulative_distances = GpSeriesModified(a, r, L, initial_value)
    cumulative_distances = [];
    current_sum = a;
    i=1;
    while(current_sum < L)
        if r ~= 1
            segment_length = a * r^(i-1);
        else
            segment_length = a;
        end
        i=i+1;
        cumulative_distances = [cumulative_distances, initial_value + current_sum];
        current_sum = current_sum + segment_length;
    end
end

function cumulative_distances = GpSeries(a,r,n,L,initial_value)

    if r == 1
        S_n = a * n;
    else
        S_n = a * (1 - r^n) / (1 - r);
    end
    scaling_factor = L / S_n;
    segment_lengths = zeros(1, n);
    for i = 1:n
        segment_lengths(i) = a * r^(i-1) * scaling_factor;
    end
    cumulative_distances = zeros(1, n);
    cumulative_distances(1) = initial_value + segment_lengths(1);
    for i = 2:n
        cumulative_distances(i) = cumulative_distances(i-1) + segment_lengths(i);
    end

end

%% This are four function to draw the GP series mesh.

function [all_x_coord_top_lines,all_y_coord_top_lines] =  PlotfanMeshGpLinesTop(Top_line1_coords,Top_line2_coords,cumulative_distances,no_of_interface)
    
    num_lines = (no_of_interface / 4) + 1;
    all_x_coord_top_lines = zeros(num_lines,size(cumulative_distances,2));
    
    for j=1:num_lines
        x1 = Top_line1_coords(j,1);
        y1 = Top_line1_coords(j,2);
        x2 = Top_line2_coords(j,1);
        y2 = Top_line2_coords(j,2);

        m = (y2 - y1) / (x2 - x1);
        
        c = y1 - m * x1;
        if isinf(m)
            x_coord = x1 * ones(size(cumulative_distances));
        else
            x_coord = (cumulative_distances - c) / m;
        end
        
       all_x_coord_top_lines(j,:) = x_coord;
       
    end
   all_x_coord_top_lines = round(all_x_coord_top_lines,3);
   all_y_coord_top_lines = repmat(cumulative_distances,size(all_x_coord_top_lines,1),1);
   all_y_coord_top_lines = round(all_y_coord_top_lines,3);
   hold on;
   for i=1:size(all_x_coord_top_lines,2)
       plot(all_x_coord_top_lines(:,i),all_y_coord_top_lines(:,i),'k-');
   end
   hold off;
end

function [all_x_coord_left_lines, all_y_coord_left_lines] = PlotfanMeshGpLinesleft(Left_line1_coords, Left_line2_coords, cum_distance_for_left, no_of_interface,isHorizontal)
    num_lines = (no_of_interface / 4) + 1;
    all_y_coord_left_lines = zeros(num_lines, length(cum_distance_for_left));
    
    for j = 1:num_lines
        x1 = Left_line1_coords(j, 1);
        y1 = Left_line1_coords(j, 2);
        x2 = Left_line2_coords(j, 1);
        y2 = Left_line2_coords(j, 2);
        
        m = (y2 - y1) / (x2 - x1);
        c = y1 - m * x1;
        
        y_coord = m * cum_distance_for_left + c;
        
        all_y_coord_left_lines(j, :) = y_coord;
    end
  
    all_y_coord_left_lines = round(all_y_coord_left_lines,3);
    all_x_coord_left_lines = repmat(cum_distance_for_left, num_lines, 1);
    all_x_coord_left_lines = round(all_x_coord_left_lines,3);
   
    hold on; 
    for i = 1:size(all_y_coord_left_lines, 2)
        plot(all_x_coord_left_lines(:, i), all_y_coord_left_lines(:, i), 'k-');
    end
    hold off;
end

function [all_x_coord_bottom_lines, all_y_coord_bottom_lines] = PlotfanMeshGpLinesBottom(Bottom_line1_coords, Bottom_line2_coords, cum_distance_for_bottom, no_of_interface)
    % Initialize the output matrices
    num_lines = (no_of_interface / 4) + 1;
    %all_y_coord_bottom_lines = zeros(num_lines, length(cum_distance_for_bottom));
    all_x_coord_bottom_lines = zeros(num_lines, length(cum_distance_for_bottom));
    % Calculate y coordinates for each line
    for j = 1:num_lines
        x1 = Bottom_line1_coords(j, 1);
        y1 = Bottom_line1_coords(j, 2);
        x2 = Bottom_line2_coords(j, 1);
        y2 = Bottom_line2_coords(j, 2);
   
        m = (y2 - y1) / (x2 - x1);
        c = y1 - m * x1;
            if isinf(m)
                x_coord = x1 * ones(size(cum_distance_for_bottom));  
            else
                x_coord = (cum_distance_for_bottom - c) / m;
            end  
           all_x_coord_bottom_lines(j, :) = x_coord;
    end
    all_x_coord_bottom_lines = round(all_x_coord_bottom_lines,3);
    
    all_y_coord_bottom_lines = repmat(cum_distance_for_bottom, num_lines, 1);
    all_y_coord_bottom_lines = round(all_y_coord_bottom_lines,3);
    hold on; 
    for i = 1:size(all_y_coord_bottom_lines, 2)
        plot(all_x_coord_bottom_lines(:, i), all_y_coord_bottom_lines(:, i), 'k-');
    end
    
end

function [all_x_coord_right_lines, all_y_coord_right_lines] = PlotfanMeshGpLinesRight(Right_line1_coords, Right_line2_coords, cum_distance_for_right, no_of_interface)
    
    num_lines = (no_of_interface / 4) + 1;
    all_y_coord_right_lines = zeros(num_lines, length(cum_distance_for_right));
    
    for j = 1:num_lines
        x1 = Right_line1_coords(j, 1);
        y1 = Right_line1_coords(j, 2);
        x2 = Right_line2_coords(j, 1);
        y2 = Right_line2_coords(j, 2);
        
        m = (y2 - y1) / (x2 - x1);
        c = y1 - m * x1;
        
        y_coord = m * cum_distance_for_right + c;
        
        all_y_coord_right_lines(j, :) = y_coord;
    end

    all_x_coord_right_lines = repmat(cum_distance_for_right, num_lines, 1);
    all_x_coord_right_lines = round(all_x_coord_right_lines,3);
    all_y_coord_right_lines = round(all_y_coord_right_lines,3);
    hold on; 
    for i = 1:size(all_y_coord_right_lines, 2)
        plot(all_x_coord_right_lines(:, i), all_y_coord_right_lines(:, i), 'k-');
    end
    hold off;
end

%% This function is for dividing the fan gp mesh and main gp mesh into triangles and storing the coordinates
function DivideGPinTriangleStoreQuads(all_x_coord_top_lines,all_y_coord_top_lines,all_x_coord_left_lines,all_y_coord_left_lines,...
      all_x_coord_bottom_lines,all_y_coord_bottom_lines,all_x_coord_right_lines,all_y_coord_right_lines)
    for k=1:4
      switch k
          case 1
              x_coord = all_x_coord_top_lines;
              y_coord = all_y_coord_top_lines;
          case 2
              x_coord = all_x_coord_left_lines;
              y_coord = all_y_coord_left_lines;
          case 3
              x_coord = all_x_coord_bottom_lines;
              y_coord = all_y_coord_bottom_lines;
          case 4
              x_coord = all_x_coord_right_lines;
              y_coord = all_y_coord_right_lines;
      end
      num_rows = size(x_coord,1);
      num_cols = size(x_coord,2);

        for col = 1:num_cols-1
             x_midpoints = zeros(1,1);
             y_midpoints = zeros(1,1);
            for row = 1:num_rows-1
               
                x1 = x_coord(row, col);
                y1 = y_coord(row, col);
                x2 = x_coord(row, col+1);
                y2 = y_coord(row, col+1);
                x3 = x_coord(row+1, col);
                y3 = y_coord(row+1, col);
                x4 = x_coord(row+1, col+1);
                y4 = y_coord(row+1, col+1);
                
                [x_coord_midpoint,y_coord_midpoint] = Divide_Quadri (x1,x3,x2,x4,y1,y3,y2,y4);
                x_midpoints = [x_midpoints; x_coord_midpoint];
                y_midpoints = [y_midpoints; y_coord_midpoint];
            end
        end
        switch k
            case 1
                x_midpoints_top = x_midpoints;
                y_midpoints_top = y_midpoints;
            case 2
                x_midpoints_left = x_midpoints;
                y_midpoints_left = y_midpoints;
            case 3
                x_midpoints_bottom = x_midpoints;
                y_midpoints_bottom = y_midpoints;
            case 4
                x_midpoints_right = x_midpoints;
                y_midpoints_right = y_midpoints;
        end
    end
end

function [x_midpoints,y_midpoints] = DivideMainMeshGPinTRiangStoreCoord(x_coord,y_coord)
      num_rows = size(x_coord,1);
      num_cols = size(x_coord,2);
        hold on;
         num_midpoints = (num_rows - 1) * (num_cols - 1);
       
        for col = 1:num_cols-1
             x_midpoints=[];
             y_midpoints = [];
            for row = 1:num_rows-1
               
                x1 = x_coord(row, col);
                y1 = y_coord(row, col);
                x2 = x_coord(row, col+1);
                y2 = y_coord(row, col+1);
                x3 = x_coord(row+1, col);
                y3 = y_coord(row+1, col);
                x4 = x_coord(row+1, col+1);
                y4 = y_coord(row+1, col+1);
                
                [x_coord_midpoint,y_coord_midpoint] = Divide_Quadri (x1,x3,x2,x4,y1,y3,y2,y4);
                hold on;
                x_midpoints = [x_midpoints; x_coord_midpoint];
                y_midpoints = [y_midpoints; y_coord_midpoint];
            end
        end   
end

