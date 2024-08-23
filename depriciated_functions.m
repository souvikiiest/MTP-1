%ActiveZoneMesh(spacing,Df_fan_mesh,B,L,Df,L_bound,L_fan_mesh_each_side,D_bound);
 %PassiveZoneMesh(spacing,Df_fan_mesh,B,L,Df,L_fan_mesh_each_side);
 %RadialShearMesh(spacing,Df_fan_mesh,B,L,Df,L_fan_mesh_each_side);

function ActiveZoneMesh(spacing,Df_fan_mesh,B,L,Df,L_bound,L_fan_mesh_each_side,D_bound)
    middle_st_line_y_coord = -(Df:spacing:(Df+Df_fan_mesh));
    middle_st_line_x_coord = (L+L_fan_mesh_each_side)*ones(1,size(middle_st_line_y_coord,2));
    footing_right_edge_x_coord = (L+B)*ones(1,size(middle_st_line_y_coord,2));
    footing_right_edge_y_coord = -(Df)*ones(1,size(middle_st_line_y_coord,2));
    footing_left_edge_x_coord = (L)*ones(1,size(middle_st_line_y_coord,2));
    footing_left_edge_y_coord = footing_right_edge_y_coord;
    hold on;

    plot([0,L,L,(L+B),(L+B),L_bound,L_bound,0,0],[0,0,-Df,-Df, 0, 0,-D_bound,-D_bound,0],'k-'); %Plots the boundary 
    %plot([0,L,(L+B),L_bound],[-Df,-Df,-Df,-Df],'k-');   %Plots the embedment level
    plot([L+L_fan_mesh_each_side,L+L_fan_mesh_each_side],[-Df,-(Df+Df_fan_mesh)],'k-');   %Plots the middle active zone st line
    plot([footing_right_edge_x_coord;middle_st_line_x_coord],[footing_right_edge_y_coord;middle_st_line_y_coord],'k-');
    plot([footing_left_edge_x_coord;middle_st_line_x_coord],[footing_left_edge_y_coord;middle_st_line_y_coord],'k-');
end

function PassiveZoneMesh(spacing,Df_fan_mesh,B,L,Df,L_fan_mesh_each_side)
    plot([L+B+L_fan_mesh_each_side,L+B+L_fan_mesh_each_side,L-L_fan_mesh_each_side,L-L_fan_mesh_each_side],[-Df,-(Df+Df_fan_mesh),-(Df+Df_fan_mesh),-Df],'k-');
    right_vert_bound_y_coord = -(Df:spacing:(Df+Df_fan_mesh));
    right_vert_bound_x_coord = (L+B+L_fan_mesh_each_side)*ones(1,size(right_vert_bound_y_coord,2));
    footing_right_edge_x_coord = (L+B)*ones(1,size(right_vert_bound_y_coord,2));
    footing_right_edge_y_coord = -(Df)*ones(1,size(right_vert_bound_y_coord,2));
    left_vert_bound_y_coord = right_vert_bound_y_coord;
    left_vert_bound_x_coord = (L-L_fan_mesh_each_side)*ones(1,size(left_vert_bound_y_coord,2));
    footing_left_edge_x_coord = (L)*ones(1,size(right_vert_bound_y_coord,2));
    footing_left_edge_y_coord = footing_right_edge_y_coord;

    plot([footing_right_edge_x_coord;right_vert_bound_x_coord],[footing_right_edge_y_coord;right_vert_bound_y_coord],'k-');
    plot([footing_left_edge_x_coord;left_vert_bound_x_coord],[footing_left_edge_y_coord;left_vert_bound_y_coord],'k-');
end

function RadialShearMesh(spacing,Df_fan_mesh,B,L,Df,L_fan_mesh_each_side)
 fan_bottom_right_x_coord = L + B / 2:spacing: L + B + L_fan_mesh_each_side;
    fan_bottom_left_x_coord = L - L_fan_mesh_each_side:spacing: L + B / 2;

    fan_bottom_right_y_coord = -(Df + Df_fan_mesh) * ones(size(fan_bottom_right_x_coord));
    fan_bottom_left_y_coord = fan_bottom_right_y_coord;

    footing_right_edge_x_coord = (L + B) * ones(size(fan_bottom_right_x_coord));
    footing_left_edge_x_coord = L * ones(size(fan_bottom_left_x_coord));

    footing_right_edge_y_coord = -Df * ones(size(fan_bottom_right_x_coord));
    footing_left_edge_y_coord = footing_right_edge_y_coord;

    plot([footing_left_edge_x_coord; fan_bottom_left_x_coord],...
        [footing_left_edge_y_coord; fan_bottom_left_y_coord], 'k-');
    plot([footing_right_edge_x_coord; fan_bottom_right_x_coord],...
        [footing_right_edge_y_coord; fan_bottom_right_y_coord],'k-');
end