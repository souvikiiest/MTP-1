tic;
Tunnel_footing;
coneStart = 9*no_of_element; % place where slack vars will start
node = no_of_element * 3; % Total number of nodes in the mesh

% Create the Gurobi model
model = struct();
model.modelsense = 'max';

% Set up the variables
A=[A2;A1];
b=[B2;B1];
model.obj = full(C_matrix(:));
model.A = sparse(A);
model.rhs = full(b);
model.sense = repmat('=', length(b), 1);

% Add second-order cone constraints
for i = 1:node
    t_idx = coneStart + 3*i - 2;
    y1_idx = coneStart + 3*i - 1;
    y2_idx = coneStart + 3*i;
    
    model.quadcon(i).Qc = sparse(length(C_matrix), length(C_matrix));
    model.quadcon(i).Qc(y1_idx, y1_idx) = 1;
    model.quadcon(i).Qc(y2_idx, y2_idx) = 1;
    model.quadcon(i).q = zeros(length(C_matrix), 1);
    model.quadcon(i).q(t_idx) = -1;
    model.quadcon(i).rhs = 0;
    model.quadcon(i).sense = '<';
end

% Set parameters
params.OutputFlag = 0;  

% Optimize the model
result = gurobi(model, params);

% Extract results
X = result.x;
fval = result.objval;

% Calculate derived values
N_c = fval/(c*B);
N_g = 2*(fval)/(gamma*B*B);

% Display results
fprintf("p = %d, c = %d, fi = %d, Df/B=%d \n", p, c, fi, Df/B);
disp("Collapse Load: " + fval);
disp("Nc: " + N_c);
disp("Ng: " + N_g);
toc;