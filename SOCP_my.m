warning('off');
tic;
Tunnel_footing;

coneStart = 9*no_of_element;  % place where slack vars will start 
node = no_of_element * 3;  % Total number of nodes in the mesh
prob.cones = cell(node,1);
for i=1:node
    prob.cones{i}.type = 'MSK_CT_QUAD';
    prob.cones{i}.sub = [coneStart + 3*i-2, coneStart + 3*i-1, coneStart + 3*i];
end

prob.c = C_matrix';
A=[A2;A1];
b=[B2;B1];
prob.a=A;       % All A matrix
prob.blc=b;    % All B matrix
prob.buc=b;   % lower bound 
prob.blx=[];    % upper bound
prob.bux=[];
param.MSK_IPAR_PRESOLVE_LINDEP_USE='MSK_OFF';
param.MSK_IPAR_LOG = 0;
[r,res]=mosekopt('maximize' ,prob,param);
X = res.sol.itr.xx;
fval = res.sol.itr.pobjval; % Total Collapse Load

N_c= fval/(c*B);
N_g = 2*(fval)/(gamma*B*B);
% N_q = -(fval);

fprintf("p = %d, c = %d, fi = %d, Df/B=%d \n",p,c,fi,Df/B);
disp("Collapse Load: "+ fval);
disp("Nc: "+N_c);
disp("Ng: "+N_g);
toc;
