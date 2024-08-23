node = Node_num; % Total number of nodes in the mesh
sub=zeros(1,node); % node = no of nodes 
pii=zeros(1,3*node); 
for in=1:node
    sub(1,in)=3*in-2;
    pii(1,3*in-2)=3*node+3*in;
    pii(1,3*in-1)=3*node+3*in-1;
    pii(1,3*in)=3*node+3*in-2;
end
% MOSEK APPLICATION
[r,res]=mosekopt('symbcon');
prob.c=-A_objective;
prob.a=A;   %[Af;Aint]; total A matrix sob gulo thakbe
prob.blc=b;  %[Bf;-Inf*ones(length(bint),1)]; sob b matrix
prob.buc=b; %[Bf;bint]; 
prob.blx=[];
prob.bux=[];
prob.cones.type=repmat(res.symbcon.MSK_CT_QUAD,1,node);
prob.cones.sub=pii;
prob.cones.subptr=sub;
param.MSK_IPAR_PRESOLVE_LINDEP_USE='MSK_OFF';
[r,res]=mosekopt('maximize',prob,param);
X= res.sol.itr.xx';
fval=res.sol.itr.pobjval; % Total Collapse Load
fval/(footing_width)/(gamma)/0.5/(footing_width); % N gamma value
Q0=fval/(footing_width); % Ultimate Bearing capacity