
fi_values = 20:10:40;

N_g_results = zeros(size(fi_values));


for i = 1:length(fi_values)
    
    phiii = fi_values(i); 
    assignin('base', 'phiii', phiii);
    % run('Tunnel_footing.m');
    run('SOCP_my.m');
    % run('failurepattern.m');
    N_g_results(i) = N_g;  

    fprintf('Completed run for fi = %d, N_g = %f\n', fi, N_g);
end

