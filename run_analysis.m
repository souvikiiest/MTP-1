
b_by_B_values = [0,1];
d_B_values = [2,3];  
fi_values = 10:10:40;  


results = {'b/B', 'd/B', 'fi', 'Nc'};  % columns 


for b_idx = 1:length(b_by_B_values)
    b_by_B = b_by_B_values(b_idx);
    

    for row_j = 1:length(d_B_values)
        d_by_B = d_B_values(row_j); 


        for row_i = 1:length(fi_values)
            phiii = fi_values(row_i); 
            assignin('base', 'phiii', phiii);  

            if row_i == 1 
                isTunnelChanging_dash = true;
            else
                isTunnelChanging_dash = false;
            end
            
            assignin('base', 'isTunnelChanging_dash', isTunnelChanging_dash);  
            
            run('SOCP_my.m'); 
            run('failurepattern.m');

            results{end+1, 1} = b_by_B;
            results{end, 2} = d_by_B;
            results{end, 3} = phiii;
            results{end, 4} = N_c;  

            fprintf('Completed run for: b/B=%.1f, d/B = %.1f, Φ = %d, Nc = %f\n', b_by_B, d_by_B, phiii, N_c);
        end
    end
end


writecell(results, 'results.xlsx');

disp('Results saved to results.xlsx');