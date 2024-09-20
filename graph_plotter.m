
[num, txt, raw] = xlsread('results.xlsx');

bB = num(:, 1);
dB = num(:, 2);
fi = num(:, 3);
Nc = num(:, 4);


unique_bB = unique(bB);


for i = 1:length(unique_bB)
    current_bB = unique_bB(i);

    mask = bB == current_bB;
    current_dB = dB(mask);
    current_fi = fi(mask);
    current_Nc = Nc(mask);

    figure;
   
    unique_fi = unique(current_fi);

    hold on;
    for j = 1:length(unique_fi)
        current_fi_value = unique_fi(j);
        fi_mask = current_fi == current_fi_value;
        
        plot(current_dB(fi_mask), current_Nc(fi_mask), 'o-', 'DisplayName', ['fi = ' num2str(current_fi_value)]);
    end
    hold off;

    title(['Nc vs d/B for b/B = ' num2str(current_bB)]);
    xlabel('d/B');
    ylabel('Nc');
    legend('show');
    grid on;
    
    % saveas(gcf, ['chart_bB_' num2str(current_bB) '.png']);
end