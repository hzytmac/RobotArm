close all;
for i = 1:4
    figure(i);
    plot(time, qnoisy(:, i), '-r', time, qclean(:, i), '-b');
    legend('Noisy', 'Clean');
end

Q_ = var(qnoisy-qclean);
save var_data.mat