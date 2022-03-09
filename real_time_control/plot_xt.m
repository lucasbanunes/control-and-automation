function plot_xt(out, save_file)
    subplot(2,2,1);
    grid;
    hold on;
    plot(out.xt.Time, out.xt.Data(:, (1:3)));
    title('Position');
    legend('p_1', 'p_2', 'p_3');
    hold off;
    subplot(2,2,2);
    grid;
    hold on;
    plot(out.xt.Time, out.xt.Data(:, (4:6)));
    title('Velocity');
    legend('v_1', 'v_2', 'v_3');
    xlabel('Time(seconds)');
    hold off;
    subplot(2,2,3);
    grid;
    hold on;
    plot(out.xt.Time, out.xt.Data(:, (7:9)));
    title('$a_b$', 'interpreter', 'latex');
    legend('$a_{b_1}$', '$a_{b_2}$', '$a_{b_3}$', 'interpreter', 'latex');
    xlabel('Time(seconds)');
    hold off;
    suptitle('xt');
    if save_file
        saveas(gcf,'imgs/xt_plot.png');
    end
end