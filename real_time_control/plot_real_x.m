function plot_real_x(out, save_file)
    figure;
    subplot(2,2,1);
    plot(out.deltax.Time, out.xt.Data(:, (1:3))-out.deltax.Data(:, (1:3)));
    title('Position');
    legend('$p_1$', '$p_2$', '$p_3$', 'interpreter', 'latex');
    grid;
    subplot(2,2,2);
    plot(out.deltax.Time, out.xt.Data(:, (4:6))-out.deltax.Data(:, (4:6)));
    title('Velocity');
    legend('$v_1$', '$\delta v_2$', '$v_3$', 'interpreter', 'latex');
    xlabel('Time(seconds)');
    grid;
    subplot(2,2,3);
    plot(out.deltax.Time, out.xt.Data(:, (7:9))-out.deltax.Data(:, (7:9)));
    title('$a_b$', 'interpreter', 'latex');
    legend('$a_{b_1}$', '$a_{b_2}$', '$a_{b_3}$', 'interpreter', 'latex');
    xlabel('Time(seconds)');
    grid;
    suptitle('x');
    if save_file
        saveas(gcf,'imgs/realx_plot.png');
    end
end

