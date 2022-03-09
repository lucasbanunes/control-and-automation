function plot_deltax(out, save_file)
    figure;
    subplot(2,2,1);
    plot(out.deltax.Time, out.deltax.Data(:, (1:3)));
    title('Error Position');
    legend('$\delta p_1$', '$\delta p_2$', '$\delta p_3$', 'interpreter', 'latex');
    grid;
    subplot(2,2,2);
    plot(out.deltax.Time, out.deltax.Data(:, (4:6)));
    title('Error Velocity');
    legend('$\delta v_1$', '$\delta v_2$', '$\delta v_3$', 'interpreter', 'latex');
    xlabel('Time(seconds)');
    grid;
    subplot(2,2,3);
    plot(out.deltax.Time, out.deltax.Data(:, (7:9)));
    title('Error $a_b$', 'interpreter', 'latex');
    legend('$\delta a_{b_1}$', '$\delta a_{b_2}$', '$\delta a_{b_3}$', 'interpreter', 'latex');
    xlabel('Time(seconds)');
    grid;
    suptitle('Error x');
    if save_file
        saveas(gcf,'imgs/deltax_plot.png');
    end
end

