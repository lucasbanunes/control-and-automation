function plot_deltax(out, save_file, dirname)
    figure;
    subplot(2,1,1);
    plot(out.deltap);
    title('Error Position');
    legend('$\delta p_1$', '$\delta p_2$', '$\delta p_3$', 'interpreter', 'latex');
    grid;
    subplot(2,1,2);
    plot(out.deltav);
    title('Error Velocity');
    legend('$\delta v_1$', '$\delta v_2$', '$\delta v_3$', 'interpreter', 'latex');
    xlabel('Time(seconds)');
    grid;
    suptitle('Error x');
    if save_file
        saveas(gcf,sprintf('%s/deltax_plot.png', dirname));
    end
end

