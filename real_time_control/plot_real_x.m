function plot_real_x(out, save_file, dirname)
    figure;
    subplot(2,1,1);
    plot(out.pt-out.deltap);
    title('Position');
    ylabel('p');
    legend('$p_1$', '$p_2$', '$p_3$', 'interpreter', 'latex');
    grid;
    subplot(2,1,2);
    plot(out.vt-out.deltav);
    title('Velocity');
    legend('$v_1$', '$\delta v_2$', '$v_3$', 'interpreter', 'latex');
    xlabel('Time(seconds)');
    ylabel('v');
    grid;
    suptitle('x');
    if save_file
        saveas(gcf,sprintf('%s/realx_plot.png', dirname));
    end
end

