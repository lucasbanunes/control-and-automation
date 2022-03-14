function plot_xt(out, save_file, dirname)
    subplot(2,1,1);
    grid;
    hold on;
    plot(out.pt);
    title('Position');
    legend('p_1', 'p_2', 'p_3');
    hold off;
    subplot(2,1,2);
    grid;
    hold on;
    plot(out.vt);
    title('Velocity');
    legend('v_1', 'v_2', 'v_3');
    xlabel('Time(seconds)');
    hold off;
    suptitle('xt');
    if save_file
        saveas(gcf,sprintf('%s/xt_plot.png', dirname));
    end
end