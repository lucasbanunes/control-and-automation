function plot_real_error(out, save_file, dirname)
    estimated_real_value = out.pt-out.deltap;
    real_error = estimated_real_value - out.p_ideal;
    figure;
    plot(real_error);
    hold on;
    title('Position real error $e_p = (p_t-\delta_p) - p_{ideal}$', 'interpreter', 'latex');
    ylabel('$e_p$', 'interpreter', 'latex');
    legend('$e_{p_1}$', '$e_{p_2}$', '$e_{p_3}$', 'interpreter', 'latex');
    grid;
    xlabel('Time(seconds)');
    hold off;
    if save_file
        saveas(gcf,sprintf('%s/real_error_plot.png', dirname));
    end
end

