function plot_xhat(out, save_file)
    figure;
    grid;
    hold on;
    plot(out.xhat);
    xlabel('Time(seconds)');
    ylabel('$\hat{x}(t)$', 'interpreter', 'latex');
    title("System estimated states");
    data_size = size(out.xhat.Data);
    n_states = data_size(2);
    plot_legend = cell(n_states, 1);
    for i=1:n_states
        plot_legend{i} = sprintf('$\\hat{x}_{%i}$', i);
    end
    legend(plot_legend, 'interpreter', 'latex');
    hold off;
    if save_file
        saveas(gcf,'imgs/estimated_states_plot.png');
    end
end

