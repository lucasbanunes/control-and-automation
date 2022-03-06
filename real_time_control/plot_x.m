function plot_x(out, save_file)
    figure;
    grid;
    hold on;
    plot(out.x)
    xlabel('Time(seconds)')
    ylabel('x(t)')
    title("System states");
    data_size = size(out.x.Data);
    n_states = data_size(2);
    plot_legend = cell(n_states, 1);
    for i=1:n_states
        plot_legend{i} = sprintf('$x_{%i}$', i);
    end
    legend(plot_legend, 'interpreter', 'latex');
    hold off;
    if save_file
        saveas(gcf,'imgs/states_plot.png');
    end
end