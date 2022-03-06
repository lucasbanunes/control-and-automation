function plot_total_cost(out, save_file)
    figure;
    grid;
    hold on;
    plot(out.total_cost, '-o');
    xlabel('Time(seconds)');
    ylabel('Cost');
    title('Total cost');
    hold off;
    if save_file
        saveas(gcf,'imgs/total_cost_plot.png');
    end
end

