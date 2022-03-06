function plot_cost(out, save_file)
    figure;
    grid;
    hold on;
    plot(out.cost, '-o');
    xlabel('Time(seconds)');
    ylabel('Cost');
    title('Instant cost');
    hold off;
    if save_file
        saveas(gcf,'imgs/cost_plot.png');
    end
end

