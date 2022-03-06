function plot_y(out, save_file)
    figure;
    grid;
    hold on;
    plot(out.y)
    xlabel('Time(seconds)')
    ylabel('y(t)')
    title("Output signal");
    hold off;
    if save_file
        saveas(gcf,'imgs/y_plot.png');
    end
end