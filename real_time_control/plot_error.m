function plot_error(reference, out, save_file)
    figure;
    grid;
    hold on;
    plot(reference - out.y)
    xlabel('Time(seconds)')
    ylabel('Error')
    title("Tracking error");
    hold off;
    if save_file
        saveas(gcf,'imgs/error_plot.png');
    end
end
