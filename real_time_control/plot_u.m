function plot_u(out, save_file)
    figure;
    grid;
    hold on;
    plot(out.u)
    xlabel('Time(seconds)')
    ylabel('u(t)')
    title("Control signal");
    hold off;
    if save_file
        saveas(gcf,'imgs/u_plot.png');
    end
end
