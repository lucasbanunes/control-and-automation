function plot_error(reference, out)
    figure;
    plot(reference - out.y)
    hold on
    plot(reference - out.y1)
    hold on
    plot(reference - out.y2)
    xlabel('Time(seconds)')
    ylabel('Error)')
    legend('C', 'V', 'MPC');
    title("Tracking error");
    grid;
    saveas(gcf,'imgs/plot_error.png');
end
