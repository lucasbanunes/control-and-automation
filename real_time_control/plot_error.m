function plot_error(reference, out)
    figure;
    plot(reference - out.y)
    hold on
    plot(reference - out.y1)
    hold on
    plot(reference - out.y2)
    hold on
    plot(reference - out.y3)
    hold on
    plot(reference - out.y4)
    xlabel('Time(seconds)')
    ylabel('Error)')
    legend('LQR_C', 'LQR_V', 'MPC', 'MPC+Kalman', 'Non optimal' );
    title("Tracking error");
    grid;
    saveas(gcf,'imgs/plot_error.png');
end
