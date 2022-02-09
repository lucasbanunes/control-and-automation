function plot_y(out)
    figure;
    plot(out.y)
    hold on
    plot(out.y1)
    hold on 
    plot(out.y2)
    hold on 
    plot(out.y3)
    hold on 
    plot(out.y4)
    xlabel('t')
    ylabel('u(t)')
    legend('LQR_C', 'LQR_V', 'MPC', 'MPC+Kalman', 'Non optimal' );
    title("Response");
    grid;
    saveas(gcf,'imgs/plot_y.png')
end