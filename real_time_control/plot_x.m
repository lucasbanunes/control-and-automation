function plot_x(out)
    figure;
    plot(out.x)
    hold on
    plot(out.x1)
    hold on
    plot(out.x2)
    xlabel('t')
    ylabel('x(t)')
    legend('x_{1_{c}}','x_{2_{c}}','x_{1_{v}}','x_{2_{v}}','x_{1_{MPC}}','x_{2_{MPC}}')
    title("States");
    grid;
    saveas(gcf,'imgs/plot_x.png')
end