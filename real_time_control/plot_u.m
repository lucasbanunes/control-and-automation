function plot_u(out)
    figure;
    plot(out.u)
    hold on
    plot(out.u1)
    hold on
    plot(out.u2)
    xlabel('t')
    ylabel('u(t)')
    legend('C', 'V', 'MPC');
    title("Control signal");
    grid;
    saveas(gcf,'imgs/plot_u.png')
end
