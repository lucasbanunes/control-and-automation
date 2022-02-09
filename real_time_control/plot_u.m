function plot_u(out)
    figure;
    plot(out.u)
    hold on
    plot(out.u1)
    hold on
    plot(out.u2)
    hold on
    plot(out.u3)
    hold on
    plot(out.u4)
    xlabel('t')
    ylabel('u(t)')
    legend('LQR_C', 'LQR_V', 'MPC', 'MPC+Kalman', 'Non optimal' );
    title("Control signal");
    grid;
    saveas(gcf,'imgs/plot_u.png')
end
