function plot_y(out)
    figure;
    plot(out.y)
    hold on
    plot(out.y1)
    hold on 
    plot(out.y2)
    xlabel('t')
    ylabel('u(t)')
    legend('C', 'V', 'MPC');
    title("Response");
    grid;
    saveas(gcf,'imgs/plot_y.png')
end