function plot_x(out)
    subplot(3,2,1);
    plot(out.x)
    legend('x_1','x_2')
    title('LQR Variable Gain states')
    xlabel('')
    ylabel('x(t)')
    grid
    
    subplot(3,2,2);
    plot(out.x1)
    legend('x_1','x_2')
    title('LQR Static Gain states')
    xlabel('')
    ylabel('')
    grid
    
    subplot(3,2,3);
    plot(out.x2)
    legend('x_1','x_2')
    title('MPC states')
    xlabel('')
    ylabel('x(t)')
    grid
    
    subplot(3,2,4);
    plot(out.x2)
    legend('x_1','x_2')
    title('MPC + Kalman states')
    xlabel('')
    ylabel('')
    grid
    
    subplot(3,2,5);
    plot(out.x4)
    legend('x_1','x_2')
    title('Non optimal states')
    xlabel('t')
    ylabel('x(t)')
    grid
    
    subplot(3,2,6);
    xlabel('t')
    ylabel('')
    saveas(gcf,'imgs/plot_x.png')
end