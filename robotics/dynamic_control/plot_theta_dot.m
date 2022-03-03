function plot_theta_dot(out, dirname, ctrl)
    figure;
    grid;
    hold on;
    plot(out.theta_dotd, 'LineWidth', 2.0);
    hold on;
    plot(out.theta_dot,'--', 'LineWidth', 2.0);
    legend('$\dot{\theta}_1^d$', '$\dot{\theta}_2^d$', '$\dot{\theta}_1$', '$\dot{\theta}_2$', 'interpreter', 'latex');
    xlabel('Segundos');
    ylabel('rad/s');
    title(sprintf('Resposta de $\\dot{\\theta}$ para %s', ctrl), 'interpreter', 'latex');
    file_ctrl = strrep(ctrl, ' ', '_');
    saveas(gcf, sprintf('%s/%s_theta_dot.png', dirname, file_ctrl));
end