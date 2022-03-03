function plot_theta_ddot(out, dirname, ctrl)
    figure;
    grid;
    hold on;
    plot(out.theta_ddotd, 'LineWidth', 2.0);
    hold on;
    plot(out.theta_ddot, '--', 'LineWidth', 2.0);
    legend('$\ddot{\theta}_1^d$', '$\ddot{\theta}_2^d$', '$\ddot{\theta}_1$', '$\ddot{\theta}_2$', 'interpreter', 'latex');
    xlabel('Segundos');
    ylabel('rad/s');
    title(sprintf('Resposta de $\\ddot{\\theta}$ para %s', ctrl), 'interpreter', 'latex');
    file_ctrl = strrep(ctrl, ' ', '_');
    saveas(gcf, sprintf('%s/%s_theta_ddot.png', dirname, file_ctrl));
end