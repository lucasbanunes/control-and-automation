function plot_theta(out, dirname, ctrl)
    figure;
    grid;
    hold on;
    plot(out.thetad, 'LineWidth', 2.0);
    hold on;
    plot(out.theta, '--', 'LineWidth', 2.0);
    legend('$\theta_1^d$', '$\theta_2^d$', '$\theta_1$', '$\theta_2$', 'interpreter', 'latex');
    xlabel('Segundos');
    ylabel('rad');
    title(sprintf('Resposta de $\\theta$ para %s', ctrl), 'Interpreter', 'latex');
    file_ctrl = strrep(ctrl, ' ', '_');
    saveas(gcf, sprintf('%s/%s_theta.png', dirname, file_ctrl));
end