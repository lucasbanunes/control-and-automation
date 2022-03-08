function plot_error(out, dirname, ctrl)
    figure;
    grid;
    hold on;
    plot(out.thetad-out.theta, 'LineWidth', 2.0);
    legend('e_1', 'e_2');
    xlabel('Segundos');
    ylabel('rad');
    title(sprintf('Erro de rastreamento para %s', ctrl), 'Interpreter', 'latex');
    file_ctrl = strrep(ctrl, ' ', '_');
    saveas(gcf, sprintf('%s/%s_error.png', dirname, file_ctrl));
end