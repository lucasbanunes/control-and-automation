function plot_am(out, save_file, dirname)
    figure;
    grid;
    hold on;
    plot(out.am)
    xlabel('Time(seconds)')
    ylabel('')
    title('$a_m$', 'interpreter', 'latex');
    legend('$a_{m_1}$', '$a_{m_2}$', '$a_{m_3}$', 'interpreter', 'latex');
    hold off;
    if save_file
        saveas(gcf,sprintf('%s/am_plot.png', dirname));
    end
end