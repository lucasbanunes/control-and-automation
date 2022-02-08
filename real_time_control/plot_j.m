function plot_j(out)
figure;
plot(out.j_real, '-o');
hold on;
plot(out.j_real1, '-o');
hold on;
plot(out.j_real2, '-o');
grid;
xlabel('Time(seconds)');
ylabel('Cost');
title('Instant cost comparison');
legend('C', 'V', 'MPC');
hold off;
saveas(gcf,'imgs/plot_j.png')
end

