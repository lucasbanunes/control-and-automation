function plot_j_total(out)
figure;
plot(out.j_real_total, '-o');
hold on;
plot(out.j_real_total1, '-o');
hold on;
plot(out.j_real_total2, '-o');
grid;
xlabel('Time(seconds)');
ylabel('Cost');
title('Total cost comparison');
legend('C', 'V', 'MPC');
hold off;
saveas(gcf,'imgs/plot_j_total.png')
end

