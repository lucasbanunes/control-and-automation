function plot_j(out)
figure;
plot(out.j_real, '-o');
hold on;
plot(out.j_real1, '-o');
hold on;
plot(out.j_real2, '-o');
hold on;
plot(out.j_real3, '-o');
hold on;
plot(out.j_real4, '-o');
grid;
xlabel('Time(seconds)');
ylabel('Cost');
title('Instant cost comparison');
legend('LQR_C', 'LQR_V', 'MPC', 'MPC+Kalman', 'Non optimal' );
hold off;
saveas(gcf,'imgs/plot_j.png')
end

