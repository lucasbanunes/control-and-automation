%definindo os parâmetros
clear;
close all;
dirname = 'imgs';

%Planta
kp = 1;
num_p = [1];
den_p = [1 0 0];
n = length(den_p)- 1;

%Adaptacao
Gamma = eye(3);
gamma = 1;

% Estados iniciais
x0p = zeros(n, 1);
x0p(1)=3;
theta0 = zeros(3 , 1);
rho0 = 0;

if not(isfolder(dirname))
    mkdir(dirname);
end

% planta 
ssp = canon(tf(kp*num_p, den_p), 'companion');
ap = ssp.A';
bp = ssp.C';
cp = ssp.B';
dp = 0;

%filtros
A = [0 1; 0 0];
e1 = [1 0]';
e2 = [0 1]';
k = 10*ones(n, 1);
A0 = A- k*e1';

%controle
c1=1;
d1=1;
c2=1;
d2=1;

%tunning
e1_tau = [1 0 0]';

% out = sim('trab6.slx' , 'StartTime' , '0' , 'StopTime', '200');
% 
% %Resposta do sistema
% figure;
% hold on;
% grid;
% plot(out.yr, 'LineWidth', 2.0);
% plot(out.y,'--', 'LineWidth', 2.0);
% legend('y', 'y_r');
% xlabel('Tempo (segundos)');
% ylabel('Sinal');
% title('Resposta do sistema');
% filepath = sprintf('%s\\resposta_do_sistema.png', dirname);
% saveas(gcf, filepath);
% hold off;
% 
% %Entrada do sistema
% figure;
% hold on;
% grid;
% plot(out.u);
% legend('u');
% xlabel('Tempo (segundos)');
% ylabel('Sinal');
% title('Entrada do sistema');
% filepath = sprintf('%s\\entrada_do_sistema.png', dirname);
% saveas(gcf, filepath)
% hold off;
% 
% %Ganhos de controle
% figure;
% hold on;
% grid;
% plot(out.theta_hat);
% hold on
% plot(out.rho_hat);
% legend('$\hat{\theta}_1$', '$\hat{\theta}_2$', '$\hat{\theta}_3$','$\hat{\rho}$', 'interpreter','latex');
% xlabel('Tempo (segundos)');
% ylabel('Sinal');
% title('Parâmetros');
% filepath = sprintf('%s\\theta_hat.png', dirname);
% saveas(gcf, filepath)
% hold off;
% 
% 
% %Erro
% figure;
% hold on;
% grid;
% plot(out.z1);
% legend('e');
% xlabel('Tempo (segundos)');
% ylabel('Sinal');
% title('Erro');
% filepath = sprintf('%s\\z1.png', dirname);
% saveas(gcf, filepath)
% hold off;
% 
% 
% %Módulo do ganho
% figure;
% hold on;
% grid;
% plot(out.norm_theta_hat);
% xlabel('Tempo (segundos)');
% ylabel('Sinal');
% title('Norma dos parâmetros');
% filepath = sprintf('%s\\norm_theta_hat.png', dirname);
% saveas(gcf, filepath)
% hold off;
