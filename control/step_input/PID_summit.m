%% Identification systeme du Summit XL - Stage FILDARIANE
% Auteur : Ronan Le Guenne
% Date : juin 2026

clear; clc; close all;

%% 1. Charger le CSV
data = readtable('ident.csv');
t = data.time;
u = data.cmd_vel_x;     % commande envoyee (m/s)
y = data.odom_vel_x;    % vitesse mesuree par odom (m/s)

%% 2. Visualisation des donnees brutes
figure('Name', 'Donnees brutes');
plot(t, u, 'b-', 'LineWidth', 1.5); hold on;
plot(t, y, 'r-', 'LineWidth', 1);
legend('cmd\_vel (commande)', 'odom (mesure)', 'Location', 'best');
xlabel('Temps (s)'); ylabel('Vitesse (m/s)');
title('Echelon de vitesse - donnees brutes');
grid on;

%% 3. Creation du dataset iddata
Ts = mean(diff(t));
fprintf('Periode d''echantillonnage : %.4f s (%.1f Hz)\n', Ts, 1/Ts);

sys_data = iddata(y, u, Ts);
sys_data.InputName  = 'cmd_vel';
sys_data.OutputName = 'odom_vel';
sys_data.TimeUnit   = 'seconds';

%% 4. Lancer la GUI d'identification interactive
systemIdentification(sys_data)

% G_summit = G_summit;   % depuis la GUI
%
% % Construction du modele complet
% integrateur = tf(1, [1 0]);                     % 1/s physique
% H_vision    = tf(1, 1, 'InputDelay', 0.1);      % retard vision estime 100 ms
% T_systeme   = G_summit * integrateur * H_vision;
%
% % Calcul des gains PID
% C = pidtune(T_systeme, 'PID');
% [Kp, Ki, Kd, Tf] = piddata(C);
%
% fprintf('\n=== Gains PID calcules ===\n');
% fprintf('Kp = %.4f\n', Kp);
% fprintf('Ki = %.4f\n', Ki);
% fprintf('Kd = %.4f\n', Kd);
%
% % Visualisation reponse boucle fermee
% sys_bf = feedback(C * T_systeme, 1);
% figure('Name', 'Reponse boucle fermee');
% step(sys_bf, 10);
% title(sprintf('Reponse echelon avec Kp=%.2f Ki=%.2f Kd=%.2f', Kp, Ki, Kd));
% grid on;