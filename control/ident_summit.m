clear
close all
clc

%% =====================================================================
%% Identification systeme Summit XL - lineaire et angulaire
%% Modele de processus : ordre 1 + retard (P1D)
%% =====================================================================
%% Gere les deux formats de CSV :
%%   - Ancien  : time, cmd_vel_x, odom_vel_x
%%   - Nouveau : time, cmd_lin, cmd_ang, odom_lin, odom_ang

%% --- Lecture des CSV ---
fprintf('=== Chargement des donnees ===\n');

data_lin = readtable('ident_linear.csv');
data_ang = readtable('ident_angular.csv');

fprintf('  ident_linear.csv  : %d lignes, colonnes = %s\n', ...
    height(data_lin), strjoin(data_lin.Properties.VariableNames, ', '));
fprintf('  ident_angular.csv : %d lignes, colonnes = %s\n', ...
    height(data_ang), strjoin(data_ang.Properties.VariableNames, ', '));

%% --- Detection automatique des colonnes ---
if ismember('cmd_lin', data_lin.Properties.VariableNames)
    u_lin = data_lin.cmd_lin;
    y_lin = data_lin.odom_lin;
elseif ismember('cmd_vel_x', data_lin.Properties.VariableNames)
    u_lin = data_lin.cmd_vel_x;
    y_lin = data_lin.odom_vel_x;
else
    error('CSV lineaire : colonnes attendues introuvables');
end

if ismember('cmd_ang', data_ang.Properties.VariableNames)
    u_ang = data_ang.cmd_ang;
    y_ang = data_ang.odom_ang;
elseif ismember('cmd_vel_x', data_ang.Properties.VariableNames)
    u_ang = data_ang.cmd_vel_x;
    y_ang = data_ang.odom_vel_x;
else
    error('CSV angulaire : colonnes attendues introuvables');
end

t_lin = data_lin.time;
t_ang = data_ang.time;

%% --- Creation des datasets iddata ---
Ts_lin = mean(diff(t_lin));
Ts_ang = mean(diff(t_ang));

sys_lin = iddata(y_lin, u_lin, Ts_lin);
sys_lin.InputName  = 'cmd_lin';
sys_lin.OutputName = 'odom_lin';
sys_lin.TimeUnit   = 'seconds';

sys_ang = iddata(y_ang, u_ang, Ts_ang);
sys_ang.InputName  = 'cmd_ang';
sys_ang.OutputName = 'odom_ang';
sys_ang.TimeUnit   = 'seconds';

%% --- Identification : ordre 1 + retard (P1D) ---
fprintf('\n=== Identification (Process Model P1D) ===\n');

opts = procestOptions('Display', 'off');
opts.SearchOptions.MaxIterations = 100;

G_lin = procest(sys_lin, 'P1D', opts);
G_ang = procest(sys_ang, 'P1D', opts);

%% --- Recuperation des fits via la fonction compare() ---
% compare() retourne le %fit utilise dans les graphiques
[~, fit_lin, ~] = compare(sys_lin, G_lin);
[~, fit_ang, ~] = compare(sys_ang, G_ang);

%% --- Affichage console ---
fprintf('\n=== Modele lineaire identifie ===\n');
fprintf('  Kp  = %.4f\n', G_lin.Kp);
fprintf('  Tp1 = %.4f s\n', G_lin.Tp1);
fprintf('  Td  = %.4f s\n', G_lin.Td);
fprintf('  Fit (estimation) : %.2f %%\n', G_lin.Report.Fit.FitPercent);
fprintf('  Fit (simulation) : %.2f %%\n', fit_lin);

fprintf('\n=== Modele angulaire identifie ===\n');
fprintf('  Kp  = %.4f\n', G_ang.Kp);
fprintf('  Tp1 = %.4f s\n', G_ang.Tp1);
fprintf('  Td  = %.4f s\n', G_ang.Td);
fprintf('  Fit (estimation) : %.2f %%\n', G_ang.Report.Fit.FitPercent);
fprintf('  Fit (simulation) : %.2f %%\n', fit_ang);

%% --- Figure unique 2x2 ---
figure('Name', 'Identification Summit XL', 'Position', [50 50 1500 900], 'Color', 'w');

% =============== Subplot 1 : Echelon lineaire (donnees brutes) ===============
subplot(2, 2, 1);
plot(t_lin, u_lin, 'b', 'LineWidth', 1.5); hold on;
plot(t_lin, y_lin, 'r', 'LineWidth', 1);
legend('cmd\_vel lineaire', 'odom lineaire', 'Location', 'best');
xlabel('Temps (s)'); ylabel('Vitesse (m/s)');
title(sprintf(['Echelon lineaire (donnees brutes)\n' ...
               'Kp = %.4f | Tp1 = %.4f s | Td = %.4f s | Fit = %.2f %%'], ...
              G_lin.Kp, G_lin.Tp1, G_lin.Td, G_lin.Report.Fit.FitPercent), ...
      'FontWeight', 'bold');
grid on;

% =============== Subplot 2 : Echelon angulaire (donnees brutes) ===============
subplot(2, 2, 2);
plot(t_ang, u_ang, 'b', 'LineWidth', 1.5); hold on;
plot(t_ang, y_ang, 'r', 'LineWidth', 1);
legend('cmd\_vel angulaire', 'odom angulaire', 'Location', 'best');
xlabel('Temps (s)'); ylabel('Vitesse angulaire (rad/s)');
title(sprintf(['Echelon angulaire (donnees brutes)\n' ...
               'Kp = %.4f | Tp1 = %.4f s | Td = %.4f s | Fit = %.2f %%'], ...
              G_ang.Kp, G_ang.Tp1, G_ang.Td, G_ang.Report.Fit.FitPercent), ...
      'FontWeight', 'bold');
grid on;

% =============== Subplot 3 : Reponse impulsionnelle lineaire ===============
t_imp = 0:0.001:0.5;
[y_imp_lin, t_out_lin] = impulse(G_lin, t_imp);

subplot(2, 2, 3);
plot(t_out_lin, y_imp_lin, 'b', 'LineWidth', 2);
xlabel('Temps (s)'); ylabel('Amplitude');
title(sprintf(['Reponse impulsionnelle - modele lineaire\n' ...
               'G(s) = %.4f / (1 + %.4f s) * exp(-%.4f s)'], ...
              G_lin.Kp, G_lin.Tp1, G_lin.Td), ...
      'FontWeight', 'bold');
grid on;
xline(G_lin.Td, '--k', 'Retard pur', 'LabelVerticalAlignment', 'bottom');

% =============== Subplot 4 : Reponse impulsionnelle angulaire ===============
[y_imp_ang, t_out_ang] = impulse(G_ang, t_imp);

subplot(2, 2, 4);
plot(t_out_ang, y_imp_ang, 'r', 'LineWidth', 2);
xlabel('Temps (s)'); ylabel('Amplitude');
title(sprintf(['Reponse impulsionnelle - modele angulaire\n' ...
               'G(s) = %.4f / (1 + %.4f s) * exp(-%.4f s)'], ...
              G_ang.Kp, G_ang.Tp1, G_ang.Td), ...
      'FontWeight', 'bold');
grid on;
xline(G_ang.Td, '--k', 'Retard pur', 'LabelVerticalAlignment', 'bottom');

% Titre general
sgtitle(sprintf(['Identification systeme Summit XL  -  Modele P1D (ordre 1 + retard)\n' ...
                 'Fs = %.0f Hz'], 1/Ts_lin), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% --- Sauvegarde ---
save('models_summit.mat', 'G_lin', 'G_ang');
fprintf('\n=== Sauvegarde ===\n');
fprintf('  Modeles sauves dans models_summit.mat\n');
fprintf('\n=== Pour Simulink ===\n');
fprintf('  Lineaire :\n');
fprintf('    Transfer Fcn  : Num=[%.4f]  Den=[%.4f 1]\n', G_lin.Kp, G_lin.Tp1);
fprintf('    Transport Delay : %.4f s\n', G_lin.Td);
fprintf('  Angulaire :\n');
fprintf('    Transfer Fcn  : Num=[%.4f]  Den=[%.4f 1]\n', G_ang.Kp, G_ang.Tp1);
fprintf('    Transport Delay : %.4f s\n', G_ang.Td);