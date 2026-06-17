%% =====================================================================
%% Figure comparative PID lineaire vs angulaire
%% Variables extraites depuis le SimulationOutput "out"
%% =====================================================================

% Consignes appliquees dans les deux schemas (a ajuster si besoin)
consigne_lin = 1.0;   % m
consigne_ang = 0.2;    % rad

% Extraction depuis out
y_lin_1dof = out.y_lin_1dof;
y_lin_2dof = out.y_lin_2dof;
y_lin_IP   = out.y_lin_IP;
y_ang_1dof = out.y_ang_1dof;
y_ang_2dof = out.y_ang_2dof;
y_ang_IP   = out.y_ang_IP;
tout       = out.tout;

%% Calcul des overshoots
os_lin_1dof = max(0, (max(y_lin_1dof) - consigne_lin) / consigne_lin * 100);
os_lin_2dof = max(0, (max(y_lin_2dof) - consigne_lin) / consigne_lin * 100);
os_lin_IP   = max(0, (max(y_lin_IP)   - consigne_lin) / consigne_lin * 100);

os_ang_1dof = max(0, (max(y_ang_1dof) - consigne_ang) / consigne_ang * 100);
os_ang_2dof = max(0, (max(y_ang_2dof) - consigne_ang) / consigne_ang * 100);
os_ang_IP   = max(0, (max(y_ang_IP)   - consigne_ang) / consigne_ang * 100);

%% Affichage console
fprintf('\n=== Resultats lineaire (consigne = %.2f m) ===\n', consigne_lin);
fprintf('  PID 1-DoF : overshoot = %.2f %%\n', os_lin_1dof);
fprintf('  PID 2-DoF : overshoot = %.2f %%\n', os_lin_2dof);
fprintf('  PID IP    : overshoot = %.2f %%\n', os_lin_IP);

fprintf('\n=== Resultats angulaire (consigne = %.2f rad) ===\n', consigne_ang);
fprintf('  PID 1-DoF : overshoot = %.2f %%\n', os_ang_1dof);
fprintf('  PID 2-DoF : overshoot = %.2f %%\n', os_ang_2dof);
fprintf('  PID IP    : overshoot = %.2f %%\n', os_ang_IP);

%% Figure
figure('Name', 'Comparaison PID lineaire vs angulaire', ...
       'Position', [50 50 1600 700], ...
       'Color', 'w');

% Couleurs coherentes entre les deux subplots
couleur_1dof = [0.929 0.694 0.125];   % jaune
couleur_2dof = [0      0.447 0.741];   % bleu
couleur_IP   = [0.851  0.325 0.098];   % orange

% =====================================================================
% Subplot 1 : LINEAIRE
% =====================================================================
subplot(1, 2, 1);

plot(tout, y_lin_1dof, 'Color', couleur_1dof, 'LineWidth', 2.5); hold on;
plot(tout, y_lin_2dof, 'Color', couleur_2dof, 'LineWidth', 2.5);
plot(tout, y_lin_IP,   'Color', couleur_IP,   'LineWidth', 2.5);
yline(consigne_lin, '--k', 'Consigne', 'LineWidth', 1.2, ...
      'LabelHorizontalAlignment', 'left');

legend(sprintf('PID 1-DoF : OS = %.2f %%', os_lin_1dof), ...
       sprintf('PID 2-DoF : OS = %.2f %%', os_lin_2dof), ...
       sprintf('PID IP (b=0, c=0) : OS = %.2f %%', os_lin_IP), ...
       'Location', 'southeast', ...
       'FontSize', 12, ...
       'Box', 'on');

xlabel('Temps (s)', 'FontSize', 13);
ylabel('Distance (m)', 'FontSize', 13);
title(sprintf('Asservissement lineaire (consigne = %.2f m)', consigne_lin), ...
      'FontSize', 14, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 12, 'LineWidth', 1);

% =====================================================================
% Subplot 2 : ANGULAIRE
% =====================================================================
subplot(1, 2, 2);

plot(tout, y_ang_1dof, 'Color', couleur_1dof, 'LineWidth', 2.5); hold on;
plot(tout, y_ang_2dof, 'Color', couleur_2dof, 'LineWidth', 2.5);
plot(tout, y_ang_IP,   'Color', couleur_IP,   'LineWidth', 2.5);
yline(consigne_ang, '--k', 'Consigne', 'LineWidth', 1.2, ...
      'LabelHorizontalAlignment', 'left');

legend(sprintf('PID 1-DoF : OS = %.2f %%', os_ang_1dof), ...
       sprintf('PID 2-DoF : OS = %.2f %%', os_ang_2dof), ...
       sprintf('PID IP (b=0, c=0) : OS = %.2f %%', os_ang_IP), ...
       'Location', 'southeast', ...
       'FontSize', 12, ...
       'Box', 'on');

xlabel('Temps (s)', 'FontSize', 13);
ylabel('Angle (rad)', 'FontSize', 13);
title(sprintf('Asservissement angulaire (consigne = %.2f rad)', consigne_ang), ...
      'FontSize', 14, 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 12, 'LineWidth', 1);

% =====================================================================
% Titre general
% =====================================================================
sgtitle('Influence de la structure PID (1-DoF, 2-DoF, IP) sur la reponse a un echelon', ...
        'FontSize', 16, 'FontWeight', 'bold');

%% Sauvegarde optionnelle de la figure
print(gcf, 'comparaison_pid_lin_ang.png', '-dpng', '-r300');
% print(gcf, 'comparaison_pid_lin_ang.pdf', '-dpdf', '-bestfit');