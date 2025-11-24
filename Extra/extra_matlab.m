clc; clear; close all;

% Parámetros motor DC ejemplo
R  = 8;         % Ohm
L  = 0.002;     % Henry
J  = 1.2e-5;    % kg*m^2
B  = 1e-4;      % Nm*s/rad
kb = 0.02;      % V*s/rad
km = 0.02;      % Nm/A 

% Función de transferencia: Omega(s)/V(s)
num = [km];
den = [L*J (R*J + L*B) (R*B + kb*km)];
G = tf(num, den);

% Tiempo de simulación
t = 0:0.001:1;

% Respuesta al impulso
[h, t_imp] = impulse(G, t);

% Obtener respuesta al escalón integrando el impulso
y_step = cumtrapz(t_imp, h);

% Convertir de rad/s a RPM
y_step_rpm = y_step * 60 / (2*pi);

% Gráfica
figure;
plot(t_imp, y_step_rpm, 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)', 'FontSize', 16)
ylabel('Velocidad (RPM)', 'FontSize', 16)
title('Respuesta al escalón del motor DC', 'FontSize', 18)
set(gca, 'FontSize', 14)


% Mostrar valor final
omega_max = y_step_rpm(end);
yline(omega_max, '--r', sprintf('Velocidad de máxima: %.2f RPM', omega_max));

% Parámetros dinámicos principales
info = stepinfo(y_step, t_imp);

fprintf("Velocidad máxima: %.2f RPM\n", omega_max);
fprintf("Tiempo de subida: %.3f s\n", info.RiseTime);
fprintf("Tiempo de establecimiento: %.3f s\n", info.SettlingTime);
