%{
Interfaz gráfica interactiva para analizar un motor DC mediante su modelo
de función de transferencia.

Incluye:
  - Sliders responsivos para parámetros
  - Respuesta al escalón
  - Diagrama de polos y ceros
  - Cálculo dinámico de la frecuencia de muestreo recomendada

Frecuencia de muestreo:
  Se calcula a partir del polo dominante:
      tau = -1 / Re(polo_dominante)
      fs ∈ [10/tau , 20/tau]
%}

function motorDC_MatLab
    close all; clc;

    % Ventana principal
    f = figure('Position',[100 100 1100 600],'Name','Motor DC Interactivo');

    % Panel de parámetros
    panel = uipanel('Parent',f, ...
                    'Units','normalized', ...
                    'Position',[0.02 0.05 0.25 0.9], ...
                    'Title','Parámetros del Motor DC', ...
                    'FontSize',12);

    % Ejes de gráficas
    ax1 = subplot('Position',[0.35 0.55 0.6 0.4]);  % Respuesta al escalón
    ax2 = subplot('Position',[0.35 0.08 0.6 0.35]); % Polos y ceros

    % Valores iniciales
    p.R  = 8;
    p.L  = 0.002;
    p.J  = 1.2e-5;
    p.B  = 1e-4;
    p.kb = 0.02;
    p.km = 0.02;
    setappdata(f, "params", p);

    % Definición de sliders
    sliders = { 
        'R', 1, 20, 'R (Ω)' % resistencia del devanado 
        'L', 0.0005, 0.01,'L (H)' % inductancia del devanado 
        'J', 1e-6, 5e-5, 'J (kg·m²)' % inercia del rotor + carga 
        'B', 1e-5, 5e-4, 'B (N·m·s/rad)' % coeficiente de fricción viscosa 
        'kb', 0.005, 0.05, 'kb (V·s/rad)' % constante de fuerza contraelectromotriz 
        'km', 0.005, 0.05, 'km (N·m/A)' % constante de torque
        };

    % Crear sliders responsivos
    n = size(sliders,1);
    for i = 1:n
        name  = sliders{i,1};
        minv  = sliders{i,2};
        maxv  = sliders{i,3};
        label = sliders{i,4};

        block_h = 1/n;
        y_low   = 1 - i*block_h;

        % Posiciones normalizadas
        label_pos  = [0.05, y_low+0.25*block_h, 0.40, 0.20*block_h];
        slider_pos = [0.50, y_low+0.25*block_h, 0.35, 0.20*block_h];
        value_pos  = [0.88, y_low+0.25*block_h, 0.10, 0.20*block_h];

        % Texto del parámetro
        uicontrol(panel,'Style','text', ...
                  'Units','normalized', ...
                  'Position',label_pos, ...
                  'String', label, ...
                  'FontSize',11, ...
                  'HorizontalAlignment','left');

        % Slider
        slider = uicontrol(panel,'Style','slider', ...
                  'Units','normalized', ...
                  'Min',minv,'Max',maxv,'Value',p.(name), ...
                  'Position',slider_pos, ...
                  'Tag',name);

        % Texto del valor numérico
        valueText = uicontrol(panel,'Style','text', ...
                     'Units','normalized', ...
                     'Position',value_pos, ...
                     'String', sprintf('%.4f', p.(name)), ...
                     'FontSize',11, ...
                     'Tag',['txt_' name]);

        % Callback
        slider.Callback = @(src,~) actualizar(src, ax1, ax2, valueText);
    end

    uicontrol(panel,'Style','text', ...
              'Units','normalized', ...
              'Position',[0.05 0.02 0.9 0.05], ...
              'String','Frecuencia de muestreo: -- Hz', ...
              'FontSize',11, ...
              'ForegroundColor',[0 0 1], ...
              'Tag','txt_fs');

    % Graficar por primera vez
    actualizar([], ax1, ax2, []);
end

%  Funcion actualizar
function actualizar(src, ax1, ax2, txtHandle)
    f = ancestor(ax1, "figure");
    p = getappdata(f, "params");

    % Si movieron un slider, guardar el nuevo valor
    if ~isempty(src)
        p.(src.Tag) = src.Value;
        setappdata(f, "params", p);
        set(txtHandle, "String", sprintf("%.6f", src.Value));
    end

    % Crear función de transferencia G(s)
    num = [p.km];
    den = [p.L*p.J  (p.R*p.J + p.L*p.B)  (p.R*p.B + p.kb*p.km)];
    G = tf(num, den);

    % Respuesta al Escalón
    t = 0:0.001:2;
    [h, t_imp] = impulse(G, t);
    y_step = cumtrapz(t_imp, h);
    y_step_rpm = y_step * 60 / (2*pi);

    cla(ax1)
    axes(ax1)
    plot(t_imp, y_step_rpm, "LineWidth", 1.5);
    grid on;
    xlabel("Tiempo (s)");
    ylabel("RPM");
    title("Respuesta al escalón");
    yline(y_step_rpm(end), "--r");

    %   Polos y ceros
    cla(ax2)
    axes(ax2)
    pzmap(G)
    grid on;
    title("Diagrama de Polos y Ceros");
end
