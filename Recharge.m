clear
close all


function [omega] = angular_speed (ax, v_i, v_f, r)
    % Tempo
    T = (v_f - v_i) / ax;
    t = 0 : 1e-3 : T;

    % Vettore delle velocità tra iniziale e finale
    n = length(t);
    v0 = repmat(v_i, 1, n);
    v = v0 + ax * t;
    omega = v / r;

end

function [Power_diss] = Power_diss (v_i, v_f, ax, Power_reg, mass, r)
    
    Power_diss = (mass * ax * r .* angular_speed(ax, v_i, v_f, r)) - Power_reg;
    Power_diss(Power_diss > 0) = 0;

end

function [Eff] = Efficiency (v_i, v_f, ax, Power_reg, mass, r)

    Eff = Power_reg ./ (Power_reg + Power_diss(v_i, v_f, ax, Power_reg, mass, r));

end

function [Load_transfer] = load_transfer (ax, ax_max)

    k = 0.35 / ax_max; %costante per calcolare i trasferimenti di carico
    Load_transfer = k * ax;

end

function [Torque] = torque (Power_reg, ax_request, v_i, v_f, mass, r)

    Power_tot = Power_reg + Power_diss(v_i, v_f, ax_request, Power_reg, mass, r);

    Torque = Power_tot ./ angular_speed (ax_request, v_i, v_f, r);

end


%Dati iniziali
g = 9.81;
v_i = 80 / 3.6;
v_f = 30 / 3.6;

Power_reg = -24000;
mass = 270;
r = 0.205;

Torque_max = -21; %Coppia max motore [Nm]
gear_box = 5 : 2 : 15;
n_gear = length(gear_box);

ax_min = Power_reg / (v_i * mass);
ax_max = -2 * g;
ax =  ax_min : -0.01 : ax_max;

n = length(ax);
Eff = zeros(n, 1);
Load_T = zeros(n, 1);

%Specificare di quale accelerazione si vuole il grafico della coppia
%Con un valore compreso tra [-19.62, -4.00]
ax_request = -10.64;

%for j = 1 : n_gear
%    current_gear = gear_box(j);
%end

for i = 1 : n
    Eff(i) = mean(Efficiency(v_i, v_f, ax(i), Power_reg, mass, r));
    Load_T(i) = load_transfer (ax(i), ax_max);
end

%Grafico Efficienza-accelerazione
figure ("Name", "Efficiency Vs acceleration", "NumberTitle", "off"), clf
hold on
grid on

yyaxis left
plot(abs(ax), Eff, 'b')

yyaxis right
plot(abs(ax), abs(ax), 'r')

T = (v_f - v_i) / ax_request;
t = 0 : 1e-3 : T;

Torque_wheel = torque (Power_reg, ax_request, v_i, v_f, mass, r);

Torque_front = (0.5 + load_transfer(ax_request, ax_max)) * Torque_wheel;
Torque_rear = (0.5 - load_transfer(ax_request, ax_max)) * Torque_wheel;

%Grafico coppia delle ruote
figure("Name","Torque Front & Rear", "NumberTitle","off"), clf
hold on
grid on
plot(t, abs(Torque_front))
plot(t, abs(Torque_rear))
ylabel('Torque [Nm]')
xlabel('Time [s]')
legend('Torque Front', 'Torque Rear', 'location', 'east')

%Grafico trasferimenti di carico
figure("Name", "Load Transfer", "NumberTitle","off"),clf
hold on
grid on
plot(abs(ax), 0.5 + Load_T)
plot(abs(ax), 0.5 - Load_T)
xlabel('Acceleration [m/s^2]')
ylabel('Force balance')
legend('Load Front', 'Unload Rear', 'Location', 'east')