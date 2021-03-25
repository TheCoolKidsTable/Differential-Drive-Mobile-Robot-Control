function u = inputTorque(t, x)

if t < 1
    u = [0;0];
elseif t < 1.5
    u = [0.1;0.1];
elseif t < 2.5
    u = [0;0];
elseif t < 4
    u = [0.1;0];
elseif t < 5
    u = [0;0];
elseif t < 6
    u = [0;0.1];
elseif t < 7
    u = [0;0];
elseif t < 10
    u = [0.1;0.1];
else
    u = [0;0];
end