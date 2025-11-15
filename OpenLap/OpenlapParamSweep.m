clear; clc; close all;

m = 1:10;
motor_pwr = 1:10;
carParams = repmat(Cars.FE12, size(m));

for i = 1:length(m)
    carParams(i).m = m(i);
    carParams(1).drivetrain.Power = motor_pwr(i);
end

V = linspace(7, 40, 2);
%%

for i = 1:length(m)
    mmd = MMD.MMD(carParams(i));
    GGV_data = generate_GGV(mmd, V);

    save("LTS_Sweep_GGV_Data" + i, "GGV_data", "carParams(i)")
end