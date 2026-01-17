%[text] # OpenLap LTS Quick Start
%[text] ## Generate a Vehicle Model
%[text] Data points of the GGV diagram need to be generated before running the lap time sim. Call the generate\_GGV function. The function is at 
%[text] Vehicle-Modeling\\Milliken Moment Method\\Performance Envelope Generation\\GGV\_generation.m
%[text] It has the following parameters
%[text] - **mmd**:                      A MMD instance
%[text] - **V** :                           Speed range
%[text] - **SA\_CG**:                  Slip angle range, optional
%[text] - **dSteer**:                   Steer ange range, optional
%[text] - **stepSize**:                Step size when sweeping within the MMD, optional
%[text] - **levelSurfStepSize**: Step size when sweeping when filling the space between the two MMDs \
%[text] Also see FE13 MDR slides for how this function works
%%
% basic use case

% Initiallize the car parameters
carParams = Cars.FE13();

% Define MMD
mmd = MMD.MMD(carParams);

% Generate the GGV data points
V = linspace(9, 40, 20); % speed range
GGV_data = generate_GGV(mmd, V) %[output:035696ac]
%%
%[text] Override the models, eg. use Sampo's weight tranfer model
models.weightTransferFn = @MMD.models.SampoWeightTransfer;
mmd = MMD.MMD(carParams, models);
GGV_data = generate_GGV(mmd, V) %[output:5c348767] %[output:72f7face]
%%
%[text] ## Run the Lap Time Sim
%[text] Load the circuit
trackfile = 'bluemax' ;
tr = load(trackfile) ;
tr = tr.TrackInfo;
tr.info.name = "blue max";
tr.info.config = "Closed";

% Some nessesary modifications if the track is created by the
% RacingLineManualGeneration.m script

% convert radias to curvature
tr.r = 1 ./ tr.r;

tr.dx = [tr.dx; 0.5]

% The bluemax track is in feet, convert it to meters
% The newer tracks are in meters
tr.dx = tr.dx .* 0.3048;
tr.x = tr.x .* 0.3048;

tr.bank = zeros(size(tr.r));
tr.incl = zeros(size(tr.r));
tr.Z = zeros(size(tr.r));
tr.X = tr.coords(:, 1);
tr.Y = tr.coords(:, 2);
%%
%[text] Loading the car
veh = carParams;
ggv = GGV_data;
veh.ggv = ggv;
veh.name = "FE99999999999999";

v_max = max(ggv(:, 3));
ggv = [ggv; [zeros(10, 1), linspace(-2, 2, 10)', ones(10, 1)*(v_max+0.1)]];

mask = ggv(:, 1) >= 0;

v = ggv(mask, 3);
ay = ggv(mask, 2);
ax = ggv(mask, 1);

veh.max_drive = scatteredInterpolant(v, ay, ax, "linear", "nearest");

v = ggv(~mask, 3);
ay = ggv(~mask, 2);
ax = ggv(~mask, 1);

veh.max_brake = scatteredInterpolant(v, ay, ax, "linear", "nearest");
%%
%[text] Run to Sim
simname = "simulation name here"
logid = ""

[sim] = OpenlapSim(veh,tr,simname,logid);

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:035696ac]
%   data: {"dataType":"text","outputData":{"text":"1\/20, 5%\nStarting parallel pool (parpool) using the 'Processes' profile ...\n","truncated":false}}
%---
%[output:5c348767]
%   data: {"dataType":"text","outputData":{"text":"1\/20, 5%\n","truncated":false}}
%---
%[output:72f7face]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Unrecognized field name \"kF\".\n\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('MMD.models.SampoWeightTransfer', 'C:\\Users\\EricTan\\Documents\\GitHub\\Vehicle-Modeling\\Milliken Moment Method\\Performance Envelope Generation\\+MMD\\+models\\SampoWeightTransfer.m', 5)\" style=\"font-weight:bold\">MMD.models.SampoWeightTransfer<\/a> (<a href=\"matlab: opentoline('C:\\Users\\EricTan\\Documents\\GitHub\\Vehicle-Modeling\\Milliken Moment Method\\Performance Envelope Generation\\+MMD\\+models\\SampoWeightTransfer.m',5,0)\">line 5<\/a>)\n    kF = carParams.kF;                                                     % For the sake of simplicity, members of carParams structure are assigned\n    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('MMD.core\/iterateOneCell', 'C:\\Users\\EricTan\\AppData\\Local\\Temp\\e33a-d5d9-3dc0-79b4\\a\\tpb4da787d_331c_4490_9bc8_c0b4ba610298\\+MMD\\+core\\iterateOneCell.m', 113)\" style=\"font-weight:bold\">MMD.core\/iterateOneCell<\/a> (<a href=\"matlab: opentoline('C:\\Users\\EricTan\\AppData\\Local\\Temp\\e33a-d5d9-3dc0-79b4\\a\\tpb4da787d_331c_4490_9bc8_c0b4ba610298\\+MMD\\+core\\iterateOneCell.m',113,0)\">line 113<\/a>)\n        [latWT_Front, latWT_Rear, longWT] = models.weightTransferFn(carParams, AxCurr, AyCurr);\n        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('MMD.core.solve', 'C:\\Users\\EricTan\\Documents\\GitHub\\Vehicle-Modeling\\Milliken Moment Method\\Performance Envelope Generation\\+MMD\\+core\\solve.m', 90)\" style=\"font-weight:bold\">MMD.core.solve<\/a> (<a href=\"matlab: opentoline('C:\\Users\\EricTan\\Documents\\GitHub\\Vehicle-Modeling\\Milliken Moment Method\\Performance Envelope Generation\\+MMD\\+core\\solve.m',90,0)\">line 90<\/a>)\n    parfor i = 1:length(constGrid.Value.dSteer)\n    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('MMD\/MMD\/evaluate', 'C:\\Users\\EricTan\\Documents\\GitHub\\Vehicle-Modeling\\Milliken Moment Method\\Performance Envelope Generation\\+MMD\\MMD.m', 32)\" style=\"font-weight:bold\">MMD\/MMD\/evaluate<\/a> (<a href=\"matlab: opentoline('C:\\Users\\EricTan\\Documents\\GitHub\\Vehicle-Modeling\\Milliken Moment Method\\Performance Envelope Generation\\+MMD\\MMD.m',32,0)\">line 32<\/a>)\n            result = MMD.core.solve(grid, obj.carParams, mode, targetCAx, prevResult, obj.models, obj.config);\n            ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('generate_GGV', 'C:\\Users\\EricTan\\Documents\\GitHub\\Vehicle-Modeling\\Milliken Moment Method\\Performance Envelope Generation\\generate_GGV.m', 26)\" style=\"font-weight:bold\">generate_GGV<\/a> (<a href=\"matlab: opentoline('C:\\Users\\EricTan\\Documents\\GitHub\\Vehicle-Modeling\\Milliken Moment Method\\Performance Envelope Generation\\generate_GGV.m',26,0)\">line 26<\/a>)\n        driveData = mmd.evaluate(grid, \"drive\", inf, driveData);\n        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"}}
%---
