clc; clear; close all;

%% Corner Performance Weighting
% This script calculates the relative importance of various cornering radii
% from previously generated track analyses and competition event point
% distributions.
%
% v1.0 (12/23/2020) - Initial Version

cd( [fileparts( which( 'CornerPerformanceWeighting.m' ) ), '\Analyses'] )

%% Loading Analyses
Directory = dir( fullfile('*.mat') );

Endurance = [];
AutoX     = [];

for i = length(Directory) : -1 : 1
    if isempty( [regexp(Directory(i).name, 'Endurance'), regexp(Directory(i).name, 'AutoX')] )
        Directory(i) = [];
    elseif regexp(Directory(i).name, 'Endurance', 'once')
        if isempty(Endurance)
            Endurance        = load( Directory(i).name ); 
        else
            Endurance(end+1) = load( Directory(i).name );
        end
    elseif regexp(Directory(i).name, 'AutoX', 'once')
        if isempty(AutoX)
            AutoX        = load( Directory(i).name ); 
        else
            AutoX(end+1) = load( Directory(i).name );
        end
    end
end

%% Importance Weighting 
BinEdges = Endurance(1).Histogram.BinEdges;

EnduranceBinVals = 0;
for i = 1 : length(Endurance)
    EnduranceBinVals = EnduranceBinVals + ...
        Endurance(i).Histogram.BinCounts ./ length(Endurance(i).Histogram.Data); 
end
EnduranceBinVals = EnduranceBinVals .* 275 / length( Endurance );
EnduranceStr = 275 - sum( EnduranceBinVals );

AutoXBinVals = 0;
for i = 1 : length(AutoX)
    AutoXBinVals = AutoXBinVals + ...
        AutoX(i).Histogram.BinCounts ./ length(AutoX(i).Histogram.Data); 
end
AutoXBinVals = AutoXBinVals .* 125 / length( AutoX );
AutoXStr = 125 - sum( AutoXBinVals );

SkidpadBinVals = zeros( size(AutoXBinVals) );
SkidpadBinVals( find( 18.25/2 < BinEdges, 1 ) - 1 ) = 75;

figure;
histogram( 'BinEdges', BinEdges, 'BinCount', EnduranceBinVals + AutoXBinVals + SkidpadBinVals, ...
    'FaceColor', [0 1 0], 'FaceAlpha', 1 );
hold on;
histogram( 'BinEdges', BinEdges, 'BinCount', EnduranceBinVals + AutoXBinVals, ...
    'FaceColor', [1 0 0], 'FaceAlpha', 1 );
histogram( 'BinEdges', BinEdges, 'BinCount', EnduranceBinVals, ...
    'FaceColor', [0 0 1], 'FaceAlpha', 1 );

plot( [4.5, 4.5], [0 80], 'k--' );
plot( [75 , 75 ], [0 80], 'k--' );

title( ['Point Distribution on Cornering Radii: ', ...
    num2str( AutoXStr+EnduranceStr+100, 3), ' Points for Straights (> 200 [m])'] ) 
ylabel( 'Associated Points [pts]' ); ylim([0 80]);
xlabel( 'Cornering Radius [m]' ); xlim([0, 200]);

legend( {'Skidpad', 'Autocross', 'Endurance'} );
