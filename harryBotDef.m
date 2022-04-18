
function mrobot=harryBotDef
%%
% L1 = Link('d', 0, 'a', 1, 'alpha', pi/2)
% L2 = Link('d', 0, 'a', 1, 'alpha', 0)
% L3 = Link('d', 0, 'a', 1, 'alpha', 0)
% clc
% close all
% clear
L1 = Link('d', 0.680, 'a', 0.200, 'alpha', pi/2)
L2 = Link('d', 0.000, 'a', 0.890+0.200, 'alpha', 0,'offset',pi/2)
L3 = Link('d', 0.000, 'a', 1.020, 'alpha', -pi/2,'offset',-pi/2)

% L1.m = 50.2255;
% L2.m = 34.1066;
% L3.m = 48.2736;
mfac = 1;

L1.m = 50.2255/mfac;
L2.m = 34.1066/mfac;
L3.m = 48.2736/mfac;
% 
L1.I = [12.6201	12.4845	10.4569]
L2.I = ([2.6964	3.1055	0.746])
L3.I = ([9.1067	9.0479	0.478])
% factor = 10
% L1.I = ([9.1067	9.0479	0.478])/factor
% L2.I = ([9.1067	9.0479	0.478])/factor
% L3.I = ([9.1067	9.0479	0.478])/factor

% L1.r = 12

mrobot = SerialLink([L1 L2 L3], 'name', 'harry robot')
end
