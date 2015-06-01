%% xyz 2015 ¶ùÍ¯½Ú

%% ·½ÏòÓàÏÒ¾ØÕó  -> ×ËÌ¬½Ç

function Attitude = C2Attitude( Cwb,NavigationFrame )


Qwb = C2Q(Cwb);
Attitude = Q2Attitude( Qwb,NavigationFrame ) ;
