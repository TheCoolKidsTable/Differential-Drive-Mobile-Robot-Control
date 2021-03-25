function wS = skidNormalForce(param)

m = param.m;
g = param.g;
c = param.c;
s = param.s;
l = param.l;

wS = m*g*(l-c)/(s+l);
