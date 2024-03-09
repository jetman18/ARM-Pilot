#ifndef __NAVIGATION__
#define __NAVIGATION__

#define MSTOSEC(x) (x*0.001f)
#define Fase 0
#define True 1


static int sign(int val){
    if (val > 0)
       val = 1;
    else if (val < 0)
       val = -1;
    return val;
}
static int signf(float val){
    int si = 0;
    if (val > 0)
       si = 1;
    else if (val < 0)
       si = -1;
    return si;
}
static float range360(float deg){
    if(deg < 0)
        deg = 360 + deg;
    else if (deg > 359)
        deg = deg - 360;
    return deg;
}
static float range180(float val){
    if(val > 180)
        val = val - 360;
    else if (val < -180)
        val = val + 360;
    return val;
}

#endif