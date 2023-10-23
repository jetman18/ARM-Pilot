#ifndef __NAVIGATION__
#define __NAVIGATION__

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
float range360(float deg){
    if(deg < 0)
        deg = 360 + deg;
    else if (deg > 359)
        deg = deg - 360;
    return deg;
}
float range180(float val){
    if(val > 180)
        val = val - 360;
    else if (val < -180)
        val = val + 360;
    return val;
}

#endif