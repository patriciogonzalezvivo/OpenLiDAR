#ifndef FNC_RGB2DEPTH
#define FNC_RGB2DEPTH
float rgb2depth(vec3 RGB) {
    float depth = 0.0; 

    float RGBMin = min(min(RGB.r, RGB.g), RGB.b);
    float RGBMax = max(max(RGB.r, RGB.g), RGB.b);
    float RGBMaxMinDiff = RGBMax - RGBMin;
    float l = (RGBMax + RGBMin) * 0.5;
    float s = 0.0;
    if (l < 0.5)
        s = RGBMaxMinDiff / (RGBMax + RGBMin);
    else
        s = RGBMaxMinDiff / (2.0 - RGBMax - RGBMin);

    vec3 d = (((RGBMax - RGB) * .1666666) + (RGBMaxMinDiff * .5)) / RGBMaxMinDiff;

    if (RGB.r == RGBMax)
        depth = d.b - d.g;
    else
        if (RGB.g == RGBMax)
            depth = (1.0 / 3.0) + d.r - d.b;
        else
            if (RGB.b == RGBMax)
                depth = (2.0 / 3.0) + d.g - d.r;

    depth = 1.-depth;
    depth *= step(.5, s);
    return depth;
}
#endif