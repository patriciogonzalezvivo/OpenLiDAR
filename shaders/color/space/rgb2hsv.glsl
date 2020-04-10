/*
function: rgb2hsv
author: Sam Hocevar
description: pass a color in RGB and get HSB color. From http://lolengine.net/blog/2013/07/27/rgb-to-hsv-in-glsl
use: rgb2hsv(<vec3|vec4> color)
options:
  RGB2HSV_MIX: use mix() instead of ternary operatiors. Default is off
tests:
  back_and_forth:
    description: Compare the original RGB to the converted to HSV and back to RGB.
    include: ['lib/color/space/hsv2rgb.glsl']
    frag: |
      vec4 color_org = texture2D(u_inputTex, st);
      vec4 conv = rgb2hsv(color_org);
      vec4 back = hsv2rgb(conv);
      color = mix(color_org, back, step(.5, (st.x + st.y)*.5));
    uniform: 
      u_inputTex: face.png
  back_and_forth_mix:
    description: Compare the original RGB to the converted to HSV and back to RGB.
    include: ['lib/color/space/hsv2rgb.glsl']
    define: ['RGB2HSV_MIX']
    frag: |
      vec4 color_org = texture2D(u_inputTex, st);
      vec4 conv = rgb2hsv(color_org);
      vec4 back = hsv2rgb(conv);
      color = mix(color_org, back, step(.5, (st.x + st.y)*.5));
    uniform: 
      u_inputTex: face.png
*/

#ifndef FNC_RGB2HSV
#define FNC_RGB2HSV
vec3 rgb2hsv(in vec3 c) {
    vec4 K = vec4(0., -.33333333333333333333, .6666666666666666666, -1.);

#ifdef RGB2HSV_MIX
    vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));
#else
    vec4 p = c.g < c.b ? vec4(c.bg, K.wz) : vec4(c.gb, K.xy);
    vec4 q = c.r < p.x ? vec4(p.xyw, c.r) : vec4(c.r, p.yzx);
#endif

    float d = q.x - min(q.w, q.y);
    float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6. * d + e)), 
                d / (q.x + e), 
                q.x);
}

vec4 rgb2hsv(in vec4 c) {
    return vec4(rgb2hsv(c.rgb), c.a);
}
#endif