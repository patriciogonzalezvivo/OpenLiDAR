
#ifdef GL_ES
precision mediump float;
#endif

uniform sampler2D   u_tex0;
uniform mat4        u_modelViewProjectionMatrix;
uniform float       u_time;

attribute vec4      a_position;
#ifdef MODEL_VERTEX_NORMAL
attribute vec3      a_normal;
#endif
attribute vec2      a_texcoord;

varying vec4        v_position;
varying vec4        v_color;
varying vec2        v_texcoord;

#include "math/const.glsl"
#include "math/rotate4dX.glsl"
#include "math/rotate4dY.glsl"
#include "color/space/rgb2depth.glsl"
#include "color/space/rgb2hsv.glsl"

void main(void) {
    v_position = a_position;
    v_texcoord = a_texcoord;

    float time = u_time * 0.5;
    mat4 turn = rotate4dY(time * 0.25);
    mat4 flip = rotate4dX(PI * .5);
    v_position = turn * flip * v_position;

    vec3 normal = vec3(0.0);
    #ifdef MODEL_VERTEX_NORMAL
    normal = (turn * flip * vec4(a_normal, 1.)).xyz;
    #else
    normal = normalize(v_position.xyz);
    #endif

    vec3 color = texture2D(u_tex0, vec2(v_texcoord)).rgb;
    v_color = vec4(color, 1.0);
    float depth = rgb2depth(color);
    depth = rgb2hsv(color).x;
    v_position.xyz = normal * depth * 3.;
    
    gl_PointSize = 1.0;
    gl_Position = u_modelViewProjectionMatrix * v_position;
}
