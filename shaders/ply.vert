
#ifdef GL_ES
precision mediump float;
#endif

uniform mat4    u_modelViewProjectionMatrix;
uniform mat4    u_projectionMatrix;
uniform mat4    u_viewMatrix;
uniform mat4    u_modelMatrix;

uniform vec3    u_camera;

uniform float   u_time;

attribute vec4  a_position;
varying vec4    v_position;

#ifdef MODEL_VERTEX_COLOR
attribute vec4  a_color;
#endif

varying vec4    v_color;

#ifdef MODEL_VERTEX_NORMAL
attribute vec3  a_normal;
varying vec3    v_normal;
#endif

#include "math/rotate4dY.glsl"

void main(void) {
    v_position = a_position;
    v_normal = a_normal;

    float time = u_time * 0.5;
    mat4 turn = rotate4dY(time * 0.25);
    vec3 normal = (turn * vec4(v_normal, 0.0)).xyz;
    float visibility = 0.2 + pow(1.0 - clamp(dot(normalize(u_camera) , normal), 0., 1.), 3.) * 0.8;

    v_position = turn * v_position;

#ifdef MODEL_VERTEX_COLOR
    v_color = a_color;
#else
    v_color = vec4(visibility);
#endif

    gl_PointSize = 1. + 2. * (1.0 - visibility);
    gl_Position = u_projectionMatrix * u_viewMatrix * v_position;
}
