
#ifdef GL_ES
precision mediump float;
#endif

uniform vec3    u_camera;
uniform vec3    u_light;
uniform vec2    u_resolution;

varying vec4    v_position;
varying vec4    v_color;
varying vec3    v_normal;

void main(void) {
    gl_FragColor = v_color;
}

