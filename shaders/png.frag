

#ifdef GL_ES
precision mediump float;
#endif

uniform sampler2D   u_tex0;
uniform vec2        u_tex0Resolution;

uniform vec3        u_camera;
uniform vec2        u_resolution;

varying vec4        v_position;
varying vec4        v_color;

#ifdef MODEL_VERTEX_NORMAL
varying vec3        v_normal;
#endif

#ifdef MODEL_VERTEX_TEXCOORD
varying vec2        v_texcoord;
#endif

void main(void) {
    vec4 color = v_color;
    vec2 st = gl_FragCoord.xy/u_resolution.xy;

    gl_FragColor = color;
}

