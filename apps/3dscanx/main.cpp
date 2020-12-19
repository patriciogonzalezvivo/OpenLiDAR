#include "OpenLiDAR.h"

#include "ada/window.h"
#include "ada/gl/gl.h"
#include "ada/gl/mesh.h"
#include "ada/gl/shader.h"

ada::Mesh rect (float _x, float _y, float _w, float _h) {
    float x = _x-1.0f;
    float y = _y-1.0f;
    float w = _w*2.0f;
    float h = _h*2.0f;

    ada::Mesh mesh;
    mesh.addVertex(glm::vec3(x, y, 0.0));
    // mesh.addColor(glm::vec4(1.0));
    // mesh.addNormal(glm::vec3(0.0, 0.0, 1.0));
    mesh.addTexCoord(glm::vec2(0.0, 0.0));

    mesh.addVertex(glm::vec3(x+w, y, 0.0));
    // mesh.addColor(glm::vec4(1.0));
    // mesh.addNormal(glm::vec3(0.0, 0.0, 1.0));
    mesh.addTexCoord(glm::vec2(1.0, 0.0));

    mesh.addVertex(glm::vec3(x+w, y+h, 0.0));
    // mesh.addColor(glm::vec4(1.0));
    // mesh.addNormal(glm::vec3(0.0, 0.0, 1.0));
    mesh.addTexCoord(glm::vec2(1.0, 1.0));

    mesh.addVertex(glm::vec3(x, y+h, 0.0));
    // mesh.addColor(glm::vec4(1.0));
    // mesh.addNormal(glm::vec3(0.0, 0.0, 1.0));
    mesh.addTexCoord(glm::vec2(0.0, 1.0));

    mesh.addIndex(0);   mesh.addIndex(1);   mesh.addIndex(2);
    mesh.addIndex(2);   mesh.addIndex(3);   mesh.addIndex(0);

    return mesh;
}

const std::string vert = R"(
#ifdef GL_ES
precision mediump float;
#endif

attribute vec4 a_position;
attribute vec2 a_texcoord;

varying vec4 v_position;
varying vec2 v_texcoord;

void main(void) {
    v_position =  a_position;
    v_texcoord = a_texcoord;
    
    gl_Position = v_position;
}
)";

const std::string frag = R"(
#ifdef GL_ES
precision mediump float;
#endif

uniform vec2    u_resolution;
uniform float   u_time;

varying vec2    v_texcoord;

void main(void) {
    vec3 color = vec3(1.0);
    vec2 st = gl_FragCoord.xy/u_resolution.xy;
    st = v_texcoord;
    
    color = vec3(st.x,st.y,abs(sin(u_time)));
    
    gl_FragColor = vec4(color, 1.0);
}
)";

// Main program
//============================================================================
int main(int argc, char **argv){
    OpenLiDAR scanner;
    OpenLiDARSettings settings;

    std::string filename = "pcl.ply";
    float toDegree = 359.0f;    // Full loop
    float atSpeed = 0.75f;      // 75% of speed

    // Initialize openGL context
    ada::initGL(argc, argv);
    glm::ivec2 screenSize = ada::getScreenSize();
    ada::setWindowSize(screenSize.x, screenSize.y);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    ada::Vbo* billboard_vbo = rect(0.0,0.0,1.0,1.0).getVbo();
    ada::Shader shader;
    shader.load(frag, vert);
    shader.use();
    
    // Render Loop
    while( ada::isGL() ) {
        glClear(GL_COLOR_BUFFER_BIT);

        // Update
        ada::updateGL();

        shader.setUniform("u_resolution", (float)ada::getWindowWidth(), (float)ada::getWindowHeight() );
        shader.setUniform("u_time", (float)ada::getTime());

        billboard_vbo->render( &shader );

        ada::renderGL();
        ada::updateViewport();
    }

    ada::closeGL();

    return 0;
}

// Events
//============================================================================
void ada::onKeyPress (int _key) {
}

void ada::onMouseMove(float _x, float _y) {
}

void ada::onMouseClick(float _x, float _y, int _button) {
}

void ada::onScroll(float _yoffset) {
}

void ada::onMouseDrag(float _x, float _y, int _button) {
}

void ada::onViewportResize(int _newWidth, int _newHeight) {
}