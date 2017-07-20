attribute vec4 a_position;
attribute vec2 a_texCoord;
uniform mat4 u_worldViewProjectionMatrix;
varying vec2 v_texCoord;

void main()
{
    v_texCoord = a_texCoord;
    vec4 pos = u_worldViewProjectionMatrix * a_position;
    gl_Position = pos.xyww;
}
