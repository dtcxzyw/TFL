#ifdef OPENGL_ES
varying vec2 v_texCoord;
void main()
{
}
#else
attribute vec2 a_position;
attribute vec2 a_texCoord;

varying vec2 v_texCoord;

void main()
{
    gl_Position = vec4(a_position,0.0f,1.0f);
    v_texCoord = a_texCoord;
}
#endif
