attribute vec4 a_position;
uniform mat4 u_matrix;
varying vec2 v_texCoord;

void main()
{
    gl_Position = u_matrix * a_position;
	v_texCoord=gl_Position.xy/gl_Position.w*0.5+0.5;
}
