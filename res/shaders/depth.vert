attribute vec4 a_position;
uniform mat4 u_matrix;
uniform mat4 u_model;

void main()
{
    gl_Position = u_matrix * u_model * a_position;
}
