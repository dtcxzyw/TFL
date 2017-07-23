attribute vec4 a_position;
attribute vec2 a_texCoord;
#ifdef WATER
uniform mat4 u_viewProjectionMatrix;
uniform mat4 u_model;
#else
uniform mat4 u_worldViewProjectionMatrix;
#endif

varying vec2 v_texCoord;
#ifdef WATER
varying vec3 v_pos;
#endif 

void main()
{
    v_texCoord = a_texCoord;
#ifdef WATER
	vec4 pos=u_model*a_position;
	pos.y=-pos.y;
	v_pos = pos.xyz;
	pos=u_viewProjectionMatrix*pos;
#else
    vec4 pos = u_worldViewProjectionMatrix * a_position;
#endif
    gl_Position = pos.xyww;
}
