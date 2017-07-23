#ifdef OPENGL_ES
#ifdef GL_FRAGMENT_PRECISION_HIGH
precision highp float;
#else
precision mediump float;
#endif
#endif

varying vec2 v_texCoord;
#ifdef WATER
varying vec3 v_pos;
#endif 
uniform sampler2D u_diffuseTexture;
void main()
{
	#ifdef WATER
	if(v_pos.y>=0.0)discard;
	#endif
	
	gl_FragColor=texture2D(u_diffuseTexture, v_texCoord);
}
