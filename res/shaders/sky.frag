#ifdef OPENGL_ES
#ifdef GL_FRAGMENT_PRECISION_HIGH
precision highp float;
#else
precision mediump float;
#endif
#endif

varying vec2 v_texCoord;
uniform sampler2D u_diffuseTexture;
void main()
{
	gl_FragColor=texture2D(u_diffuseTexture, v_texCoord);
}
