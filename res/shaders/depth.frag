#ifdef OPENGL_ES
#ifdef GL_FRAGMENT_PRECISION_HIGH
precision highp float;
#else
precision mediump float;
#endif
#endif


void main()
{      
	gl_FragColor=vec4(gl_FragCoord.z,1.0,1.0,gl_FragCoord.z); 
}
