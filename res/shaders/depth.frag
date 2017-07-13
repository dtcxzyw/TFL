#ifdef OPENGL_ES
#ifdef GL_FRAGMENT_PRECISION_HIGH
precision highp float;
#else
precision mediump float;
#endif
#endif


void main()
{      
#ifdef OPENGL_ES
	int a=int(gl_FragCoord.z*256.0*256.0*256.0);
	gl_FragColor.z=float(a-a/256*256)/256.0;
	a/=256;
	gl_FragColor.y=float(a-a/256*256)/256.0;
	a/=256;
	gl_FragColor.x=float(a)/256.0;
	gl_FragColor.w=1.0;
	
#else
    gl_FragColor=vec4(1.0,1.0,1.0,gl_FragCoord.z); 
#endif
}
