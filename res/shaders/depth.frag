void main()
{      
#ifndef OPENGL_ES
	gl_FragColor=vec4(1.0,1.0,1.0,gl_FragCoord.z); 
#endif
}
