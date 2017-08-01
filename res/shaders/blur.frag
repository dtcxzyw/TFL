#ifdef OPENGL_ES
#ifdef GL_FRAGMENT_PRECISION_HIGH
precision highp float;
#else
precision mediump float;
#endif
#endif

uniform sampler2D u_texture;
uniform vec2 u_offset;

varying vec2 v_texCoord;

const int r=4;

void main()
{
	vec4 res=vec4(0.0,0.0,0.0,0.0);
	for(int i=0;i<r;++i)
        res+=(texture2D(u_texture, v_texCoord-u_offset*i)+texture2D(u_texture, v_texCoord-u_offset*i))*(r-i);
	res/=r*(r+1)*2.0;
	gl_FragColor=res;
}
