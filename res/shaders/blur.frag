#ifdef OPENGL_ES
#ifdef GL_FRAGMENT_PRECISION_HIGH
precision highp float;
#else
precision mediump float;
#endif
#endif

uniform sampler2D u_texture;
uniform vec2 u_pixel;

varying vec2 v_texCoord;

const int r=10,sum=r*(r+1);

void main()
{
    vec4 res=vec4(0.0,0.0,0.0,0.0);
    
    for(int i=0;i<r;++i)
        res+=(texture2D(u_texture, v_texCoord-u_pixel*float(i))+texture2D(u_texture, v_texCoord-u_pixel*float(i)))*float(r-i);
    res/=float(sum);
    gl_FragColor=res;
}
