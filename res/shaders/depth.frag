#ifdef OPENGL_ES
#ifdef GL_FRAGMENT_PRECISION_HIGH
precision highp float;
#else
precision mediump float;
#endif
#endif

vec4 packDepth(const in float depth)
{
    const vec4 bit_shift = vec4(256.0*256.0*256.0, 256.0*256.0, 256.0, 1.0);
    const vec4 bit_mask  = vec4(0.0, 1.0/256.0, 1.0/256.0, 1.0/256.0);
    vec4 res = fract(depth * bit_shift);
    res -= res.xxyz * bit_mask;
    return res;
}

void main()
{      
    gl_FragColor=packDepth(gl_FragCoord.z); 
}
