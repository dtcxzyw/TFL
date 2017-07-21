uniform int u_mapSize;
uniform float u_bias;
uniform sampler2D u_shadowMap;
float unpackDepth(const in vec4 rgba_depth)
{
    const vec4 bit_shift = vec4(1.0/(256.0*256.0*256.0), 1.0/(256.0*256.0), 1.0/256.0, 1.0);
    float depth = dot(rgba_depth, bit_shift);
    return depth;
}
float getShadowValue()
{
    if(u_mapSize==1)
        return 1.0;
        
    vec3 projCoords = (v_pos.xyz/v_pos.w)*0.5+0.5;
    float currentDepth = projCoords.z;
    #ifndef OPENGL_ES
        vec2 mapSize = textureSize(u_shadowMap, 0);
    #endif
    if(projCoords.x<0.0 || projCoords.x>1.0 || projCoords.y<0.0 || projCoords.y>1.0 
        || projCoords.z>1.0 || projCoords.z<0.0)
            return 1.0;
	
    #ifdef OPENGL_ES
		float depth = unpackDepth(texture2D(u_shadowMap, projCoords.xy));
        float shadow = currentDepth-u_bias > depth ? 0.2 : 1.0;        
    #else
        float shadow=0.0;
        vec2 texelSize = 1.0/mapSize;
        for(int x = -1; x <= 1; ++x)
        {
            for(int y = -1; y <= 1; ++y)
            {
                float depth = unpackDepth(texture2D(u_shadowMap, projCoords.xy + vec2(x, y) * texelSize));
                shadow += currentDepth-u_bias > depth ? 0.2 : 1.0;        
            }    
        }
        shadow/=9.0;
    #endif
    return shadow;
}
