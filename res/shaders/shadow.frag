uniform sampler2D u_shadowMap;
uniform int u_mapSize;
uniform float u_bias;
float getShadow()
{
    if(u_mapSize==1)
        return 1.0;
        
    vec3 projCoords = (v_pos.xyz/v_pos.w)*0.5+0.5;
    float currentDepth = projCoords.z;
    #ifndef OPENGL_ES
        vec2 mapSize = textureSize(u_shadowMap, 0);
    #endif
    if(projCoords.x<0.0 || projCoords.x>1.0 || projCoords.y<0.0 || projCoords.y>1.0 
        || projCoords.z>1.0 || projCoords.z<0.0
        #ifndef OPENGL_ES
            || int(mapSize.x)!=u_mapSize
        #endif
        )
            return 1.0;
    #ifdef OPENGL_ES
        float depth=texture2D(u_shadowMap, projCoords.xy).r;
        float shadow = currentDepth-u_bias > depth ? 0.2 : 1.0;        
    #else
        float shadow=0.0;
        vec2 texelSize = 1.0/mapSize;
        for(int x = -1; x <= 1; ++x)
        {
            for(int y = -1; y <= 1; ++y)
            {
                float pcfDepth = texture2D(u_shadowMap, projCoords.xy + vec2(x, y) * texelSize).a;
                shadow += currentDepth-u_bias > pcfDepth ? 0.2 : 1.0;        
            }    
        }
        shadow/=9.0;
    #endif
    return shadow;
}
