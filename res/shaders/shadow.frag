uniform sampler2D u_shadowMap;
uniform int u_mapSize;
float getShadow()
{
	float shadow=0.0;
    vec3 projCoords = (v_pos.xyz/v_pos.w)*0.5+0.5;
	float currentDepth = projCoords.z;
	vec2 mapSize = textureSize(u_shadowMap, 0);
	if(projCoords.x<0.0 || projCoords.x>1.0 || projCoords.y<0.0 || projCoords.y>1.0 
		|| projCoords.z>1.0 || projCoords.z<0.0 || mapSize!=u_mapSize)
			return 1.0;
	vec2 texelSize = 1.0/mapSize;
	for(int x = -1; x <= 1; ++x)
	{
		for(int y = -1; y <= 1; ++y)
		{
			float pcfDepth = texture(u_shadowMap, projCoords.xy + vec2(x, y) * texelSize).r; 
			shadow += currentDepth-0.0025 > pcfDepth ? 0.2 : 1.0;        
		}    
	}
	shadow/=9.0;
    return shadow;
}
