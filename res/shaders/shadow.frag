uniform sampler2D u_shadowMap;
uniform int u_mapSize;
float getShadow()
{
	float shadow=0.0;
    vec3 projCoords = (v_pos.xyz/v_pos.w)*0.5+0.5;
	float currentDepth = projCoords.z;
	#ifdef OPENGL_ES
		vec2 mapSize = vec2(u_mapSize,u_mapSize);
	#else
		vec2 mapSize = textureSize(u_shadowMap, 0);
	#endif
	if(projCoords.x<0.0 || projCoords.x>1.0 || projCoords.y<0.0 || projCoords.y>1.0 
		|| projCoords.z>1.0 || projCoords.z<0.0 ||
		#ifdef OPENGL_ES
			u_mapSize==1
		#else
			int(mapSize.x)!=u_mapSize
		#endif
		)
			return 1.0;
	vec2 texelSize = 1.0/mapSize;
	for(int x = -1; x <= 1; ++x)
	{
		for(int y = -1; y <= 1; ++y)
		{
			float pcfDepth = texture2D(u_shadowMap, projCoords.xy + vec2(x, y) * texelSize).
			#ifdef OPENGL_ES
			r
			#else
			a
			#endif
			; 
			shadow += currentDepth-0.0025 > pcfDepth ? 0.2 : 1.0;        
		}    
	}
	shadow/=9.0;
    return shadow;
}
