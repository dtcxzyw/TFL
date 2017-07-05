float getShadow()
{
	float shadow=0.0;
    vec3 projCoords = (v_pos.xyz/v_pos.w)*0.5+0.5;
	if(projCoords.x<0.0 || projCoords.x>1.0 || projCoords.y<0.0 || projCoords.y>1.0 || projCoords.z>1.0)
		return 1.0;
    float currentDepth = projCoords.z;
	vec2 texelSize = 1.0 / textureSize(u_shadowMap, 0);
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
