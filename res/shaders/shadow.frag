float getShadow()
{
    vec3 projCoords = v_pos.xyz / v_pos.w;
    projCoords = projCoords * 0.5 + 0.5;
    float closestDepth = texture(u_shadowMap, projCoords.xy).r; 
    float currentDepth = projCoords.z;
    float shadow = currentDepth > closestDepth  ? 0.0 : 1.0;
    return shadow;
}
