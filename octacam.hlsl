#define PI 3.14159

#define POSTERIZE(v, steps) (floor((v) * steps) / (steps))

float invLerp(float from, float to, float value){
  return clamp((value - from) / (to - from),0,1);
}

float remap(float iMin, float iMax, float oMin, float oMax, float value) {
    return lerp(oMin, oMax, invLerp(iMin,iMax,value));
}

float3 lerpVec(float3 a, float3 b, float t) {
    return float3(
        lerp(a.x,b.x,t),
        lerp(a.y,b.y,t),
        lerp(a.z,b.z,t)
    );
}

float3 remapVec(float iMin, float iMax, float3 oMin, float3 oMax, float value) {
    return lerp(oMin, oMax, invLerp(iMin,iMax,value));
}

float3 Barycentric(float3 p, float3 a, float3 b, float3 c)
{
    float3 v0 = b - a, v1 = c - a, v2 = p - a;
    float den = v0.x * v1.y - v1.x * v0.y;
    float v = (v2.x * v1.y - v1.x * v2.y) / den;
    float w = (v0.x * v2.y - v2.x * v0.y) / den;
    float u = 1.0f - v - w;
    return float3(u,v,w);
}

float sdOctahedron( float3 p, float s)
{
  p = abs(p);
  return (p.x+p.y+p.z-s)*0.57735027;
}

float3 rotateX(float3 p, float angle) {
    return float3x3(1,0,0,0,cos(angle),sin(-angle),0,sin(angle),cos(angle)) * p;
}
float3 rotateY(float3 p, float angle) {
    return float3x3(cos(angle),0,sin(angle),0,1,0,sin(-angle),0,cos(angle)) * p;
}
float3 rotateZ(float3 p, float angle) {
    return float3x3(cos(angle),sin(-angle),0,sin(angle),cos(angle),0,0,0,1) * p;
}

float sdShape(float3 p, out float3 octaPos) {
    p -= float3(0,.1,3.2);
    p = rotateX(p,.25);
    p = rotateY(p,builtin_elapsed_time/4);
    octaPos = p;
    return sdOctahedron(p,1);
}

float3 getNormal(float3 p) {
    float3 octaPos;
    float d = sdShape(p, octaPos);
    float2 e = float2(.001,0);
    float3 n = d - float3(
        sdShape(p+e.xyy,octaPos),
        sdShape(p+e.yxy,octaPos),
        sdShape(p+e.yyx,octaPos)
    );
    return normalize(n);
}

float getLight(float3 p) {
    float3 dirLight = normalize(float3(0,0,1));
    return max(.05,dot(getNormal(p), dirLight));
}

float2 pointToUv(float3 p) {
    
    
    p = abs(p);

    p = p - dot(p,float3(1,1,1))/3*float3(1,1,1);

    float2 uv = float2(dot(normalize(float3(-1,0,1)),p), dot(normalize(float3(-.5,1,-.5)),p));

    float yMax = dot(float3(0,1,0),normalize(float3(-.5,1,-.5)));
    float yMin = dot(float3(.5,0,.5),normalize(float3(-.5,1,-.5)));

    float xMin = dot(float3(1,0,0),normalize(float3(-1,0,1)));
    float xMax = dot(float3(0,0,1),normalize(float3(-1,0,1)));

    uv.x = remap(xMin, xMax, 0, 1, uv.x);
    uv.y = remap(yMin, yMax, 1, 0, uv.y);

    return uv;
}

#define STEP_LIMIT 100
#define SURFACE_DIST .001
#define MAX_DIST 100
float4 raymarch(float3 ro, float3 rd) {
    float3 p = ro;
    float3 octaPos;
    for (int i = 0; i < STEP_LIMIT; i++) {
        float dist = sdShape(p,octaPos);
        if (dist < SURFACE_DIST) break;
        if (dist > MAX_DIST) return float4(0,0,0,0);
        p += rd * dist;
    }

    float3 bary = Barycentric(abs(octaPos), float3(1,0,0),float3(0,1,0),float3(0,0,1));

    float minEdge = min(bary.x,min(bary.y,bary.z));

    bool isEdge = minEdge < .03;

    float3 albedo = !isEdge ? image.Sample(builtin_texture_sampler, pointToUv(octaPos)).xyz : float3(.5,1,1);

    albedo = isEdge ? albedo : remapVec(0,.08,float3(.5,1,1),albedo,minEdge);

    float light = getLight(p);

    if (isEdge) light = max(.85,light);

    // float3 normal = getNormal(p);

    return float4(albedo * light,1);
    // return float4(getNormal(p)/.5+.5,1);
}

float4 render(float2 uv) {
    // sample the source texture and return its color to be displayed
    float3 ro = float3(0);
    float3 rd = normalize(float3((uv-float2(.5))*2/3, 1));
    float4 col = raymarch(ro,rd);
    return col;
}
