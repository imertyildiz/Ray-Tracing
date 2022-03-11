#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <stdlib.h>
#include <cmath>
#include <cstring>
typedef unsigned char RGB[3];

parser::Vec3f multS(parser::Vec3f a, float s)
{
    parser::Vec3f result;
    result.x = a.x * s;
    result.y = a.y * s;
    result.z = a.z * s;
    return result;
}
parser::Vec3i multSInteger(parser::Vec3i a, float s)
{
    parser::Vec3i result;
    result.x = a.x * s;
    result.y = a.y * s;
    result.z = a.z * s;
    return result;
}

parser::Vec3f divideS(parser::Vec3f a, float s)
{
    parser::Vec3f result;
    result.x = a.x / s;
    result.y = a.y / s;
    result.z = a.z / s;
    return result;
}

parser::Vec3f add(parser::Vec3f a, parser::Vec3f b)
{
    parser::Vec3f result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

parser::Vec3i addI(parser::Vec3i a, parser::Vec3i b)
{
    parser::Vec3i result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

parser::Vec3f subtract(parser::Vec3f a, parser::Vec3f b)
{
    parser::Vec3f result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

float dotProduct(parser::Vec3f vector1, parser::Vec3f vector2)
{
    // be careful about the order of the vectors
    float result;
    result = (vector1.x * vector2.x) + (vector1.y * vector2.y) + (vector1.z * vector2.z);
    return result;
}

parser::Vec3f crossProduct(parser::Vec3f a, parser::Vec3f b)
{
    parser::Vec3f result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}

parser::Ray generateRay(int i, int j, parser::Camera camera)
{
    parser::Ray result;
    float su, sv;
    parser::Vec3f m, q, s;

    su = (i + 0.5) * (camera.near_plane.y - camera.near_plane.x) / camera.image_width;
    sv = (j + 0.5) * (camera.near_plane.w - camera.near_plane.z) / camera.image_height;

    m = add(camera.position, multS(camera.gaze, camera.near_distance));

    q = add(m, add(multS(camera.upCrossMinusGaze, camera.near_plane.x), multS(camera.up, camera.near_plane.w)));

    s = add(q, add(multS(camera.upCrossMinusGaze, su), multS(camera.up, -sv)));

    result.origin = camera.position;
    result.direction = add(s, multS(camera.position, -1));

    return result;
}
parser::Vec3f normalize(parser::Vec3f a)
{
    return multS(a, 1.0 / sqrt(dotProduct(a, a)));
}

float intersectTriangleIsInFront(parser::Ray ray, parser::Vec3f a, parser::Vec3f b, parser::Vec3f c, parser::Vec3f normalOfTriangle)
{
    // #1 intersect the ray with the plane of the triangle
    // #1.1 find normal of the triangle
    float dividing, divisor;
    // #find t
    dividing = dotProduct(subtract(a, ray.origin), normalOfTriangle);
    divisor = dotProduct(ray.direction, normalOfTriangle);
    if (divisor == 0)
        return -1;
    else
        return dividing / divisor;
}

float intersectSphere(parser::Ray ray, parser::Sphere sphere, parser::Vec3f center)
{
    float A, B, C; //constants for the quadratic equation

    float delta;

    float t, t1, t2;

    C = (ray.origin.x - center.x) * (ray.origin.x - center.x) + (ray.origin.y - center.y) * (ray.origin.y - center.y) + (ray.origin.z - center.z) * (ray.origin.z - center.z) - sphere.radius * sphere.radius;

    B = 2 * ray.direction.x * (ray.origin.x - center.x) + 2 * ray.direction.y * (ray.origin.y - center.y) + 2 * ray.direction.z * (ray.origin.z - center.z);

    A = ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y + ray.direction.z * ray.direction.z;

    delta = B * B - 4 * A * C;

    if (delta < 0)
        return -1;
    else if (delta == 0)
    {
        t = -B / (2 * A);
    }
    else
    {
        delta = sqrt(delta);
        A = 2 * A;
        t1 = (-B + delta) / A;
        t2 = (-B - delta) / A;

        if (t1 < t2)
            t = t1;
        else
            t = t2;
    }

    return t;
}

bool intersectTriangle(float t, parser::Ray ray, parser::Vec3f a, parser::Vec3f b, parser::Vec3f c, float epsilon)
{
    parser::Vec3f p;
    bool result = false;
    parser::Vec3f vectorp, vectora, vectorb, vectorc;
    p = add(ray.origin, multS(ray.direction, t));
    // for point a
    vectorp = crossProduct(subtract(p, c), subtract(b, c));
    vectora = crossProduct(subtract(a, c), subtract(b, c));
    if (dotProduct(vectorp, vectora) >= 0)
        result = true;
    else
        return false;
    // for point b
    vectorp = crossProduct(subtract(p, a), subtract(c, a));
    vectorb = crossProduct(subtract(b, a), subtract(c, a));
    if (dotProduct(vectorp, vectorb) >= 0)
        result = true;
    else
        return false;
    // for point c
    vectorp = crossProduct(subtract(p, b), subtract(a, b));
    vectorc = crossProduct(subtract(c, b), subtract(a, b));
    if (dotProduct(vectorp, vectorc) >= 0)
        return true;
    else
        return false;
}

parser::Vec3i computeColor(parser::Ray ray, parser::Scene scene, int howManyTimes)
{
    parser::Vec3i color;
    if (howManyTimes > scene.max_recursion_depth)
    {
        color.x = 0;
        color.y = 0;
        color.z = 0;
        return color;
    }
    float minT = 999999;
    float t, dotProdWiNormal, dotProdNormalH;
    int i, minI, minJ;
    bool isIntersect = false;
    parser::Vec3f Wi, normal, P, H;
    color.x = scene.background_color.x;
    color.y = scene.background_color.y;
    color.z = scene.background_color.z;
    minI = -1;
    minJ = -1;
    int whichIsThat = -1; // 0 == sphere, 1 == triangle, 2 == mesh
    for (i = 0; i < scene.spheres.size(); i++)
    {
        t = intersectSphere(ray, scene.spheres[i], scene.vertex_data[scene.spheres[i].center_vertex_id - 1]);
        if (((double)minT) > ((double)t) && t >= 0)
        {
            minI = i;
            minT = t;
            whichIsThat = 0;
        }
    }
    for (i = 0; i < scene.triangles.size(); i++)
    {
        t = intersectTriangleIsInFront(ray, scene.vertex_data[scene.triangles[i].indices.v0_id - 1], scene.vertex_data[scene.triangles[i].indices.v1_id - 1], scene.vertex_data[scene.triangles[i].indices.v2_id - 1], scene.triangles[i].normalOfTriangle);
        if (((double)minT) > ((double)t) && t >= 0)
        {
            if (dotProduct(ray.direction, scene.triangles[i].normalOfTriangle) > ((double)0.0))
            {
                continue;
            }
            if (intersectTriangle(t, ray, scene.vertex_data[scene.triangles[i].indices.v0_id - 1], scene.vertex_data[scene.triangles[i].indices.v1_id - 1], scene.vertex_data[scene.triangles[i].indices.v2_id - 1], scene.shadow_ray_epsilon))
            {
                minI = i;
                minT = t;
                whichIsThat = 1;
            }
        }
    }
    for (i = 0; i < scene.meshes.size(); i++)
    {
        for (int j = 0; j < scene.meshes[i].faces.size(); j++)
        {
            t = intersectTriangleIsInFront(ray, scene.vertex_data[scene.meshes[i].faces[j].v0_id - 1], scene.vertex_data[scene.meshes[i].faces[j].v1_id - 1], scene.vertex_data[scene.meshes[i].faces[j].v2_id - 1], scene.meshes[i].normalOfTriangle[j]);
            if (((double)minT) > ((double)t) && t >= 0)
            {
                if (dotProduct(ray.direction, scene.meshes[i].normalOfTriangle[j]) > ((double)0.0))
                {
                    continue;
                }
                if (intersectTriangle(t, ray, scene.vertex_data[scene.meshes[i].faces[j].v0_id - 1], scene.vertex_data[scene.meshes[i].faces[j].v1_id - 1], scene.vertex_data[scene.meshes[i].faces[j].v2_id - 1], scene.shadow_ray_epsilon))
                {
                    minI = i;
                    minT = t;
                    minJ = j;
                    whichIsThat = 2;
                }
            }
        }
    }

    if (howManyTimes > 0 && whichIsThat == -1)
    {
        color.x = 0;
        color.y = 0;
        color.z = 0;
        return color;
    }

    if (whichIsThat == 0)
    {
        color.x = scene.materials[scene.spheres[minI].material_id - 1].ambient.x * scene.ambient_light.x;
        color.y = scene.materials[scene.spheres[minI].material_id - 1].ambient.y * scene.ambient_light.y;
        color.z = scene.materials[scene.spheres[minI].material_id - 1].ambient.z * scene.ambient_light.z;
        P = add(ray.origin, multS(ray.direction, minT));
        normal = add(P, multS(scene.vertex_data[scene.spheres[minI].center_vertex_id - 1], -1));
        normal = normalize(normal);
        for (int i = 0; i < scene.point_lights.size(); i++)
        {
            isIntersect = false;
            parser::Vec3f colorDiffuseFloat, colorSpecularFloat;
            parser::Vec3i colorDiffuseInteger, colorSpecularInteger;
            parser::Ray shadowRay;
            shadowRay.origin = add(P, multS(normal, scene.shadow_ray_epsilon));
            shadowRay.direction = subtract(scene.point_lights[i].position, P);
            for (int j = 0; j < scene.spheres.size(); j++)
            {
                t = intersectSphere(shadowRay, scene.spheres[j], scene.vertex_data[scene.spheres[j].center_vertex_id - 1]);
                if (t < (double)1.0 && t >= 0)
                {
                    isIntersect = true;
                }
            }
            for (int j = 0; j < scene.triangles.size(); j++)
            {
                t = intersectTriangleIsInFront(shadowRay, scene.vertex_data[scene.triangles[j].indices.v0_id - 1], scene.vertex_data[scene.triangles[j].indices.v1_id - 1], scene.vertex_data[scene.triangles[j].indices.v2_id - 1], scene.triangles[j].normalOfTriangle);
                if (t < (double)1.0 && t >= 0)
                {
                    if (intersectTriangle(t, shadowRay, scene.vertex_data[scene.triangles[j].indices.v0_id - 1], scene.vertex_data[scene.triangles[j].indices.v1_id - 1], scene.vertex_data[scene.triangles[j].indices.v2_id - 1], scene.shadow_ray_epsilon))
                    {
                        isIntersect = true;
                    }
                }
            }
            for (int j = 0; j < scene.meshes.size(); j++)
            {
                for (int k = 0; k < scene.meshes[j].faces.size(); k++)
                {
                    t = intersectTriangleIsInFront(shadowRay, scene.vertex_data[scene.meshes[j].faces[k].v0_id - 1], scene.vertex_data[scene.meshes[j].faces[k].v1_id - 1], scene.vertex_data[scene.meshes[j].faces[k].v2_id - 1], scene.meshes[j].normalOfTriangle[k]);
                    if (t < (double)1.0 && t >= 0)
                    {
                        if (intersectTriangle(t, shadowRay, scene.vertex_data[scene.meshes[j].faces[k].v0_id - 1], scene.vertex_data[scene.meshes[j].faces[k].v1_id - 1], scene.vertex_data[scene.meshes[j].faces[k].v2_id - 1], scene.shadow_ray_epsilon))
                        {
                            isIntersect = true;
                        }
                    }
                }
            }
            if (!isIntersect)
            {
                colorDiffuseFloat.x = scene.materials[scene.spheres[minI].material_id - 1].diffuse.x;
                colorDiffuseFloat.x *= (scene.point_lights[i].intensity.x / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorDiffuseFloat.y = scene.materials[scene.spheres[minI].material_id - 1].diffuse.y;
                colorDiffuseFloat.y *= (scene.point_lights[i].intensity.y / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorDiffuseFloat.z = scene.materials[scene.spheres[minI].material_id - 1].diffuse.z;
                colorDiffuseFloat.z *= (scene.point_lights[i].intensity.z / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                Wi = add(scene.point_lights[i].position, multS(P, -1));

                Wi = normalize(Wi);
                dotProdWiNormal = dotProduct(Wi, normal);
                if (dotProdWiNormal < 0)
                {
                    dotProdWiNormal = 0;
                }
                colorDiffuseFloat.x = colorDiffuseFloat.x * dotProdWiNormal;
                colorDiffuseFloat.y = colorDiffuseFloat.y * dotProdWiNormal;
                colorDiffuseFloat.z = colorDiffuseFloat.z * dotProdWiNormal;
                colorDiffuseInteger.x = (int)(colorDiffuseFloat.x + 0.5);
                colorDiffuseInteger.y = (int)(colorDiffuseFloat.y + 0.5);
                colorDiffuseInteger.z = (int)(colorDiffuseFloat.z + 0.5);
                color = addI(color, colorDiffuseInteger);
                H = add(Wi, normalize(add(ray.origin, multS(P, -1))));
                H = normalize(H);
                dotProdNormalH = dotProduct(normal, H);
                if (dotProdNormalH < 0)
                {
                    dotProdNormalH = 0;
                }
                dotProdNormalH = pow(dotProdNormalH, scene.materials[scene.spheres[minI].material_id - 1].phong_exponent);
                colorSpecularFloat.x = scene.materials[scene.spheres[minI].material_id - 1].specular.x;
                colorSpecularFloat.x *= (scene.point_lights[i].intensity.x / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorSpecularFloat.y = scene.materials[scene.spheres[minI].material_id - 1].specular.y;
                colorSpecularFloat.y *= (scene.point_lights[i].intensity.y / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorSpecularFloat.z = scene.materials[scene.spheres[minI].material_id - 1].specular.z;
                colorSpecularFloat.z *= (scene.point_lights[i].intensity.z / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorSpecularFloat.x = colorSpecularFloat.x * dotProdNormalH;
                colorSpecularFloat.y = colorSpecularFloat.y * dotProdNormalH;
                colorSpecularFloat.z = colorSpecularFloat.z * dotProdNormalH;
                colorSpecularInteger.x = (int)(colorSpecularFloat.x + 0.5);
                colorSpecularInteger.y = (int)(colorSpecularFloat.y + 0.5);
                colorSpecularInteger.z = (int)(colorSpecularFloat.z + 0.5);
                color = addI(color, colorSpecularInteger);
            }
        }
        if (scene.materials[scene.spheres[minI].material_id - 1].is_mirror)
        {
            parser::Ray mirrorRay; // is W0 negative ?
            parser::Vec3f wR;
            parser::Vec3i colorV2;
            wR = subtract(multS(multS(normal, 2), dotProduct(normal, multS(normalize(ray.direction), -1))), multS(normalize(ray.direction), -1));
            mirrorRay.origin = add(P, multS(normal, scene.shadow_ray_epsilon));
            mirrorRay.direction = wR;
            colorV2 = computeColor(mirrorRay, scene, ++howManyTimes);
            colorV2.x = (int)(colorV2.x * scene.materials[scene.spheres[minI].material_id - 1].mirror.x + 0.5);
            colorV2.y = (int)(colorV2.y * scene.materials[scene.spheres[minI].material_id - 1].mirror.y + 0.5);
            colorV2.z = (int)(colorV2.z * scene.materials[scene.spheres[minI].material_id - 1].mirror.z + 0.5);
            color = addI(color, colorV2);
        }
    }
    else if (whichIsThat == 1)
    {
        color.x = scene.materials[scene.triangles[minI].material_id - 1].ambient.x * scene.ambient_light.x;
        color.y = scene.materials[scene.triangles[minI].material_id - 1].ambient.y * scene.ambient_light.y;
        color.z = scene.materials[scene.triangles[minI].material_id - 1].ambient.z * scene.ambient_light.z;
        P = add(ray.origin, multS(ray.direction, minT));
        normal = scene.triangles[minI].normalOfTriangle;
        normal = normalize(normal);
        for (int i = 0; i < scene.point_lights.size(); i++)
        {
            float newT = 999999;
            isIntersect = false;
            parser::Vec3f colorDiffuseFloat, colorSpecularFloat;
            parser::Vec3i colorDiffuseInteger, colorSpecularInteger;
            parser::Ray shadowRay;
            shadowRay.origin = add(P, multS(normal, scene.shadow_ray_epsilon));
            shadowRay.direction = subtract(scene.point_lights[i].position, P);
            for (int j = 0; j < scene.spheres.size(); j++)
            {
                t = intersectSphere(shadowRay, scene.spheres[j], scene.vertex_data[scene.spheres[j].center_vertex_id - 1]);
                if (t < 1.0 && t >= 0)
                {
                    isIntersect = true;
                }
            }
            for (int j = 0; j < scene.triangles.size(); j++)
            {
                t = intersectTriangleIsInFront(shadowRay, scene.vertex_data[scene.triangles[j].indices.v0_id - 1], scene.vertex_data[scene.triangles[j].indices.v1_id - 1], scene.vertex_data[scene.triangles[j].indices.v2_id - 1], scene.triangles[j].normalOfTriangle);
                if (t < 1.0 && t >= 0)
                {
                    if (intersectTriangle(t, shadowRay, scene.vertex_data[scene.triangles[j].indices.v0_id - 1], scene.vertex_data[scene.triangles[j].indices.v1_id - 1], scene.vertex_data[scene.triangles[j].indices.v2_id - 1], scene.shadow_ray_epsilon))
                    {
                        isIntersect = true;
                    }
                }
            }
            for (int j = 0; j < scene.meshes.size(); j++)
            {
                for (int k = 0; k < scene.meshes[j].faces.size(); k++)
                {
                    t = intersectTriangleIsInFront(shadowRay, scene.vertex_data[scene.meshes[j].faces[k].v0_id - 1], scene.vertex_data[scene.meshes[j].faces[k].v1_id - 1], scene.vertex_data[scene.meshes[j].faces[k].v2_id - 1], scene.meshes[j].normalOfTriangle[k]);
                    if (t < 1.0 && t >= 0)
                    {
                        if (intersectTriangle(t, shadowRay, scene.vertex_data[scene.meshes[j].faces[k].v0_id - 1], scene.vertex_data[scene.meshes[j].faces[k].v1_id - 1], scene.vertex_data[scene.meshes[j].faces[k].v2_id - 1], scene.shadow_ray_epsilon))
                        {
                            isIntersect = true;
                        }
                    }
                }
            }
            if (!isIntersect)
            {
                colorDiffuseFloat.x = scene.materials[scene.triangles[minI].material_id - 1].diffuse.x;
                colorDiffuseFloat.x *= (scene.point_lights[i].intensity.x / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorDiffuseFloat.y = scene.materials[scene.triangles[minI].material_id - 1].diffuse.y;
                colorDiffuseFloat.y *= (scene.point_lights[i].intensity.y / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorDiffuseFloat.z = scene.materials[scene.triangles[minI].material_id - 1].diffuse.z;
                colorDiffuseFloat.z *= (scene.point_lights[i].intensity.z / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                Wi = add(scene.point_lights[i].position, multS(P, -1));

                Wi = normalize(Wi);
                dotProdWiNormal = dotProduct(Wi, normal);
                if (dotProdWiNormal < 0)
                {
                    dotProdWiNormal = 0;
                }
                colorDiffuseFloat.x = colorDiffuseFloat.x * dotProdWiNormal;
                colorDiffuseFloat.y = colorDiffuseFloat.y * dotProdWiNormal;
                colorDiffuseFloat.z = colorDiffuseFloat.z * dotProdWiNormal;
                colorDiffuseInteger.x = (int)(colorDiffuseFloat.x + 0.5);
                colorDiffuseInteger.y = (int)(colorDiffuseFloat.y + 0.5);
                colorDiffuseInteger.z = (int)(colorDiffuseFloat.z + 0.5);
                color = addI(color, colorDiffuseInteger);
                H = add(Wi, normalize(add(ray.origin, multS(P, -1))));
                H = normalize(H);
                dotProdNormalH = dotProduct(normal, H);
                if (dotProdNormalH < 0)
                {
                    dotProdNormalH = 0;
                }
                dotProdNormalH = pow(dotProdNormalH, scene.materials[scene.triangles[minI].material_id - 1].phong_exponent);
                colorSpecularFloat.x = scene.materials[scene.triangles[minI].material_id - 1].specular.x;
                colorSpecularFloat.x *= (scene.point_lights[i].intensity.x / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorSpecularFloat.y = scene.materials[scene.triangles[minI].material_id - 1].specular.y;
                colorSpecularFloat.y *= (scene.point_lights[i].intensity.y / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorSpecularFloat.z = scene.materials[scene.triangles[minI].material_id - 1].specular.z;
                colorSpecularFloat.z *= (scene.point_lights[i].intensity.z / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorSpecularFloat.x = colorSpecularFloat.x * dotProdNormalH;
                colorSpecularFloat.y = colorSpecularFloat.y * dotProdNormalH;
                colorSpecularFloat.z = colorSpecularFloat.z * dotProdNormalH;
                colorSpecularInteger.x = (int)(colorSpecularFloat.x + 0.5);
                colorSpecularInteger.y = (int)(colorSpecularFloat.y + 0.5);
                colorSpecularInteger.z = (int)(colorSpecularFloat.z + 0.5);
                color = addI(color, colorSpecularInteger);
            }
        }
        if (scene.materials[scene.triangles[minI].material_id - 1].is_mirror)
        {
            parser::Ray mirrorRay; // is W0 negative ?
            parser::Vec3f wR;
            parser::Vec3i colorV2;
            wR = subtract(multS(multS(normal, 2), dotProduct(normal, multS(normalize(ray.direction), -1))), multS(normalize(ray.direction), -1));
            mirrorRay.origin = add(P, multS(normal, scene.shadow_ray_epsilon));
            mirrorRay.direction = wR;
            colorV2 = computeColor(mirrorRay, scene, ++howManyTimes);
            colorV2.x = (int)(colorV2.x * scene.materials[scene.triangles[minI].material_id - 1].mirror.x + 0.5);
            colorV2.y = (int)(colorV2.y * scene.materials[scene.triangles[minI].material_id - 1].mirror.y + 0.5);
            colorV2.z = (int)(colorV2.z * scene.materials[scene.triangles[minI].material_id - 1].mirror.z + 0.5);
            color = addI(color, colorV2);
        }
    }
    else if (whichIsThat == 2)
    {
        color.x = scene.materials[scene.meshes[minI].material_id - 1].ambient.x * scene.ambient_light.x;
        color.y = scene.materials[scene.meshes[minI].material_id - 1].ambient.y * scene.ambient_light.y;
        color.z = scene.materials[scene.meshes[minI].material_id - 1].ambient.z * scene.ambient_light.z;
        P = add(ray.origin, multS(ray.direction, minT));
        normal = scene.meshes[minI].normalOfTriangle[minJ];
        normal = normalize(normal);
        for (int i = 0; i < scene.point_lights.size(); i++)
        {
            float newT = 99999;
            isIntersect = false;
            parser::Vec3f colorDiffuseFloat, colorSpecularFloat;
            parser::Vec3i colorDiffuseInteger, colorSpecularInteger;
            parser::Ray shadowRay;
            shadowRay.origin = add(P, multS(normal, scene.shadow_ray_epsilon));
            shadowRay.direction = subtract(scene.point_lights[i].position, P);
            for (int j = 0; j < scene.spheres.size(); j++)
            {
                t = intersectSphere(shadowRay, scene.spheres[j], scene.vertex_data[scene.spheres[j].center_vertex_id - 1]);
                if (t < 1.0 && t >= 0)
                {
                    isIntersect = true;
                }
            }
            for (int j = 0; j < scene.triangles.size(); j++)
            {
                t = intersectTriangleIsInFront(shadowRay, scene.vertex_data[scene.triangles[j].indices.v0_id - 1], scene.vertex_data[scene.triangles[j].indices.v1_id - 1], scene.vertex_data[scene.triangles[j].indices.v2_id - 1], scene.triangles[j].normalOfTriangle);
                if (t < 1.0 && t >= 0)
                {
                    if (intersectTriangle(t, shadowRay, scene.vertex_data[scene.triangles[j].indices.v0_id - 1], scene.vertex_data[scene.triangles[j].indices.v1_id - 1], scene.vertex_data[scene.triangles[j].indices.v2_id - 1], scene.shadow_ray_epsilon))
                    {
                        isIntersect = true;
                    }
                }
            }
            for (int j = 0; j < scene.meshes.size(); j++)
            {
                for (int k = 0; k < scene.meshes[j].faces.size(); k++)
                {
                    t = intersectTriangleIsInFront(shadowRay, scene.vertex_data[scene.meshes[j].faces[k].v0_id - 1], scene.vertex_data[scene.meshes[j].faces[k].v1_id - 1], scene.vertex_data[scene.meshes[j].faces[k].v2_id - 1], scene.meshes[j].normalOfTriangle[k]);
                    if (t < 1.0 && t >= 0)
                    {
                        if (intersectTriangle(t, shadowRay, scene.vertex_data[scene.meshes[j].faces[k].v0_id - 1], scene.vertex_data[scene.meshes[j].faces[k].v1_id - 1], scene.vertex_data[scene.meshes[j].faces[k].v2_id - 1], scene.shadow_ray_epsilon))
                        {
                            isIntersect = true;
                        }
                    }
                }
            }
            if (!isIntersect)
            {
                colorDiffuseFloat.x = scene.materials[scene.meshes[minI].material_id - 1].diffuse.x;
                colorDiffuseFloat.x *= (scene.point_lights[i].intensity.x / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorDiffuseFloat.y = scene.materials[scene.meshes[minI].material_id - 1].diffuse.y;
                colorDiffuseFloat.y *= (scene.point_lights[i].intensity.y / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorDiffuseFloat.z = scene.materials[scene.meshes[minI].material_id - 1].diffuse.z;
                colorDiffuseFloat.z *= (scene.point_lights[i].intensity.z / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));

                Wi = add(scene.point_lights[i].position, multS(P, -1));
                Wi = normalize(Wi);
                dotProdWiNormal = dotProduct(Wi, normal);
                if (dotProdWiNormal < 0)
                {
                    dotProdWiNormal = 0;
                }

                colorDiffuseFloat.x = colorDiffuseFloat.x * dotProdWiNormal;
                colorDiffuseFloat.y = colorDiffuseFloat.y * dotProdWiNormal;
                colorDiffuseFloat.z = colorDiffuseFloat.z * dotProdWiNormal;
                colorDiffuseInteger.x = (int)(colorDiffuseFloat.x + 0.5);
                colorDiffuseInteger.y = (int)(colorDiffuseFloat.y + 0.5);
                colorDiffuseInteger.z = (int)(colorDiffuseFloat.z + 0.5);
                color = addI(color, colorDiffuseInteger);
                H = add(Wi, normalize(add(ray.origin, multS(P, -1))));
                H = normalize(H);
                dotProdNormalH = dotProduct(normal, H);
                if (dotProdNormalH < 0)
                {
                    dotProdNormalH = 0;
                }
                dotProdNormalH = pow(dotProdNormalH, scene.materials[scene.meshes[minI].material_id - 1].phong_exponent);
                colorSpecularFloat.x = scene.materials[scene.meshes[minI].material_id - 1].specular.x;
                colorSpecularFloat.x *= (scene.point_lights[i].intensity.x / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorSpecularFloat.y = scene.materials[scene.meshes[minI].material_id - 1].specular.y;
                colorSpecularFloat.y *= (scene.point_lights[i].intensity.y / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorSpecularFloat.z = scene.materials[scene.meshes[minI].material_id - 1].specular.z;
                colorSpecularFloat.z *= (scene.point_lights[i].intensity.z / (pow((scene.point_lights[i].position.x - P.x), 2) + pow((scene.point_lights[i].position.y - P.y), 2) + pow((scene.point_lights[i].position.z - P.z), 2)));
                colorSpecularFloat.x = colorSpecularFloat.x * dotProdNormalH;
                colorSpecularFloat.y = colorSpecularFloat.y * dotProdNormalH;
                colorSpecularFloat.z = colorSpecularFloat.z * dotProdNormalH;
                colorSpecularInteger.x = (int)(colorSpecularFloat.x + 0.5);
                colorSpecularInteger.y = (int)(colorSpecularFloat.y + 0.5);
                colorSpecularInteger.z = (int)(colorSpecularFloat.z + 0.5);
                color = addI(color, colorSpecularInteger);
            }
        }
        if (scene.materials[scene.meshes[minI].material_id - 1].is_mirror)
        {
            parser::Ray mirrorRay; // is W0 negative ?
            parser::Vec3f wR;
            parser::Vec3i colorV2;
            wR = subtract(multS(multS(normal, 2), dotProduct(normal, multS(normalize(ray.direction), -1))), multS(normalize(ray.direction), -1));
            mirrorRay.origin = add(P, multS(normal, scene.shadow_ray_epsilon));
            mirrorRay.direction = wR;
            colorV2 = computeColor(mirrorRay, scene, ++howManyTimes);
            colorV2.x = (int)(colorV2.x * scene.materials[scene.meshes[minI].material_id - 1].mirror.x + 0.5);
            colorV2.y = (int)(colorV2.y * scene.materials[scene.meshes[minI].material_id - 1].mirror.y + 0.5);
            colorV2.z = (int)(colorV2.z * scene.materials[scene.meshes[minI].material_id - 1].mirror.z + 0.5);
            color = addI(color, colorV2);
        }
    }
    if (color.x > 255)
    {
        color.x = 255;
    }
    if (color.y > 255)
    {
        color.y = 255;
    }
    if (color.z > 255)
    {
        color.z = 255;
    }
    return color;
}

int main(int argc, char *argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);

    // Triangle normal calculation. Not sure about Order :( // doğru cnm
    for (int i = 0; i < scene.triangles.size(); i++)
    {
        scene.triangles[i].normalOfTriangle = crossProduct(subtract(scene.vertex_data[scene.triangles[i].indices.v2_id - 1], scene.vertex_data[scene.triangles[i].indices.v1_id - 1]), (subtract(scene.vertex_data[scene.triangles[i].indices.v0_id - 1], scene.vertex_data[scene.triangles[i].indices.v1_id - 1])));
    }
    for (int i = 0; i < scene.meshes.size(); i++)
    {
        for (int j = 0; j < scene.meshes[i].faces.size(); j++)
        {
            scene.meshes[i].normalOfTriangle.push_back(crossProduct(subtract(scene.vertex_data[scene.meshes[i].faces[j].v2_id - 1], scene.vertex_data[scene.meshes[i].faces[j].v1_id - 1]), (subtract(scene.vertex_data[scene.meshes[i].faces[j].v0_id - 1], scene.vertex_data[scene.meshes[i].faces[j].v1_id - 1]))));
        }
    }

    for (int c = 0; c < scene.cameras.size(); c++)
    {
        scene.cameras[c].upCrossMinusGaze = crossProduct(scene.cameras[c].up, multS(scene.cameras[c].gaze, -1));
        unsigned char *image = new unsigned char[scene.cameras[c].image_width * scene.cameras[c].image_height * 3];

        //raytracing loop
        int x = 0;
        for (int j = 0; j < scene.cameras[c].image_height; j++)
        {
            for (int i = 0; i < scene.cameras[c].image_width; i++)
            {
                parser::Ray myRay;
                myRay = generateRay(i, j, scene.cameras[c]);
                parser::Vec3i color = computeColor(myRay, scene, 0);
                image[x++] = color.x;
                image[x++] = color.y;
                image[x++] = color.z;
            }
        }
        write_ppm(scene.cameras[c].image_name.c_str(), image, scene.cameras[c].image_width, scene.cameras[c].image_height);
    }
}
