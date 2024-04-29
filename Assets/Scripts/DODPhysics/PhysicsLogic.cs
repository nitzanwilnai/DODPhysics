using System;
using System.Security.Cryptography.X509Certificates;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;

namespace DODPhysics
{
    public struct RectCollisionData
    {
        public int rectIdx1; // rect index 1
        public int rectIdx2; // rect index 2
        public Vector2 perp; // perpendicular vector
        public Vector2 pen; // penetration vector
        public Vector2 cp; // collision point
    }

    public static class PhysicsLogic
    {
        public static void AllocatePhysicsCircles(PhysicsData physicsData, int maxCircles)
        {
            physicsData.CircleCount = 0;
            physicsData.MaxCircles = maxCircles;
            physicsData.CircleRadius = new float[maxCircles];
            physicsData.CirclePosition = new Vector2[maxCircles];
            physicsData.CircleDirection = new Vector2[maxCircles];
            physicsData.CircleMass = new float[maxCircles];
        }

        public static void AllocatePhysicsRects(PhysicsData physicsData, int maxRects)
        {
            physicsData.RectCount = 0;
            physicsData.RectVertices = new Vector2[maxRects * 4];
            physicsData.RectWidth = new float[maxRects];
            physicsData.RectHeight = new float[maxRects];
            physicsData.RectPosition = new Vector2[maxRects];
            physicsData.RectDirection = new Vector2[maxRects];
            physicsData.RectMass = new float[maxRects];
            physicsData.RectAngle = new float[maxRects];
        }

        public static void AddCircle(PhysicsData physicsData, Vector2 pos, Vector2 dir, float radius, float mass)
        {
            Debug.Log("physicsData.CircleCount " + physicsData.CircleCount + " AddCircle(pos (" + pos.x + "," + pos.y + ") dir (" + dir.x + "," + dir.y + ") radius " + radius + " mass " + mass + ")");
            physicsData.CircleRadius[physicsData.CircleCount] = radius;
            physicsData.CirclePosition[physicsData.CircleCount] = pos;
            physicsData.CircleDirection[physicsData.CircleCount] = dir;
            physicsData.CircleMass[physicsData.CircleCount] = mass;
            physicsData.CircleCount++;
        }

        public static unsafe void Tick(PhysicsData physicsData, float dt)
        {
            bool* circleCollisionHappened = stackalloc bool[physicsData.CircleCount];
            for (int i = 0; i < physicsData.CircleCount; i++)
                circleCollisionHappened[i] = false;

            checkCicleWallCollision(physicsData, circleCollisionHappened);

            // check collision between balls
            for (int i1 = 0; i1 < physicsData.CircleCount; i1++)
            {
                for (int i2 = i1 + 1; i2 < physicsData.CircleCount; i2++)
                {
                    checkCircleCircleCollision(physicsData, circleCollisionHappened, i1, i2);
                }
            }

            Vector2 gravity = new Vector2(0.0f, physicsData.Gravity);

            // apply gravity
            for (int i = 0; i < physicsData.CircleCount; i++)
                if (circleCollisionHappened[i])
                    physicsData.CircleDirection[i] *= 0.9f;
                else
                {
                    // modify physicsData.CircleDireciton.Y by adding gravity and normalizing again
                    physicsData.CircleDirection[i] += gravity * dt;
                }

            // move circles
            for (int i = 0; i < physicsData.CircleCount; i++)
            {
                physicsData.CirclePosition[i] = physicsData.CirclePosition[i] + physicsData.CircleDirection[i] * dt;
            }

        }

        private static unsafe void checkCicleWallCollision(PhysicsData physicsData, bool* circleCollisionHappened)
        {
            // check collision with floor and walls
            for (int i = 0; i < physicsData.CircleCount; i++)
            {
                // floor
                if (physicsData.CirclePosition[i].y - physicsData.CircleRadius[i] < physicsData.Floor)
                {
                    physicsData.CirclePosition[i].y = physicsData.Floor + physicsData.CircleRadius[i];
                    physicsData.CircleDirection[i].y = -physicsData.CircleDirection[i].y;
                    physicsData.CircleDirection[i] *= 0.75f;
                }
                // ceiling
                if (physicsData.CirclePosition[i].y + physicsData.CircleRadius[i] > physicsData.Ceiling)
                {
                    physicsData.CirclePosition[i].y = physicsData.Ceiling - physicsData.CircleRadius[i];
                    physicsData.CircleDirection[i].y = -physicsData.CircleDirection[i].y;
                    physicsData.CircleDirection[i] *= 0.75f;
                }
                // wall left
                if (physicsData.CirclePosition[i].x - physicsData.CircleRadius[i] < physicsData.WallLeft)
                {
                    physicsData.CirclePosition[i].x = physicsData.WallLeft + physicsData.CircleRadius[i];
                    physicsData.CircleDirection[i].x = -physicsData.CircleDirection[i].x;
                    circleCollisionHappened[i] = true;
                }
                // wall right
                if (physicsData.CirclePosition[i].x + physicsData.CircleRadius[i] > physicsData.WallRight)
                {
                    physicsData.CirclePosition[i].x = physicsData.WallRight - physicsData.CircleRadius[i];
                    physicsData.CircleDirection[i].x = -physicsData.CircleDirection[i].x;
                    circleCollisionHappened[i] = true;
                }
            }
        }

        private static unsafe void checkCircleCircleCollision(PhysicsData physicsData, bool* circleCollisionHappened, int i1, int i2)
        {
            float distance = (physicsData.CirclePosition[i1] - physicsData.CirclePosition[i2]).magnitude;
            // Debug.Log("i1 " + i1 + " i2 " + i2 + " distance " + distance + " totalRadius " + (physicsData.CircleRadius[i1] + physicsData.CircleRadius[i2]));
            float circlesRadius = physicsData.CircleRadius[i1] + physicsData.CircleRadius[i2];
            if (distance < circlesRadius && Mathf.Abs(distance - circlesRadius) > 0.0001f)
            {
                //float dot = Vector2.Dot(physicsData.CircleDirection[i1], physicsData.CircleDirection[i2]);
                // Debug.Log("dot " + dot + " dir1 " + physicsData.CircleDirection[i1] + " dir2 " + physicsData.CircleDirection[i2]);
                //if (dot < 0)
                {
                    Vector2 diffVector = (physicsData.CirclePosition[i1] - physicsData.CirclePosition[i2]).normalized;
                    float penetration = (physicsData.CircleRadius[i1] + physicsData.CircleRadius[i2]) - distance;
                    float halfPenetration = penetration / 2.0f;
                    physicsData.CirclePosition[i1] += diffVector * halfPenetration * 1.01f;
                    physicsData.CirclePosition[i2] -= diffVector * halfPenetration * 1.01f;

                    Vector2 v1 = physicsData.CircleDirection[i1];
                    Vector2 v2 = physicsData.CircleDirection[i2];

                    float combinedMass = physicsData.CircleMass[i1] + physicsData.CircleMass[i2];
                    // physicsData.CircleDirection[i1] = ((physicsData.CircleMass[i1] - physicsData.CircleMass[i2]) / combinedMass) * v1 + ((2.0f * physicsData.CircleMass[i2]) / combinedMass) * v2;
                    // physicsData.CircleDirection[i2] = ((physicsData.CircleMass[i2] - physicsData.CircleMass[i1]) / combinedMass) * v2 + ((2.0f * physicsData.CircleMass[i1]) / combinedMass) * v1;

                    // physicsData.CircleDirection[i1] = physicsData.CircleDirection[i1] + ((physicsData.CirclePosition[i1] - physicsData.CirclePosition[i2]) / 2.0f);
                    // physicsData.CircleDirection[i2] = physicsData.CircleDirection[i2] + ((physicsData.CirclePosition[i2] - physicsData.CirclePosition[i1]) / 2.0f);

                    float m1 = physicsData.CircleMass[i1];
                    float m2 = physicsData.CircleMass[i2];
                    Vector2 p1 = physicsData.CirclePosition[i1];
                    Vector2 p2 = physicsData.CirclePosition[i2];
                    physicsData.CircleDirection[i1] = v1 - (2.0f * m2 / combinedMass) * Vector2.Dot(v1 - v2, p1 - p2) / (p1 - p2).sqrMagnitude * (p1 - p2);
                    physicsData.CircleDirection[i2] = v2 - (2.0f * m1 / combinedMass) * Vector2.Dot(v2 - v1, p2 - p1) / (p2 - p1).sqrMagnitude * (p2 - p1);

                    circleCollisionHappened[i1] = true;
                    circleCollisionHappened[i2] = true;

                    // Debug.Log("penetration " + penetration + " dir1 " + physicsData.CircleDirection[i1] + " dir2 " + physicsData.CircleDirection[i2]);
                }
            }
        }

        public static void AddRect(PhysicsData physicsData, Vector2 pos, Vector2 dir, float width, float height, float mass)
        {
            physicsData.RectWidth[physicsData.RectCount] = width;
            physicsData.RectHeight[physicsData.RectCount] = height;
            physicsData.RectPosition[physicsData.RectCount] = pos;
            physicsData.RectDirection[physicsData.RectCount] = dir;
            physicsData.RectMass[physicsData.RectCount] = mass;
            physicsData.RectAngle[physicsData.RectCount] = 0.0f;

            RotateRect(physicsData, physicsData.RectCount);

            // physicsData.RectVertices[physicsData.RectCount * 4 + 0] = new Vector2(-width / 2.0f, -height / 2.0f);
            // physicsData.RectVertices[physicsData.RectCount * 4 + 1] = new Vector2(-width / 2.0f, height / 2.0f);
            // physicsData.RectVertices[physicsData.RectCount * 4 + 2] = new Vector2(width / 2.0f, height / 2.0f);
            // physicsData.RectVertices[physicsData.RectCount * 4 + 3] = new Vector2(width / 2.0f, -height / 2.0f);
            physicsData.RectCount++;

        }

        public static void SetRectVertices(PhysicsData physicsData, int idx)
        {
            Vector2 pos = physicsData.RectPosition[idx];

            float halfWidth = physicsData.RectWidth[idx] / 2.0f;
            float halfHeight = physicsData.RectHeight[idx] / 2.0f;

            Quaternion quaternion = Quaternion.Euler(0.0f, 0.0f, physicsData.RectAngle[idx]);

            physicsData.RectVertices[idx * 4 + 0] = pos + (Vector2)(quaternion * new Vector3(-halfWidth, -halfHeight));
            physicsData.RectVertices[idx * 4 + 1] = pos + (Vector2)(quaternion * new Vector3(-halfWidth, halfHeight));
            physicsData.RectVertices[idx * 4 + 2] = pos + (Vector2)(quaternion * new Vector3(halfWidth, halfHeight));
            physicsData.RectVertices[idx * 4 + 3] = pos + (Vector2)(quaternion * new Vector3(halfWidth, -halfHeight));
        }

        public static void RotateRect(PhysicsData physicsData, int idx)
        {
            Quaternion quaternion = Quaternion.Euler(0.0f, 0.0f, physicsData.RectAngle[idx]);
            physicsData.RectDirection[idx] = quaternion * new Vector2(0.0f, 1.0f);

            SetRectVertices(physicsData, idx);
        }

        public static bool SeparatingAxisTheorem(PhysicsData physicsData, int idx1, int idx2)
        {
            float rect1min;
            float rect1max;
            float rect2min;
            float rect2max;
            getRectMinMaxForAxis(physicsData, idx1, Vector2.Perpendicular(physicsData.RectDirection[idx1]), out rect1min, out rect1max);
            getRectMinMaxForAxis(physicsData, idx2, Vector2.Perpendicular(physicsData.RectDirection[idx1]), out rect2min, out rect2max);
            if (Mathf.Min(rect1max, rect2max) - Mathf.Max(rect1min, rect2min) < 0.0f)
                return false;

            getRectMinMaxForAxis(physicsData, idx1, physicsData.RectDirection[idx1], out rect1min, out rect1max);
            getRectMinMaxForAxis(physicsData, idx2, physicsData.RectDirection[idx1], out rect2min, out rect2max);
            if (Mathf.Min(rect1max, rect2max) - Mathf.Max(rect1min, rect2min) < 0.0f)
                return false;

            getRectMinMaxForAxis(physicsData, idx1, Vector2.Perpendicular(physicsData.RectDirection[idx2]), out rect1min, out rect1max);
            getRectMinMaxForAxis(physicsData, idx2, Vector2.Perpendicular(physicsData.RectDirection[idx2]), out rect2min, out rect2max);
            if (Mathf.Min(rect1max, rect2max) - Mathf.Max(rect1min, rect2min) < 0.0f)
                return false;

            getRectMinMaxForAxis(physicsData, idx1, physicsData.RectDirection[idx2], out rect1min, out rect1max);
            getRectMinMaxForAxis(physicsData, idx2, physicsData.RectDirection[idx2], out rect2min, out rect2max);
            if (Mathf.Min(rect1max, rect2max) - Mathf.Max(rect1min, rect2min) < 0.0f)
                return false;

            return true;
        }

        private static void getRectMinMaxForAxis(PhysicsData physicsData, int idx, Vector2 axis, out float min, out float max)
        {
            min = max = Vector2.Dot(axis, physicsData.RectVertices[idx * 4 + 0]);
            for (int v = 1; v < 4; v++)
            {
                float p = Vector2.Dot(axis, physicsData.RectVertices[idx * 4 + v]);
                if (p < min)
                    min = p;
                if (p > max)
                    max = p;
            }
        }


        // private static unsafe bool checkRectRectCollision(PhysicsData physicsData, int i1, int i2)
        // {
        //     Vector2 r1p1 = new Vector2(rect1.x, rect1.y);
        //     Vector2 r1p2 = new Vector2(rect1.x, rect1.w);
        //     Vector2 r1p3 = new Vector2(rect1.z, rect1.w);
        //     Vector2 r1p4 = new Vector2(rect1.z, rect1.y);

        //     Vector2 collisionP;
        //     if (checkLineRectCollision(r1p1, r1p2, rect2, out collisionP))
        //     {
        //         return true;
        //     }
        //     else if (checkLineRectCollision(r1p2, r1p3, rect2, out collisionP))
        //     {
        //         return true;
        //     }
        //     else if (checkLineRectCollision(r1p3, r1p4, rect2, out collisionP))
        //     {
        //         return true;
        //     }
        //     else if (checkLineRectCollision(r1p4, r1p1, rect2, out collisionP))
        //     {
        //         return true;
        //     }
        //     return false;
        // }

        private static unsafe bool checkLineRectCollision(Vector2 p1, Vector2 p2, Vector4 rect, out Vector2 collisionP)
        {
            Vector2 rectP1 = new Vector2(rect.x, rect.y);
            Vector2 rectP2 = new Vector2(rect.x, rect.w);
            Vector2 rectP3 = new Vector2(rect.z, rect.w);
            Vector2 rectP4 = new Vector2(rect.z, rect.y);

            collisionP = Vector2.zero;
            if (checkLineLineCollision(p1, p2, rectP1, rectP2, out collisionP))
            {
                return true;
            }
            else if (checkLineLineCollision(p1, p2, rectP2, rectP3, out collisionP))
            {
                return true;
            }
            else if (checkLineLineCollision(p1, p2, rectP3, rectP4, out collisionP))
            {
                return true;
            }
            else if (checkLineLineCollision(p1, p2, rectP4, rectP1, out collisionP))
            {
                return true;
            }
            return false;
        }

        private static unsafe bool checkLineLineCollision(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, out Vector2 collisionP)
        {
            float uA = ((p4.x - p4.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x)) / ((p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y));

            float uB = ((p2.x - p1.x) * (p1.y - p3.y) - (p2.y - p1.y) * (p1.x - p3.x)) / ((p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y));

            collisionP = Vector2.zero;
            if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1)
            {
                collisionP.x = p1.x + (uA * (p2.x - p1.x));
                collisionP.y = p1.y + (uA * (p2.y - p1.y));
                return true;
            }
            return false;
        }
    }
}