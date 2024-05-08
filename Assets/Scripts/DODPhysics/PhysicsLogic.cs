using System;
using System.Threading;
using UnityEngine;

namespace DODPhysics
{
    [Serializable]
    public struct CollisionData
    {
        public int Idx1; // rect index 1
        public int Idx2; // rect index 2
        public Vec2 Perpendicular; // perpendicular vector
        public float Penetration; // penetration vector
        public Vec2 CollisionPoint; // collision point

        public CollisionData(int rectIdx1, int rectIdx2, Vec2 perpendicular, float penetration, Vec2 collisionPoint)
        {
            Idx1 = rectIdx1;
            Idx2 = rectIdx2;
            Perpendicular = perpendicular;
            Penetration = penetration;
            CollisionPoint = collisionPoint;
        }
    }

    [Serializable]
    public struct SATOutputData
    {
        public float Penetration;
        public Vec2 Axis;
        public Vec2 CollisionVertex;
    }

    public static class PhysicsLogic
    {
        public static void AllocatePhysics(PhysicsData physicsData, int maxObjects, float friction)
        {
            physicsData.MaxObjects = maxObjects;
            physicsData.ObjectCount = 0;
            physicsData.Shape = new SHAPE[maxObjects];
            physicsData.Vertices = new Vec2[maxObjects][];
            physicsData.NumAxis = new int[maxObjects];

            physicsData.Friction = friction;

            physicsData.Gravity = new Vec2[maxObjects];

            physicsData.Position = new Vec2[maxObjects];
            physicsData.Direction = new Vec2[maxObjects];
            physicsData.Velocity = new Vec2[maxObjects];
            physicsData.AngularVelocity = new float[maxObjects];
            physicsData.Mass = new float[maxObjects];
            physicsData.InvMass = new float[maxObjects];
            physicsData.Angle = new float[maxObjects];
            physicsData.Inertia = new float[maxObjects];
            physicsData.InvInertia = new float[maxObjects];
            physicsData.Elasticity = new float[maxObjects];

            physicsData.Radius = new float[maxObjects];

            physicsData.RectWidth = new float[maxObjects];
            physicsData.RectHeight = new float[maxObjects];

            physicsData.CollisionHappenedPrevFrame = new bool[maxObjects];

        }

        public static void AddWall(PhysicsData physicsData, Vec2 p1, Vec2 p2, Vec2 gravity)
        {
            p1 *= 100.0f;
            p2 *= 100.0f;

            physicsData.Shape[physicsData.ObjectCount] = SHAPE.WALL;
            physicsData.NumAxis[physicsData.ObjectCount] = 1;

            physicsData.Gravity[physicsData.ObjectCount] = gravity;

            physicsData.Vertices[physicsData.ObjectCount] = new Vec2[2];
            physicsData.Vertices[physicsData.ObjectCount][0] = p1;
            physicsData.Vertices[physicsData.ObjectCount][1] = p2;

            physicsData.Direction[physicsData.ObjectCount] = (p2 - p1).Normal();
            physicsData.Position[physicsData.ObjectCount] = (p2 + p1) / 2.0f;

            physicsData.Velocity[physicsData.ObjectCount] = Vec2.Zero();
            physicsData.Mass[physicsData.ObjectCount] = 0.0f;
            physicsData.InvMass[physicsData.ObjectCount] = 0.0f;
            physicsData.Angle[physicsData.ObjectCount] = 0.0f; // ???
            physicsData.Elasticity[physicsData.ObjectCount] = 1.0f;

            physicsData.ObjectCount++;
        }

        public static void AddCircle(PhysicsData physicsData, Vec2 pos, Vec2 velocity, float radius, float mass, Vec2 gravity)
        {
            pos *= 100.0f;
            velocity *= 100.0f;
            radius *= 100.0f;

            physicsData.Shape[physicsData.ObjectCount] = SHAPE.CIRCLE;
            physicsData.NumAxis[physicsData.ObjectCount] = 1;

            physicsData.Radius[physicsData.ObjectCount] = radius;
            physicsData.Vertices[physicsData.ObjectCount] = new Vec2[2];

            physicsData.Inertia[physicsData.ObjectCount] = mass * (radius * radius + radius * radius) / 12.0f;

            addCommon(physicsData, pos, velocity, mass, gravity);

            physicsData.Elasticity[physicsData.ObjectCount] = 0.8f;

            physicsData.ObjectCount++;
        }

        public static void AddRect(PhysicsData physicsData, Vec2 pos, Vec2 velocity, float width, float height, float mass, Vec2 gravity)
        {
            pos *= 100.0f;
            width *= 100.0f;
            height *= 100.0f;

            physicsData.Shape[physicsData.ObjectCount] = SHAPE.RECTANGLE;
            physicsData.NumAxis[physicsData.ObjectCount] = 2;

            physicsData.RectWidth[physicsData.ObjectCount] = width;
            physicsData.RectHeight[physicsData.ObjectCount] = height;

            physicsData.Radius[physicsData.ObjectCount] = Mathf.Sqrt(width * width + height * height);

            physicsData.Inertia[physicsData.ObjectCount] = mass * (width * width + height * height) / 12.0f;

            addCommon(physicsData, pos, velocity, mass, gravity);

            Vec2 dir = new Vec2(0.0f, 1.0f);
            dir.RotateRad(physicsData.Angle[physicsData.ObjectCount]);
            physicsData.Direction[physicsData.ObjectCount] = dir;

            physicsData.Vertices[physicsData.ObjectCount] = new Vec2[4];
            SetRectVertices(physicsData, physicsData.ObjectCount);

            physicsData.Elasticity[physicsData.ObjectCount] = 0.9f;

            physicsData.ObjectCount++;

        }

        private static void addCommon(PhysicsData physicsData, Vec2 pos, Vec2 velocity, float mass, Vec2 gravity)
        {
            physicsData.Gravity[physicsData.ObjectCount] = gravity;

            physicsData.InvInertia[physicsData.ObjectCount] = physicsData.Inertia[physicsData.ObjectCount] > 0.0f ? 1.0f / physicsData.Inertia[physicsData.ObjectCount] : 0.0f;

            physicsData.Position[physicsData.ObjectCount] = pos;
            physicsData.Direction[physicsData.ObjectCount] = new Vec2(0.0f, 1.0f);
            physicsData.Velocity[physicsData.ObjectCount] = velocity;
            physicsData.Mass[physicsData.ObjectCount] = mass;
            physicsData.InvMass[physicsData.ObjectCount] = mass > 0.0f ? 1.0f / mass : 0.0f;
            physicsData.Angle[physicsData.ObjectCount] = 0.0f;
            physicsData.Elasticity[physicsData.ObjectCount] = 1.0f;
        }

        public static unsafe void Tick(PhysicsData physicsData, float dt)
        {
            for (int i = 0; i < physicsData.ObjectCount; i++)
            {
                if (physicsData.Mass[i] > 0.0f)
                {
                    physicsData.Velocity[i] *= 1.0f - physicsData.Friction * dt;

                    if (!physicsData.CollisionHappenedPrevFrame[i])
                        physicsData.Velocity[i] += physicsData.Gravity[i] * dt;

                    physicsData.Position[i] += physicsData.Velocity[i] * dt * 60.0f;

                    physicsData.Angle[i] += physicsData.AngularVelocity[i] * dt * 60.0f;

                    //make sure we don't exit our bounds
                    if (physicsData.Position[i].x < physicsData.WallLeft)
                        physicsData.Position[i].x = physicsData.WallLeft + 1.0f;
                    if (physicsData.Position[i].x > physicsData.WallRight)
                        physicsData.Position[i].x = physicsData.WallRight - 1.0f;
                    if (physicsData.Position[i].y < physicsData.Floor)
                        physicsData.Position[i].y = physicsData.Floor + 1.0f;
                    if (physicsData.Position[i].y > physicsData.Ceiling)
                        physicsData.Position[i].y = physicsData.Ceiling - 1.0f;
                }

                if (physicsData.Shape[i] == SHAPE.RECTANGLE)
                    SetRectVertices(physicsData, i);
            }

            for (int i = 0; i < physicsData.ObjectCount; i++)
                physicsData.CollisionHappenedPrevFrame[i] = false;


            int collisionDataSize = physicsData.ObjectCount * 10;
            CollisionData* collisionData = stackalloc CollisionData[collisionDataSize];
            int rectCollisionDataCount = 0;
            for (int i1 = 0; i1 < physicsData.ObjectCount - 1; i1++)
            {
                for (int i2 = i1 + 1; i2 < physicsData.ObjectCount; i2++)
                {
                    if (physicsData.Mass[i1] > 0 || physicsData.Mass[i2] > 0)
                    {
                        bool checkCollision = (physicsData.Shape[i1] == SHAPE.WALL || physicsData.Shape[i2] == SHAPE.WALL);
                        if (!checkCollision)
                        {
                            // do a quick circle to circle bounds check (even for boxes)
                            float radius = physicsData.Radius[i1] + physicsData.Radius[i2];
                            float radiusSquared = radius * radius;
                            float dstSquared = (physicsData.Position[i2] - physicsData.Position[i1]).MagnitudeSqr();
                            checkCollision = dstSquared < radiusSquared;
                        }
                        if (checkCollision)
                        {
                            // do a more in depth check using the Separating Axis Theorem
                            SATOutputData satOutputData;
                            if (SeparatingAxisTheorem(physicsData, i1, i2, out satOutputData) && rectCollisionDataCount < collisionDataSize)
                                collisionData[rectCollisionDataCount++] = new CollisionData(i1, i2, satOutputData.Axis, satOutputData.Penetration, satOutputData.CollisionVertex);
                            if (rectCollisionDataCount >= collisionDataSize)
                                Debug.LogError("rectCollisionDataCount" + rectCollisionDataCount + " >= collisionDataSize " + collisionDataSize);
                        }
                    }
                }
            }

            for (int i = 0; i < rectCollisionDataCount; i++)
            {
                int idx1 = collisionData[i].Idx1;
                int idx2 = collisionData[i].Idx2;

                physicsData.CollisionHappenedPrevFrame[idx1] = true;
                physicsData.CollisionHappenedPrevFrame[idx2] = true;

                // penetration resolution
                PenetrationResolution(physicsData, collisionData[i]);
                // todo collision response
                CollisionResponse(physicsData, collisionData[i]);
            }
        }

        public static void SetRectVertices(PhysicsData physicsData, int idx)
        {
            Vec2 pos = physicsData.Position[idx];

            float halfWidth = physicsData.RectWidth[idx] / 2.0f;
            float halfHeight = physicsData.RectHeight[idx] / 2.0f;

            Vec2 dir = new Vec2(0.0f, 1.0f);
            dir.RotateRad(physicsData.Angle[idx]);

            physicsData.Direction[idx] = dir;

            physicsData.Vertices[idx][0] = pos + Vec2.RotateRad(new Vec2(-halfWidth, -halfHeight), physicsData.Angle[idx]);
            physicsData.Vertices[idx][1] = pos + Vec2.RotateRad(new Vec2(halfWidth, -halfHeight), physicsData.Angle[idx]);
            physicsData.Vertices[idx][2] = pos + Vec2.RotateRad(new Vec2(halfWidth, halfHeight), physicsData.Angle[idx]);
            physicsData.Vertices[idx][3] = pos + Vec2.RotateRad(new Vec2(-halfWidth, halfHeight), physicsData.Angle[idx]);
        }

        public static unsafe bool SeparatingAxisTheorem(PhysicsData physicsData, int idx1, int idx2, out SATOutputData satOutputData)
        {
            satOutputData = new SATOutputData();

            float minOverlap = float.MaxValue;
            int vertexRectIdx = idx1;

            Vec2* axis = stackalloc Vec2[4];
            int axisCount = getAxisForShapes(physicsData, idx1, idx2, axis);
            float firstShapeAxisIdxs = physicsData.NumAxis[idx1];
            Vec2 smallestAxis = axis[0];

            float proj1min;
            float proj1max;
            float proj2min;
            float proj2max;
            Vec2 collisionVertex;

            for (int i = 0; i < axisCount; i++)
            {
                getMinMaxForAxis(physicsData, idx1, axis[i], out proj1min, out proj1max, out collisionVertex);
                getMinMaxForAxis(physicsData, idx2, axis[i], out proj2min, out proj2max, out collisionVertex);
                float overlap = Mathf.Min(proj1max, proj2max) - Mathf.Max(proj1min, proj2min);
                if (overlap < 0.0f)
                    return false;

                // take into account if we are inside another object, or vice versa
                if ((proj1max > proj2max && proj1min < proj2min) || (proj1max < proj2max && proj1min > proj2min))
                {
                    float mins = Mathf.Abs(proj1min - proj2min);
                    float maxs = Mathf.Abs(proj1max - proj2max);
                    if (mins < maxs)
                        overlap += mins;
                    else
                    {
                        overlap += maxs;
                        axis[i] = axis[i] * -1.0f;
                    }

                }

                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    smallestAxis = axis[i];
                    if (i < firstShapeAxisIdxs)
                    {
                        vertexRectIdx = idx2;
                        if (proj1max > proj2max)
                            smallestAxis = axis[i] * -1.0f;
                    }
                    else
                    {
                        vertexRectIdx = idx1;
                        if (proj1max < proj2max)
                            smallestAxis = axis[i] * -1.0f;
                    }
                }

            }

            float min;
            float max;
            getMinMaxForAxis(physicsData, vertexRectIdx, smallestAxis, out min, out max, out collisionVertex);

            if (vertexRectIdx == idx2)
                smallestAxis = smallestAxis * -1.0f;

            satOutputData.Penetration = minOverlap;
            satOutputData.Axis = smallestAxis;
            satOutputData.CollisionVertex = collisionVertex;

            return true;
        }

        private static unsafe int getAxisForShapes(PhysicsData physicsData, int idx1, int idx2, Vec2* axis)
        {
            int axisCount = 0;

            if (physicsData.Shape[idx1] == SHAPE.CIRCLE && physicsData.Shape[idx2] == SHAPE.CIRCLE)
                axis[axisCount++] = (physicsData.Position[idx2] - physicsData.Position[idx1]).Normal();
            else if (physicsData.Shape[idx1] == SHAPE.CIRCLE)
            {
                axis[axisCount++] = (closestVertexToPoint(physicsData, idx2, physicsData.Position[idx1]) - physicsData.Position[idx1]).Normal();
                axis[axisCount++] = physicsData.Direction[idx2].Perpendicular();
                if (physicsData.Shape[idx2] == SHAPE.RECTANGLE)
                    axis[axisCount++] = physicsData.Direction[idx2];
            }
            else if (physicsData.Shape[idx2] == SHAPE.CIRCLE)
            {
                axis[axisCount++] = physicsData.Direction[idx1].Perpendicular();
                if (physicsData.Shape[idx1] == SHAPE.RECTANGLE)
                    axis[axisCount++] = physicsData.Direction[idx1];
                axis[axisCount++] = (closestVertexToPoint(physicsData, idx1, physicsData.Position[idx2]) - physicsData.Position[idx2]).Normal();
            }
            else
            {
                axis[axisCount++] = physicsData.Direction[idx1].Perpendicular();
                if (physicsData.Shape[idx1] == SHAPE.RECTANGLE)
                    axis[axisCount++] = physicsData.Direction[idx1];
                axis[axisCount++] = physicsData.Direction[idx2].Perpendicular();
                if (physicsData.Shape[idx2] == SHAPE.RECTANGLE)
                    axis[axisCount++] = physicsData.Direction[idx2];
            }
            return axisCount;
        }

        private static Vec2 closestVertexToPoint(PhysicsData physicsData, int idx, Vec2 p)
        {
            Vec2 closestVertex = Vec2.Zero();
            float minDist = float.MaxValue;
            for (int i = 0; i < physicsData.Vertices[idx].Length; i++)
            {
                float mag = (p - physicsData.Vertices[idx][i]).MagnitudeSqr();
                if (mag < minDist)
                {
                    closestVertex = physicsData.Vertices[idx][i];
                    minDist = mag;
                }
            }
            return closestVertex;
        }

        private static void getMinMaxForAxis(PhysicsData physicsData, int idx, Vec2 axis, out float min, out float max, out Vec2 collisionVertex)
        {
            if (physicsData.Shape[idx] == SHAPE.CIRCLE)
                setBallVerticesAlongAxis(physicsData, idx, axis);

            min = max = Vec2.Dot(axis, physicsData.Vertices[idx][0]);
            collisionVertex = physicsData.Vertices[idx][0];
            for (int v = 1; v < physicsData.Vertices[idx].Length; v++)
            {
                float p = Vec2.Dot(axis, physicsData.Vertices[idx][v]);
                if (p < min)
                {
                    min = p;
                    collisionVertex = physicsData.Vertices[idx][v];
                }
                if (p > max)
                    max = p;
            }
        }

        private static void setBallVerticesAlongAxis(PhysicsData physicsData, int idx, Vec2 axis)
        {
            physicsData.Vertices[idx][0] = physicsData.Position[idx] + (axis.Normal() * -physicsData.Radius[idx]);
            physicsData.Vertices[idx][1] = physicsData.Position[idx] + (axis.Normal() * physicsData.Radius[idx]);
        }

        public static void PenetrationResolution(PhysicsData physicsData, CollisionData rectCollisionData)
        {
            int idx1 = rectCollisionData.Idx1;
            int idx2 = rectCollisionData.Idx2;
            if (rectCollisionData.Penetration > 0.01f && physicsData.InvMass[idx1] > 0.0f || physicsData.InvMass[idx2] > 0.0f)
            {
                Vec2 penResolution = rectCollisionData.Perpendicular * (rectCollisionData.Penetration / (physicsData.InvMass[idx1] + physicsData.InvMass[idx2])) * 1.1f;
                physicsData.Position[idx1] += penResolution * physicsData.InvMass[idx1];
                physicsData.Position[idx2] += penResolution * -physicsData.InvMass[idx2];
            }
        }

        public static void CollisionResponse(PhysicsData physicsData, CollisionData rectCollisionData)
        {
            int idx1 = rectCollisionData.Idx1;
            int idx2 = rectCollisionData.Idx2;


            Vec2 collArm1 = rectCollisionData.CollisionPoint - physicsData.Position[idx1];
            Vec2 rotVel1 = new Vec2(-physicsData.AngularVelocity[idx1] * collArm1.y, physicsData.AngularVelocity[idx1] * collArm1.x);
            Vec2 closVel1 = physicsData.Velocity[idx1] + rotVel1;

            Vec2 collArm2 = rectCollisionData.CollisionPoint - physicsData.Position[idx2];
            Vec2 rotVel2 = new Vec2(-physicsData.AngularVelocity[idx2] * collArm2.y, physicsData.AngularVelocity[idx2] * collArm2.x);
            Vec2 closVel2 = physicsData.Velocity[idx2] + rotVel2;


            float impAug1 = Vec2.Cross(collArm1, rectCollisionData.Perpendicular);
            impAug1 = impAug1 * physicsData.InvInertia[idx1] * impAug1;
            float impAug2 = Vec2.Cross(collArm2, rectCollisionData.Perpendicular);
            impAug2 = impAug2 * physicsData.InvInertia[idx2] * impAug2;


            Vec2 retVel = closVel1 - closVel2;
            float sepVel = Vec2.Dot(retVel, rectCollisionData.Perpendicular);
            float newSepVel = -sepVel * Mathf.Min(physicsData.Elasticity[idx1], physicsData.Elasticity[idx2]);
            float vsepDiff = newSepVel - sepVel;

            float invMass1 = physicsData.InvMass[idx1];
            float invMass2 = physicsData.InvMass[idx2];


            float impulse = (invMass1 + invMass2 + impAug1 + impAug2) > 0.0f ? vsepDiff / (invMass1 + invMass2 + impAug1 + impAug2) : 0.0f;
            Vec2 impulseVec = rectCollisionData.Perpendicular * impulse;

            physicsData.Velocity[idx1] += impulseVec * invMass1;
            physicsData.Velocity[idx2] -= impulseVec * invMass2;

            physicsData.AngularVelocity[idx1] += physicsData.InvInertia[idx1] * Vec2.Cross(collArm1, impulseVec);
            physicsData.AngularVelocity[idx2] -= physicsData.InvInertia[idx2] * Vec2.Cross(collArm2, impulseVec);
        }
    }
}