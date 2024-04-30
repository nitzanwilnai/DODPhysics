using System;
using System.Security.Cryptography;
using System.Security.Cryptography.X509Certificates;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;

namespace DODPhysics
{
    [Serializable]
    public struct RectCollisionData
    {
        public int RectIdx1; // rect index 1
        public int RectIdx2; // rect index 2
        public Vector2 Perpendicular; // perpendicular vector
        public float Penetration; // penetration vector
        public Vector2 CollisionPoint; // collision point

        public RectCollisionData(int rectIdx1, int rectIdx2, Vector2 perpendicular, float penetration, Vector2 collisionPoint)
        {
            RectIdx1 = rectIdx1;
            RectIdx2 = rectIdx2;
            Perpendicular = perpendicular;
            Penetration = penetration;
            CollisionPoint = collisionPoint;
        }
    }

    [Serializable]
    public struct SATOutputData
    {
        public float Penetration;
        public Vector2 Axis;
        public Vector2 CollisionVertex;
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
            physicsData.RectVelocity = new Vector2[maxRects];
            physicsData.RectAngularVelocity = new float[maxRects];
            physicsData.RectMass = new float[maxRects];
            physicsData.RectInvMass = new float[maxRects];
            physicsData.RectAngle = new float[maxRects];
            physicsData.RectInertia = new float[maxRects];
            physicsData.RectInvInertia = new float[maxRects];
            physicsData.RectElasticity = new float[maxRects];
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

            RectCollisionData* rectCollisionData = stackalloc RectCollisionData[physicsData.RectCount];
            int rectCollisionDataCount = 0;
            for (int i1 = 0; i1 < physicsData.RectCount; i1++)
            {
                for (int i2 = i1 + 1; i2 < physicsData.RectCount; i2++)
                {
                    SATOutputData satOutputData;
                    if (SeparatingAxisTheorem(physicsData, i1, i2, out satOutputData))
                        rectCollisionData[rectCollisionDataCount++] = new RectCollisionData(i1, i2, satOutputData.Axis, satOutputData.Penetration, satOutputData.CollisionVertex);
                }
            }

            for (int i = 0; i < rectCollisionDataCount; i++)
            {
                // penetration resolution
                PenetrationResolution(physicsData, rectCollisionData[i]);
                // todo collision response
                CollisionResponse(physicsData, rectCollisionData[i]);
            }

            for (int i = 0; i < physicsData.RectCount; i++)
            {
                physicsData.RectPosition[i] += physicsData.RectVelocity[i] * dt;
                physicsData.RectAngle[i] += physicsData.RectAngularVelocity[i] * dt;

                SetRectVertices(physicsData, i);
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

        public static void AddRect(PhysicsData physicsData, Vector2 pos, Vector2 velocity, float width, float height, float mass)
        {
            physicsData.RectWidth[physicsData.RectCount] = width;
            physicsData.RectHeight[physicsData.RectCount] = height;
            physicsData.RectPosition[physicsData.RectCount] = pos;
            physicsData.RectDirection[physicsData.RectCount] = new Vector2(0.0f, 1.0f);
            physicsData.RectVelocity[physicsData.RectCount] = velocity;
            physicsData.RectMass[physicsData.RectCount] = mass;
            physicsData.RectInvMass[physicsData.RectCount] = 1.0f / mass;
            physicsData.RectAngle[physicsData.RectCount] = 0.0f;
            physicsData.RectInertia[physicsData.RectCount] = mass * (width * width + height * height) / 12.0f;
            physicsData.RectInvInertia[physicsData.RectCount] = 1.0f / physicsData.RectInertia[physicsData.RectCount];
            physicsData.RectElasticity[physicsData.RectCount] = 1.0f;

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

        public static unsafe bool SeparatingAxisTheorem(PhysicsData physicsData, int idx1, int idx2, out SATOutputData satOutputData)
        {
            satOutputData = new SATOutputData();

            float minOverlap = float.MaxValue;
            int vertexRectIdx = idx1;

            int axisCount = 4; // todo handle circles and lines
            Vector2* axis = stackalloc Vector2[axisCount];
            axis[0] = VectorPerpendicular2D(physicsData.RectDirection[idx1]);
            axis[1] = physicsData.RectDirection[idx1];
            axis[2] = VectorPerpendicular2D(physicsData.RectDirection[idx2]);
            axis[3] = physicsData.RectDirection[idx2];
            float firstShapeAxisIdxs = 2; // todo , handle circles and lines

            Vector2 smallestAxis = axis[0];

            float proj1min;
            float proj1max;
            float proj2min;
            float proj2max;
            Vector2 collisionVertex;

            for (int i = 0; i < axisCount; i++)
            {
                getRectMinMaxForAxis(physicsData, idx1, axis[i], out proj1min, out proj1max, out collisionVertex);
                getRectMinMaxForAxis(physicsData, idx2, axis[i], out proj2min, out proj2max, out collisionVertex);
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
            getRectMinMaxForAxis(physicsData, vertexRectIdx, smallestAxis, out min, out max, out collisionVertex);

            if (vertexRectIdx == idx2)
            {
                smallestAxis = smallestAxis * -1.0f;
            }

            satOutputData.Penetration = minOverlap;
            satOutputData.Axis = smallestAxis;
            satOutputData.CollisionVertex = collisionVertex;

            return true;
        }

        private static void getRectMinMaxForAxis(PhysicsData physicsData, int idx, Vector2 axis, out float min, out float max, out Vector2 collisionVertex)
        {
            min = max = Vector2.Dot(axis, physicsData.RectVertices[idx * 4 + 0]);
            collisionVertex = physicsData.RectVertices[idx * 4 + 0];
            for (int v = 1; v < 4; v++)
            {
                float p = Vector2.Dot(axis, physicsData.RectVertices[idx * 4 + v]);
                if (p < min)
                {
                    min = p;
                    collisionVertex = physicsData.RectVertices[idx * 4 + v];
                }
                if (p > max)
                    max = p;
            }
        }

        public static void PenetrationResolution(PhysicsData physicsData, RectCollisionData rectCollisionData)
        {
            // let penResolution = this.normal.mult(this.pen / (this.o1.inv_m + this.o2.inv_m));
            // this.o1.pos = this.o1.pos.add(penResolution.mult(this.o1.inv_m));
            // this.o2.pos = this.o2.pos.add(penResolution.mult(-this.o2.inv_m));

            int idx1 = rectCollisionData.RectIdx1;
            int idx2 = rectCollisionData.RectIdx2;
            Vector2 penResolution = rectCollisionData.Perpendicular * (rectCollisionData.Penetration / (physicsData.RectInvMass[idx1] + physicsData.RectInvMass[idx2]));
            physicsData.RectPosition[idx1] += penResolution * physicsData.RectInvMass[idx1];
            physicsData.RectPosition[idx2] += penResolution * -physicsData.RectInvMass[idx2];
        }

        public static float VectorCross2D(Vector2 v1, Vector2 v2)
        {
            return v1.x * v2.y - v1.y * v2.x;
        }

        public static Vector2 VectorPerpendicular2D(Vector2 v)
        {
            return new Vector2(-v.y, v.x).normalized;
        }

        public static void CollisionResponse(PhysicsData physicsData, RectCollisionData rectCollisionData)
        {
            int idx1 = rectCollisionData.RectIdx1;
            int idx2 = rectCollisionData.RectIdx2;

            //1. Closing velocity
            // let collArm1 = this.cp.subtr(this.o1.comp[0].pos);
            // let rotVel1 = new Vector(-this.o1.angVel * collArm1.y, this.o1.angVel * collArm1.x);
            // let closVel1 = this.o1.vel.add(rotVel1);
            // let collArm2 = this.cp.subtr(this.o2.comp[0].pos);
            // let rotVel2= new Vector(-this.o2.angVel * collArm2.y, this.o2.angVel * collArm2.x);
            // let closVel2 = this.o2.vel.add(rotVel2);

            Vector2 collArm1 = rectCollisionData.CollisionPoint - physicsData.RectPosition[idx1];
            Vector2 rotVel1 = new Vector2(-physicsData.RectAngularVelocity[idx1] * collArm1.y, physicsData.RectAngularVelocity[idx1] * collArm1.x);
            Vector2 closVel1 = physicsData.RectVelocity[idx1] + rotVel1;

            Vector2 collArm2 = rectCollisionData.CollisionPoint - physicsData.RectPosition[idx2];
            Vector2 rotVel2 = new Vector2(-physicsData.RectAngularVelocity[idx2] * collArm2.y, physicsData.RectAngularVelocity[idx2] * collArm2.x);
            Vector2 closVel2 = physicsData.RectVelocity[idx2] + rotVel2;

            // //2. Impulse augmentation
            // let impAug1 = Vector.cross(collArm1, this.normal);
            // impAug1 = impAug1 * this.o1.inv_inertia * impAug1;
            // let impAug2 = Vector.cross(collArm2, this.normal);
            // impAug2 = impAug2 * this.o2.inv_inertia * impAug2;

            float impAug1 = VectorCross2D(collArm1, rectCollisionData.Perpendicular);
            impAug1 = impAug1 * physicsData.RectInvInertia[idx1] * impAug1;
            float impAug2 = VectorCross2D(collArm2, rectCollisionData.Perpendicular);
            impAug2 = impAug2 * physicsData.RectInvInertia[idx2] * impAug2;

            // let relVel = closVel1.subtr(closVel2);
            // let sepVel = Vector.dot(relVel, this.normal);
            // let new_sepVel = -sepVel * Math.min(this.o1.elasticity, this.o2.elasticity);
            // let vsep_diff = new_sepVel - sepVel;

            Vector2 retVel = closVel1 - closVel2;
            float sepVel = Vector2.Dot(retVel, rectCollisionData.Perpendicular);
            float newSepVel = -sepVel * Mathf.Min(physicsData.RectElasticity[idx1], physicsData.RectElasticity[idx2]);
            float vsepDiff = newSepVel - sepVel;

            // let impulse = vsep_diff / (this.o1.inv_m + this.o2.inv_m + impAug1 + impAug2);
            // let impulseVec = this.normal.mult(impulse);
            float impulse = vsepDiff / (physicsData.RectInvMass[idx1] + physicsData.RectInvMass[idx2] + impAug1 + impAug2);
            Vector2 impulseVec = rectCollisionData.Perpendicular * impulse;

            // //3. Changing the velocities
            // this.o1.vel = this.o1.vel.add(impulseVec.mult(this.o1.inv_m));
            // this.o2.vel = this.o2.vel.add(impulseVec.mult(-this.o2.inv_m));
            physicsData.RectVelocity[idx1] += impulseVec * physicsData.RectInvMass[idx1];
            physicsData.RectVelocity[idx2] += impulseVec * -physicsData.RectInvMass[idx2];

            // this.o1.angVel += this.o1.inv_inertia * Vector.cross(collArm1, impulseVec);
            // this.o2.angVel -= this.o2.inv_inertia * Vector.cross(collArm2, impulseVec); 
            physicsData.RectAngularVelocity[idx1] += 3.0f * (physicsData.RectInvInertia[idx1] * VectorCross2D(collArm1, impulseVec));
            physicsData.RectAngularVelocity[idx2] -= 3.0f * (physicsData.RectInvInertia[idx2] * VectorCross2D(collArm2, impulseVec));
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