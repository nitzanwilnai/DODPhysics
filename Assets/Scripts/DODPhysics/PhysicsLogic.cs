using System;
using UnityEngine;

namespace DODPhysics
{
    [Serializable]
    public struct CollisionData
    {
        public int Idx1; // rect index 1
        public int Idx2; // rect index 2
        public Vector2 Perpendicular; // perpendicular vector
        public float Penetration; // penetration vector
        public Vector2 CollisionPoint; // collision point

        public CollisionData(int rectIdx1, int rectIdx2, Vector2 perpendicular, float penetration, Vector2 collisionPoint)
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
        public Vector2 Axis;
        public Vector2 CollisionVertex;
    }

    public static class PhysicsLogic
    {
        public static void AllocatePhysics(PhysicsData physicsData, int maxObjects, float friction)
        {
            physicsData.MaxObjects = maxObjects;
            physicsData.ObjectCount = 0;
            physicsData.Shape = new SHAPE[maxObjects];
            physicsData.Vertices = new Vector2[maxObjects][];
            physicsData.NumAxis = new int[maxObjects];

            physicsData.Friction = friction;

            physicsData.Gravity = new Vector2[maxObjects];

            physicsData.Position = new Vector2[maxObjects];
            physicsData.Direction = new Vector2[maxObjects];
            physicsData.Velocity = new Vector2[maxObjects];
            physicsData.AngularVelocity = new float[maxObjects];
            physicsData.Mass = new float[maxObjects];
            physicsData.InvMass = new float[maxObjects];
            physicsData.Angle = new float[maxObjects];
            physicsData.Inertia = new float[maxObjects];
            physicsData.InvInertia = new float[maxObjects];
            physicsData.Elasticity = new float[maxObjects];

            physicsData.CircleRadius = new float[maxObjects];

            physicsData.RectWidth = new float[maxObjects];
            physicsData.RectHeight = new float[maxObjects];

        }

        public static void AddWall(PhysicsData physicsData, Vector2 p1, Vector2 p2, Vector2 gravity)
        {
            p1 *= 100.0f;
            p2 *= 100.0f;

            physicsData.Shape[physicsData.ObjectCount] = SHAPE.WALL;
            physicsData.NumAxis[physicsData.ObjectCount] = 1;

            physicsData.Gravity[physicsData.ObjectCount] = gravity;

            physicsData.Vertices[physicsData.ObjectCount] = new Vector2[2];
            physicsData.Vertices[physicsData.ObjectCount][0] = p1;
            physicsData.Vertices[physicsData.ObjectCount][1] = p2;

            physicsData.Direction[physicsData.ObjectCount] = (p2 - p1).normalized;
            physicsData.Position[physicsData.ObjectCount] = (p2 + p1) / 2.0f;

            physicsData.Velocity[physicsData.ObjectCount] = Vector2.zero;
            physicsData.Mass[physicsData.ObjectCount] = 0.0f;
            physicsData.InvMass[physicsData.ObjectCount] = 0.0f;
            physicsData.Angle[physicsData.ObjectCount] = 0.0f; // ???
            physicsData.Elasticity[physicsData.ObjectCount] = 1.0f;

            physicsData.ObjectCount++;
        }

        public static void AddBall(PhysicsData physicsData, Vector2 pos, Vector2 velocity, float radius, float mass, Vector2 gravity)
        {
            pos *= 100.0f;
            velocity *= 100.0f;
            radius *= 100.0f;

            physicsData.Shape[physicsData.ObjectCount] = SHAPE.CIRCLE;
            physicsData.NumAxis[physicsData.ObjectCount] = 1;

            physicsData.CircleRadius[physicsData.ObjectCount] = radius;
            physicsData.Vertices[physicsData.ObjectCount] = new Vector2[2];

            physicsData.Inertia[physicsData.ObjectCount] = mass * (radius * radius + radius * radius) / 12.0f;
            physicsData.InvInertia[physicsData.ObjectCount] = 1.0f / physicsData.Inertia[physicsData.ObjectCount];

            addCommon(physicsData, pos, velocity, mass, gravity);

            physicsData.ObjectCount++;
        }

        public static void AddRect(PhysicsData physicsData, Vector2 pos, Vector2 velocity, float width, float height, float mass, Vector2 gravity)
        {
            pos *= 100.0f;
            width *= 100.0f;
            height *= 100.0f;

            physicsData.Shape[physicsData.ObjectCount] = SHAPE.RECTANGLE;
            physicsData.NumAxis[physicsData.ObjectCount] = 2;

            physicsData.RectWidth[physicsData.ObjectCount] = width;
            physicsData.RectHeight[physicsData.ObjectCount] = height;

            physicsData.Inertia[physicsData.ObjectCount] = mass * (width * width + height * height) / 12.0f;
            physicsData.InvInertia[physicsData.ObjectCount] = 1.0f / physicsData.Inertia[physicsData.ObjectCount];

            addCommon(physicsData, pos, velocity, mass, gravity);

            Quaternion quaternion = Quaternion.Euler(0.0f, 0.0f, physicsData.Angle[physicsData.ObjectCount] * Mathf.Rad2Deg);
            physicsData.Direction[physicsData.ObjectCount] = quaternion * new Vector2(0.0f, 1.0f);
            physicsData.Vertices[physicsData.ObjectCount] = new Vector2[4];
            SetRectVertices(physicsData, physicsData.ObjectCount);

            physicsData.Elasticity[physicsData.ObjectCount] = 0.9f;

            physicsData.ObjectCount++;

        }

        private static void addCommon(PhysicsData physicsData, Vector2 pos, Vector2 velocity, float mass, Vector2 gravity)
        {
            physicsData.Gravity[physicsData.ObjectCount] = gravity;

            physicsData.Position[physicsData.ObjectCount] = pos;
            physicsData.Direction[physicsData.ObjectCount] = new Vector2(0.0f, 1.0f);
            physicsData.Velocity[physicsData.ObjectCount] = velocity;
            physicsData.Mass[physicsData.ObjectCount] = mass;
            physicsData.InvMass[physicsData.ObjectCount] = mass > 0.0f ? 1.0f / mass : 0.0f;
            physicsData.Angle[physicsData.ObjectCount] = 0.0f;
            physicsData.Elasticity[physicsData.ObjectCount] = 1.0f;
        }

        static float maxVel = 0.0f;
        static float maxAngVel = 0.0f;

        static int frame = 0;
        public static unsafe void Tick(PhysicsData physicsData, float dt)
        {
            // bool* collisionHappened = stackalloc bool[physicsData.ObjectCount];
            // for (int i = 0; i < physicsData.ObjectCount; i++)
            //     collisionHappened[i] = false;


            for (int i = 0; i < physicsData.ObjectCount; i++)
            {
                // physicsData.Velocity[i] *= 0.9f;
                if (physicsData.Shape[i] == SHAPE.CIRCLE)
                {
                    Debug.Log(frame + " physicsData.Velocity[" + i + "] " + physicsData.Velocity[i] + " start");
                    physicsData.Velocity[i] *= 1.0f - physicsData.Friction;
                    Debug.Log(frame + " physicsData.Velocity[" + i + "] " + physicsData.Velocity[i] + " post friction");
                    physicsData.Velocity[i] += physicsData.Gravity[i];
                    Debug.Log(frame + " physicsData.Velocity[" + i + "] " + physicsData.Velocity[i] + " post gravity");
                    physicsData.Position[i] += physicsData.Velocity[i] * dt * 60.0f;
                    Debug.Log(frame + " physicsData.Position[" + i + "] " + physicsData.Position[i]);
                    physicsData.Angle[i] += physicsData.AngularVelocity[i] * dt * 60.0f;

                    if (physicsData.Position[i].x < physicsData.WallLeft)
                        physicsData.Position[i].x = physicsData.WallLeft + 1.0f;
                    if (physicsData.Position[i].x > physicsData.WallRight)
                        physicsData.Position[i].x = physicsData.WallRight - 1.0f;
                    if (physicsData.Position[i].y < physicsData.Floor)
                        physicsData.Position[i].y = physicsData.Floor + 1.0f;
                    if (physicsData.Position[i].y > physicsData.Ceiling)
                        physicsData.Position[i].y = physicsData.Ceiling - 1.0f;
                }

                // Debug.Log(frame + " " + i + " " + physicsData.Shape[i].ToString() + " physicsData.Position " + physicsData.Position[i].ToString() + " physicsData.Velocity[i] " + physicsData.Velocity[i].ToString() + " physicsData.Angle " + physicsData.Angle[i].ToString() + " physicsData.AngularVelocity " + physicsData.AngularVelocity[i]);
                // if (physicsData.Velocity[i].magnitude > maxVel)
                //     maxVel = physicsData.Velocity[i].magnitude;
                // if (physicsData.AngularVelocity[i] > maxAngVel)
                //     maxAngVel = physicsData.AngularVelocity[i];

                if (physicsData.Shape[i] == SHAPE.RECTANGLE)
                    SetRectVertices(physicsData, i);
            }
            frame++;

            // Debug.Log("maxVel " + maxVel + " maxAngVel " + maxAngVel);

            int collisionDataSize = physicsData.ObjectCount * 10;
            CollisionData* collisionData = stackalloc CollisionData[collisionDataSize];
            int rectCollisionDataCount = 0;
            for (int i1 = 0; i1 < physicsData.ObjectCount - 1; i1++)
            {
                for (int i2 = i1 + 1; i2 < physicsData.ObjectCount; i2++)
                {
                    if (physicsData.Shape[i1] != SHAPE.WALL || physicsData.Shape[i2] != SHAPE.WALL)
                    {
                        SATOutputData satOutputData;
                        if (SeparatingAxisTheorem(physicsData, i1, i2, out satOutputData) && rectCollisionDataCount < physicsData.ObjectCount)
                            collisionData[rectCollisionDataCount++] = new CollisionData(i1, i2, satOutputData.Axis, satOutputData.Penetration, satOutputData.CollisionVertex);
                        if (rectCollisionDataCount >= collisionDataSize)
                            Debug.LogError("rectCollisionDataCount" + rectCollisionDataCount + " physicsData.ObjectCount " + physicsData.ObjectCount);
                    }
                }
            }

            for (int i = 0; i < rectCollisionDataCount; i++)
            {
                int idx1 = collisionData[i].Idx1;
                int idx2 = collisionData[i].Idx2;

                // penetration resolution
                PenetrationResolution(physicsData, collisionData[i]);
                // todo collision response
                CollisionResponse(physicsData, collisionData[i]);
            }

            // for (int i = 0; i < physicsData.ObjectCount; i++)
            // {
            //     if (!physicsData.Fixed[i])
            //     {
            //         physicsData.Position[i] += physicsData.Velocity[i] * dt;
            //         physicsData.Angle[i] += physicsData.AngularVelocity[i] * dt * Mathf.Rad2Deg;
            //     }

            //     SetRectVertices(physicsData, i);
            // }

            // Vector2 gravity = new Vector2(0.0f, physicsData.GravityY);

            // // apply gravity
            // for (int i = 0; i < physicsData.ObjectCount; i++)
            //     if (collisionHappened[i])
            //     {
            //         physicsData.Velocity[i] *= 0.9f;
            //         physicsData.AngularVelocity[i] *= 0.9f;
            //     }
            //     else if (physicsData.Gravity[i])
            //     {
            //         // modify physicsData.CircleDireciton.Y by adding gravity and normalizing again
            //         physicsData.Velocity[i] += gravity * dt;
            //     }
        }

        public static void SetRectVertices(PhysicsData physicsData, int idx)
        {
            Vector2 pos = physicsData.Position[idx];

            float halfWidth = physicsData.RectWidth[idx] / 2.0f;
            float halfHeight = physicsData.RectHeight[idx] / 2.0f;

            Quaternion quaternion = Quaternion.Euler(0.0f, 0.0f, physicsData.Angle[idx] * Mathf.Rad2Deg);
            physicsData.Direction[idx] = quaternion * new Vector2(0.0f, 1.0f);

            physicsData.Vertices[idx][0] = pos + (Vector2)(quaternion * new Vector3(-halfWidth, -halfHeight));
            physicsData.Vertices[idx][1] = pos + (Vector2)(quaternion * new Vector3(halfWidth, -halfHeight));
            physicsData.Vertices[idx][2] = pos + (Vector2)(quaternion * new Vector3(halfWidth, halfHeight));
            physicsData.Vertices[idx][3] = pos + (Vector2)(quaternion * new Vector3(-halfWidth, halfHeight));

            // physicsData.Vertices[idx * 4 + 0] = pos + (Vector2)(quaternion * new Vector3(-halfWidth, -halfHeight));
            // physicsData.Vertices[idx * 4 + 1] = pos + (Vector2)(quaternion * new Vector3(-halfWidth, halfHeight));
            // physicsData.Vertices[idx * 4 + 2] = pos + (Vector2)(quaternion * new Vector3(halfWidth, halfHeight));
            // physicsData.Vertices[idx * 4 + 3] = pos + (Vector2)(quaternion * new Vector3(halfWidth, -halfHeight));
        }

        public static unsafe bool SeparatingAxisTheorem(PhysicsData physicsData, int idx1, int idx2, out SATOutputData satOutputData)
        {
            satOutputData = new SATOutputData();

            float minOverlap = float.MaxValue;
            int vertexRectIdx = idx1;

            Vector2* axis = stackalloc Vector2[4];
            int axisCount = getAxisForShapes(physicsData, idx1, idx2, axis);
            float firstShapeAxisIdxs = physicsData.NumAxis[idx1];
            Vector2 smallestAxis = axis[0];

            float proj1min;
            float proj1max;
            float proj2min;
            float proj2max;
            Vector2 collisionVertex;

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

            // Debug.Log(physicsData.Shape[idx1].ToString() + " pos " + idx1 + " " + physicsData.Position[idx1].ToString() + " " + physicsData.Shape[idx2].ToString() + " pos " + idx2 + " " + physicsData.Position[idx2].ToSafeString() + " collisionVertex " + collisionVertex.ToString());
            // Debug.Log(physicsData.Shape[idx1].ToString() + " velocity " + physicsData.Velocity[idx1].ToString());
            // Debug.Log(physicsData.Shape[idx2].ToString() + " velocity " + physicsData.Velocity[idx2].ToString());
            // Debug.Log(physicsData.Shape[idx1].ToString() + " direction " + physicsData.Direction[idx1].ToString());
            // Debug.Log(physicsData.Shape[idx2].ToString() + " direction " + physicsData.Direction[idx2].ToString());
            // Debug.Log(physicsData.Shape[idx1].ToString() + " angle " + physicsData.Angle[idx1].ToString());
            // Debug.Log(physicsData.Shape[idx2].ToString() + " angle " + physicsData.Angle[idx2].ToString());

            if (vertexRectIdx == idx2)
            {
                smallestAxis = smallestAxis * -1.0f;
            }

            //Debug.Log("penetration " + satOutputData.Penetration + " axis " + satOutputData.Axis.ToString() + " collisionVertex " + satOutputData.CollisionVertex.ToString());

            satOutputData.Penetration = minOverlap;
            satOutputData.Axis = smallestAxis;
            satOutputData.CollisionVertex = collisionVertex;

            return true;
        }

        private static unsafe int getAxisForShapes(PhysicsData physicsData, int idx1, int idx2, Vector2* axis)
        {
            int axisCount = 0;

            if (physicsData.Shape[idx1] == SHAPE.CIRCLE && physicsData.Shape[idx2] == SHAPE.CIRCLE)
                axis[axisCount++] = (physicsData.Position[idx2] - physicsData.Position[idx1]).normalized;
            else if (physicsData.Shape[idx1] == SHAPE.CIRCLE)
            {
                axis[axisCount++] = (closestVertexToPoint(physicsData, idx2, physicsData.Position[idx1]) - physicsData.Position[idx1]).normalized;
                axis[axisCount++] = VectorPerpendicular2D(physicsData.Direction[idx2]);
                if (physicsData.Shape[idx2] == SHAPE.RECTANGLE)
                    axis[axisCount++] = physicsData.Direction[idx2];
            }
            else if (physicsData.Shape[idx2] == SHAPE.CIRCLE)
            {
                axis[axisCount++] = VectorPerpendicular2D(physicsData.Direction[idx1]);
                if (physicsData.Shape[idx1] == SHAPE.RECTANGLE)
                    axis[axisCount++] = physicsData.Direction[idx1];
                axis[axisCount++] = (closestVertexToPoint(physicsData, idx1, physicsData.Position[idx2]) - physicsData.Position[idx2]).normalized;
            }
            else
            {
                axis[axisCount++] = VectorPerpendicular2D(physicsData.Direction[idx1]);
                if (physicsData.Shape[idx1] == SHAPE.RECTANGLE)
                    axis[axisCount++] = physicsData.Direction[idx1];
                axis[axisCount++] = VectorPerpendicular2D(physicsData.Direction[idx2]);
                if (physicsData.Shape[idx2] == SHAPE.RECTANGLE)
                    axis[axisCount++] = physicsData.Direction[idx2];
            }
            return axisCount;
        }

        private static Vector2 closestVertexToPoint(PhysicsData physicsData, int idx, Vector2 p)
        {
            Vector2 closestVertex = Vector2.zero;
            float minDist = float.MaxValue;
            for (int i = 0; i < physicsData.Vertices[idx].Length; i++)
            {
                float mag = (p - physicsData.Vertices[idx][i]).magnitude;
                if (mag < minDist)
                {
                    closestVertex = physicsData.Vertices[idx][i];
                    minDist = mag;
                }
            }
            return closestVertex;
        }

        private static void getMinMaxForAxis(PhysicsData physicsData, int idx, Vector2 axis, out float min, out float max, out Vector2 collisionVertex)
        {
            if (physicsData.Shape[idx] == SHAPE.CIRCLE)
                setBallVerticesAlongAxis(physicsData, idx, axis);

            min = max = Vector2.Dot(axis, physicsData.Vertices[idx][0]);
            collisionVertex = physicsData.Vertices[idx][0];
            for (int v = 1; v < physicsData.Vertices[idx].Length; v++)
            {
                float p = Vector2.Dot(axis, physicsData.Vertices[idx][v]);
                if (p < min)
                {
                    min = p;
                    collisionVertex = physicsData.Vertices[idx][v];
                }
                if (p > max)
                    max = p;
            }
        }

        private static void setBallVerticesAlongAxis(PhysicsData physicsData, int idx, Vector2 axis)
        {
            physicsData.Vertices[idx][0] = physicsData.Position[idx] + (axis.normalized * -physicsData.CircleRadius[idx]);
            physicsData.Vertices[idx][1] = physicsData.Position[idx] + (axis.normalized * physicsData.CircleRadius[idx]);
        }

        public static void PenetrationResolution(PhysicsData physicsData, CollisionData rectCollisionData)
        {
            // let penResolution = this.normal.mult(this.pen / (this.o1.inv_m + this.o2.inv_m));
            // this.o1.pos = this.o1.pos.add(penResolution.mult(this.o1.inv_m));
            // this.o2.pos = this.o2.pos.add(penResolution.mult(-this.o2.inv_m));

            int idx1 = rectCollisionData.Idx1;
            int idx2 = rectCollisionData.Idx2;
            if (physicsData.InvMass[idx1] > 0.0f || physicsData.InvMass[idx2] > 0.0f)
            {
                Vector2 penResolution = rectCollisionData.Perpendicular * (rectCollisionData.Penetration / (physicsData.InvMass[idx1] + physicsData.InvMass[idx2]));
                physicsData.Position[idx1] += penResolution * physicsData.InvMass[idx1];
                physicsData.Position[idx2] += penResolution * -physicsData.InvMass[idx2];
            }
        }

        public static float VectorCross2D(Vector2 v1, Vector2 v2)
        {
            return v1.x * v2.y - v1.y * v2.x;
        }

        public static Vector2 VectorPerpendicular2D(Vector2 v)
        {
            return new Vector2(-v.y, v.x).normalized;
        }

        public static void CollisionResponse(PhysicsData physicsData, CollisionData rectCollisionData)
        {
            int idx1 = rectCollisionData.Idx1;
            int idx2 = rectCollisionData.Idx2;

            //1. Closing velocity
            // let collArm1 = this.cp.subtr(this.o1.comp[0].pos);
            // let rotVel1 = new Vector(-this.o1.angVel * collArm1.y, this.o1.angVel * collArm1.x);
            // let closVel1 = this.o1.vel.add(rotVel1);
            // let collArm2 = this.cp.subtr(this.o2.comp[0].pos);
            // let rotVel2= new Vector(-this.o2.angVel * collArm2.y, this.o2.angVel * collArm2.x);
            // let closVel2 = this.o2.vel.add(rotVel2);

            Vector2 collArm1 = rectCollisionData.CollisionPoint - physicsData.Position[idx1];
            Vector2 rotVel1 = new Vector2(-physicsData.AngularVelocity[idx1] * collArm1.y, physicsData.AngularVelocity[idx1] * collArm1.x);
            Vector2 closVel1 = physicsData.Velocity[idx1] + rotVel1;

            Vector2 collArm2 = rectCollisionData.CollisionPoint - physicsData.Position[idx2];
            Vector2 rotVel2 = new Vector2(-physicsData.AngularVelocity[idx2] * collArm2.y, physicsData.AngularVelocity[idx2] * collArm2.x);
            Vector2 closVel2 = physicsData.Velocity[idx2] + rotVel2;

            // //2. Impulse augmentation
            // let impAug1 = Vector.cross(collArm1, this.normal);
            // impAug1 = impAug1 * this.o1.inv_inertia * impAug1;
            // let impAug2 = Vector.cross(collArm2, this.normal);
            // impAug2 = impAug2 * this.o2.inv_inertia * impAug2;

            float impAug1 = VectorCross2D(collArm1, rectCollisionData.Perpendicular);
            impAug1 = impAug1 * physicsData.InvInertia[idx1] * impAug1;
            float impAug2 = VectorCross2D(collArm2, rectCollisionData.Perpendicular);
            impAug2 = impAug2 * physicsData.InvInertia[idx2] * impAug2;

            // let relVel = closVel1.subtr(closVel2);
            // let sepVel = Vector.dot(relVel, this.normal);
            // let new_sepVel = -sepVel * Math.min(this.o1.elasticity, this.o2.elasticity);
            // let vsep_diff = new_sepVel - sepVel;

            Vector2 retVel = closVel1 - closVel2;
            float sepVel = Vector2.Dot(retVel, rectCollisionData.Perpendicular);
            float newSepVel = -sepVel * Mathf.Min(physicsData.Elasticity[idx1], physicsData.Elasticity[idx2]);
            float vsepDiff = newSepVel - sepVel;

            float invMass1 = physicsData.InvMass[idx1];
            float invMass2 = physicsData.InvMass[idx2];

            // let impulse = vsep_diff / (this.o1.inv_m + this.o2.inv_m + impAug1 + impAug2);
            // let impulseVec = this.normal.mult(impulse);
            float impulse = (invMass1 + invMass2 + impAug1 + impAug2) > 0.0f ? vsepDiff / (invMass1 + invMass2 + impAug1 + impAug2) : 0.0f;
            // if (impulse > 0.0f && impulse < 1.0f)
            //     impulse = 1.1f;
            // else if (impulse < 0.0f && impulse < -1.0f)
            //     impulse = -1.1f;
            // Debug.Log("impulse " + impulse);
            Vector2 impulseVec = rectCollisionData.Perpendicular * impulse;

            // //3. Changing the velocities
            // this.o1.vel = this.o1.vel.add(impulseVec.mult(this.o1.inv_m));
            // this.o2.vel = this.o2.vel.add(impulseVec.mult(-this.o2.inv_m));
            // if (physicsData.Fixed[idx1])
            //     invMass2 = physicsData.InvMass[idx1] + physicsData.InvMass[idx2];
            // if (physicsData.Fixed[idx2])
            //     invMass1 = physicsData.InvMass[idx1] + physicsData.InvMass[idx2];

            physicsData.Velocity[idx1] += impulseVec * invMass1;
            physicsData.Velocity[idx2] -= impulseVec * invMass2;

            // this.o1.angVel += this.o1.inv_inertia * Vector.cross(collArm1, impulseVec);
            // this.o2.angVel -= this.o2.inv_inertia * Vector.cross(collArm2, impulseVec); 
            physicsData.AngularVelocity[idx1] += physicsData.InvInertia[idx1] * VectorCross2D(collArm1, impulseVec);
            physicsData.AngularVelocity[idx2] -= physicsData.InvInertia[idx2] * VectorCross2D(collArm2, impulseVec);

            // Debug.Log("physicsData.AngularVelocity[" + idx1 + "] " + physicsData.AngularVelocity[idx1]); 
            // Debug.Log("physicsData.AngularVelocity[" + idx2 + "] " + physicsData.AngularVelocity[idx2]);
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