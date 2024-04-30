using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DODPhysics
{
    public class PhysicsData
    {
        public float Gravity;

        public float WallLeft;
        public float WallRight;
        public float Floor;
        public float Ceiling;

        // circles
        public int MaxCircles;
        public int CircleCount;
        public float[] CircleRadius;
        public Vector2[] CirclePosition;
        public Vector2[] CircleDirection;
        public float[] CircleMass;

        public int MaxRects;
        public int RectCount;
        public float[] RectWidth;
        public float[] RectHeight;
        public Vector2[] RectVertices;
        public Vector2[] RectPosition;
        public Vector2[] RectDirection;
        public Vector2[] RectVelocity;
        public float[] RectAngle;
        public float[] RectAngularVelocity;
        public float[] RectMass;
        public float[] RectInvMass;
        public float[] RectInertia;
        public float[] RectInvInertia;
        public float[] RectElasticity;
    }
}