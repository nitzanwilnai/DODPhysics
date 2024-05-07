using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DODPhysics
{
    public enum SHAPE { WALL, CIRCLE, RECTANGLE };
    public class PhysicsData
    {
        public float WallLeft;
        public float WallRight;
        public float Floor;
        public float Ceiling;

        // circles
        public float[] Radius;

        // rect
        public float[] RectWidth;
        public float[] RectHeight;

        public float Friction;

        // common
        public int MaxObjects;
        public int ObjectCount;
        public SHAPE[] Shape;
        public Vector2[][] Vertices;
        public int[] NumAxis;

        public Vector2[] Gravity;
        public int[] UnFixedIdxs;

        public Vector2[] Position;
        public Vector2[] Direction;
        public Vector2[] Velocity;
        public float[] Angle;
        public float[] AngularVelocity;
        public float[] Mass;
        public float[] InvMass;
        public float[] Inertia;
        public float[] InvInertia;
        public float[] Elasticity;

        public bool[] CollisionHappenedPrevFrame;
    }
}