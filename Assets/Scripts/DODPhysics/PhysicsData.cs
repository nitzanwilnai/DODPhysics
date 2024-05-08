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
        public float[] Radius; // also used for quick collision detection

        // rect
        public float[] RectWidth;
        public float[] RectHeight;

        // common
        public float Friction;

        public int MaxObjects;
        public int ObjectCount;
        public SHAPE[] Shape;
        public Vec2[][] Vertices;
        public int[] NumAxis;

        public Vec2[] Gravity;

        public Vec2[] Position;
        public Vec2[] Direction;
        public Vec2[] Velocity;
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