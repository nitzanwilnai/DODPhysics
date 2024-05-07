using UnityEngine;
using DODPhysics;
using TMPro;

public class Board : MonoBehaviour
{
    public Camera Camera;

    PhysicsData m_physicsData = new PhysicsData();
    public GameObject FloorGO;
    public GameObject CeilingGO;
    public GameObject WallLeftGO;
    public GameObject WallRightGO;

    public Vector2 Gravity;
    public float Friction;

    public int MaxObjects;

    public GameObject CirclePrefab;
    GameObject[] m_circlesGO;

    public GameObject RectPrefab;
    public GameObject[] m_rectGO;
    public SpriteRenderer[] m_rectSR;

    bool m_debug = false;
    bool m_objectText = true;
    public Mesh[] m_rectMesh;
    TextMeshPro[] m_objectTMPro;


    float m_addTime = 0.01f;
    float m_addTimer = 0.0f;

    public bool FrameByFrame = false;

    void Awake()
    {
#if !UNITY_EDITOR
#endif
        // Application.targetFrameRate = 999999;
    }

    // Start is called before the first frame update
    void Start()
    {
        float floor = FloorGO.transform.localPosition.y + FloorGO.transform.localScale.y / 2.0f;
        float ceiling = CeilingGO.transform.localPosition.y + CeilingGO.transform.localScale.y / 2.0f;
        float wallLeft = WallLeftGO.transform.localPosition.x + WallLeftGO.transform.localScale.x / 2.0f;
        float wallRight = WallRightGO.transform.localPosition.x - WallRightGO.transform.localScale.x / 2.0f;

        m_circlesGO = new GameObject[MaxObjects];
        m_rectGO = new GameObject[MaxObjects];
        m_rectSR = new SpriteRenderer[MaxObjects];

        if (m_debug)
            m_rectMesh = new Mesh[MaxObjects];

        if (m_objectText)
            m_objectTMPro = new TextMeshPro[MaxObjects];

        PhysicsLogic.AllocatePhysics(m_physicsData, MaxObjects, Friction);

        // PhysicsLogic.AddLine(m_physicsData, new Vector2(wallLeft, floor), new Vector2(wallRight, floor));

        // addCircle(new Vector2(-1.5f, -2.0f), new Vector2(0.0f, 0.0f), 0.5f, 999.0f);
        // addCircle(new Vector2(-1.6f, 2.0f),  new Vector2(0.0f, -1.0f), 0.25f, 1.0f);

        // addCircle(new Vector2(0.1f, 2.0f), new Vector2(0.0f, 0.0f), 0.5f, 1.0f);
        // addCircle(new Vector2(0.0f, 2.1f),  new Vector2(0.0f, 0.0f), 0.5f, 1.0f);

        int seed = 838587520;
        Debug.Log("seed " + seed);
        Random.InitState(seed);
        // for (int i = 0; i < 30; i++)
        //     addCircle();

        // RECTANGLE pos 0 (2.009793,2.1) CIRCLE pos 2 (-7.606146,2.567009) collisionVertex (1.810378,1.998839)

        // TEST1
        // let Box1 = new Box(200, 200, 240, 200, 20, 1);
        // let Box2 = new Box(100, 200, 120, 200, 40, 1);
        // addRect(new Vector2(2.1f, 2.1f), 0.4f, 0.2f, new Vector2(-1.0f, 0.0f));
        // addRect(new Vector2(0.9f, 2.2f), 0.2f, 0.4f, Vector2.zero);
        // addCircle(new Vector2(0.0f, 0.0f), new Vector2(0.0f, 0.0f), 0.25f, 1.0f);
        // addRect(new Vector2(2.2f, 0.0f), 0.4f, 0.2f, new Vector2(-1.0f, 0.0f));

        // TEST2 - was causing collision
        // addCircle(new Vector2(-8.51f, 3.75f), new Vector2(0.0f, 0.0f), 0.25f, 1.0f);
        // addRect(new Vector2(2.55f, 2.03f), 0.4f, 0.2f, new Vector2(-1.0f, 0.0f));
        // m_physicsData.Velocity[0] = new Vector2(0.34f, 1.32f);
        // m_physicsData.Velocity[1] = new Vector2(-0.99f, 0.12f);
        // m_physicsData.Angle[0] = 142.3341f;
        // m_physicsData.Angle[1] = 55.20527f;

        // addBox(FloorGO.transform.localPosition, FloorGO.transform.localScale.x, FloorGO.transform.localScale.y, Vector2.zero, 1.0f, false);
        // addBox(CeilingGO.transform.localPosition, CeilingGO.transform.localScale.x, CeilingGO.transform.localScale.y, Vector2.zero, 1000.0f, false);
        // addBox(WallLeftGO.transform.localPosition, WallLeftGO.transform.localScale.x, WallLeftGO.transform.localScale.y, Vector2.zero, 1000.0f, false);
        // addBox(WallRightGO.transform.localPosition, WallRightGO.transform.localScale.x, WallRightGO.transform.localScale.y, Vector2.zero, 1000.0f, false);

        // floor = 0.0f;
        // addWall(new Vector2(0.0f, floor), new Vector2(400.0f, floor)); // floor
        addWall(new Vector2(wallLeft, floor), new Vector2(wallRight, floor)); // floor
        addWall(new Vector2(wallLeft, ceiling), new Vector2(wallRight, ceiling)); // ceiling
        addWall(new Vector2(wallLeft, floor), new Vector2(wallLeft, ceiling)); // wall left
        addWall(new Vector2(wallRight, floor), new Vector2(wallRight, ceiling)); // wall right

        for (int i = 0; i < m_physicsData.ObjectCount; i++)
            m_physicsData.Elasticity[i] = 0.8f;

        m_physicsData.Floor = floor * 100.0f;
        m_physicsData.Ceiling = ceiling * 100.0f;
        m_physicsData.WallLeft = wallLeft * 100.0f;
        m_physicsData.WallRight = wallRight * 100.0f;

        for (int i = 0; i < MaxObjects; i++)
        {
            Vector2 pos = new Vector2(Random.value * (wallRight - wallLeft) + wallLeft, Random.value * (ceiling - floor) + floor);
            if (i % 2 == 0)
                addBall(pos, new Vector2(0.0f, 0.0f), 0.25f, 1.0f, Gravity);
            else
                addBox(pos, 0.5f, 0.5f, new Vector2(0.0f, 0.0f), 1.0f, Gravity);
        }

        double time = Time.realtimeSinceStartupAsDouble;
        float dt = 1.0f / 60.0f;
        for (int i = 0; i < 1000; i++)
            PhysicsLogic.Tick(m_physicsData, dt);
        Debug.Log("Time: " + (Time.realtimeSinceStartupAsDouble - time).ToString("G4"));

        double totalTime = 0.0d;
        for (int i = 0; i < 1000; i++)
        {
            time = Time.realtimeSinceStartupAsDouble;
            PhysicsLogic.Tick(m_physicsData, dt);
            totalTime += (Time.realtimeSinceStartupAsDouble - time);
        }
        Debug.Log("Average Time: " + (totalTime / 1000.0f).ToString("G4"));


        // static ball
        // addBall(new Vector2(0.0f, 0.0f), new Vector2(0.0f, 0.0f), 1.0f, 0.0f, new Vector2(0.0f, 0.0f));

        // static box
        // addBox(Vector2.zero, 2.0f, 0.5f, Vector2.zero, 0.0f, Vector2.zero);
        // m_physicsData.Angle[m_physicsData.ObjectCount-1] = 45.0f * Mathf.Deg2Rad;

        // addBall(new Vector2(0.0f, 3.0f), new Vector2(0.0f, 0.0f), 0.5f, 1.0f, new Vector2(0.0f, -0.1f));

        // addBox(new Vector2(1.0f, floor + 0.5f), 1.0f, 1.0f, new Vector2(0.0f, 0.0f), 1.0f, Gravity);
        // m_physicsData.Elasticity[m_physicsData.ObjectCount-1] = 0.9f;
        // addBox(new Vector2(1.95f, floor + 1.5f), 1.0f, 1.0f, new Vector2(0.0f, 0.0f), 1.0f, Gravity);
        // m_physicsData.Elasticity[m_physicsData.ObjectCount-1] = 0.9f;

        // addBox(new Vector2(1.0f, 1.0f), 1.0f, 1.0f, new Vector2(0.0f, 0.0f), 1.0f, Gravity);
        // m_physicsData.Elasticity[m_physicsData.ObjectCount-1] = 1.0f;
        // m_physicsData.Velocity[m_physicsData.ObjectCount-1] = new Vector2(0.0f, 1.0f);
        // addBox(new Vector2(1.95f, 3.0f), 1.0f, 1.0f, new Vector2(0.0f, 0.0f), 1.0f, Gravity);
        // m_physicsData.Elasticity[m_physicsData.ObjectCount-1] = 1.0f;
        // m_physicsData.Velocity[m_physicsData.ObjectCount-1] = new Vector2(0.0f, -1.0f);


    }

    // Update is called once per frame
    void Update()
    {
        // double time = Time.realtimeSinceStartupAsDouble;
        float dt = Time.deltaTime;
        int numIterations = 2;
        if (!FrameByFrame)
            for (int i = 0; i < numIterations; i++)
                PhysicsLogic.Tick(m_physicsData, dt / (float)numIterations);
        // Debug.Log(m_physicsData.CircleCount + " time " + (Time.realtimeSinceStartupAsDouble - time).ToString("G5"));

        // m_addTimer += Time.deltaTime;
        // if (m_addTimer > m_addTime)
        // {
        //     m_addTimer -= m_addTime;
        //     addCircle();
        // }

        if (Input.GetMouseButtonUp(0))
        {
            if (m_physicsData.ObjectCount % 2 == 0)
                addBox(new Vector2(Random.value * 2.0f - 1.0f, 3.0f), 0.5f, 0.5f, new Vector2(0.0f, 0.0f), 1.0f, Gravity);
            else
                addBall(new Vector2(Random.value * 2.0f - 1.0f, 3.0f), new Vector2(0.0f, 0.0f), 0.25f, 1.0f, Gravity);
        }

        if (Input.GetKeyUp(KeyCode.S))
        {
            for (int i = 0; i < m_physicsData.ObjectCount; i++)
                m_physicsData.Gravity[i] = Vector2.zero;
        }
        if (Input.GetKeyUp(KeyCode.D))
        {
            for (int i = 0; i < m_physicsData.ObjectCount; i++)
                m_physicsData.Gravity[i] = Gravity;
        }


        if (FrameByFrame && Input.GetKeyUp(KeyCode.Space))
        {
            PhysicsLogic.Tick(m_physicsData, 1.0f / 60.0f);
        }

        ShowVisual(m_physicsData);
    }

    private void addWall(Vector2 p1, Vector2 p2)
    {
        PhysicsLogic.AddWall(m_physicsData, p1, p2, Vector2.zero);

        GameObject go = Instantiate(RectPrefab);
        m_rectGO[m_physicsData.ObjectCount] = go;
        m_rectSR[m_physicsData.ObjectCount] = go.GetComponentInChildren<SpriteRenderer>();
        m_rectSR[m_physicsData.ObjectCount].gameObject.SetActive(false);

        if (m_debug)
        {
            MeshFilter meshFilter = go.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = go.AddComponent<MeshRenderer>();
            Mesh mesh = new Mesh();
            mesh.vertices = new Vector3[4];
            int[] triangles = { 0, 3, 2, 0, 2, 1 };
            mesh.triangles = triangles;
            meshFilter.mesh = mesh;

            m_rectMesh[m_physicsData.ObjectCount] = mesh;
        }
    }

    private void addBall(Vector2 pos, Vector2 velocity, float radius, float mass, Vector2 gravity)
    {
        if (m_physicsData.ObjectCount < m_physicsData.MaxObjects)
        {
            m_circlesGO[m_physicsData.ObjectCount] = Instantiate(CirclePrefab);
            m_circlesGO[m_physicsData.ObjectCount].GetComponent<SpriteRenderer>().color = Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
            m_circlesGO[m_physicsData.ObjectCount].transform.localPosition = pos;
            m_circlesGO[m_physicsData.ObjectCount].transform.localScale = new Vector3(radius * 2.0f, radius * 2.0f, 1.0f);

            if (m_objectText)
            {
                m_objectTMPro[m_physicsData.ObjectCount] = m_circlesGO[m_physicsData.ObjectCount].GetComponentInChildren<TextMeshPro>();
                m_objectTMPro[m_physicsData.ObjectCount].text = m_physicsData.ObjectCount.ToString();
            }
            else
                m_circlesGO[m_physicsData.ObjectCount].GetComponentInChildren<TextMeshPro>().gameObject.SetActive(false);

            PhysicsLogic.AddCircle(m_physicsData, pos, velocity, radius, mass, gravity);
        }
    }

    private void addBox(Vector2 pos, float width, float height, Vector2 velocity, float mass, Vector2 gravity)
    {
        if (m_physicsData.ObjectCount < m_physicsData.MaxObjects)
        {
            GameObject go = Instantiate(RectPrefab);
            m_rectGO[m_physicsData.ObjectCount] = go;

            m_rectSR[m_physicsData.ObjectCount] = go.GetComponentInChildren<SpriteRenderer>();
            m_rectSR[m_physicsData.ObjectCount].color = Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
            m_rectSR[m_physicsData.ObjectCount].transform.localScale = new Vector3(width, height, 1.0f);
            m_rectSR[m_physicsData.ObjectCount].transform.localPosition = pos;

            if (m_debug)
            {
                MeshFilter meshFilter = go.AddComponent<MeshFilter>();
                Mesh mesh = new Mesh();
                mesh.vertices = new Vector3[4];
                int[] triangles = { 0, 3, 2, 0, 2, 1 };
                mesh.triangles = triangles;
                meshFilter.mesh = mesh;
                m_rectMesh[m_physicsData.ObjectCount] = mesh;
            }

            if (m_objectText)
            {
                m_objectTMPro[m_physicsData.ObjectCount] = go.GetComponentInChildren<TextMeshPro>();
                m_objectTMPro[m_physicsData.ObjectCount].text = m_physicsData.ObjectCount.ToString();
            }
            else
                go.GetComponentInChildren<TextMeshPro>().gameObject.SetActive(false);

            PhysicsLogic.AddRect(m_physicsData, pos, velocity, width, height, mass, gravity);
        }
    }

    void ShowVisual(PhysicsData physicsData)
    {
        for (int i = 0; i < physicsData.ObjectCount; i++)
        {
            if (physicsData.Shape[i] == SHAPE.CIRCLE)
            {
                m_circlesGO[i].transform.localPosition = physicsData.Position[i] / 100.0f;
            }
            else if (m_physicsData.Shape[i] == SHAPE.RECTANGLE)
            {
                if (m_debug)
                {
                    Vector3[] vertices = m_rectMesh[i].vertices;
                    for (int v = 0; v < m_physicsData.Vertices[i].Length; v++)
                        vertices[v] = m_physicsData.Vertices[i][v] / 100.0f;
                    m_rectMesh[i].SetVertices(vertices);
                }

                m_rectSR[i].transform.localPosition = physicsData.Position[i] / 100.0f;
                m_rectSR[i].transform.localRotation = Quaternion.Euler(0.0f, 0.0f, physicsData.Angle[i] * Mathf.Rad2Deg);
            }
        }

    }
}
