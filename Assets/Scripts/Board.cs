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
    TextMeshPro[] m_circleTMPro;

    public GameObject RectPrefab;
    public GameObject[] m_rectGO;
    public Mesh[] m_rectMesh;

    float m_addTime = 0.01f;
    float m_addTimer = 0.0f;

    public bool FrameByFrame = false;

    // Start is called before the first frame update
    void Start()
    {
        float floor = FloorGO.transform.localPosition.y + FloorGO.transform.localScale.y / 2.0f;
        float ceiling = CeilingGO.transform.localPosition.y + CeilingGO.transform.localScale.y / 2.0f;
        float wallLeft = WallLeftGO.transform.localPosition.x + WallLeftGO.transform.localScale.x / 2.0f;
        float wallRight = WallRightGO.transform.localPosition.x - WallRightGO.transform.localScale.x / 2.0f;

        m_circlesGO = new GameObject[MaxObjects];
        m_rectGO = new GameObject[MaxObjects];
        m_rectMesh = new Mesh[MaxObjects];

        m_circleTMPro = new TextMeshPro[MaxObjects];

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


        addBall(new Vector2(0.0f, 0.0f), new Vector2(0.0f, 0.0f), 1.0f, 0.0f, new Vector2(0.0f, 0.0f));

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
        int numIterations = 1;
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

        if (Input.GetKeyUp(KeyCode.A))
        {
            // if (Random.value < 0.5f)
            //     addBox(new Vector2(Random.value * 2.0f - 1.0f, 3.0f), 0.5f, 0.5f, new Vector2(0.0f, 0.0f), 1.0f, Gravity);
            // else
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

        // SATOutputData satOutputData;
        // if (PhysicsLogic.SeparatingAxisTheorem(m_physicsData, 0, 1, out satOutputData))
        // {
        //     m_rectGO[0].GetComponent<MeshRenderer>().material.color = Color.red;
        //     m_rectGO[1].GetComponent<MeshRenderer>().material.color = Color.red;
        // }
        // else
        // {
        //     m_rectGO[0].GetComponent<MeshRenderer>().material.color = Color.white;
        //     m_rectGO[1].GetComponent<MeshRenderer>().material.color = Color.white;
        // }

        ShowVisual(m_physicsData);
    }

    // void addBall()
    // {
    //     float radius = Random.value + 0.25f;
    //     radius = 0.25f;
    //     float minX = m_physicsData.WallLeft + radius;
    //     float maxX = m_physicsData.WallRight - radius;
    //     float posX = Random.value * (maxX - minX) + minX;
    //     float posY = 9.0f;
    //     Vector2 pos = new Vector2(posX, posY);
    //     pos = new Vector2(Random.value * 4.0f - 2.0f, Random.value * 3.0f + 1.0f);
    //     Vector2 dir = Vector2.zero;
    //     addBall(pos, dir, radius, radius, Gravity);
    // }

    private void addWall(Vector2 p1, Vector2 p2)
    {
        PhysicsLogic.AddWall(m_physicsData, p1, p2, Vector2.zero);

        GameObject go = Instantiate(RectPrefab);
        m_rectGO[m_physicsData.ObjectCount] = go;
        MeshFilter meshFilter = go.AddComponent<MeshFilter>();
        MeshRenderer meshRenderer = go.AddComponent<MeshRenderer>();
        meshRenderer.sharedMaterial = new Material(Shader.Find("Standard"));
        Mesh mesh = new Mesh();
        mesh.vertices = new Vector3[4];
        int[] triangles = { 0, 3, 2, 0, 2, 1 };
        mesh.triangles = triangles;
        meshFilter.mesh = mesh;

        go.GetComponentInChildren<TextMeshPro>().gameObject.SetActive(false);

        m_rectMesh[m_physicsData.ObjectCount] = mesh;
    }

    private void addBall(Vector2 pos, Vector2 velocity, float radius, float mass, Vector2 gravity)
    {
        if (m_physicsData.ObjectCount < m_physicsData.MaxObjects)
        {
            m_circlesGO[m_physicsData.ObjectCount] = Instantiate(CirclePrefab);
            m_circlesGO[m_physicsData.ObjectCount].GetComponent<SpriteRenderer>().color = Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
            m_circlesGO[m_physicsData.ObjectCount].transform.localPosition = pos;
            m_circlesGO[m_physicsData.ObjectCount].transform.localScale = new Vector3(radius * 2.0f, radius * 2.0f, 1.0f);

            m_circleTMPro[m_physicsData.ObjectCount] = m_circlesGO[m_physicsData.ObjectCount].GetComponentInChildren<TextMeshPro>();
            m_circleTMPro[m_physicsData.ObjectCount].text = m_physicsData.ObjectCount.ToString();

            PhysicsLogic.AddBall(m_physicsData, pos, velocity, radius, mass, gravity);
        }
    }

    private void addBox(Vector2 pos, float width, float height, Vector2 velocity, float mass, Vector2 gravity)
    {
        if (m_physicsData.ObjectCount < m_physicsData.MaxObjects)
        {
            GameObject go = Instantiate(RectPrefab);
            m_rectGO[m_physicsData.ObjectCount] = go;

            MeshFilter meshFilter = go.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = go.AddComponent<MeshRenderer>();
            meshRenderer.sharedMaterial = new Material(Shader.Find("Standard"));
            meshRenderer.sharedMaterial.color = Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
            Mesh mesh = new Mesh();
            mesh.vertices = new Vector3[4];
            int[] triangles = { 0, 3, 2, 0, 2, 1 };
            // int[] triangles = { 0, 1, 2, 0, 2, 3 };
            mesh.triangles = triangles;
            meshFilter.mesh = mesh;
            m_rectMesh[m_physicsData.ObjectCount] = mesh;

            go.GetComponentInChildren<TextMeshPro>().text = m_physicsData.ObjectCount.ToString();

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
                // m_circleTMPro[i].text = i + "\n" + "V " + (m_physicsData.Velocity[i].y * 10.0f).ToString("G0");
            }
            else if (m_physicsData.Shape[i] == SHAPE.RECTANGLE)
            {
                Vector3[] vertices = m_rectMesh[i].vertices;
                for (int v = 0; v < m_physicsData.Vertices[i].Length; v++)
                    vertices[v] = m_physicsData.Vertices[i][v] / 100.0f;
                m_rectMesh[i].SetVertices(vertices);
            }
        }

    }
}
