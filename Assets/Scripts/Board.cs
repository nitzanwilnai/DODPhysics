using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using DODPhysics;

public class Board : MonoBehaviour
{
    PhysicsData m_physicsData = new PhysicsData();
    public GameObject FloorGO;
    public GameObject CeilingGO;
    public GameObject WallLeftGO;
    public GameObject WallRightGO;

    public float Gravity;

    public GameObject CirclePrefab;
    public int MaxCircles;
    GameObject[] m_circlesGO;

    public GameObject RectPrefab;
    public int MaxRects;
    public GameObject[] m_rectGO;
    public Mesh[] m_rectMesh;

    float m_addTime = 0.01f;
    float m_addTimer = 0.0f;



    // Start is called before the first frame update
    void Start()
    {
        m_physicsData.Floor = FloorGO.transform.localPosition.y + FloorGO.transform.localScale.y / 2.0f;
        m_physicsData.Ceiling = CeilingGO.transform.localPosition.y + CeilingGO.transform.localScale.y / 2.0f;
        m_physicsData.WallLeft = WallLeftGO.transform.localPosition.x + WallLeftGO.transform.localScale.x / 2.0f;
        m_physicsData.WallRight = WallRightGO.transform.localPosition.x - WallRightGO.transform.localScale.x / 2.0f;

        m_physicsData.Gravity = Gravity;

        m_circlesGO = new GameObject[MaxCircles];
        m_rectGO = new GameObject[MaxRects];
        m_rectMesh = new Mesh[MaxRects];

        PhysicsLogic.AllocatePhysicsCircles(m_physicsData, MaxCircles);
        PhysicsLogic.AllocatePhysicsRects(m_physicsData, MaxRects);

        // addCircle(new Vector2(-1.5f, -2.0f), new Vector2(0.0f, 0.0f), 0.5f, 999.0f);
        // addCircle(new Vector2(-1.6f, 2.0f),  new Vector2(0.0f, -1.0f), 0.25f, 1.0f);

        // addCircle(new Vector2(0.1f, 2.0f), new Vector2(0.0f, 0.0f), 0.5f, 1.0f);
        // addCircle(new Vector2(0.0f, 2.1f),  new Vector2(0.0f, 0.0f), 0.5f, 1.0f);

        Vector2 p1 = new Vector2(0.1007806f, 3.109174f);
        Vector2 p2 = new Vector2(0.02804411f, 3.194554f);
        Vector2 p3 = new Vector2(0.03291607f, 3.133624f);

        int seed = 838587520;
        Debug.Log("seed " + seed);
        Random.InitState(seed);
        // for (int i = 0; i < 30; i++)
        //     addCircle();

        AddRect(new Vector2(0.0f, 0.0f), 2.0f, 1.0f);
        AddRect(new Vector2(0.0f, 0.0f), 1.0f, 2.0f);
    }

    // Update is called once per frame
    void Update()
    {
        // double time = Time.realtimeSinceStartupAsDouble;
        PhysicsLogic.Tick(m_physicsData, Time.deltaTime);
        // Debug.Log(m_physicsData.CircleCount + " time " + (Time.realtimeSinceStartupAsDouble - time).ToString("G5"));
        ShowVisual(m_physicsData);

        // m_addTimer += Time.deltaTime;
        // if (m_addTimer > m_addTime)
        // {
        //     m_addTimer -= m_addTime;
        //     addCircle();
        // }

        // if (Input.GetMouseButtonUp(0))
        // {
        //     addCircle();
        // }

        if (Input.GetKey(KeyCode.LeftArrow))
        {
            m_physicsData.RectAngle[0]++;
            PhysicsLogic.RotateRect(m_physicsData, 0);
        }
        if (Input.GetKey(KeyCode.RightArrow))
        {
            m_physicsData.RectAngle[0]--;
            PhysicsLogic.RotateRect(m_physicsData, 0);
        }
        if (Input.GetKeyUp(KeyCode.W))
        {
            m_physicsData.RectPosition[0].y += 0.1f;
            PhysicsLogic.RotateRect(m_physicsData, 0);
        }
        if (Input.GetKeyUp(KeyCode.S))
        {
            m_physicsData.RectPosition[0].y -= 0.1f;
            PhysicsLogic.RotateRect(m_physicsData, 0);
        }
        if (Input.GetKeyUp(KeyCode.A))
        {
            m_physicsData.RectPosition[0].x -= 0.1f;
            PhysicsLogic.RotateRect(m_physicsData, 0);
        }
        if (Input.GetKeyUp(KeyCode.D))
        {
            m_physicsData.RectPosition[0].x += 0.1f;
            PhysicsLogic.RotateRect(m_physicsData, 0);
        }

        if (PhysicsLogic.SeparatingAxisTheorem(m_physicsData, 0, 1))
        {
            m_rectGO[0].GetComponent<MeshRenderer>().material.color = Color.red;
            m_rectGO[1].GetComponent<MeshRenderer>().material.color = Color.red;
        }
        else
        {
            m_rectGO[0].GetComponent<MeshRenderer>().material.color = Color.white;
            m_rectGO[1].GetComponent<MeshRenderer>().material.color = Color.white;
        }
    }

    void addCircle()
    {
        float radius = Random.value + 0.25f;
        radius = 0.25f;
        float minX = m_physicsData.WallLeft + radius;
        float maxX = m_physicsData.WallRight - radius;
        float posX = Random.value * (maxX - minX) + minX;
        float posY = 9.0f;
        Vector2 pos = new Vector2(posX, posY);
        pos = new Vector2(Random.value * 4.0f - 2.0f, Random.value * 3.0f + 1.0f);
        Vector2 dir = Vector2.zero;
        addCircle(pos, dir, radius, radius);
    }

    private void addCircle(Vector2 pos, Vector2 dir, float radius, float mass)
    {
        if (m_physicsData.CircleCount < m_physicsData.MaxCircles)
        {
            m_circlesGO[m_physicsData.CircleCount] = Instantiate(CirclePrefab);
            m_circlesGO[m_physicsData.CircleCount].GetComponent<SpriteRenderer>().color = Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f);
            m_circlesGO[m_physicsData.CircleCount].transform.localPosition = pos;
            m_circlesGO[m_physicsData.CircleCount].transform.localScale = new Vector3(radius * 2.0f, radius * 2.0f, 1.0f);

            PhysicsLogic.AddCircle(m_physicsData, pos, dir, radius, mass);
        }
    }

    private void AddRect(Vector2 pos, float width, float height)
    {
        GameObject go = Instantiate(RectPrefab);
        m_rectGO[m_physicsData.RectCount] = go;

        MeshFilter meshFilter = go.AddComponent<MeshFilter>();
        MeshRenderer meshRenderer = go.AddComponent<MeshRenderer>();
        meshRenderer.sharedMaterial = new Material(Shader.Find("Standard"));
        Mesh mesh = new Mesh();
        mesh.vertices = new Vector3[4];
        int[] triangles = { 0, 1, 2, 0, 2, 3 };
        mesh.triangles = triangles;
        meshFilter.mesh = mesh;
        m_rectMesh[m_physicsData.RectCount] = mesh;

        PhysicsLogic.AddRect(m_physicsData, pos, new Vector2(0.0f, 0.0f), width, height, 1.0f);
        //PhysicsLogic.RotateRect(m_physicsData, 0);
    }

    void ShowVisual(PhysicsData physicsData)
    {
        for (int i = 0; i < physicsData.CircleCount; i++)
        {
            m_circlesGO[i].transform.localPosition = physicsData.CirclePosition[i];
        }

        for (int i = 0; i < physicsData.RectCount; i++)
        {
            Vector3[] vertices = m_rectMesh[i].vertices;

            for (int v = 0; v < 4; v++)
                vertices[v] = m_physicsData.RectVertices[i * 4 + v];

            m_rectMesh[i].SetVertices(vertices);
            // m_rectGO[i].GetComponent<MeshFilter>().mesh.SetVertices(vertices);
        }

    }
}
