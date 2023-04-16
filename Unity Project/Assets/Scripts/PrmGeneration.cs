using System.Collections.Generic;
using UnityEngine;
using Random = System.Random;

public class PrmGeneration : MonoBehaviour
{
    #region Variables

    private float _localMultiplicationScale;

    //Visualization Assets
    [Header("Visualization Assets")] [SerializeField]
    public GameObject vertexModel;

    [Header("Visualization Assets")] [SerializeField]
    private LineRenderer lineModel;

    [Header("Visualization Assets")] [SerializeField]
    private string visualizationLayer;

    private int _mappingLayerMask;
    private bool _firstBoot;

    //Mapping Parameters
    [Header("Mapping Parameters")] [SerializeField]
    private ObstacleGenerator obstacleField;

    [Header("Mapping Parameters")] [SerializeField] [Range(100, 5000)]
    private int maxNodeCount;

    [Header("Mapping Parameters")] [SerializeField] [Range(10, 100)]
    public int neighborSearchRadiusInMeters;

    private int obstaclePadding;
    private bool _generateGraph;
    public bool graphGenerated;
    private readonly Vector3 _invalidPosition = new(float.MaxValue, float.MaxValue, float.MaxValue);
    private LayerMask _obstacleLayer;

    private bool mapRotationAngle;

    //Graph Standards
    public class ClassVertexInGraph
    {
        public GameObject Vertex { get; }
        public List<ClassVertexInGraph> NeighborVertices { get; }
        public List<LineRenderer> Edges { get; }

        private readonly Transform _parentTransform;
        private readonly LineRenderer _lineModel;
        private readonly int _visualizationLayer;

        public ClassVertexInGraph(Vector3 vertexPosition, Quaternion rotation, Transform parent,
            int visualizationLayer, LineRenderer lineModel, GameObject vertexModel)
        {
            _parentTransform = parent;
            _lineModel = lineModel;
            _visualizationLayer = visualizationLayer;

            Vertex = Instantiate(vertexModel, _parentTransform);
            Vertex.transform.rotation = rotation;
            Vertex.layer = _visualizationLayer;
            Vertex.transform.position = vertexPosition;

            NeighborVertices = new List<ClassVertexInGraph>();
            Edges = new List<LineRenderer>();
        }

        public void AddNeighbors(List<ClassVertexInGraph> neighbors)
        {
            foreach (var neighbor in neighbors)
            {
                NeighborVertices.Add(neighbor);
                var line = Instantiate(_lineModel, _parentTransform);
                line.gameObject.layer = _visualizationLayer;
                line.GetComponent<LineRenderer>().SetPosition(0, Vertex.transform.position);
                line.GetComponent<LineRenderer>().SetPosition(1, neighbor.Vertex.transform.position);
                Edges.Add(line);
            }
        }

        public void Destroy_Vertex_GameObject()
        {
            Destroy(Vertex);
            foreach (var neighbor in NeighborVertices)
                Destroy(neighbor.Vertex);
            foreach (var edge in Edges)
                Destroy(edge);
            NeighborVertices.Clear();
            Edges.Clear();
        }
    }

    private List<ClassVertexInGraph> _prmVertexGraph;

    #endregion Variables

    #region Main Methods

    private void Start()
    {
        _firstBoot = true;
        _localMultiplicationScale = 1 / transform.localScale.x;
        _obstacleLayer = LayerMask.GetMask("Obstacles");
        neighborSearchRadiusInMeters = (int)Mathf.Pow(neighborSearchRadiusInMeters, 2.0f);
        _prmVertexGraph = new List<ClassVertexInGraph>();
        _mappingLayerMask = LayerMask.NameToLayer(visualizationLayer);
    }

    private readonly List<ClassVertexInGraph> _possibleVertexList = new();

    private int prmVertexIndex;

    private void Update()
    {
        if (obstacleField.environmentGenerated && _generateGraph && !graphGenerated)
        {
            if (_firstBoot)
            {
                _firstBoot = false;
            }
            else
            {
                if (maxNodeCount > 0)
                {
                    var possibleVertexPosition = FetchNewVertexPosition();
                    if (possibleVertexPosition != _invalidPosition)
                    {
                        var rotationAngle = 0;
                        if (mapRotationAngle)
                            rotationAngle = (int)(new Random().Next(0, 360) / 5.0f);
                        var quaternionRotation = Quaternion.Euler(0, rotationAngle, 0);
                        var newVertex = new ClassVertexInGraph(
                            possibleVertexPosition,
                            quaternionRotation,
                            transform,
                            visualizationLayer: _mappingLayerMask,
                            lineModel: lineModel,
                            vertexModel: vertexModel);

                        _possibleVertexList.Add(newVertex);
                        maxNodeCount--;
                    }
                }
                else
                {
                    if (prmVertexIndex >= _possibleVertexList.Count)
                    {
                        _generateGraph = false;
                        graphGenerated = true;
                    }
                    else
                    {
                        AddEdges(_possibleVertexList[prmVertexIndex], neighborSearchRadiusInMeters,
                            _possibleVertexList);
                        prmVertexIndex++;
                    }
                }
            }
        }
    }

    #endregion Main Methods

    #region Custom Methods

    private Vector3 FetchNewVertexPosition()
    {
        Random rnd = new();
        var x = rnd.Next(-125, 125);
        var z = rnd.Next(-125, 125);
        var y = 0.5f;

        Vector3 vertexPosition = new(x, y, z);

        if (Physics.CheckSphere(vertexPosition, 1 * obstaclePadding, _obstacleLayer))
            return _invalidPosition;
        return vertexPosition;
    }

    private void AddEdges(ClassVertexInGraph currentVertex, int sqrSearchRadius,
        List<ClassVertexInGraph> tempVertexList)
    {
        List<ClassVertexInGraph> possibleNeighbors = new();

        foreach (var possibleNeighbor in tempVertexList)
        {
            var distance = (possibleNeighbor.Vertex.transform.position - currentVertex.Vertex.transform.position)
                .sqrMagnitude;
            if (distance < sqrSearchRadius)
                possibleNeighbors.Add(possibleNeighbor);
        }

        possibleNeighbors.Remove(currentVertex);
        if (possibleNeighbors.Count == 0)
        {
            _possibleVertexList.Remove(currentVertex);
            currentVertex.Destroy_Vertex_GameObject();
            return;
        }

        List<ClassVertexInGraph> neighbors = new();

        foreach (var neighbor in possibleNeighbors)
        {
            var currentPosition = currentVertex.Vertex.transform.position;
            var neighborPosition = neighbor.Vertex.transform.position;
            var edgeCreatingIsValid = true;
            for (var i = 0; i < 10; i++)
            {
                var position = Vector3.Lerp(currentPosition, neighborPosition, i / 10.0f);
                if (Physics.CheckSphere(position, 1 * obstaclePadding, _obstacleLayer))
                {
                    edgeCreatingIsValid = false;
                    break;
                }
            }

            if (edgeCreatingIsValid)
                neighbors.Add(neighbor);
        }

        currentVertex.AddNeighbors(neighbors);
        _prmVertexGraph.Add(currentVertex);
    }

    public void GeneratePrmGraph(int obstaclePaddingValue, bool mapRotationAngleFlag)
    {
        obstaclePadding = obstaclePaddingValue;
        mapRotationAngle = mapRotationAngleFlag;
        _generateGraph = true;
    }

    public List<ClassVertexInGraph> GetPrmGraph()
    {
        return _prmVertexGraph;
    }

    #endregion Custom Methods
}
