using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Random = UnityEngine.Random;

public class Wumpus : MonoBehaviour
{
    #region Variables

    //Ignition Parameters
    [Header("Ignition Parameters")] [SerializeField]
    private float minimumTimeNeededToIgniteObstacles;

    [Header("Ignition Parameters")] [SerializeField]
    private float ignitionRadiusInM;

    private float _counterToMeasureTimeAtPosition;

    //PRM Parameters
    [Header("PRM Parameters")] [SerializeField]
    private PrmGeneration prmGenerator;

    private bool _nodeMappingCompleted;


    //Obstacle Parameters
    [Header("Obstacles Parameters")] [SerializeField]
    private ObstacleGenerator obstacleField;

    private Vector3 selectedObstaclePosition;

    //Visualization Parameters
    [Header("Visualization Parameters")] [SerializeField]
    private LineRenderer lineModel;

    [Header("Visualization Parameters")] [SerializeField]
    private string visualizationLayerString;

    [Header("Visualization Parameters")] [SerializeField]
    private Material wumpusVisualizationMaterial;

    [Header("Visualization Parameters")] [SerializeField]
    private Material defaultVisualizationMaterial;

    [Header("Visualization Parameters")] [SerializeField]
    private float maxWumpusSpeed;

    private Animator wumpusAnimator;
    private int inMotionHash;
    private int fireObstacleHash;

    private Transform wumpusObject;
    private int _visualizationLayer;
    private bool _firstRun;
    private float _localMultiplicationScale;

    //A* Mapping Parameters
    private class ClassAStarNode
    {
        public GameObject Node { get; }
        public ClassAStarNode ParentNode { get; private set; }
        public List<ClassAStarNode> NeighborNodes { get; }
        private List<LineRenderer> _neighborEdges;
        private LineRenderer _pathTravelEdge;
        private int _hCost = int.MaxValue; //H-Cost
        private int _gCost = int.MaxValue; //G-Cost
        private int _fCost = int.MaxValue; //F-Cost
        private readonly int _visualizationLayer;

        private readonly Transform _parentTransform;
        private readonly Material _defaultMaterial;
        private readonly Material _pathMaterial;
        private readonly LineRenderer _lineModel;

        public void Reset()
        {
            ParentNode = null;
            Destroy(_pathTravelEdge);
            Node.GetComponent<MeshRenderer>().material = _defaultMaterial;
            Node.transform.localScale = new Vector3(1.0f, 1.0f, 1.0f);
            Node.layer = _visualizationLayer;
            _hCost = int.MaxValue; //H-Cost
            _gCost = int.MaxValue; //G-Cost
            _fCost = int.MaxValue; //F-Cost
        }

        public ClassAStarNode(GameObject prmNode, Transform parentTransform, int visualizationLayer,
            LineRenderer lineModel, Material defaultMaterial, Material pathMaterial)
        {
            _parentTransform = parentTransform;
            _visualizationLayer = visualizationLayer;
            _lineModel = lineModel;
            _defaultMaterial = defaultMaterial;
            _pathMaterial = pathMaterial;

            Node = prmNode;
            Node.layer = _visualizationLayer;
            Node.GetComponent<MeshRenderer>().material = _defaultMaterial;


            NeighborNodes = new List<ClassAStarNode>();
            _neighborEdges = new List<LineRenderer>();
            ParentNode = null;
        }

        public void AddNeighbors(List<ClassAStarNode> neighbors, List<LineRenderer> edges)
        {
            foreach (var neighbor in neighbors)
                NeighborNodes.Add(neighbor);

            _neighborEdges = edges;
            foreach (var prmEdge in _neighborEdges)
                prmEdge.gameObject.layer = _visualizationLayer;
        }

        public int GetFCost()
        {
            return _fCost;
        }

        public int GetHCost()
        {
            return _hCost;
        }

        public int GetGCost()
        {
            return _gCost;
        }

        public void SetCost(int newFCost, int newHCost, int newGCost)
        {
            _fCost = newFCost;
            _hCost = newHCost;
            _gCost = newGCost;
        }

        public void SetParent(ClassAStarNode parentNode)
        {
            ParentNode = parentNode;
            _pathTravelEdge = Instantiate(_lineModel, _parentTransform);
            _pathTravelEdge.GetComponent<LineRenderer>().SetPosition(0, Node.transform.position);
            _pathTravelEdge.GetComponent<LineRenderer>().SetPosition(1, parentNode.Node.transform.position);
            _pathTravelEdge.gameObject.layer = _visualizationLayer;
        }

        public void SelectNodeForPath()
        {
            Node.GetComponent<MeshRenderer>().material = _pathMaterial;
            Node.layer = 0;
            Node.transform.localScale = new Vector3(1.5f, 1.5f, 1.5f);

            _pathTravelEdge.gameObject.layer = 0;
            _pathTravelEdge.material = _pathMaterial;
            _pathTravelEdge.widthMultiplier = 0.5f;
        }
    }

    private bool _generateNewPath, _pathGenerated, _motionInitiated, _motionCompleted;
    private List<ClassAStarNode> _aStarNodeGraph, _returnPath;

    //Temporary Variables
    private List<PrmGeneration.ClassVertexInGraph> tempPrmGraph;
    private int prmGraphIndex;
    private bool prmNodesTransferred;

    //Motion Variables
    private float motionTime;
    private int motionNextPositionIndex = 1;

    #endregion Variables

    #region Main Methods

    private void Start()
    {
        wumpusObject = gameObject.transform.Find("Wumpus");
        wumpusAnimator = GetComponentInChildren<Animator>();
        inMotionHash = Animator.StringToHash("InMotion");
        fireObstacleHash = Animator.StringToHash("FireObstacles");

        _firstRun = true;
        _returnPath = new List<ClassAStarNode>();
        _localMultiplicationScale = 1 / transform.localScale.x;
        _visualizationLayer = LayerMask.NameToLayer(visualizationLayerString);
        ignitionRadiusInM = Mathf.Pow(ignitionRadiusInM, 2);
        obstacleField.SetIgnitionRadius(ignitionRadiusInM);
        _aStarNodeGraph = new List<ClassAStarNode>();
        prmGenerator.GeneratePrmGraph(2, false);
    }

    private readonly List<ClassAStarNode> _tempAStarNodeGraph = new();

    private void Update()
    {
        if (!_nodeMappingCompleted && prmGenerator.graphGenerated)
        {
            if (_firstRun)
            {
                _firstRun = false;
                tempPrmGraph = prmGenerator.GetPrmGraph();
            }
            else
            {
                if (!prmNodesTransferred)
                {
                    ClassAStarNode node = new(
                        tempPrmGraph[prmGraphIndex].Vertex,
                        transform,
                        _visualizationLayer,
                        lineModel,
                        defaultVisualizationMaterial,
                        wumpusVisualizationMaterial);
                    _tempAStarNodeGraph.Add(node);
                    prmGraphIndex++;

                    if (prmGraphIndex == tempPrmGraph.Count)
                    {
                        prmNodesTransferred = true;
                        prmGraphIndex = 0;
                    }
                }
                else
                {
                    var prmNode = tempPrmGraph[prmGraphIndex];
                    List<ClassAStarNode> neighbors = new();
                    foreach (var aStarNode in _tempAStarNodeGraph)
                        if (aStarNode.Node.GetInstanceID() == prmNode.Vertex.GetInstanceID())
                        {
                            var prmNeighbors = prmNode.NeighborVertices;
                            foreach (var prmNeighbor in prmNeighbors)
                            foreach (var aStarNeighbor in _tempAStarNodeGraph)
                                if (prmNeighbor.Vertex.GetInstanceID() == aStarNeighbor.Node.GetInstanceID())
                                {
                                    neighbors.Add(aStarNeighbor);
                                    break;
                                }

                            aStarNode.AddNeighbors(neighbors, prmNode.Edges);
                            _aStarNodeGraph.Add(aStarNode);
                            break;
                        }

                    prmGraphIndex++;
                    if (prmGraphIndex == tempPrmGraph.Count)
                    {
                        _nodeMappingCompleted = true;
                        _generateNewPath = true;
                        _pathGenerated = false;
                        _motionInitiated = false;
                        _motionCompleted = false;
                    }
                }
            }
        }
        else if (_generateNewPath)
        {
            var availableObstacles = obstacleField.GetAvailableObstaclesList();
            var listCount = availableObstacles.Count;
            if (listCount > 0)
            {
                var selectedObstacle = Random.Range(0, listCount);
                selectedObstaclePosition = availableObstacles[selectedObstacle].GetObstacleObject().transform.position;
                Debug.Log("Wumpus Target Position: " + selectedObstaclePosition);
                ClassAStarNode endNode;
                (_pathGenerated, endNode) = GenerateAStarGraph(selectedObstaclePosition);
                _returnPath.Clear();
                if (_pathGenerated)
                {
                    _returnPath = retrace_path(endNode);
                    _generateNewPath = false;
                    _pathGenerated = true;
                }
            }

            wumpusAnimator.SetBool(inMotionHash, true);
            _motionInitiated = true;

        }
        else if (_motionInitiated)
        {
            var motionStartTransform = _returnPath[motionNextPositionIndex - 1].Node.transform;
            var nextTransform = _returnPath[motionNextPositionIndex].Node.transform;
            var travelDistance = (motionStartTransform.position - nextTransform.position).magnitude;
            if (travelDistance > 0)
            {
                var duration = travelDistance / maxWumpusSpeed;
                var newPosition = Vector3.Lerp(motionStartTransform.position, nextTransform.position,
                    motionTime / duration);
                wumpusObject.transform.position = newPosition;
                wumpusObject.LookAt(nextTransform.position);
                motionTime += Time.deltaTime;

                if (motionTime >= duration)
                {
                    motionNextPositionIndex += 1;
                    motionTime = 0;
                }
            }
            else
            {
                motionNextPositionIndex += 1;
            }

            if (motionNextPositionIndex >= _returnPath.Count)
            {
                motionNextPositionIndex = 1;
                _counterToMeasureTimeAtPosition = Time.realtimeSinceStartup;
                wumpusAnimator.SetBool(inMotionHash, false);
                wumpusAnimator.SetBool(fireObstacleHash, true);
                wumpusObject.LookAt(selectedObstaclePosition);
                _motionInitiated = false;
                _motionCompleted = true;
            }
        }
        else if (_motionCompleted)
        {
            if (Time.realtimeSinceStartup - _counterToMeasureTimeAtPosition > minimumTimeNeededToIgniteObstacles)
            {
                obstacleField.BurnSurroundingObstacles(wumpusObject.position);
                ResetAStarNodes();
                _generateNewPath = true;
                _pathGenerated = false;
                _motionInitiated = false;
                _motionCompleted = false;
                wumpusAnimator.SetBool(fireObstacleHash, false);
            }
        }
    }

    #endregion Main Methods

    #region Custom Methods

    private ClassAStarNode GetClosestNode(Vector3 position)
    {
        var distance = int.MaxValue;
        var returnNodeID = 0;
        foreach (var node in _aStarNodeGraph)
        {
            var tempDistance = (int)(node.Node.transform.position - position).sqrMagnitude;
            if (tempDistance < distance)
            {
                distance = tempDistance;
                returnNodeID = node.Node.gameObject.GetInstanceID();
            }
        }

        foreach (var node in _aStarNodeGraph)
            if (returnNodeID == node.Node.gameObject.GetInstanceID())
                return node;
        return null;
    }

    private void ResetAStarNodes()
    {
        foreach (var aStarNode in _aStarNodeGraph)
            aStarNode.Reset();
    }

    private (bool, ClassAStarNode) GenerateAStarGraph(Vector3 endPosition)
    {
        List<ClassAStarNode> openList = new();
        List<ClassAStarNode> closedList = new();

        var currentNode = GetClosestNode(wumpusObject.position);
        var hCost = (int)(currentNode.Node.transform.position - endPosition).sqrMagnitude;
        var fCost = hCost;
        currentNode.SetCost(fCost, hCost, 0);
        currentNode.SetParent(currentNode);
        openList.Add(currentNode);

        while (openList.Any())
        {
            //Find Node with lowest Cost
            currentNode = openList[0];
            foreach (var nodeInOpenList in openList)
            {
                float tempHCost = nodeInOpenList.GetHCost(); //H-Cost
                float tempFCost = nodeInOpenList.GetFCost(); //F-Cost

                if (tempFCost < currentNode.GetFCost() ||
                    (Math.Abs(tempFCost - currentNode.GetFCost()) < 0.1f * _localMultiplicationScale &&
                     tempHCost < currentNode.GetHCost()))
                    currentNode = nodeInOpenList;
            }

            closedList.Add(currentNode);
            openList.Remove(currentNode);

            var neighbors = currentNode.NeighborNodes;
            foreach (var neighbor in neighbors)
                if (!closedList.Contains(neighbor))
                {
                    var neighborPosition = neighbor.Node.transform.position;
                    var tempHCost = (int)(neighborPosition - endPosition).sqrMagnitude;
                    var tempGCost = (int)(currentNode.GetGCost() +
                                          (neighborPosition - currentNode.Node.transform.position).sqrMagnitude);
                    var tempFCost = tempGCost + tempHCost;

                    if (!openList.Contains(neighbor))
                    {
                        neighbor.SetCost(tempFCost, tempHCost, tempGCost);
                        neighbor.SetParent(currentNode);
                        openList.Add(neighbor);
                    }
                    else if (tempGCost < neighbor.GetGCost())
                    {
                        neighbor.SetCost(tempFCost, tempHCost, tempGCost);
                        neighbor.SetParent(currentNode);
                        if (tempHCost < ignitionRadiusInM)
                        {
                            Debug.Log("Path Found");
                            return (true, neighbor);
                        }
                    }
                }
        }

        Debug.Log("Path Not Found");
        return (false, null);
    }

    private List<ClassAStarNode> retrace_path(ClassAStarNode endNode)
    {
        List<ClassAStarNode> returnPath = new() { endNode };

        var currentNode = endNode;
        while (currentNode.ParentNode != currentNode)
        {
            currentNode.SelectNodeForPath();
            currentNode = currentNode.ParentNode;
            returnPath.Add(currentNode);
        }

        returnPath.Reverse();
        return returnPath;
    }

    #endregion Custom Methods
}
