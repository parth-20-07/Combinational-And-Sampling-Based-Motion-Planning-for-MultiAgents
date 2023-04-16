using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Random = UnityEngine.Random;

public class FireTruck : MonoBehaviour
{
    #region Variables

    //Ignition Parameters
    [Header("Ignition Parameters")] [SerializeField]
    private float minimumTimeNeededToExtinguishObstacles;

    [Header("Ignition Parameters")] [SerializeField]
    private float extinguisherRadiusInM;

    private float _extinguisherInitiationTimeInSeconds;

    //Ackermann Steering Parameters
    [Header("Ackermann Steering Parameters")] [SerializeField]
    private float carWidthInM;

    [Header("Ackermann Steering Parameters")] [SerializeField]
    private float carLengthInM;

    [Header("Ackermann Steering Parameters")] [SerializeField]
    private float carWheelbaseInM;

    [Header("Ackermann Steering Parameters")] [SerializeField]
    private int minimumTurningRadiusInM;

    [Header("Ackermann Steering Parameters")] [SerializeField]
    private int maximumVelocityInMps;

    private readonly int steeringCost = 1000;
    private readonly int velocityCost = 100;


    private int maximumSteeringAngle;

    private List<(int Velocity, int SteeringAngle, int Cost)>
        _possibleCombinationsOfMoves; //(Velocity,Steering Angle)

    /*
     * ------------------------------------------------
     * ---------- Ackermann Steering Math -------------
     * ------------------------------------------------
     * Parameters
     * v => car speed
     * theta => rotation angle about y
     * phi => steering angle
     * R => Turning Radius
     * wb => WheelBase
     * tw => Tire Width
     * ------------------------------------------------
     * ------------------------------------------------
     * ------------------------------------------------
     * Formulae
     * dX = v*cos(theta)
     * => X(t) = X(t-1) + v*cos(theta)
     *
     * dZ = v*sin(theta)
     * => Z(t) = Z(t-1) + v*sin(theta)
     *
     * dtheta = (v/carLength)*tan(phi)
     * => theta(t) = theta(t-1) + (v/carLength)*tan(phi)
     *
     * R = (wb/sin(phi)) + (tw/2)
     * Assume tw = 0
     * sin(phi) = wb/R
     * => phi = sin-1(wb/R)
     * ------------------------------------------------
     * ------------------------------------------------
     * ------------------------------------------------
     * Max Limits
     * |v_max| = maximumVelocityInMps
     * |phi_max| = sin-1(carWheelbaseInM/minimumTurningRadiusInM)
     * ------------------------------------------------
     * ------------------------------------------------
     * ------------------------------------------------
     */

    //PRM Parameters
    [Header("PRM Parameters")] [SerializeField]
    private PrmGeneration prmGenerator;

    private bool _nodeMappingCompleted;


    //Obstacle Parameters
    [Header("Obstacles Parameters")] [SerializeField]
    private ObstacleGenerator obstacleField;


    //Visualization Parameters
    [Header("Visualization Parameters")] [SerializeField]
    private LineRenderer lineModel;

    [Header("Visualization Parameters")] [SerializeField]
    private string visualizationLayerString;

    [Header("Visualization Parameters")] [SerializeField]
    private Material fireTruckVisualizationMaterial;

    [Header("Visualization Parameters")] [SerializeField]
    private Material defaultVisualizationMaterial;

    [SerializeField] private GameObject fireTruckObject;
    private int _visualizationLayer;
    private bool _firstRun;
    private float _localMultiplicationScale;

    //A* Mapping Parameters
    private class ClassHybridAStarNode
    {
        public GameObject Node { get; }
        public ClassHybridAStarNode ParentNode { get; private set; }
        public List<ClassHybridAStarNode> NeighborNodes { get; }
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
            Node.transform.localScale = new Vector3(1000, 1000, 1000);
            Node.layer = _visualizationLayer;
            _hCost = int.MaxValue; //H-Cost
            _gCost = int.MaxValue; //G-Cost
            _fCost = int.MaxValue; //F-Cost
        }

        public void DestroyNode()
        {
            Destroy(Node);
            ParentNode = null;
        }

        public ClassHybridAStarNode(GameObject prmNode, Transform parentTransform, int visualizationLayer,
            LineRenderer lineModel, Material defaultMaterial, Material pathMaterial)
        {
            _parentTransform = parentTransform;
            _visualizationLayer = visualizationLayer;
            _lineModel = lineModel;
            _defaultMaterial = defaultMaterial;
            _pathMaterial = pathMaterial;

            Node = prmNode;
            Node.GetComponent<MeshRenderer>().material = _defaultMaterial;
            Node.layer = _visualizationLayer;

            NeighborNodes = new List<ClassHybridAStarNode>();
            _neighborEdges = new List<LineRenderer>();
            ParentNode = null;
        }

        public void AddNeighbors(List<ClassHybridAStarNode> neighbors, List<LineRenderer> edges)
        {
            foreach (var neighbor in neighbors)
                NeighborNodes.Add(neighbor);

            _neighborEdges = edges;
            foreach (var prmEdge in _neighborEdges)
            {
                prmEdge.gameObject.layer = _visualizationLayer;
                prmEdge.material = _defaultMaterial;
            }
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

        public void SetParent(ClassHybridAStarNode parentNode)
        {
            ParentNode = parentNode;
            if (parentNode != null)
            {
                _pathTravelEdge = Instantiate(_lineModel, _parentTransform);
                _pathTravelEdge.GetComponent<LineRenderer>().SetPosition(0, Node.transform.position);
                _pathTravelEdge.GetComponent<LineRenderer>().SetPosition(1, parentNode.Node.transform.position);
                _pathTravelEdge.gameObject.layer = _visualizationLayer;
            }
        }

        public void SelectNodeForPath()
        {
            Node.GetComponent<MeshRenderer>().material = _pathMaterial;
            Node.layer = 0;
            Node.transform.localScale = new Vector3(1.5f, 1.5f, 1.5f);

            _pathTravelEdge.gameObject.layer = 0;
            _pathTravelEdge.widthMultiplier = 0.5f;
            _pathTravelEdge.material = _pathMaterial;
        }
    }

    private bool _generateNewPath, _pathGenerated, _motionInitiated, _motionCompleted;

    private List<ClassHybridAStarNode> _aStarNodeGraph, _returnPath, _pseudoNodes;

    //Temporary Variables
    private readonly List<ClassHybridAStarNode> _tempAStarNodeGraph = new();
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
        _firstRun = true;
        _returnPath = new List<ClassHybridAStarNode>();
        _pseudoNodes = new List<ClassHybridAStarNode>();
        _localMultiplicationScale = 1 / transform.localScale.x;
        _visualizationLayer = LayerMask.NameToLayer(visualizationLayerString);
        extinguisherRadiusInM = Mathf.Pow(extinguisherRadiusInM, 2);
        obstacleField.SetExtinguisherRadius(extinguisherRadiusInM);
        _aStarNodeGraph = new List<ClassHybridAStarNode>();

        var possibleVelocityInMps = new List<int>
        {
            -maximumVelocityInMps,
            (int)(-(4.0f / 5.0f) * maximumVelocityInMps),
//            (int)(-(3.0f / 5.0f) * maximumVelocityInMps),
            (int)(-(2.0f / 5.0f) * maximumVelocityInMps),
//            (int)(-(1.0f / 5.0f) * maximumVelocityInMps),
            (int)(-(1.0f / 10.0f) * maximumVelocityInMps),
            (int)(1.0f / 10.0f) * maximumVelocityInMps,
//            (int)(1.0f / 5.0f) * maximumVelocityInMps,
            (int)(2.0f / 5.0f) * maximumVelocityInMps,
//            (int)(3.0f / 5.0f) * maximumVelocityInMps,
            (int)(4.0f / 5.0f) * maximumVelocityInMps,
            maximumVelocityInMps
        };

        maximumSteeringAngle = (int)(Mathf.Asin(carWheelbaseInM / minimumTurningRadiusInM) * (180.0f / Mathf.PI));
        var possibleSteeringAngle = new List<int>
        {
            -maximumSteeringAngle,
            (int)(-(4.0f / 5.0f) * maximumSteeringAngle),
//            (int)(-(3.0f / 5.0f) * maximumSteeringAngle),
            (int)(-(2.0f / 5.0f) * maximumSteeringAngle),
//            (int)(-(1.0f / 5.0f) * maximumSteeringAngle),
            (int)(-(1.0f / 10.0f) * maximumSteeringAngle),
            0,
            (int)(1.0f / 10.0f) * maximumSteeringAngle,
//            (int)(1.0f / 5.0f) * maximumSteeringAngle,
            (int)(2.0f / 5.0f) * maximumSteeringAngle,
//            (int)(3.0f / 5.0f) * maximumSteeringAngle,
            (int)(4.0f / 5.0f) * maximumSteeringAngle,
            maximumSteeringAngle
        };

        _possibleCombinationsOfMoves = new List<(int Velocity, int SteeringAngle, int Cost)>();
        foreach (var velocity in possibleVelocityInMps)
        foreach (var steeringAngle in possibleSteeringAngle)
        {
            var cost = 0;
            if (velocity < 0)
                cost += -velocity * 3 * velocityCost;
            else
                cost += velocity * velocityCost;

            if (steeringAngle != 0)
                cost += Mathf.Abs(steeringAngle) * steeringCost;

            _possibleCombinationsOfMoves.Add((velocity, steeringAngle, cost));
        }

        prmGenerator.GeneratePrmGraph((int)carWidthInM, true);
        _nodeMappingCompleted = false;
        _generateNewPath = false;
        _pathGenerated = false;
        _motionInitiated = false;
        _motionCompleted = false;
    }

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
                    ClassHybridAStarNode node = new(
                        tempPrmGraph[prmGraphIndex].Vertex,
                        transform,
                        _visualizationLayer,
                        lineModel,
                        defaultVisualizationMaterial,
                        fireTruckVisualizationMaterial);
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
                    List<ClassHybridAStarNode> neighbors = new();
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
            var burningObstacles = obstacleField.GetIgnitedObstaclesList();
            var burningObstaclesCount = burningObstacles.Count;
            if (burningObstaclesCount > 0)
            {
                var selectedObstacles = Random.Range(0, burningObstaclesCount);
                var obstaclePosition = burningObstacles[selectedObstacles].GetObstacleObject().transform.position;
                Debug.Log("FireTruck: Travel Position: " + obstaclePosition);
                ClassHybridAStarNode endNode;
                (_pathGenerated, endNode) = GenerateAStarGraph(obstaclePosition);
                if (_pathGenerated)
                {
                    _returnPath.Clear();
                    _returnPath = RetracePath(endNode);
                    _generateNewPath = false;
                    _motionInitiated = true;
                }
            }
        }
        else if (_motionInitiated)
        {
            var motionStartTransform = _returnPath[motionNextPositionIndex - 1].Node.transform;
            var nextTransform = _returnPath[motionNextPositionIndex].Node.transform;
            var travelDistance = (motionStartTransform.position - nextTransform.position).magnitude;
            if (travelDistance > 0)
            {
                var duration = travelDistance / maximumVelocityInMps;
                var newPosition = Vector3.Lerp(motionStartTransform.position, nextTransform.position,
                    motionTime / duration);
                fireTruckObject.transform.position = newPosition;
                fireTruckObject.transform.rotation = nextTransform.rotation;
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
                _motionInitiated = false;
                motionNextPositionIndex = 1;
                _extinguisherInitiationTimeInSeconds = Time.realtimeSinceStartup;
                _motionCompleted = true;
            }
        }
        else if (_motionCompleted)
        {
            if (Time.realtimeSinceStartup - _extinguisherInitiationTimeInSeconds >
                minimumTimeNeededToExtinguishObstacles)
            {
                obstacleField.ExtinguishSurroundingObstacles(transform.position);
                ResetHybridAStarNodes();
                _returnPath.Clear();

                _generateNewPath = true;
                _pathGenerated = false;
                _motionInitiated = false;
                _motionCompleted = false;
            }
        }
    }

    #endregion Main Methods

    #region Custom Methods

    private ClassHybridAStarNode GetClosestNode(Vector3 position)
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

    private void ResetHybridAStarNodes()
    {
        foreach (var aStarNode in _aStarNodeGraph)
            aStarNode.Reset();
        foreach (var pseudoNode in _pseudoNodes)
        {
            pseudoNode.Reset();
            pseudoNode.DestroyNode();
        }

        _pseudoNodes.Clear();
    }

    private ClassHybridAStarNode SelectLeastCostNode(List<ClassHybridAStarNode> openList)
    {
        var currentNode = openList[0];
        foreach (var nodeInOpenList in openList)
        {
            float tempHCost = nodeInOpenList.GetHCost(); //H-Cost
            float tempFCost = nodeInOpenList.GetFCost(); //F-Cost

            if (tempFCost < currentNode.GetFCost() ||
                (Math.Abs(tempFCost - currentNode.GetFCost()) < 0.1f * _localMultiplicationScale &&
                 tempHCost < currentNode.GetHCost()))
                currentNode = nodeInOpenList;
        }

        return currentNode;
    }

    private (bool, ClassHybridAStarNode) GenerateAStarGraph(Vector3 endPosition)
    {
        var startNode = GetClosestNode(fireTruckObject.transform.position);
        var endNode = GetClosestNode(endPosition);

        List<ClassHybridAStarNode> openList = new();
        List<ClassHybridAStarNode> closedList = new();

        var startPosition = fireTruckObject.transform.position;
        var startRotation = fireTruckObject.transform.rotation;

        var startGameObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        startGameObject.transform.position = startPosition;
        startGameObject.transform.rotation = startRotation;

        var startAStarNode = new ClassHybridAStarNode(
            startGameObject,
            transform,
            _visualizationLayer,
            lineModel,
            defaultVisualizationMaterial,
            fireTruckVisualizationMaterial);

        var startHCost = (int)((startGameObject.transform.position - endPosition).sqrMagnitude +
                               Mathf.Abs(startGameObject.transform.rotation.eulerAngles.y -
                                         endNode.Node.transform.rotation.eulerAngles.y) * steeringCost);
        var startFCost = startHCost;
        startAStarNode.SetCost(startFCost, startHCost, 0);
        startAStarNode.SetParent(null);

        var currentNode = startNode;
        var gCost = (int)((startAStarNode.Node.transform.position - endNode.Node.transform.position).sqrMagnitude +
                          Mathf.Abs(startAStarNode.Node.transform.rotation.eulerAngles.y -
                                    endNode.Node.transform.rotation.eulerAngles.y) * steeringCost);
        var hCost = (int)((currentNode.Node.transform.position - endNode.Node.transform.position).sqrMagnitude +
                          Mathf.Abs(currentNode.Node.transform.rotation.eulerAngles.y -
                                    endNode.Node.transform.rotation.eulerAngles.y) * steeringCost);
        var fCost = gCost + hCost;
        currentNode.SetCost(fCost, hCost, gCost);
        openList.Add(currentNode);

        while (openList.Any())
        {
            //Find Node with lowest Cost
            currentNode = SelectLeastCostNode(openList);
            closedList.Add(currentNode);
            openList.Remove(currentNode);

            var neighbors = currentNode.NeighborNodes;
            foreach (var neighbor in neighbors)
                if (!closedList.Contains(neighbor))
                {
                    int tempHCost, tempGCost, tempFCost;
                    (tempGCost, tempHCost, tempFCost) = CalculateNeighborCost(currentNode, neighbor, endNode);

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
                        if (tempHCost < extinguisherRadiusInM*5)
                        {
                            Debug.Log("Firetruck Path Found");
                            return (true, neighbor);
                        }
                    }
                }
        }

        Debug.Log("FireTruck Path Not Found");
        return (false, null);
    }

    private (int gCost, int hCost, int fCost) CalculateNeighborCost(ClassHybridAStarNode currentNode,
        ClassHybridAStarNode neighborNode, ClassHybridAStarNode endNode)
    {
        var neighborNodePosition = neighborNode.Node.transform.position;
        var neighborNodeRotation = neighborNode.Node.transform.rotation;

        var hCost = (int)((neighborNodePosition - endNode.Node.transform.position).sqrMagnitude +
                          Mathf.Abs(neighborNodeRotation.eulerAngles.y -
                                    endNode.Node.transform.rotation.eulerAngles.y) *
                          steeringCost);

        var gCost = (int)(currentNode.GetGCost() +
                          ((neighborNodePosition - currentNode.Node.transform.position).sqrMagnitude +
                           Mathf.Abs(neighborNodeRotation.eulerAngles.y -
                                     currentNode.Node.transform.rotation.eulerAngles.y) * steeringCost));

        var fCost = gCost + hCost;

        return (gCost, hCost, fCost);
    }

    private List<ClassHybridAStarNode> InterpolateNodesInPath(ClassHybridAStarNode startNode,
        ClassHybridAStarNode endNode)
    {
        var endNodeFound = false;

        List<ClassHybridAStarNode> openList = new();
        List<ClassHybridAStarNode> closedList = new();
        openList.Add(startNode);

        while (openList.Any() && !endNodeFound)
        {
            var currentNode = SelectLeastCostNode(openList);
            closedList.Add(currentNode);
            openList.Remove(currentNode);

            foreach (var possibleStep in _possibleCombinationsOfMoves)
            {
                int gCost, hCost, fCost;
                Vector3 position;
                Quaternion rotation;

                ((position, rotation), gCost, hCost, fCost) =
                    CalculateCostOfInterpolatedNode(currentNode, endNode, possibleStep);

                var elementInClosedList = false;
                var elementInOpenList = false;
                foreach (var closedElement in closedList)
                    if (position == closedElement.Node.transform.position &&
                        rotation == closedElement.Node.transform.rotation)
                    {
                        elementInClosedList = true;
                        break;
                    }

                if (elementInClosedList)
                    break;

                foreach (var openElement in openList)
                    if (position == openElement.Node.transform.position &&
                        rotation == openElement.Node.transform.rotation)
                    {
                        elementInOpenList = true;
                        if (gCost < openElement.GetGCost())
                        {
                            openElement.SetCost(fCost, hCost, gCost);
                            openElement.SetParent(currentNode);
                            if (hCost <= 10)
                            {
                                endNode.SetParent(openElement);
                                endNodeFound = true;
                            }
                        }

                        break;
                    }

                if (!elementInOpenList)
                {
                    var newNodeGameObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    newNodeGameObject.transform.position = position;
                    newNodeGameObject.transform.rotation = rotation;

                    var newNode = new ClassHybridAStarNode(
                        newNodeGameObject,
                        transform,
                        _visualizationLayer,
                        lineModel,
                        pathMaterial: fireTruckVisualizationMaterial,
                        defaultMaterial: defaultVisualizationMaterial);
                    newNode.SetCost(fCost, hCost, gCost);
                    newNode.SetParent(currentNode);

                    openList.Add(newNode);
                    _pseudoNodes.Add(newNode);
                }
            }
        }

        var retracingNode = endNode;
        var returnList = new List<ClassHybridAStarNode> { retracingNode };
        while (retracingNode.ParentNode != startNode)
        {
            retracingNode = retracingNode.ParentNode;
            returnList.Add(retracingNode);
        }

        returnList.Reverse();
        return returnList;
    }

    private ((Vector3 position, Quaternion rotation), int Gcost, int Hcost, int FCost) CalculateCostOfInterpolatedNode
    (ClassHybridAStarNode currentNode, ClassHybridAStarNode endNode,
        (int Velocity, int SteeringAngle, int Cost) possibleStep)
    {
        var node = currentNode.Node;
        var nodePosition = node.transform.position;
        var nodeRotationInEuler = node.transform.rotation.eulerAngles;
        var theta = (int)nodeRotationInEuler.y;

        /*
         * ------------------------------------------------
         * ---------- Ackermann Steering Math -------------
         * ------------------------------------------------
         * Parameters
         * v => car speed
         * theta => rotation angle about y
         * phi => steering angle
         * R => Turning Radius
         * wb => WheelBase
         * tw => Tire Width
         * ------------------------------------------------
         * ------------------------------------------------
         * ------------------------------------------------
         * Formulae
         * dX = v*cos(theta)
         * => X(t) = X(t-1) + v*cos(theta)
         *
         * dZ = v*sin(theta)
         * => Z(t) = Z(t-1) + v*sin(theta)
         *
         * dtheta = (v/carLength)*tan(phi)
         * => theta(t) = theta(t-1) + (v/carLength)*tan(phi)
         *
         * R = (wb/sin(phi)) + (tw/2)
         * Assume tw = 0
         * sin(phi) = wb/R
         * => phi = sin-1(wb/R)
         * ------------------------------------------------
         * ------------------------------------------------
         * ------------------------------------------------
         * Max Limits
         * |v_max| = maximumVelocityInMps
         * |phi_max| = sin-1(carWheelbaseInM/minimumTurningRadiusInM)
         * ------------------------------------------------
         * ------------------------------------------------
         * ------------------------------------------------
         */
        var tanPhi = (int)(Mathf.Tan(possibleStep.SteeringAngle) * 180 / Mathf.PI);
        var nTheta = theta + possibleStep.Velocity / carLengthInM * tanPhi;
        var nX = (int)(nodePosition.x + possibleStep.Velocity * Mathf.Cos(theta + nTheta));
        var nZ = (int)(nodePosition.z + possibleStep.Velocity * Mathf.Sin(theta + nTheta));
        var newPosition = new Vector3(nX, nodePosition.y, nZ);
        var newRotationInQuaternion = Quaternion.Euler(nodeRotationInEuler.x, nTheta, nodeRotationInEuler.z);

        var tempGCost = (int)((newPosition - nodePosition).sqrMagnitude + (newRotationInQuaternion.eulerAngles.y - nodeRotationInEuler.y) * steeringCost);
        var gCost = currentNode.GetGCost() + tempGCost;

        var hCost = (int)((endNode.Node.transform.position - newPosition).sqrMagnitude +
                          (endNode.Node.transform.rotation.eulerAngles.y - newRotationInQuaternion.eulerAngles.y) * steeringCost);

        var fCost = gCost + hCost + possibleStep.Cost;

        return ((newPosition, newRotationInQuaternion), gCost, hCost, fCost);
    }

    private List<ClassHybridAStarNode> RetracePath(ClassHybridAStarNode endNode)
    {
        List<ClassHybridAStarNode> nonInterpolatedList = new();
        var currentNode = endNode;
        nonInterpolatedList.Add(endNode);

        while (currentNode != null)
        {
            nonInterpolatedList.Add(currentNode);
            currentNode = currentNode.ParentNode;
        }

        nonInterpolatedList.Reverse();


        List<ClassHybridAStarNode> returnPath = new() { nonInterpolatedList[0] };
        for (var i = 1; i < nonInterpolatedList.Count; i++)
        {
            var firstNode = nonInterpolatedList[i - 1];
            var nextNode = nonInterpolatedList[i];

            var interpolatedList = InterpolateNodesInPath(firstNode, nextNode);
            foreach (var interpolatedElement in interpolatedList)
            {
                interpolatedElement.SelectNodeForPath();
                returnPath.Add(interpolatedElement);
            }
        }

        return returnPath;
    }

    #endregion Custom Methods
}
