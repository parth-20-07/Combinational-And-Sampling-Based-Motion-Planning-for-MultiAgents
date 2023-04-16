using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;
using Random = System.Random;

public class ObstacleGenerator : MonoBehaviour
{
    #region Variables

    private float _localScale;

    [Header("Environment Population Parameters")] [SerializeField] [Range(1, 15)]
    private int coveragePercentage;

    [SerializeField] private List<GameObject> possibleObstacles;
    [SerializeField] private List<Material> possibleMaterials;
    private int _obstacleGenerationOption, _obstaclesToInstantiate;
    private LayerMask _obstacleLayer;
    public bool environmentGenerated;

    [Header("Obstacle Behaviour Parameters")] [SerializeField] [Range(1, 15)]
    private int maxBurnTime;

    //Obstacle Management Parameters
    public enum PossibleObstacleState
    {
        ObstacleIntact,
        ObstacleBurning,
        ObstacleDamaged,
        ObstacleDestroyed
    }

    public class ClassObstacle
    {
        private readonly GameObject _obstacleGameObject;
        private PossibleObstacleState _obstacleState;
        private int _ignitionTime;

        public ClassObstacle(GameObject obstacleGameObject, LayerMask obstacleLayer)
        {
            _obstacleGameObject = obstacleGameObject;
            _obstacleGameObject.layer = obstacleLayer;
            _obstacleGameObject.transform.GetChild(0).gameObject.layer = obstacleLayer;
            _obstacleGameObject.transform.GetChild(0).AddComponent<BoxCollider>();
            _obstacleState = PossibleObstacleState.ObstacleIntact;
            _ignitionTime = 0;
        }

        public GameObject GetObstacleObject()
        {
            return _obstacleGameObject;
        }

        public void SetObstacleState(PossibleObstacleState state)
        {
            _obstacleState = state;
        }

        public PossibleObstacleState GetObstacleState()
        {
            return _obstacleState;
        }

        public int GetIgnitionTime()
        {
            return _ignitionTime;
        }

        public void SetIgnitionTime(int ignitionTime)
        {
            _ignitionTime = ignitionTime;
        }
    }

    private List<ClassObstacle> _allObstaclesList;
    private List<ClassObstacle> _availableObstacleList;
    private List<ClassObstacle> _ignitedObstacleList;
    private List<ClassObstacle> _destroyedObstacleList;

    [Header("Materials")] private Material _intactMaterial, _burningMaterial, _damagedMaterial, _destroyedMaterial;

    [Header("Interaction Parameters")] private float _burnRadius, _extinguisherRadius;

    [Header("Evaluation Metrics")] private int _intactObstaclesCount,
        _burnedObstaclesCount,
        _extinguishedObstaclesCount,
        _totalObstaclesInstantiatedCount;

    #endregion Variables

    #region Main Methods

    private void Start()
    {
        _localScale = transform.localScale.x * 1000;
        foreach (var mat in possibleMaterials)
            switch (mat.name)
            {
                case "Intact Material":
                    _intactMaterial = mat;
                    break;
                case "Burning Material":
                    _burningMaterial = mat;
                    break;
                case "Damaged Material":
                    _damagedMaterial = mat;
                    break;
                case "Burnt Material":
                    _destroyedMaterial = mat;
                    break;
                default:
                    Debug.LogError("Material Unknown");
                    break;
            }

        _allObstaclesList = new List<ClassObstacle>();
        _availableObstacleList = new List<ClassObstacle>();
        _destroyedObstacleList = new List<ClassObstacle>();
        _ignitedObstacleList = new List<ClassObstacle>();

        _obstacleLayer = LayerMask.NameToLayer("Obstacles");
        gameObject.layer = _obstacleLayer;
        _obstacleGenerationOption = possibleObstacles.Count;
        _obstaclesToInstantiate = 250 * 250 * coveragePercentage / 100 / (5 * 5 * 4);
    }

    private void Update()
    {
        if (_obstaclesToInstantiate > 0)
        {
            InstantiateObstacle();
            _obstaclesToInstantiate--;
        }
        else
        {
            environmentGenerated = true;
        }

        foreach (var obstacle in from obstacle in _ignitedObstacleList.ToList() let currentTime = (int)Time.realtimeSinceStartup where currentTime - obstacle.GetIgnitionTime() > maxBurnTime select obstacle)
        {
            DestroyObstacle(obstacle);
        }
    }

    #endregion Main Methods

    #region Custom Methods

    private void InstantiateObstacle()
    {
        Random rnd = new();
        float x = rnd.Next((int)(-115 * _localScale), (int)(115 * _localScale));
        float z = rnd.Next((int)(-115 * _localScale), (int)(115 * _localScale));
        const float y = 0;

        var copyObject = possibleObstacles[rnd.Next(_obstacleGenerationOption)];
        Vector3 position = new(x, y, z);
        Vector3 rotationAngle = new(0, rnd.Next(4) * 90, 0);
        var rotation = Quaternion.Euler(rotationAngle);
        var parent = transform;

        ClassObstacle newObstacle = new(Instantiate(copyObject, position, rotation, parent), _obstacleLayer);
        SetMaterialToObstacle(newObstacle.GetObstacleObject(), _intactMaterial);

        _allObstaclesList.Add(newObstacle);
        _availableObstacleList.Add(newObstacle);
    }

    private void IgniteObstacle(ClassObstacle obstacle)
    {
        obstacle.SetObstacleState(PossibleObstacleState.ObstacleBurning);
        obstacle.SetIgnitionTime((int)Time.realtimeSinceStartup);

        _availableObstacleList.Remove(obstacle);
        _ignitedObstacleList.Add(obstacle);
        _burnedObstaclesCount += 1;

        SetMaterialToObstacle(obstacle.GetObstacleObject(), _burningMaterial);
    }

    private void ExtinguishObstacle(ClassObstacle obstacle)
    {
        obstacle.SetObstacleState(PossibleObstacleState.ObstacleDamaged);
        _ignitedObstacleList.Remove(obstacle);
        _availableObstacleList.Add(obstacle);
        _extinguishedObstaclesCount++;

        SetMaterialToObstacle(obstacle.GetObstacleObject(), _damagedMaterial);
    }

    private void DestroyObstacle(ClassObstacle obstacle)
    {
        obstacle.SetObstacleState(PossibleObstacleState.ObstacleDestroyed);
        _ignitedObstacleList.Remove(obstacle);
        _destroyedObstacleList.Add(obstacle);

        SetMaterialToObstacle(obstacle.GetObstacleObject(), _destroyedMaterial);
    }

    private static void SetMaterialToObstacle(GameObject obstacle, Material mat)
    {
        obstacle.GetComponentInChildren<Renderer>().material = mat;
    }

    public void SetIgnitionRadius(float radius)
    {
        _burnRadius = radius;
    }

    public List<ClassObstacle> GetAvailableObstaclesList()
    {
        return _availableObstacleList;
    }

    public void BurnSurroundingObstacles(Vector3 position)
    {
        foreach (var obstacle in from obstacle in _availableObstacleList.ToList() let sqrDistance = (int)(position - obstacle.GetObstacleObject().transform.position).sqrMagnitude where sqrDistance <= _burnRadius select obstacle)
        {
            IgniteObstacle(obstacle);
        }
    }

    public void SetExtinguisherRadius(float radius)
    {
        _extinguisherRadius = radius;
    }

    public List<ClassObstacle> GetIgnitedObstaclesList()
    {
        return _ignitedObstacleList;
    }

    public void ExtinguishSurroundingObstacles(Vector3 position)
    {
        foreach (var obstacle in from obstacle in _ignitedObstacleList.ToList() let sqrDistance = (int)(position - obstacle.GetObstacleObject().transform.position).sqrMagnitude where sqrDistance <= _extinguisherRadius select obstacle)
        {
            ExtinguishObstacle(obstacle);
        }
    }


    public (int intactObstaclesCount, int burnedObstaclesCount, int extinguisedObstaclesCount, int destroyedObstaclesCount, int totalObstaclesInstantiatedCount) get_simulation_metrics()
    {
        foreach (var obstacle in _allObstaclesList.ToList().Where(obstacle => obstacle.GetObstacleState() == PossibleObstacleState.ObstacleIntact))
            _intactObstaclesCount++;

        var destroyedObstaclesCount = _destroyedObstacleList.Count;

        (int intactObstaclesCount, int burnedObstaclesCount, int extinguisedObstaclesCount, int destroyedObstaclesCount,
            int totalObstaclesInstantiatedCount) metrics = new(_intactObstaclesCount, _burnedObstaclesCount,
                _extinguishedObstaclesCount,
                destroyedObstaclesCount, _totalObstaclesInstantiatedCount);
        return metrics;
    }

    #endregion Custom Methods
}
