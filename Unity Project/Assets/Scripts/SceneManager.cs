using UnityEngine;

public class SceneManager : MonoBehaviour
{
    [SerializeField] private ObstacleGenerator obstacleField;
    private int startTime;

    private void Start()
    {
        startTime = (int)Time.realtimeSinceStartup;
    }

    private void Update()
    {
        if ((int)Time.realtimeSinceStartup - startTime > 3600)
        {
            int intactObstaclesCount,
                burnedObstaclesCount,
                extinguisedObstaclesCount,
                destroyedObstaclesCount,
                totalObstaclesInstantiatedCount;
            (intactObstaclesCount, burnedObstaclesCount, extinguisedObstaclesCount, destroyedObstaclesCount,
                totalObstaclesInstantiatedCount) = obstacleField.get_simulation_metrics();
            Application.Quit();

            Debug.Log(
                "Intact: "+intactObstaclesCount+
                      " | Burned: "+burnedObstaclesCount+
                      " | Extinguished: "+extinguisedObstaclesCount+
                      " | Destroyed: "+destroyedObstaclesCount+
                      " | Total: "+totalObstaclesInstantiatedCount);
        }
    }
}
