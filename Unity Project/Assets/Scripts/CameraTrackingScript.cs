using UnityEngine;

public class CameraTrackingScript : MonoBehaviour
{

    [SerializeField] private Transform tracer;
    private Vector3 relativePosition;

    private void Start()
    {
        relativePosition = transform.position-tracer.transform.position;
    }

    private void Update()
    {
        transform.position = relativePosition + tracer.position;
    }
}
