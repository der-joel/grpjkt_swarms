using System;
using UnityEngine;
using Random = UnityEngine.Random;

public class Spawn : MonoBehaviour
{
    [Header("Spawn Position")]
    public Vector3 center;
    public Vector3 size;
    [Header("Spawn Properties")]
    public Transform agentPrefab;
    public float agentCount;
    [Header("Swarm Targets")]
    public Transform seekTarget;
    public Transform arrivalTarget;
    public Transform fleeTarget;
    public Transform hideTarget;
    
    // Start is called before the first frame update
    void Start()
    {
        // spawn agents
        for (int i = 0; i < agentCount; i++) SpawnAgent();
    }

    // Spawn a new agent instance
    void SpawnAgent()
    {
        // calculate spawn position
        Vector3 position = new Vector3(
            Random.Range(-size.x/2, size.x/2),
            Random.Range(-size.y/2, size.y/2),
            Random.Range(-size.z/2, size.z/2)) + center;
        // instantiate agent
        var agent = Instantiate(agentPrefab, position, Quaternion.identity).GetComponent<Agent>();
        // set targets
        agent.seekTarget = seekTarget;
        agent.arrivalTarget = arrivalTarget;
        agent.fleeTarget = fleeTarget;
        agent.hideTarget = hideTarget;
    }

    // Draw the spawn area in cyan if the spawner is selected (using gizmos)
    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.cyan;
        Gizmos.DrawCube(center, size);
    }
}
