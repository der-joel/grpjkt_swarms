using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class SteeringBehaviours : MonoBehaviour
{
    private Rigidbody _rb;
    private float _maxSensorRange;
    private Vector3 _wanderTarget;
    
    // TODO: default to "perfect" velocity
    [Header("Seek")]
    public float maxSeekVelocity = 3.5f;
    [Header("Arrival")]
    public float maxArrivalVelocity = 3.5f;
    public float arrivalSlowingDistance = 3.5f;
    [Header("Cohesion")]
    public float maxCohesionVelocity = 3.5f;
    [Header("Separation")]
    public float maxSeparationVelocity = 3.5f;
    [Header("Alignment")]
    // TODO: use or remove
    public float maxAlignmentVelocity = 3.5f;
    [Header("Wander")]
    public float wanderJitter = 20;
    public float wanderCircleRadius = 1.2f;
    public float wanderDistance = 2f;
    public float maxWanderVelocity = 3.5f;
    [Header("Flee")]
    public float maxFleeDistance = 20;
    public float maxFleeVelocity = 3.5f;
    [Header("Obstacle Avoidance")]
    public float maxObstacleAvoidanceVelocity = 3.5f;

    // Start is called before the first frame update
    void Start()
    {
        // set variables to the attached components
        _rb = GetComponent<Rigidbody>();
        // initialize wander circle
        var theta = Random.value * 2 * Mathf.PI;
        _wanderTarget = new Vector3(wanderCircleRadius * Mathf.Cos(theta), 0f, wanderCircleRadius * Mathf.Sin(theta));
        // calculate maximum sensor range from the attached colliders (used for obstacle avoidance force)
        foreach (var collider in GetComponents<Collider>())
        {
            if (collider.isTrigger)
            {
                var colliderBounds = collider.bounds;
                var colliderSize = Vector3.Magnitude(colliderBounds.max - colliderBounds.center);
                if (_maxSensorRange < colliderSize) _maxSensorRange = colliderSize;
            }
        }
        _maxSensorRange = 10;
    }
    
    // "Seek" Steering Behaviour
    public Vector3 Seek(Vector3 targetPosition)
    {
        // calculate normalized vector pointing at the target position
        // multiply it with the max seek velocity
        return Vector3.Normalize(targetPosition - transform.position) * maxSeekVelocity;
    }
    
    // "Arrival" Steering Behaviour
    public Vector3 Arrival(Vector3 targetPosition)
    {
        var ownPosition = transform.position;
        var targetDistance = Vector3.Distance(targetPosition, ownPosition);
        // calculate desired velocity
        float desiredVelocity;
        if (targetDistance > arrivalSlowingDistance)
        {
            desiredVelocity = maxArrivalVelocity * (targetDistance / arrivalSlowingDistance);
        }
        else
        {
            desiredVelocity = maxArrivalVelocity;
        }
        // multiply normalized vector pointing at the target position with the desired velocity
        return Vector3.Normalize(targetPosition - ownPosition) * desiredVelocity;
    }
    
    // "Cohesion" Steering Behaviour
    public Vector3 Cohesion(IReadOnlyCollection<Collider> neighbors)
    {
        if (neighbors.Count <= 0) return Vector3.zero;
        // calculate center of all neighbors
        var neighborCenter = new Vector3(
            neighbors.Average(o => o.transform.position.x),
            neighbors.Average(o => o.transform.position.y),
            neighbors.Average(o => o.transform.position.z));
        // multiply normalized vector pointing at the neighbors center of mass with the maximum velocity
        return Vector3.Normalize(neighborCenter - transform.position) * maxCohesionVelocity;
    }
    
    // "Separation" Steering Behaviour
    public Vector3 Separation(IReadOnlyCollection<Collider> neighbors)
    {
        var acceleration = Vector3.zero;
        var ownPosition = transform.position;
        // sum up repulsive forces of all neighbors
        foreach (var neighbor in neighbors)
        {
            var neighborPosition = neighbor.transform.position;
            // calculate repulsive force for this neighbor
            var repulsiveForce = Vector3.Normalize(ownPosition - neighborPosition) /
                                 Mathf.Pow(Vector3.Distance(ownPosition, neighborPosition), 2);
            acceleration += repulsiveForce;
            /* another possible solution is calculating the repulsive force as float
             and multiply it with the normalized vector pointing at the neighbor:
             
             var strength = sepMaxAcceleration * (maxSepDist - dist) / (maxSepDist - rb.Radius - r.Radius);
             acceleration += Vector3.Normalize(neighborPosition - ownPosition) * strength;
             */
        }
        // multiply normalized sum of all repulsive forces with maximum velocity
        // TODO: which return statement is better?
        return acceleration.normalized * maxSeparationVelocity;
        // return acceleration;
    }
    
    // "Alignment" Steering Behaviour
    public Vector3 Alignment(IReadOnlyCollection<Collider> neighbors)
    {
        if (neighbors.Count <= 0) return Vector3.zero;
        // calculate average neighbor velocity
        var neighborVelocity = new Vector3(
            neighbors.Average(o => o.attachedRigidbody.velocity.x),
            neighbors.Average(o => o.attachedRigidbody.velocity.y),
            neighbors.Average(o => o.attachedRigidbody.velocity.z));
        return neighborVelocity;
    }

    // "Flee" Steering Behaviour
    public Vector3 Flee(Vector3 targetPosition)
    {
        var ownPosition = transform.position;
        // if the predator is out of range don't flee
        if (Vector3.Distance(targetPosition, ownPosition) > maxFleeDistance) return Vector3.zero;
        // else flee
        return Vector3.Normalize(ownPosition - targetPosition) * maxFleeVelocity;
        
    }

    // "CollisionAvoidance" Steering Behaviour
    public Vector3 ObstacleAvoidance(IReadOnlyCollection<Collider> obstacles)
    {
        var acceleration = Vector3.zero;
        var ownPosition = transform.position;
        foreach (var obstacle in obstacles)
        {
            // calculate obstacle position (closest point on the surface) and vector pointing at the obstacle 
            var obstaclePosition = obstacle.ClosestPoint(ownPosition);
            var toObstacle = obstaclePosition - ownPosition;
            Debug.DrawLine(ownPosition, obstaclePosition, Color.red);
            // if the obstacle is in front of the agent
            if (Vector3.Dot(_rb.velocity, toObstacle) >= 0)
            {
                // calculate force multiplier based on obstacle distance in relation to maximum trigger size
                var forceMultiplier = maxObstacleAvoidanceVelocity * ((_maxSensorRange - toObstacle.magnitude) / _maxSensorRange);

                // calculate auxiliary vector that is orthogonal to current velocity and vector pointing at the obstacle
                var aux = Vector3.Cross(_rb.velocity, toObstacle);
                
                // calculate direction of the repulsive force for the given obstacle
                var forceDirection = ownPosition + Vector3.Cross(toObstacle, aux);
                
                acceleration += forceDirection.normalized * forceMultiplier;
                Debug.DrawLine(ownPosition, ownPosition + (forceDirection.normalized * forceMultiplier), Color.blue);
            }
        }
        return acceleration;
    }
    
    // "Wander" Steering Behaviour
    public Vector3 Wander()
    {
        var ownPosition = transform.position;
        // displace target a random amount using jitter
        var jitter = wanderJitter * Time.deltaTime;
        _wanderTarget += new Vector3(Random.Range(-1f, 1f) * jitter, 0f, Random.Range(-1f, 1f) * jitter);
        // project target back onto circle
        _wanderTarget.Normalize();
        _wanderTarget *= wanderCircleRadius;
        // calculate target for the agent and return vector pointing at it
        var target = ownPosition + transform.forward * wanderDistance + _wanderTarget;;
        return Vector3.Normalize(target - ownPosition) * maxWanderVelocity;
    }
    
    // "Hide" Steering Behaviour
    public Vector3 Hide()
    {
        // TODO: finish this behaviour
        return Vector3.zero;
    }
}