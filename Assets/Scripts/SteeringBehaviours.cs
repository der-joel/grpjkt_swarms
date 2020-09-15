using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Random = UnityEngine.Random;

public class SteeringBehaviours
{
    private float _maxSensorRange;
    private Vector3 _wanderTarget;
    public readonly float  maxSeekVelocity;
    public readonly float maxArrivalVelocity;
    public readonly float arrivalSlowingDistance;
    public readonly float maxCohesionVelocity;
    public readonly float maxSeparationVelocity;
    public readonly float maxAlignmentVelocity;
    public readonly float wanderJitter;
    public readonly float wanderCircleRadius;
    public readonly float wanderDistance;
    public readonly float maxWanderVelocity;
    public readonly float maxFleeDistance;
    public readonly float maxFleeVelocity;
    public readonly float maxObstacleAvoidanceVelocity;
    public readonly float hideObstacleDistance;
    public readonly float maxHideVelocity;

    public SteeringBehaviours(float agentMaxSensorRange, float maxSeekVelocity, float maxArrivalVelocity, float arrivalSlowingDistance, float maxCohesionVelocity, float maxSeparationVelocity, float maxAlignmentVelocity, float wanderJitter, float wanderCircleRadius, float wanderDistance, float maxWanderVelocity, float maxFleeDistance, float maxFleeVelocity, float maxObstacleAvoidanceVelocity, float hideObstacleDistance, float maxHideVelocity)
    {
        // set variables
        _maxSensorRange = agentMaxSensorRange;
        this.maxSeekVelocity = maxSeekVelocity;
        this.maxArrivalVelocity = maxArrivalVelocity;
        this.arrivalSlowingDistance = arrivalSlowingDistance;
        this.maxCohesionVelocity = maxCohesionVelocity;
        this.maxSeparationVelocity = maxSeparationVelocity;
        this.maxAlignmentVelocity = maxAlignmentVelocity;
        this.wanderJitter = wanderJitter;
        this.wanderCircleRadius = wanderCircleRadius;
        this.wanderDistance = wanderDistance;
        this.maxWanderVelocity = maxWanderVelocity;
        this.maxFleeDistance = maxFleeDistance;
        this.maxFleeVelocity = maxFleeVelocity;
        this.maxObstacleAvoidanceVelocity = maxObstacleAvoidanceVelocity;
        this.hideObstacleDistance = hideObstacleDistance;
        this.maxHideVelocity = maxHideVelocity;
        // initialize wander circle
        var theta = Random.value * 2 * Mathf.PI;
        _wanderTarget = new Vector3(wanderCircleRadius * Mathf.Cos(theta), 0f, wanderCircleRadius * Mathf.Sin(theta));
    }

    // "Seek" Steering Behaviour
    public Vector3 Seek(Vector3 ownPosition, Vector3 targetPosition)
    {
        // calculate normalized vector pointing at the target position
        // multiply it with the max seek velocity
        return Vector3.Normalize(targetPosition - ownPosition) * maxSeekVelocity;
    }
    
    // "Arrival" Steering Behaviour
    public Vector3 Arrival(Vector3 ownPosition, Vector3 targetPosition)
    {
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
    public Vector3 Cohesion(Vector3 ownPosition, IReadOnlyCollection<Collider> neighbors)
    {
        if (neighbors.Count <= 0) return Vector3.zero;
        // calculate center of all neighbors
        var neighborCenter = new Vector3(
            neighbors.Average(o => o.transform.position.x),
            neighbors.Average(o => o.transform.position.y),
            neighbors.Average(o => o.transform.position.z));
        // multiply normalized vector pointing at the neighbors center of mass with the maximum velocity
        return Vector3.Normalize(neighborCenter - ownPosition) * maxCohesionVelocity;
    }
    
    // "Separation" Steering Behaviour
    public Vector3 Separation(Vector3 ownPosition, IReadOnlyCollection<Collider> neighbors)
    {
        var acceleration = Vector3.zero;
        // sum up repulsive forces of all neighbors
        foreach (var neighbor in neighbors)
        {
            var neighborPosition = neighbor.transform.position;
            // calculate repulsive force for this neighbor
            var repulsiveForce = Vector3.Normalize(ownPosition - neighborPosition) /
                                 Mathf.Pow(Vector3.Distance(ownPosition, neighborPosition), 2);
            acceleration += repulsiveForce;
        }
        // multiply normalized sum of all repulsive forces with maximum velocity
        return acceleration.normalized * maxSeparationVelocity;
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
        // limit velocity
        if (neighborVelocity.magnitude > maxAlignmentVelocity) neighborVelocity = neighborVelocity.normalized * maxAlignmentVelocity;
        return neighborVelocity;
    }

    // "Flee" Steering Behaviour
    public Vector3 Flee(Vector3 ownPosition, Vector3 targetPosition)
    {
        // if the predator is out of range don't flee
        if (Vector3.Distance(targetPosition, ownPosition) > maxFleeDistance) return Vector3.zero;
        // else flee
        return Vector3.Normalize(ownPosition - targetPosition) * maxFleeVelocity;
        
    }

    // "CollisionAvoidance" Steering Behaviour
    public Vector3 ObstacleAvoidance(Vector3 ownPosition, Vector3 currentVelocity, IReadOnlyCollection<Collider> obstacles)
    {
        var acceleration = Vector3.zero;
        foreach (var obstacle in obstacles)
        {
            // calculate obstacle position (closest point on the surface) and vector pointing at the obstacle 
            var obstaclePosition = obstacle.ClosestPoint(ownPosition);
            var toObstacle = obstaclePosition - ownPosition;
            // if the obstacle is in front of the agent
            if (Vector3.Dot(currentVelocity, toObstacle) > 0)
            {
                // calculate force multiplier based on obstacle distance in relation to maximum trigger size
                var forceMultiplier = maxObstacleAvoidanceVelocity * ((_maxSensorRange - toObstacle.magnitude) / _maxSensorRange);

                // calculate direction of the repulsive force for the given obstacle
                // its the current velocity vector reflected by the layer that is orthogonal to the toObstacle vector
                var forceDirection = Vector3.Reflect(currentVelocity, toObstacle);
                
                // uncomment these lines to draw vectors of the reflection layer (normal is yellow)
                //Debug.DrawLine(ownPosition, ownPosition + Vector3.Cross(currentVelocity, toObstacle), Color.black);
                //Debug.DrawLine(ownPosition, ownPosition + Vector3.Cross(toObstacle, Vector3.Cross(currentVelocity, toObstacle)), Color.black);
                //Debug.DrawLine(ownPosition, obstaclePosition, Color.yellow);

                acceleration += forceDirection.normalized * forceMultiplier;
            }
        }
        return acceleration.normalized * maxObstacleAvoidanceVelocity;
    }

    // "Wander" Steering Behaviour
    public Vector3 Wander(Vector3 ownPosition, Vector3 forward)
    {
        // displace target a random amount using jitter
        var jitter = wanderJitter * Time.deltaTime;
        _wanderTarget += new Vector3(Random.Range(-1f, 1f) * jitter, 0f, Random.Range(-1f, 1f) * jitter);
        // project target back onto circle
        _wanderTarget.Normalize();
        _wanderTarget *= wanderCircleRadius;
        // calculate target for the agent and return vector pointing at it
        var target = ownPosition + forward * wanderDistance + _wanderTarget;;
        return Vector3.Normalize(target - ownPosition) * maxWanderVelocity;
    }
    
    // "Hide" Steering Behaviour
    public Vector3 Hide(Vector3 ownPosition, Vector3 targetPosition, IReadOnlyCollection<Collider> obstacles)
    {
        if (obstacles.Count == 0) return Vector3.zero;
        // get closest obstacle
        Collider closestObstacle = null;
        float minDistance = float.MaxValue;
        foreach (var obstacle in obstacles)
        {
            var distance = Vector3.Distance(ownPosition, obstacle.ClosestPoint(ownPosition));
            if (distance < minDistance)
            {
                minDistance = distance;
                closestObstacle = obstacle;
            }
        }
        // calculate point behind obstacle
        var obstaclePosition = closestObstacle.ClosestPoint(targetPosition);
        var toTarget = targetPosition - obstaclePosition;
        if (toTarget == Vector3.zero) return Vector3.zero;
        var toTargetDistance = toTarget.magnitude;
        toTarget.Normalize();
        var behindObstacle = obstaclePosition - 2 * closestObstacle.bounds.max.magnitude * toTarget;
        // cast ray from behind the obstacle to get optimal hiding spot
        RaycastHit hit;
        if (closestObstacle.Raycast(new Ray(behindObstacle, toTarget), out hit,
            2 * closestObstacle.bounds.max.magnitude * toTargetDistance))
        {
            var hidingSpot = hit.point - toTarget * hideObstacleDistance;
            return Vector3.Normalize(hidingSpot - ownPosition) * maxHideVelocity;
        }
        // if the ray hit nothing return zero
        return Vector3.zero;
    }
}