using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class SteeringBehaviours : MonoBehaviour
{
    private Rigidbody _rb;
    private float _maxSensorRange;
    
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
    public float maxWanderJitter = 20;
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
        // calculate maximum sensor range from the attached colliders (used for obstacle avoidance force)
        foreach (var collider in GetComponents<Collider>())
        {
            if (collider.isTrigger)
            {
                var colliderSize = collider.bounds.max.magnitude;
                if (_maxSensorRange < colliderSize) _maxSensorRange = colliderSize;
            }
        }
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
    
    // "Wander" Steering Behaviour
    public Vector3 Wander()
    {
        // TODO: finish this behaviour
        return Vector3.zero;
    }
    
    // "CollisionAvoidance" Steering Behaviour
    public Vector3 ObstacleAvoidance(IReadOnlyCollection<Collider> obstacles)
    {
        var acceleration = Vector3.zero;
        var ownPosition = transform.position;
        foreach (var obstacle in obstacles)
        {
            // calculate vector pointing at the obstacle and obstacle position
            var obstaclePosition = obstacle.ClosestPoint(ownPosition);
            Debug.DrawLine(ownPosition, obstaclePosition, Color.blue);
            //return ownPosition - obstaclePosition;
            var toObstacle = ownPosition - obstaclePosition;
            // if the angle between toObstacle and the current velocity is more than 90 degree ignore obstacle (its behind the agent)
            if (Vector3.Dot(_rb.velocity, toObstacle) <= 0)
            {
                // calculate relative velocity and time until possible collision
                var relativeVelocity = _rb.velocity;
                if (obstacle.attachedRigidbody) relativeVelocity -= obstacle.attachedRigidbody.velocity;
                var collisionTime = Vector3.Dot(toObstacle, relativeVelocity) /
                                    relativeVelocity.magnitude * relativeVelocity.magnitude;
                Debug.Log(collisionTime);
                if (collisionTime > 0)
                {
                    // check separation between agent and obstacle at the calculated collision time
                    var separation = toObstacle + relativeVelocity * collisionTime;
                    Debug.Log("Check1 complete " + separation);
                    // check if they will collide
                    if (separation.magnitude > (ownPosition - _rb.ClosestPointOnBounds(ownPosition)).magnitude)
                    {
                        // if collision will happen without separation or if collision is happening at the moment
                        if (separation.magnitude <= 0 || toObstacle.magnitude <= 0)
                        {
                            // calculate acceleration based on the current position
                            acceleration += ownPosition - obstaclePosition;
                        }
                        // else calculate acceleration based on the future position
                        else acceleration += separation;
                        Debug.Log("Check2 complete " + acceleration);
                    }
                }
            }
        }
        Debug.DrawLine(transform.position, acceleration, Color.red);
        return acceleration.normalized * maxObstacleAvoidanceVelocity;
    }
    
    public Vector3 ObstacleAvoidanceOwn(IReadOnlyCollection<Collider> obstacles)
    {
        var acceleration = Vector3.zero;
        var ownPosition = transform.position;
        foreach (var obstacle in obstacles)
        {
            
            // calculate vector pointing at the obstacle and obstacle position
            var obstaclePosition = obstacle.ClosestPoint(ownPosition);
            var toObstacle = obstaclePosition - ownPosition;
            if (Vector3.Dot(_rb.velocity, toObstacle) >= 0)
            {
                // calculate force multiplier based on obstacle distance
                var forceMultiplier = 2 - toObstacle.magnitude / _maxSensorRange;
                
                Debug.DrawLine(transform.position, transform.position + _rb.velocity, Color.blue);
                Debug.DrawLine(transform.position, obstaclePosition, Color.red);
                Debug.DrawLine(transform.position, transform.position + Vector3.Cross(toObstacle, _rb.velocity), Color.black);

                var forceDir = Vector3.Cross(toObstacle, transform.position + Vector3.Cross(toObstacle, _rb.velocity));
                Debug.DrawLine(transform.position, forceDir - transform.position, Color.yellow);
                acceleration += forceMultiplier * forceDir;
            }
        }
        return acceleration.normalized * maxObstacleAvoidanceVelocity;
    }
    
    // "Hide" Steering Behaviour
    public Vector3 Hide()
    {
        // TODO: finish this behaviour
        return Vector3.zero;
    }
}