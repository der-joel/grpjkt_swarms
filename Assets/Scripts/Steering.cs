using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Steering : MonoBehaviour
{
    [Header("General")]
    private Rigidbody _rb;
    [Header("Movement")]
    // TODO: maybe define different min/max for all seek 
    [Header("Seek")]
    public float maxSeekVelocity;
    [Header("Arrival")]
    public float maxArrivalVelocity;
    public float arrivalSlowingDistance;
    [Header("Cohesion")]
    public float maxCohesionVelocity;
    [Header("Separation")]
    public float maxSeparationVelocity;
    [Header("Alignment")]
    public float maxAlignmentVelocity;
    
    // "Seek" Steering Behaviour
    public Vector3 Seek(Vector3 targetPosition)
    {
        // calculate normalized vector pointing at the target position
        // multiply it with the max seek velocity
        return Vector3.Normalize(targetPosition - transform.position) * maxSeekVelocity;
    }
    
    // "Arrival" Steering Behaviour
    private Vector3 Arrival(Vector3 targetPosition)
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
    private Vector3 Cohesion(IReadOnlyCollection<Agent> neighbors)
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
    private Vector3 Separation(IReadOnlyCollection<Agent> neighbors)
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
        // return a vector pointing at the desired position
        return acceleration;
    }
    
    // "Alignment" Steering Behaviour
    private Vector3 Alignment(IReadOnlyCollection<Agent> neighbors)
    {
        if (neighbors.Count <= 0) return Vector3.zero;
        // calculate average neighbor velocity
        var neighborVelocity = new Vector3(
            neighbors.Average(o => o.GetVelocity().x),
            neighbors.Average(o => o.GetVelocity().y),
            neighbors.Average(o => o.GetVelocity().z));
        return neighborVelocity;
    }
}